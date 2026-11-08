#!/usr/bin/env python3
"""RViz 的 2D Pose Estimate（/initialpose）-> 用这个位姿重开 Cartographer 定位轨迹。

Cartographer 是图优化，不订阅 /initialpose；唯一的入口是开新轨迹时（/start_trajectory）带初始位姿。
所以收到 /initialpose 后：结束当前活动的定位轨迹（/finish_trajectory），再以点选的位姿开一条新的。
新轨迹在点选位置周围 1.5 m 的窗口里做局部匹配（见 carto_3d_localization.lua），点得差一点会被纠正，
但要车动起来才纠正：静止时 Cartographer 不插新节点，定位就停在点选的位置（仿真实测点偏 0.58 m，开 15 m 后剩 0.12 m）。

z 取 0（地面）：RViz 点不出高度。车在夹层平台上时要用 navigation.launch.py 的 initial_z 启动。
"""
import math

import rclpy
from cartographer_ros_msgs.msg import TrajectoryStates
from cartographer_ros_msgs.srv import FinishTrajectory, GetTrajectoryStates, StartTrajectory
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

FROZEN_TRAJECTORY = 0       # 加载的建图轨迹；初始位姿相对它的起点（= map 原点）


class InitialPose(Node):
    def __init__(self):
        super().__init__('initial_pose')
        self.config_dir = self.declare_parameter('configuration_directory', '').value
        self.config_base = self.declare_parameter('configuration_basename', 'carto_3d_localization.lua').value
        self.get_states = self.create_client(GetTrajectoryStates, 'get_trajectory_states')
        self.finish = self.create_client(FinishTrajectory, 'finish_trajectory')
        self.start = self.create_client(StartTrajectory, 'start_trajectory')
        self.busy = False
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_pose, 1)

    def on_pose(self, msg):
        if msg.header.frame_id != 'map':
            self.get_logger().error(f'初始位姿要在 map 系下给（收到的是 {msg.header.frame_id!r}，RViz 的 Fixed Frame 设成 map）')
            return
        if self.busy:
            self.get_logger().warn('上一次重定位还没完成，这次忽略')
            return
        if not self.get_states.service_is_ready():
            self.get_logger().error('Cartographer 的服务不在（定位起来了吗？）')
            return
        self.busy = True
        pose = msg.pose.pose
        q = pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.get_logger().info(f'重定位到 ({pose.position.x:.2f}, {pose.position.y:.2f}, {math.degrees(yaw):.1f}°)')
        self.get_states.call_async(GetTrajectoryStates.Request()).add_done_callback(
            lambda f: self.finish_active(f, pose.position.x, pose.position.y, yaw))

    def finish_active(self, future, x, y, yaw):
        states = future.result().trajectory_states
        active = [i for i, s in zip(states.trajectory_id, states.trajectory_state)
                  if s == TrajectoryStates.ACTIVE and i != FROZEN_TRAJECTORY]
        self.finish_next(active, x, y, yaw)

    def finish_next(self, ids, x, y, yaw):
        if not ids:
            self.start_new(x, y, yaw)
            return
        req = FinishTrajectory.Request(trajectory_id=ids[0])
        self.finish.call_async(req).add_done_callback(
            lambda f: (self.log_status('结束定位轨迹 %d' % ids[0], f.result().status),
                       self.finish_next(ids[1:], x, y, yaw)))

    def start_new(self, x, y, yaw):
        req = StartTrajectory.Request()
        req.configuration_directory, req.configuration_basename = self.config_dir, self.config_base
        req.use_initial_pose, req.relative_to_trajectory_id = True, FROZEN_TRAJECTORY
        req.initial_pose.position.x, req.initial_pose.position.y = x, y
        req.initial_pose.orientation.z, req.initial_pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.start.call_async(req).add_done_callback(self.started)

    def started(self, future):
        r = future.result()
        self.log_status('开始定位轨迹 %d' % r.trajectory_id, r.status)
        self.busy = False

    def log_status(self, what, status):
        if status.code == 0:
            self.get_logger().info(f'{what}：OK')
        else:
            self.get_logger().error(f'{what}失败：{status.message}')


def main():
    rclpy.init()
    node = InitialPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

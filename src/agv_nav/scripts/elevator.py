#!/usr/bin/env python3
"""进电梯：导航到门外等待点 -> 点云提取轿厢 -> 沿轿厢中心线开进去、居中停车；或者沿中心线倒出到等待点。

    ros2 run agv_nav elevator.py teach  <电梯文件>   # 示教：车停在门外、车头大致对着门，提取轿厢，写电梯文件
    ros2 run agv_nav elevator.py enter  <电梯文件>   # 导航到等待点 -> 提取 -> 进梯居中；成功返回 0，失败返回 1
    ros2 run agv_nav elevator.py exit   <电梯文件>   # 沿中心线倒出到等待点
    ros2 run agv_nav elevator.py detect <电梯文件>   # 只提取，1 Hz 打印并发布结果，不动车

电梯文件（如 maps/wh/wh_elevator.yaml）是部署时在这张地图上示教出来的轿厢位姿和等待点（map 系），
和定位用的是同一张地图，地图本身的误差两边一致；轿厢尺寸来自电梯规格（参数 cabin_width / cabin_depth）。
进梯这一段以实时点云看到的轿厢为准，不以地图为准：轿厢只比车宽 0.9 m，导航的到点容差和定位误差都太大。
提取和控制都在 odom 系里做（连续、不跳）；电梯文件只用来框选区域。
速度发到 cmd_vel_nav（velocity_smoother 的输入），进出梯期间没有导航目标，导航控制器不发指令。
"""
import math
import os
import sys
import time

import numpy as np
import rclpy
import tf2_ros
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker

import elevator_fit as ef

MODES = ('teach', 'enter', 'exit', 'detect')


class Failed(Exception):
    pass


class Elevator(Node):
    def __init__(self, elevator_file, teach):
        super().__init__('elevator')
        P = self.declare_parameter
        self.elevator_file = elevator_file
        self.odom_frame = P('odom_frame', 'odom').value
        self.base_frame = P('base_frame', 'base_link').value
        self.base_height = P('base_height', 0.183).value
        self.speed_in = P('speed_in', 0.2).value
        self.speed_out = P('speed_out', 0.15).value
        self.lookahead = P('lookahead', 1.0).value
        self.max_curvature = P('max_curvature', 0.4348).value
        self.slow_dist = P('slow_dist', 0.3).value
        self.min_frames = P('min_frames', 5).value
        self.detect_timeout = P('detect_timeout', 10.0).value
        self.stall_timeout = P('stall_timeout', 10.0).value
        self.tol_lateral = P('tol_lateral', 0.05).value
        self.tol_yaw = math.radians(P('tol_yaw_deg', 2.0).value)
        self.tol_depth = P('tol_depth', 0.10).value
        self.footprint_pad = P('footprint_pad', 0.05).value
        self.wait_distance = P('wait_distance', 5.45).value
        self.params = ef.FitParams(width=P('cabin_width', 2.0).value, depth=P('cabin_depth', 2.6).value)
        self.follow = teach             # 示教：先验只是车正前方的粗估计，框选区域跟着提取结果走
        if not teach:
            with open(self.elevator_file) as f:
                e = yaml.safe_load(f)
            self.prior_map = ef.Cabin(e['cabin_center'][0], e['cabin_center'][1], e['cabin_yaw'])
            self.wait_map = tuple(e['wait_point'])

        self.tf_buffer = tf2_ros.Buffer()
        # 不开 spin_thread：那样整个节点会被挂到第二个 executor 上，和这里按顺序执行的 spin_once 抢回调
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, P('cmd_topic', 'cmd_vel_nav').value, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/elevator/pose', 10)
        self.walls_pub = self.create_publisher(Marker, '/elevator/walls', 10)
        self.create_subscription(PointCloud2, P('cloud_topic', '/livox/lidar_PointCloud2').value, self.on_cloud, 10)
        self.create_subscription(PointCloud2, P('obstacle_topic', 'terrain_obstacles').value, self.on_obstacles, 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.prior = None               # 先验，odom 系
        self.wait = None                # 等待点，odom 系
        self.filter = ef.CabinFilter()
        self.rng = np.random.default_rng(0)
        self.obstacles = None           # 最新一帧障碍点，base_link 系 xy
        self.obstacles_time = 0.0       # 收到的时刻（墙钟）

    # ------------------------------------------------------------ TF
    def lookup(self, target, source, when=None):
        """不等待（TF 回调和这里在同一个线程）；点云时刻的 TF 还没到时用最新的；拿不到返回 None"""
        for t in ((when,) if when is not None else ()) + (Time(),):
            try:
                return self.tf_buffer.lookup_transform(target, source, t)
            except tf2_ros.TransformException:
                pass
        return None

    @staticmethod
    def matrix(tf):
        q, t = tf.transform.rotation, tf.transform.translation
        return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(), np.array([t.x, t.y, t.z])

    def base_pose(self):
        """base_link 在 odom 系里的 (x, y, z, yaw)；拿不到返回 None"""
        tf = self.lookup(self.odom_frame, self.base_frame)
        if tf is None:
            return None
        R, t = self.matrix(tf)
        return t[0], t[1], t[2], math.atan2(R[1, 0], R[0, 0])

    def wait_tf(self, target, source):
        """等到 target<-source 和 odom->base_link 都有，返回 (R, t, yaw)；detect_timeout 内拿不到就失败"""
        t0 = time.time()
        while time.time() - t0 < self.detect_timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            tf = self.lookup(target, source)
            if tf is not None and self.base_pose() is not None:
                R, t = self.matrix(tf)
                return R, t, math.atan2(R[1, 0], R[0, 0])
        raise Failed(f'{self.detect_timeout:.0f} s 内没有 map->{self.odom_frame} / {self.odom_frame}->{self.base_frame}'
                     '（导航和定位起来了吗？）')

    def set_prior(self):
        """把先验和等待点从 map 转到 odom"""
        R, t, yaw = self.wait_tf(self.odom_frame, 'map')
        c = R @ (self.prior_map.x, self.prior_map.y, 0.0) + t
        w = R @ (self.wait_map[0], self.wait_map[1], 0.0) + t
        self.prior = ef.Cabin(float(c[0]), float(c[1]), self.prior_map.yaw + yaw)
        self.wait = (float(w[0]), float(w[1]))
        self.filter = ef.CabinFilter()

    # ------------------------------------------------------------ 回调
    def on_cloud(self, msg):
        if self.prior is None:
            return
        tf = self.lookup(self.odom_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp))
        pose = self.base_pose()
        if tf is None or pose is None:
            return
        R, t = self.matrix(tf)
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True) @ R.T + t
        last = self.filter.mean()
        if self.follow and last is None:        # 示教还没找到轿厢：车头航向可能偏得多，绕车搜
            c = ef.search_cabin(pts, pose[2] - self.base_height, *self.teach_pose, self.wait_distance,
                                self.params, self.rng)
            self.publish([], self.filter.update(c), msg.header.stamp)
            return
        prior = last if self.follow else self.prior
        xy = ef.roi_points(pts, prior, pose[2] - self.base_height, self.params)
        lines = ef.fit_lines(xy, self.params, self.rng)
        cabin = self.filter.update(ef.cabin_from_lines(lines, prior, last, self.params))
        self.publish(lines, cabin, msg.header.stamp)

    def on_obstacles(self, msg):
        tf = self.lookup(self.base_frame, msg.header.frame_id)
        if tf is None:
            return
        R, t = self.matrix(tf)
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        self.obstacles = (pts @ R.T + t)[:, :2]
        self.obstacles_time = time.time()

    def publish(self, lines, cabin, stamp):
        m = Marker()
        m.header.frame_id, m.header.stamp = self.odom_frame, stamp
        m.ns, m.type, m.scale.x = 'elevator_walls', Marker.LINE_LIST, 0.03
        m.color.g, m.color.a = 1.0, 1.0
        for ln in lines:
            for s in (-ln.length / 2, ln.length / 2):
                q = ln.point + s * ln.direction
                m.points.append(Point(x=float(q[0]), y=float(q[1]), z=0.5))
        self.walls_pub.publish(m)
        if cabin is not None:
            p = PoseStamped()
            p.header = m.header
            p.pose.position.x, p.pose.position.y = cabin.x, cabin.y
            p.pose.orientation.z, p.pose.orientation.w = math.sin(cabin.yaw / 2), math.cos(cabin.yaw / 2)
            self.pose_pub.publish(p)

    # ------------------------------------------------------------ 动作
    def stop(self):
        for _ in range(3):
            self.cmd_pub.publish(Twist())
            time.sleep(0.02)

    def spin_for(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)

    def pose_or_fail(self):
        pose = self.base_pose()
        if pose is None:
            raise Failed(f'没有 {self.odom_frame}->{self.base_frame}')
        return pose

    def navigate_to_wait_point(self):
        if not self.nav.wait_for_server(timeout_sec=self.detect_timeout):
            raise Failed('navigate_to_pose 动作服务器不在（导航起来了吗？）')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = self.wait_map
        yaw = self.prior_map.yaw
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        future = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            raise Failed('导航拒绝了去等待点的目标')
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result)
        if result.result().status != GoalStatus.STATUS_SUCCEEDED:
            raise Failed(f'导航到等待点失败（status {result.result().status}）')

    def wait_frames(self):
        t0 = time.time()
        while self.filter.count < self.min_frames:
            if time.time() - t0 > self.detect_timeout:
                raise Failed(f'{self.detect_timeout:.0f} s 内没有 {self.min_frames} 帧有效的轿厢提取')
            rclpy.spin_once(self, timeout_sec=0.05)

    def drive(self, direction, v_max, remaining_of):
        """沿当前提取到的中心线开，remaining_of(depth, cabin) <= 0 时停；20 Hz"""
        best, best_time = None, time.time()
        try:
            while True:
                t0 = time.time()
                while time.time() - t0 < 0.05:
                    rclpy.spin_once(self, timeout_sec=0.01)
                cabin = self.filter.mean()
                x, y, _, yaw = self.pose_or_fail()
                depth, _, _ = ef.to_cabin_frame(x, y, yaw, cabin)
                remaining = remaining_of(depth, cabin)
                v = ef.approach_speed(remaining, v_max, self.slow_dist)
                if v == 0.0:
                    return
                if time.time() - self.obstacles_time > 1.0:
                    raise Failed('1 s 没收到 terrain_obstacles，没法做安全检查')
                hits = ef.footprint_hits(self.obstacles, pad=self.footprint_pad)
                if hits:
                    raise Failed(f'车体外扩 {self.footprint_pad} m 内有 {hits} 个障碍点')
                if best is None or remaining < best - 0.02:
                    best, best_time = remaining, time.time()
                elif time.time() - best_time > self.stall_timeout:
                    raise Failed(f'{self.stall_timeout:.0f} s 没有前进（还差 {remaining:.2f} m）')
                k = ef.pursuit_curvature(x, y, yaw, cabin, direction, self.lookahead,
                                         max_curvature=self.max_curvature)
                cmd = Twist()
                cmd.linear.x, cmd.angular.z = direction * v, k * v
                self.cmd_pub.publish(cmd)
        finally:
            self.stop()

    def teach(self):
        self.wait_tf(self.odom_frame, 'map')
        x, y, _, yaw = self.pose_or_fail()
        self.teach_pose = (x, y, yaw)
        self.prior = ef.cabin_ahead(x, y, yaw, self.wait_distance)
        self.wait_frames()
        self.spin_for(2.0)              # 再收 2 s，滤波窗口里都是框选收敛之后的帧
        c = self.filter.mean()
        R, t, myaw = self.wait_tf('map', self.odom_frame)
        m = R @ (c.x, c.y, 0.0) + t
        cabin = ef.Cabin(float(m[0]), float(m[1]), math.remainder(c.yaw + myaw, 2 * math.pi))
        wait = ef.wait_point(cabin, self.wait_distance)
        with open(self.elevator_file, 'w') as f:
            f.write(f'# elevator.py teach 示教生成（{time.strftime("%Y-%m-%d %H:%M")}）：'
                    '电梯在这张地图里的位置，map 坐标系；轿厢尺寸在 elevator_params.yaml\n')
            yaml.safe_dump({'frame': 'map',
                            'cabin_center': [round(cabin.x, 3), round(cabin.y, 3)],
                            'cabin_yaw': round(cabin.yaw, 4),
                            'wait_point': [round(wait[0], 3), round(wait[1], 3)]}, f, sort_keys=False)
        print(f'ELEVATOR_TEACH cabin(map)=({cabin.x:.3f}, {cabin.y:.3f}, {math.degrees(cabin.yaw):.1f}°) '
              f'wait(map)=({wait[0]:.3f}, {wait[1]:.3f}) -> {self.elevator_file}', flush=True)
        return True

    def enter(self):
        self.set_prior()
        x, y, _, _ = self.pose_or_fail()
        if ef.inside_cabin(x, y, self.prior, self.params):
            raise Failed('车已经在轿厢里')
        self.navigate_to_wait_point()
        self.set_prior()                # 导航途中 map->odom 可能变了
        self.wait_frames()
        self.drive(+1, self.speed_in, lambda depth, cabin: -depth)
        self.spin_for(2.0)              # 停稳 1 s，再用接下来 1 s 的提取结果（滤波窗口 5 帧 = 0.5 s）
        cabin = self.filter.mean()
        x, y, _, yaw = self.pose_or_fail()
        depth, lat, yerr = ef.to_cabin_frame(x, y, yaw, cabin)
        ok = abs(lat) < self.tol_lateral and abs(yerr) < self.tol_yaw and abs(depth) < self.tol_depth
        print(f'ELEVATOR_ENTER depth={depth:+.3f} lateral={lat:+.3f} yaw_deg={math.degrees(yerr):+.2f} '
              f'{"OK" if ok else "FAIL"}', flush=True)
        return ok

    def exit(self):
        self.set_prior()
        x, y, _, _ = self.pose_or_fail()
        if not ef.inside_cabin(x, y, self.prior, self.params):
            raise Failed('车不在轿厢里')
        self.wait_frames()

        def remaining(depth, cabin):
            return depth - ef.to_cabin_frame(self.wait[0], self.wait[1], cabin.yaw, cabin)[0]
        self.drive(-1, self.speed_out, remaining)
        cabin = self.filter.mean()
        x, y, _, yaw = self.pose_or_fail()
        depth, lat, _ = ef.to_cabin_frame(x, y, yaw, cabin)
        print(f'ELEVATOR_EXIT depth={remaining(depth, cabin):+.3f} lateral={lat:+.3f}', flush=True)
        return True

    def detect(self):
        self.set_prior()
        while rclpy.ok():
            self.spin_for(1.0)
            c = self.filter.mean()
            hits = 0 if self.obstacles is None else ef.footprint_hits(self.obstacles, pad=self.footprint_pad)
            if c is None:
                print(f'没有有效提取  footprint_hits={hits}', flush=True)
                continue
            x, y, _, yaw = self.pose_or_fail()
            depth, lat, yerr = ef.to_cabin_frame(x, y, yaw, c)
            print(f'cabin(odom)=({c.x:.3f}, {c.y:.3f}, {math.degrees(c.yaw):.1f}°) frames={self.filter.count}  '
                  f'base: depth={depth:+.3f} lateral={lat:+.3f} yaw_deg={math.degrees(yerr):+.2f}  '
                  f'footprint_hits={hits}', flush=True)


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else None
    path = sys.argv[2] if len(sys.argv) > 2 and not sys.argv[2].startswith('-') else None
    if mode not in MODES or path is None:
        print(f'用法：elevator.py {"|".join(MODES)} <电梯文件，如 maps/wh/wh_elevator.yaml> [--ros-args ...]',
              file=sys.stderr)
        return 2
    if mode != 'teach' and not os.path.isfile(path):
        print(f'没有电梯文件 {path}：先把车停在门外、车头对着门，跑 elevator.py teach {path}', file=sys.stderr)
        return 2
    args = list(sys.argv)
    if '--params-file' not in args:
        f = os.path.join(get_package_share_directory('agv_nav'), 'config', 'elevator_params.yaml')
        args += ['--ros-args', '--params-file', f]
    rclpy.init(args=args)
    node = Elevator(os.path.abspath(path), mode == 'teach')
    code = 1
    try:
        code = 0 if getattr(node, mode)() else 1
    except Failed as e:
        node.get_logger().error(str(e))
        print(f'ELEVATOR_{mode.upper()} FAIL: {e}', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())

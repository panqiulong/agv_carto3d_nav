#!/usr/bin/env python3
"""仿真的轮式里程计：把 Ackermann 插件的 /wheel/truth（没有噪声，等于车的真实位姿）变成带误差的 /wheel/odom。

每一步取真值的相对运动，按走过的路程 ds 加误差，再累加成里程计位姿：
    路程：比例误差 scale（轮径不准）+ 随机游走 sigma_s（打滑），单位 m/√m
    航向：曲率偏置 curvature_bias（转向零位不准，rad/m，倒车时反向）+ 随机游走 sigma_yaw（rad/√m）
误差只随路程累积，车停着不漂，和编码器一样。

和实车的差别：z 和俯仰、横滚来自真值的相对运动，实车的轮速测不到这些。EKF 只用这里的车体速度，
不受影响；Cartographer 用的位姿在坡道上比实车的好。
"""
import math
from collections import namedtuple

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation

WheelOdom = namedtuple('WheelOdom', 'position orientation linear angular')


class WheelOdomModel:
    def __init__(self, scale, sigma_s, curvature_bias, sigma_yaw, seed=0):
        self.scale, self.sigma_s = scale, sigma_s
        self.curvature_bias, self.sigma_yaw = curvature_bias, sigma_yaw
        self.rng = np.random.default_rng(seed)
        self.truth = None       # 上一帧真值 (位置, 姿态, 时刻)
        self.position, self.rotation = None, None

    def update(self, position, orientation, stamp):
        """position: 真值位置；orientation: 真值四元数 xyzw；stamp: 秒。返回 WheelOdom，时间没有前进时返回 None"""
        rotation = Rotation.from_quat(orientation)
        if self.truth is None:
            self.truth = position, rotation, stamp
            self.position, self.rotation = position.copy(), rotation
            return None
        last_position, last_rotation, last_stamp = self.truth
        dt = stamp - last_stamp
        if dt <= 0.0:
            return None
        self.truth = position, rotation, stamp

        move = last_rotation.inv().apply(position - last_position)      # 车体系下的相对运动
        turn = last_rotation.inv() * rotation
        ds = np.linalg.norm(move)
        root = math.sqrt(ds)
        if ds > 0.0:
            move = move * (1.0 + self.scale + self.sigma_s * self.rng.standard_normal() / root)
        yaw_error = self.curvature_bias * math.copysign(ds, move[0]) + self.sigma_yaw * root * self.rng.standard_normal()
        turn = Rotation.from_euler('z', yaw_error) * turn

        self.position = self.position + self.rotation.apply(move)
        self.rotation = self.rotation * turn
        return WheelOdom(self.position, self.rotation.as_quat(), move / dt, turn.as_rotvec() / dt)


class WheelOdomSim(Node):
    def __init__(self):
        super().__init__('wheel_odom_sim')
        self.model = WheelOdomModel(
            self.declare_parameter('scale', 0.01).value,
            self.declare_parameter('sigma_s', 0.01).value,
            self.declare_parameter('curvature_bias', 0.002).value,
            self.declare_parameter('sigma_yaw', 0.003).value)
        self.pub = self.create_publisher(Odometry, 'wheel/odom', 50)
        self.create_subscription(Odometry, 'wheel/truth', self.on_truth, qos_profile_sensor_data)

    def on_truth(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        out = self.model.update(np.array([p.x, p.y, p.z]), [q.x, q.y, q.z, q.w], stamp)
        if out is None:
            return
        odom = Odometry()
        odom.header = msg.header
        odom.child_frame_id = msg.child_frame_id
        pose, twist = odom.pose.pose, odom.twist.twist
        pose.position.x, pose.position.y, pose.position.z = out.position.tolist()
        o = pose.orientation
        o.x, o.y, o.z, o.w = out.orientation.tolist()
        twist.linear.x, twist.linear.y, twist.linear.z = out.linear.tolist()
        twist.angular.x, twist.angular.y, twist.angular.z = out.angular.tolist()
        # EKF 按协方差分配权重：航向角速度要明显比陀螺差（轮速的航向有偏置）
        odom.twist.covariance[0] = odom.twist.covariance[7] = odom.twist.covariance[14] = 0.05 ** 2
        odom.twist.covariance[35] = 0.05 ** 2
        odom.pose.covariance[0] = odom.pose.covariance[7] = odom.pose.covariance[14] = 0.1 ** 2
        odom.pose.covariance[21] = odom.pose.covariance[28] = odom.pose.covariance[35] = 0.1 ** 2
        self.pub.publish(odom)


def main():
    rclpy.init()
    rclpy.spin(WheelOdomSim())


if __name__ == '__main__':
    main()

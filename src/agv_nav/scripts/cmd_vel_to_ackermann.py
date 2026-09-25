#!/usr/bin/env python3
"""把标准 Twist（linear.x 速度, angular.z 角速度）换算成 gazebo_ros_ackermann_drive 要的
Twist（linear.x 速度, angular.z 前轮转角 rad）。

插件内部会把转角乘以 sign(linear.x)，所以这里按 |v| 算：
    steer = atan(wheelbase * wz / |v|)
倒车时插件自动翻转符号，得到的实际偏航角速度仍然是 wz。

插件的速度环只用右后轮转速做反馈，转弯时右后轮不在后轴中心的圆上，
所以把期望的后轴中心速度换算成右后轮速度再发：v_right = v * (1 + (track/2) * tan(实际转角) / wheelbase)，
左转（右轮在外侧）放大，右转缩小，否则左转时车身会比指令慢、右转会比指令快。
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelToAckermann(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_ackermann')
        self.wheelbase = self.declare_parameter('wheelbase', 0.57012).value
        self.max_steer = self.declare_parameter('max_steer', 0.414).value
        self.rear_track = self.declare_parameter('rear_track', 0.62).value
        self.min_speed = self.declare_parameter('min_speed', 1e-3).value
        self.pub = self.create_publisher(Twist, 'cmd_vel_ackermann', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)

    def on_cmd(self, msg):
        v = msg.linear.x
        steer = 0.0
        if abs(v) > self.min_speed:
            steer = math.atan(self.wheelbase * msg.angular.z / abs(v))
            steer = max(-self.max_steer, min(self.max_steer, steer))
        actual_steer = steer if v >= 0.0 else -steer
        out = Twist()
        out.linear.x = v * (1.0 + self.rear_track / 2.0 * math.tan(actual_steer) / self.wheelbase)
        out.angular.z = steer
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(CmdVelToAckermann())


if __name__ == '__main__':
    main()

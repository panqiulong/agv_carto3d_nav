#!/usr/bin/env python3
"""把标准 Twist（linear.x 速度, angular.z 角速度）换算成 gazebo_ros_ackermann_drive 要的
Twist（linear.x 速度, angular.z 前轮转角 rad）。

插件内部会把转角乘以 sign(linear.x)，所以这里按 |v| 算：
    steer = atan(wheelbase * wz / |v|)
倒车时插件自动翻转符号，得到的实际偏航角速度仍然是 wz。

插件的速度环只用右后轮转速做反馈，转弯时右后轮不在后轴中心的圆上，
所以把期望的后轴中心速度换算成右后轮速度再发：v_right = v * (1 + (track/2) * tan(实际转角) / wheelbase)，
左转（右轮在外侧）放大，右转缩小，否则左转时车身会比指令慢、右转会比指令快。

前轮实际转角的变化速率限制在 max_steer_rate 以内：velocity_smoother 只平滑 v 和 wz，
低速时 wz 的小变化就会让 atan(L * wz / |v|) 大幅跳变。按实际转角（乘过 sign(v)）限幅，
前进 / 倒车切换时车轮不会瞬间翻转。
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelToAckermann(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_ackermann')
        self.wheelbase = self.declare_parameter('wheelbase', 0.85518).value
        self.max_steer = self.declare_parameter('max_steer', 0.414).value
        self.rear_track = self.declare_parameter('rear_track', 0.93).value
        self.min_speed = self.declare_parameter('min_speed', 1e-3).value
        self.max_steer_rate = self.declare_parameter('max_steer_rate', 0.8).value     # rad/s
        self.last_steer = 0.0           # 上一次发出的实际转角
        self.last_time = None
        self.pub = self.create_publisher(Twist, 'cmd_vel_ackermann', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)

    def on_cmd(self, msg, dt=None):
        if dt is None:
            now = self.get_clock().now()
            dt = 0.0 if self.last_time is None else (now - self.last_time).nanoseconds * 1e-9
            self.last_time = now
        dt = min(max(dt, 0.0), 0.2)     # 指令中断很久后也不允许一步打满；仿真时间回退时不动
        v = msg.linear.x
        steer = 0.0
        if abs(v) > self.min_speed:
            steer = math.atan(self.wheelbase * msg.angular.z / abs(v))
            steer = max(-self.max_steer, min(self.max_steer, steer))
        target = steer if v >= 0.0 else -steer
        max_delta = self.max_steer_rate * dt
        actual_steer = self.last_steer + max(-max_delta, min(max_delta, target - self.last_steer))
        self.last_steer = actual_steer
        steer = actual_steer if v >= 0.0 else -actual_steer
        out = Twist()
        out.linear.x = v * (1.0 + self.rear_track / 2.0 * math.tan(actual_steer) / self.wheelbase)
        out.angular.z = steer
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(CmdVelToAckermann())


if __name__ == '__main__':
    main()

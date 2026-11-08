#!/usr/bin/env python3
"""高精地图命令行：
  hdmap_cli.py goto <x> <y> [yaw]     发一个目标（map 坐标）到 /hdmap/goal，等到 arrived（返回 0）或 failed（返回 1）
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


def goto(node, x, y, yaw, timeout):
    latest = []
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/hdmap/status', lambda m: latest.append(m.data), latched)
    pub = node.create_publisher(PoseStamped, '/hdmap/goal', 10)
    t0 = time.time()
    while pub.get_subscription_count() == 0 or not latest:    # 等导航节点：订阅上目标，且收到它的当前状态
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - t0 > 10.0:
            print('hdmap_navigator 没有运行（导航 launch 要带 hdmap:=<.osm>）', file=sys.stderr)
            return 1
    del latest[:]
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.position.x, msg.pose.position.y = x, y
    msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    pub.publish(msg)
    t0 = time.time()
    while time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        while latest:
            s = latest.pop(0)
            print(s, flush=True)
            if s == 'arrived':
                return 0
            if s.startswith('failed'):
                return 1
    print('超时', file=sys.stderr)
    return 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    g = sub.add_parser('goto', help='按车道开到 (x, y)，map 坐标')
    g.add_argument('x', type=float)
    g.add_argument('y', type=float)
    g.add_argument('yaw', type=float, nargs='?', default=0.0, help='不影响到达朝向（取车道方向），只是消息里带上')
    g.add_argument('--timeout', type=float, default=900.0)
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = Node('hdmap_cli')
    try:
        return goto(node, args.x, args.y, args.yaw, args.timeout)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())

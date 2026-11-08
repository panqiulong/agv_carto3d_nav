#!/usr/bin/env python3
"""语义地图命令行：
  semantic.py add <名字> [--overwrite]   把车当前位置保存为地点
  semantic.py remove <名字>
  semantic.py list
  semantic.py goto <名字>                按名字导航（Ctrl+C 取消）
  semantic.py where                      车当前所在区域
"""
import argparse
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from agv_semantic.action import GoToPlace
from agv_semantic.srv import PlaceEdit

SERVER = '/semantic_server'


def call(node, srv_type, name, request):
    client = node.create_client(srv_type, SERVER + '/' + name)
    if not client.wait_for_service(timeout_sec=5.0):
        print('semantic_server 没有运行（导航 launch 要带 semantic:=<语义文件>）', file=sys.stderr)
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    return future.result()


def edit(node, name, place, overwrite=False):
    res = call(node, PlaceEdit, name, PlaceEdit.Request(name=place, overwrite=overwrite))
    if res is None:
        return 1
    print(res.message, file=sys.stdout if res.success else sys.stderr)
    return 0 if res.success else 1


def goto(node, place):
    client = ActionClient(node, GoToPlace, SERVER + '/goto')
    if not client.wait_for_server(timeout_sec=5.0):
        print('semantic_server 没有运行（导航 launch 要带 semantic:=<语义文件>）', file=sys.stderr)
        return 1
    last = [0.0]

    def on_feedback(fb):
        if time.time() - last[0] > 2.0:
            last[0] = time.time()
            print('剩余 %.1f m' % fb.feedback.distance_remaining, flush=True)

    send = client.send_goal_async(GoToPlace.Goal(name=place), feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, send)
    handle = send.result()
    if not handle.accepted:
        print('semantic_server 拒绝了请求', file=sys.stderr)
        return 1
    done = handle.get_result_async()
    try:
        rclpy.spin_until_future_complete(node, done)
    except KeyboardInterrupt:
        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel, timeout_sec=5.0)
        print('已取消', file=sys.stderr)
        return 1
    res = done.result().result
    print(res.message, file=sys.stdout if res.success else sys.stderr)
    return 0 if res.success else 1


def where(node):
    got = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/current_zone', lambda m: got.append(m.data), qos)
    end = time.time() + 3.0
    while not got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not got:
        print('没有收到 /current_zone（semantic_server 没有运行，或还没有定位）', file=sys.stderr)
        return 1
    print(got[-1] or '（不在任何区域）')
    return 0


def main():
    parser = argparse.ArgumentParser(prog='semantic.py', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('add'); p.add_argument('name'); p.add_argument('--overwrite', action='store_true')
    p = sub.add_parser('remove'); p.add_argument('name')
    sub.add_parser('list')
    p = sub.add_parser('goto'); p.add_argument('name')
    sub.add_parser('where')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = Node('semantic_cli')
    try:
        if args.cmd == 'add':
            code = edit(node, 'add_place', args.name, args.overwrite)
        elif args.cmd == 'remove':
            code = edit(node, 'remove_place', args.name)
        elif args.cmd == 'list':
            res = call(node, Trigger, 'list', Trigger.Request())
            code = 1 if res is None else (print(res.message) or 0)
        elif args.cmd == 'goto':
            code = goto(node, args.name)
        else:
            code = where(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()

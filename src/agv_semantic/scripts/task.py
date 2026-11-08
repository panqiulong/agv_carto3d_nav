#!/usr/bin/env python3
"""出入库任务命令行：
  task.py submit inbound  [--from R1] [--to A3]   入库单（收货月台 → 库位），不写就自动选
  task.py submit outbound [--from A3] [--to S1]   出库单（库位 → 发货月台）
  task.py list                                     当前任务、队列、最近结束的任务、库存
  task.py cancel <编号>
  task.py resume                                   失败或取消后，继续执行队列
  task.py stock <库位> full|empty                  手工设置库存
"""
import argparse
import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from agv_semantic.srv import SlotSet, TaskId, TaskSubmit

SERVER = '/task_manager'


def call(node, srv_type, name, request):
    client = node.create_client(srv_type, SERVER + '/' + name)
    if not client.wait_for_service(timeout_sec=5.0):
        print('task_manager 没有运行（导航 launch 要带 semantic:=<语义文件> tasks:=<任务配置>）', file=sys.stderr)
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    return future.result()


def report(res):
    if res is None:
        return 1
    print(res.message, file=sys.stdout if res.success else sys.stderr)
    return 0 if res.success else 1


def main():
    parser = argparse.ArgumentParser(prog='task.py', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('submit')
    p.add_argument('type', choices=['inbound', 'outbound'])
    p.add_argument('--from', dest='source', default='')
    p.add_argument('--to', dest='target', default='')
    sub.add_parser('list')
    p = sub.add_parser('cancel'); p.add_argument('id', type=int)
    sub.add_parser('resume')
    p = sub.add_parser('stock'); p.add_argument('slot'); p.add_argument('state', choices=['full', 'empty'])
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = Node('task_cli')
    try:
        if args.cmd == 'submit':
            code = report(call(node, TaskSubmit, 'submit',
                               TaskSubmit.Request(type=args.type, source=args.source, target=args.target)))
        elif args.cmd == 'list':
            code = report(call(node, Trigger, 'list', Trigger.Request()))
        elif args.cmd == 'cancel':
            code = report(call(node, TaskId, 'cancel', TaskId.Request(id=args.id)))
        elif args.cmd == 'resume':
            code = report(call(node, Trigger, 'resume', Trigger.Request()))
        else:
            code = report(call(node, SlotSet, 'set_stock', SlotSet.Request(slot=args.slot, full=args.state == 'full')))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()

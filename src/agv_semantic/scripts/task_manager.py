#!/usr/bin/env python3
"""出入库任务节点：提交 / 取消 / 继续 / 库存设置 / 列表；逐个执行任务（调用 semantic_server 的 goto）。

规则见 docs/superpowers/specs/2026-09-30-warehouse-tasks-design.md 第 6、7 节。
"""
import json
import math
import sys
import threading
import time

import rclpy
import tf2_ros
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger

from agv_semantic.action import GoToPlace
from agv_semantic.srv import SlotSet, TaskId, TaskSubmit
import semantic_map as S
import tasks as T

CANCELED = '已取消'


class Canceled(Exception):
    pass


class TaskFailed(Exception):
    pass


class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        self.path = self.declare_parameter('tasks_file', '').value
        semantic_file = self.declare_parameter('semantic_file', '').value
        goto_action = self.declare_parameter('goto_action', '/semantic_server/goto').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.home_radius = self.declare_parameter('home_radius', 1.0).value
        if not self.path or not semantic_file:
            raise T.TaskError('必须给出参数 tasks_file 和 semantic_file')
        self.sm = S.load(semantic_file)
        self.board = T.TaskBoard(T.load(self.path, {p.name for p in self.sm.places}))
        self.lock = threading.RLock()
        self.cancel_active = False
        self.home_done = False
        self.last_status = None

        cb = ReentrantCallbackGroup()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/task_status', latched)
        self.create_service(TaskSubmit, '~/submit', self.on_submit, callback_group=cb)
        self.create_service(TaskId, '~/cancel', self.on_cancel, callback_group=cb)
        self.create_service(Trigger, '~/resume', self.on_resume, callback_group=cb)
        self.create_service(SlotSet, '~/set_stock', self.on_set_stock, callback_group=cb)
        self.create_service(Trigger, '~/list', self.on_list, callback_group=cb)
        self.goto_client = ActionClient(self, GoToPlace, goto_action, callback_group=cb)
        self.publish_status()
        self.get_logger().info('loaded %s: %d slots' % (self.path, len(self.board.cfg.slots)))

    # ------------------------------------------------------------ 状态与保存
    def publish_status(self):
        # 比较和发布放在同一段锁内：否则工作线程可能在服务线程之后发出旧状态，latched 话题会一直停在旧状态
        with self.lock:
            text = json.dumps(self.board.status(), ensure_ascii=False, sort_keys=True)
            if text != self.last_status and rclpy.ok():        # 节点关闭后不再发布
                self.last_status = text
                self.status_pub.publish(String(data=text))

    def save(self):
        """保存失败只警告：内存里的库存照常更新，任务不中断"""
        try:
            with self.lock:
                T.save(self.board.cfg, self.path)
        except (OSError, T.TaskError) as e:
            self.get_logger().warn('库存保存失败：%s' % e)

    # ------------------------------------------------------------ 服务
    def on_submit(self, req, res):
        with self.lock:
            try:
                task = self.board.submit(req.type.strip(), req.source, req.target)
            except T.TaskError as e:
                res.success, res.message, res.id = False, str(e), 0
                return res
        res.success, res.id = True, task.id
        res.message = '已提交 #%d：%s %s → %s' % (task.id, T.TYPE_NAMES[task.type], task.source, task.target)
        self.publish_status()
        return res

    def on_cancel(self, req, res):
        with self.lock:
            if self.board.active is not None and self.board.active.id == req.id:
                self.cancel_active = True
                res.success, res.message = True, '正在取消执行中的任务 #%d，之后队列暂停' % req.id
                return res
            try:
                self.board.cancel_queued(req.id)
            except T.TaskError as e:
                res.success, res.message = False, str(e)
                return res
        res.success, res.message = True, '已取消 #%d' % req.id
        self.publish_status()
        return res

    def on_resume(self, req, res):
        with self.lock:
            was = self.board.resume()
        res.success, res.message = True, '队列继续执行' if was else '队列本来就没有暂停'
        self.publish_status()
        return res

    def on_set_stock(self, req, res):
        with self.lock:
            try:
                self.board.set_stock(req.slot.strip(), req.full)
            except T.TaskError as e:
                res.success, res.message = False, str(e)
                return res
        self.save()
        res.success, res.message = True, '库位 %s 设为%s' % (req.slot.strip(), '有货' if req.full else '空')
        self.publish_status()
        return res

    def on_list(self, req, res):
        with self.lock:
            res.success, res.message = True, self.board.describe()
        return res

    # ------------------------------------------------------------ 执行
    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time()).transform
        except tf2_ros.TransformException:
            return None
        q = t.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.translation.x, t.translation.y, yaw

    def goto(self, name, should_cancel):
        """调用 semantic_server 的 goto；should_cancel() 为真时取消并返回 (False, CANCELED)"""
        if not self.goto_client.wait_for_server(timeout_sec=5.0):
            return False, 'semantic_server 的 goto 不可用'
        send = self.goto_client.send_goal_async(GoToPlace.Goal(name=name))
        while not send.done():                    # 节点关闭时退出，见 semantic_server.on_goto
            if not rclpy.ok():
                return False, CANCELED
            time.sleep(0.05)
        handle = send.result()
        if not handle.accepted:
            return False, 'goto 拒绝了目标'
        done = handle.get_result_async()
        while not done.done():
            if should_cancel() or not rclpy.ok():
                handle.cancel_goal_async()
                end = time.time() + 10.0          # 等它真正结束，免得和下一次 goto 重叠
                while not done.done() and time.time() < end:
                    time.sleep(0.05)
                return False, CANCELED
            time.sleep(0.1)
        r = done.result().result
        return r.success, r.message

    def set_state(self, state):
        with self.lock:
            self.board.set_state(state)
        self.publish_status()

    def navigate(self, name):
        message = ''
        for attempt in (1, 2):                      # 失败自动重试 1 次
            ok, message = self.goto(name, lambda: self.cancel_active)
            if ok:
                return
            if message == CANCELED:
                raise Canceled()
            self.get_logger().warn('去 %s 失败（第 %d 次）：%s' % (name, attempt, message))
        raise TaskFailed('去 %s 失败：%s' % (name, message))

    def check_cancel(self):
        """停留刚结束、库存还没更新时到达的取消也要生效（取消服务已经回复了“之后队列暂停”）"""
        if self.cancel_active:
            raise Canceled()

    def dwell(self):
        end = self.get_clock().now().nanoseconds + int(self.board.cfg.dwell * 1e9)
        while self.get_clock().now().nanoseconds < end:
            if self.cancel_active or not rclpy.ok():
                raise Canceled()
            time.sleep(0.05)

    def run(self, task):
        try:
            self.navigate(task.source)
            self.set_state('loading')
            self.dwell()
            with self.lock:
                self.check_cancel()
                self.board.picked()
            self.save()
            self.publish_status()
            self.navigate(task.target)
            self.set_state('unloading')
            self.dwell()
            with self.lock:
                self.check_cancel()
                self.board.dropped()
            self.save()
            self.get_logger().info('任务 #%d 完成' % task.id)
        except Canceled:
            with self.lock:
                self.board.end_active('执行中取消', canceled=True)
            self.get_logger().warn('任务 #%d 已取消，队列暂停' % task.id)
        except TaskFailed as e:
            with self.lock:
                self.board.end_active(str(e))
            self.get_logger().error('任务 #%d 失败：%s，队列暂停' % (task.id, e))
        except Exception as e:                      # 意外异常也要结束任务，否则它会一直占着“执行中”，队列永远卡住
            with self.lock:
                self.board.end_active('内部错误：%r' % e)
            self.get_logger().error('任务 #%d 内部错误：%r，队列暂停' % (task.id, e))
        self.publish_status()

    def go_home(self):
        pose = self.robot_pose()
        if pose is None:
            return False                            # 定位还没起来，稍后再试
        if self.board.queue:
            return True                             # 判定空闲之后来了任务：不发回家的目标，下一步直接执行
        home = self.sm.place(self.board.cfg.home)
        if math.hypot(pose[0] - home.x, pose[1] - home.y) > self.home_radius:
            ok, message = self.goto(home.name, lambda: bool(self.board.queue))
            if message == CANCELED:
                return True                         # 有新任务：下一步直接执行，home_done 保持 False
            if not ok:
                self.get_logger().warn('回 %s 失败：%s；不再重试，等下一个任务' % (home.name, message))
        self.home_done = True
        return True

    def step(self):
        """执行一步：有任务就完整执行一个；否则队列空且没暂停时回 HOME。返回是否做了事"""
        with self.lock:
            task = self.board.start_next()
            if task is not None:
                self.cancel_active = False
                self.home_done = False
            idle = task is None and not self.board.paused and not self.board.queue
        if task is not None:
            self.publish_status()
            self.run(task)
            return True
        if idle and not self.home_done:
            return self.go_home()
        return False


def main():
    rclpy.init()
    try:
        node = TaskManager()
    except (T.TaskError, S.SemanticMapError) as e:
        rclpy.logging.get_logger('task_manager').fatal('任务配置有误：%s' % e)
        rclpy.shutdown()
        sys.exit(1)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    def work():
        while rclpy.ok():
            try:
                if not node.step():
                    time.sleep(0.5)
            except Exception as e:                  # 工作线程不能静默退出
                node.get_logger().error('任务执行出错：%r' % e)
                time.sleep(1.0)

    threading.Thread(target=work, daemon=True).start()
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()

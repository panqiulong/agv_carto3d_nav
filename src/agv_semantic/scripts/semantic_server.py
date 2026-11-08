#!/usr/bin/env python3
"""语义地图服务：地点的增删查、按名字导航、车当前所在区域、RViz 标记。

数据文件格式见 docs/superpowers/specs/2026-09-27-semantic-places-zones-design.md。
"""
import math
import os
import sys
import time

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from agv_semantic.action import GoToPlace
from agv_semantic.srv import PlaceEdit
import semantic_map as S

COLORS = {'keepout': (0.9, 0.1, 0.1), 'speed': (0.95, 0.8, 0.1), 'plain': (0.6, 0.6, 0.6), 'place': (0.1, 0.5, 0.95)}


class SemanticServer(Node):
    def __init__(self):
        super().__init__('semantic_server')
        self.path = self.declare_parameter('semantic_file', '').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        zone_rate = self.declare_parameter('zone_rate', 2.0).value
        if not self.path:
            raise S.SemanticMapError('必须给出参数 semantic_file')
        self.sm = S.load(self.path)
        self.check_masks()

        cb = ReentrantCallbackGroup()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.zone_pub = self.create_publisher(String, '/current_zone', latched)
        self.marker_pub = self.create_publisher(MarkerArray, '/semantic_markers', latched)
        self.speed_pub = self.create_publisher(SpeedLimit, '/speed_limit', 10)
        self.last_limit = None
        self.create_service(PlaceEdit, '~/add_place', self.on_add, callback_group=cb)
        self.create_service(PlaceEdit, '~/remove_place', self.on_remove, callback_group=cb)
        self.create_service(Trigger, '~/list', self.on_list, callback_group=cb)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=cb)
        self.goto_server = ActionServer(self, GoToPlace, '~/goto', self.on_goto, callback_group=cb,
                                        cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.last_zone = None
        self.create_timer(1.0 / zone_rate, self.on_zone_timer, callback_group=cb)
        self.publish_markers()
        self.get_logger().info('loaded %s: %d places, %d zones' % (self.path, len(self.sm.places), len(self.sm.zones)))

    def check_masks(self):
        sha1 = S.mask_paths(self.path)['sha1']
        if os.path.exists(sha1):
            if open(sha1).read().strip() != S.zones_hash(self.sm):
                self.get_logger().warn('区域已修改，请重新生成掩码（make_filter_masks.py）并重启导航')
        elif any(z.keepout or z.speed_limit is not None for z in self.sm.zones):
            self.get_logger().warn('有禁行 / 限速区域但还没生成掩码（make_filter_masks.py），禁行区不会生效（限速仍由 semantic_server 按车的位置发布）')

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time()).transform
        except tf2_ros.TransformException:
            return None
        q = t.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.translation.x, t.translation.y, yaw

    def commit(self, places):
        """先写文件再改内存：保存失败时内存与文件保持一致，节点也不退出"""
        new = S.SemanticMap(places=places, zones=self.sm.zones)
        try:
            S.save(new, self.path)
        except (OSError, S.SemanticMapError) as e:
            return '保存失败：%s' % e
        self.sm = new
        self.publish_markers()
        return None

    def names(self):
        return '、'.join(p.name for p in self.sm.places) or '（没有地点）'

    def on_add(self, req, res):
        name = req.name.strip()
        pose = self.robot_pose()
        if not name:
            res.success, res.message = False, '名字不能为空'
        elif pose is None:
            res.success, res.message = False, '拿不到车的位置（TF %s->%s），定位是否已启动？' % (self.map_frame, self.base_frame)
        elif self.sm.place(name) is not None and not req.overwrite:
            res.success, res.message = False, '地点 %s 已存在，要覆盖请加 --overwrite' % name
        else:
            place = S.Place(name, *pose)
            error = self.commit([p for p in self.sm.places if p.name != name] + [place])
            if error:
                res.success, res.message = False, error
                return res
            res.success = True
            res.message = '已保存 %s：(%.2f, %.2f, yaw %.2f)' % (name, *pose)
            z = S.keepout_zone_at(self.sm, pose[0], pose[1])
            if z is not None:
                res.message += '；注意：它在禁行区 %s 里，按名字导航会被拒绝' % z.name
        return res

    def on_remove(self, req, res):
        name = req.name.strip()
        if self.sm.place(name) is None:
            res.success, res.message = False, '没有地点 %s。已有：%s' % (name, self.names())
        else:
            error = self.commit([p for p in self.sm.places if p.name != name])
            res.success, res.message = (False, error) if error else (True, '已删除 %s' % name)
        return res

    def on_list(self, req, res):
        lines = ['地点：'] + ['  %s  (%.2f, %.2f, yaw %.2f)' % (p.name, p.x, p.y, p.yaw) for p in self.sm.places]
        lines.append('区域：')
        for z in self.sm.zones:
            rule = '禁行' if z.keepout else ('限速 %.2f m/s' % z.speed_limit if z.speed_limit is not None else '')
            lines.append('  %s  %d 个顶点  %s' % (z.name, len(z.polygon), rule))
        res.success, res.message = True, '\n'.join(lines)
        return res

    def on_zone_timer(self):
        pose = self.robot_pose()
        if pose is None:
            return
        zone = ','.join(z.name for z in S.zones_at(self.sm, pose[0], pose[1]))
        if zone != self.last_zone:
            self.last_zone = zone
            self.zone_pub.publish(String(data=zone))
        # MPPI 每收到新的跟踪目标（包括恢复动作之后）都会把速度上限重置为 vx_max，而限速过滤器只在限速变化时
        # 发一次 /speed_limit：车在限速区里卡住、恢复后就不再限速。所以在限速区内周期性重发，离开时发一次“不限速”
        limit = S.speed_limit_at(self.sm, pose[0], pose[1])
        if limit is not None or self.last_limit is not None:
            self.speed_pub.publish(SpeedLimit(percentage=False, speed_limit=float(limit or 0.0)))
        self.last_limit = limit

    def on_goto(self, goal_handle):
        name = goal_handle.request.name.strip()
        result = GoToPlace.Result()

        def fail(message):
            goal_handle.abort()
            result.success, result.message = False, message
            self.get_logger().warn('goto %s: %s' % (name, message))
            return result

        place = self.sm.place(name)
        if place is None:
            return fail('没有地点 %s。已有：%s' % (name, self.names()))
        zone = S.keepout_zone_at(self.sm, place.x, place.y)
        if zone is not None:
            return fail('地点 %s 在禁行区 %s 里，拒绝导航' % (name, zone.name))
        if not self.nav.wait_for_server(timeout_sec=5.0):
            return fail('Nav2 的 navigate_to_pose 不可用，导航是否已启动？')

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = place.x, place.y
        goal.pose.pose.orientation.z = math.sin(place.yaw / 2)
        goal.pose.pose.orientation.w = math.cos(place.yaw / 2)

        def forward(fb):
            goal_handle.publish_feedback(GoToPlace.Feedback(distance_remaining=fb.feedback.distance_remaining))

        send = self.nav.send_goal_async(goal, feedback_callback=forward)
        # 等待循环都要在节点关闭时退出：否则 launch 关闭时正在执行的 goto 永远等不到结果，
        # 执行器线程一直卡着，进程收到 SIGTERM 也退不出去
        while not send.done():
            if not rclpy.ok():
                return fail('节点正在关闭')
            time.sleep(0.05)
        nav_handle = send.result()
        if not nav_handle.accepted:
            return fail('Nav2 拒绝了目标')
        done = nav_handle.get_result_async()
        while not done.done():
            if not rclpy.ok():
                return fail('节点正在关闭')
            if goal_handle.is_cancel_requested:
                nav_handle.cancel_goal_async()
                deadline = time.time() + 10.0                  # 等 Nav2 目标真正结束，否则紧接着的新目标会被它打断
                while not done.done() and time.time() < deadline:
                    time.sleep(0.05)
                goal_handle.canceled()
                result.success, result.message = False, '已取消'
                return result
            time.sleep(0.1)
        if done.result().status != GoalStatus.STATUS_SUCCEEDED:
            return fail('Nav2 导航失败（状态 %d）' % done.result().status)
        goal_handle.succeed()
        result.success, result.message = True, '已到达 %s' % name
        return result

    def publish_markers(self):
        arr = MarkerArray()
        arr.markers.append(Marker(action=Marker.DELETEALL))
        stamp = self.get_clock().now().to_msg()

        def marker(mid, mtype, color, ns):
            m = Marker()
            m.header.frame_id, m.header.stamp = self.map_frame, stamp
            m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
            m.color.r, m.color.g, m.color.b = color
            m.color.a = 1.0
            m.pose.orientation.w = 1.0
            return m

        for i, p in enumerate(self.sm.places):
            a = marker(i, Marker.ARROW, COLORS['place'], 'places')
            a.pose.position.x, a.pose.position.y = p.x, p.y
            a.pose.orientation.z, a.pose.orientation.w = math.sin(p.yaw / 2), math.cos(p.yaw / 2)
            a.scale.x, a.scale.y, a.scale.z = 0.8, 0.15, 0.15
            t = marker(i, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0), 'place_names')
            t.pose.position.x, t.pose.position.y, t.pose.position.z = p.x, p.y, 0.8
            t.scale.z, t.text = 0.4, p.name
            arr.markers += [a, t]
        for i, z in enumerate(self.sm.zones):
            kind = 'keepout' if z.keepout else ('speed' if z.speed_limit is not None else 'plain')
            line = marker(i, Marker.LINE_STRIP, COLORS[kind], 'zones')
            line.scale.x = 0.08
            line.points = [Point(x=x, y=y, z=0.05) for x, y in z.polygon + [z.polygon[0]]]
            cx = sum(v[0] for v in z.polygon) / len(z.polygon)
            cy = sum(v[1] for v in z.polygon) / len(z.polygon)
            t = marker(i, Marker.TEXT_VIEW_FACING, COLORS[kind], 'zone_names')
            t.pose.position.x, t.pose.position.y, t.pose.position.z = cx, cy, 0.5
            t.scale.z = 0.4
            t.text = z.name + ('（禁行）' if z.keepout else ('（限速 %.2f）' % z.speed_limit if z.speed_limit else ''))
            arr.markers += [line, t]
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    try:
        node = SemanticServer()
    except S.SemanticMapError as e:
        rclpy.logging.get_logger('semantic_server').fatal('语义文件有误：%s' % e)
        rclpy.shutdown()
        sys.exit(1)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()

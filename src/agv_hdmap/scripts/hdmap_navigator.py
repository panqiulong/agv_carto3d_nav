#!/usr/bin/env python3
"""高精地图导航：收 /hdmap/goal，按车道路由、生成路径，交给 Nav2 controller_server 的 follow_path 去跟；
沿途按车道发限速，前方红灯 / 黄灯时把路径截到停止线前。车或目标在仓库里时分段：仓库里那段交给 Nav2 的
navigate_to_pose，在交接车道 gate_out / gate_in 的端点换手。

    话题：/hdmap/goal（PoseStamped，map 系）、/hdmap/traffic_light_states
        → /hdmap/path、/hdmap/route_markers、/hdmap/current_lanelet、/hdmap/status、/speed_limit
    /hdmap/status：idle / driving / waiting_light / arrived / failed: <原因>
"""
import math
import os
import sys

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from autoware_perception_msgs.msg import TrafficLightElement, TrafficLightGroupArray
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import FollowPath, NavigateToPose
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hdmap  # noqa: E402


COLOR = {TrafficLightElement.RED: 'red', TrafficLightElement.AMBER: 'yellow', TrafficLightElement.GREEN: 'green'}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class HdMapNavigator(Node):
    def __init__(self):
        super().__init__('hdmap_navigator')
        osm = self.declare_parameter('hdmap_file', '').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.goal_checker = self.declare_parameter('goal_checker_id', 'general_goal_checker').value
        flat = self.declare_parameter('indoor_polygon', [-27.0, -11.0, 23.0, -11.0, 23.0, 19.0, -27.0, 19.0]).value
        self.indoor = list(zip(flat[0::2], flat[1::2]))   # 仓库内部（map 系），默认是墙内表面
        self.m = hdmap.load(osm)
        self.get_logger().info('高精地图 %s：%d 条车道' % (osm, len(self.m.by_name)))

        self.goal_handle = None
        self.gen = 0                 # 每发一次动作目标加一：被顶掉的旧目标，结果不再改状态
        self.legs = []               # 还没开始的分段 [('nav2' | 'lane', (x, y, yaw))]
        self.kind = None             # 当前这段是 'nav2' 还是 'lane'
        self.leg = None              # lane 段的 FollowPath 是开到段终点（'final'）还是开到停止线前（'stop'）
        self.lane = None
        self.limit_sent = 0.0
        self.plan = None             # 当前的 (路由, 完整路径)
        self.robot_i = 0             # 车在完整路径上的下标
        self.stop_k = None           # 现在发出去的路径截到哪个下标（None = 没截）
        self.colors = {}             # 红绿灯 id → 颜色
        self.committed = set()       # 黄灯时离线太近、决定直接通过的灯：变红也不再停
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '/hdmap/path', latched)
        self.route_pub = self.create_publisher(MarkerArray, '/hdmap/route_markers', latched)
        self.status_pub = self.create_publisher(String, '/hdmap/status', latched)
        self.lane_pub = self.create_publisher(String, '/hdmap/current_lanelet', latched)
        self.speed_pub = self.create_publisher(SpeedLimit, '/speed_limit', 10)
        self.create_subscription(PoseStamped, '/hdmap/goal', self.on_goal, 10)
        self.create_subscription(TrafficLightGroupArray, '/hdmap/traffic_light_states', self.on_lights, 10)
        self.follow = ActionClient(self, FollowPath, 'follow_path')
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_cancel = self.create_client(CancelGoal, 'navigate_to_pose/_action/cancel_goal')
        self.set_status('idle')
        self.create_timer(0.2, self.on_timer)

    def set_status(self, s):
        self.status = s
        self.status_pub.publish(String(data=s))
        if s.startswith('failed'):
            self.get_logger().warn(s)
        else:
            self.get_logger().info(s)

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time()).transform
        except tf2_ros.TransformException:
            return None
        return t.translation.x, t.translation.y, yaw_of(t.rotation)

    def on_timer(self):
        pose = self.robot_pose()
        ll = hdmap.locate(self.m, *pose) if pose else None
        name = ll.attributes['name'] if ll is not None else ''
        changed = name != self.lane
        if changed:
            self.lane = name
            self.lane_pub.publish(String(data=name))
        now = self.get_clock().now().nanoseconds * 1e-9
        if ll is not None and self.plan is not None and (changed or now - self.limit_sent > 0.5):
            self.publish_speed_limit(ll)

    def publish_speed_limit(self, ll):
        # 每换一条车道都发：语义区域的 SpeedFilter 只在进出区域时发，车从仓库限速区开上公路后要马上换成公路的值。
        # 沿车道开的时候每 0.5 s 重发：MPPI 每收到新的跟踪目标（换路径、红绿灯截路径）都把速度上限重置成 vx_max
        self.speed_pub.publish(SpeedLimit(percentage=False, speed_limit=hdmap.speed_limit(ll)))
        self.limit_sent = self.get_clock().now().nanoseconds * 1e-9

    def on_lights(self, msg):
        self.colors = {g.traffic_light_group_id: COLOR.get(g.elements[0].color, 'red') if g.elements else 'red'
                       for g in msg.traffic_light_groups}
        self.update_stop()

    def update_robot_index(self):
        pose = self.robot_pose()
        if pose is None:
            return
        pts = self.plan[1]
        lo, hi = max(0, self.robot_i - 20), min(len(pts), self.robot_i + 400)     # 只在附近找：路径可能绕回来
        self.robot_i = min(range(lo, hi), key=lambda i: math.dist(pts[i][:2], pose[:2]))

    def update_stop(self, force=False):
        """按灯的状态决定路径截不截、截到哪；结果变了就重发 FollowPath"""
        if self.plan is None:
            return
        self.update_robot_index()
        lanes, pts = self.plan
        nl = hdmap.next_light(self.m, lanes, pts, self.robot_i)
        if nl is not None and self.colors.get(nl[0]) == 'yellow' and nl[1] - nl[2] < hdmap.YELLOW_PASS:
            self.committed.add(nl[0])
        colors = {i: 'green' if i in self.committed else c for i, c in self.colors.items()}
        k = hdmap.stop_index(self.m, lanes, pts, self.robot_i, colors)
        if k == self.stop_k and not force:
            return
        self.stop_k = k
        i = self.robot_i
        if k is None:
            if self.send(pts[i:], 'final'):
                self.set_status('driving')
        elif k > i + 1:
            if self.send(pts[i:k + 1], 'stop'):
                self.set_status('waiting_light')
        else:                                        # 已经在停车点上：停下等
            self.gen += 1
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None
            self.set_status('waiting_light')

    def on_goal(self, msg):
        pose = self.robot_pose()
        if pose is None:
            return self.fail('no localization')
        goal = (msg.pose.position.x, msg.pose.position.y, yaw_of(msg.pose.orientation))
        legs = hdmap.plan_legs(self.m, pose, goal, self.indoor)
        if isinstance(legs, str):
            return self.fail(legs)
        self.get_logger().info('分段：' + ' → '.join(k for k, _ in legs))
        if self.goal_handle is not None and self.kind != legs[0][0]:   # 同一种动作直接抢占，换动作时先取消
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.legs, self.kind = legs, None
        if legs[0][0] == 'lane' and self.nav_cancel.service_is_ready():
            # 车道段和 Nav2 行为树共用 controller_server：别人（RViz 的 Nav2 Goal）发的 navigate_to_pose 还在跑的话，
            # 行为树会不停重发 FollowPath 把车道路径顶掉。先取消所有 navigate_to_pose 目标（goal_id 全 0、时间 0），
            # 收到回复再发车道路径
            self.gen += 1
            gen = self.gen

            def canceled(_):
                if gen == self.gen:
                    self.next_leg()
            self.nav_cancel.call_async(CancelGoal.Request()).add_done_callback(canceled)
            return
        self.next_leg()

    def next_leg(self):
        if not self.legs:
            self.plan, self.kind = None, None
            return self.set_status('arrived')
        kind, target = self.legs.pop(0)
        prev, self.kind = self.kind, kind
        if kind == 'nav2':
            self.plan = None
            if prev == 'lane':
                # 回到仓库、交给 Nav2：先清掉公路限速（0 = 不限速），语义区域的限速过滤器从干净的状态接手
                self.speed_pub.publish(SpeedLimit(percentage=False, speed_limit=0.0))
            return self.send_nav2(target)
        # Nav2 到点有容差（0.375 m），车可能停在 gate_out 起点线之前，不在任何车道内，但这时要走的车道是确定的
        self.start_lane(target, self.m.by_name['gate_out'] if prev == 'nav2' else None)

    def send_nav2(self, target):
        if not self.nav.wait_for_server(timeout_sec=2.0):
            return self.fail('nav2 bt_navigator not available')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = target[0], target[1]
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(target[2] / 2), math.cos(target[2] / 2)
        self.gen += 1
        gen = self.gen
        self.nav.send_goal_async(goal).add_done_callback(lambda f: self.on_accepted(f, gen))
        self.set_status('driving')

    def start_lane(self, target, handover=None):
        pose = self.robot_pose()
        if pose is None:
            return self.fail('lane no localization')
        gx, gy = target[:2]
        start = hdmap.locate(self.m, *pose)
        if start is None:
            start = handover
        if start is None:
            return self.fail('lane robot off road')
        goal = hdmap.locate_goal(self.m, gx, gy)
        if goal is None:
            return self.fail('lane goal off road')
        lanes = hdmap.route(self.m, start, goal, hdmap.project(start, *pose[:2]), hdmap.project(goal, gx, gy))
        if lanes is None:
            return self.fail('lane no route')
        self.get_logger().info('路由：' + ' → '.join(ll.attributes['name'] for ll in lanes))
        self.publish_route(lanes)
        self.plan = (lanes, hdmap.path(self.m, lanes, pose[:2], (gx, gy)))
        self.robot_i, self.stop_k, self.committed = 0, None, set()
        self.publish_speed_limit(start)
        self.update_stop(force=True)

    def publish_route(self, lanes):
        """路由上的车道：半透明的车道面 + 蓝色边界（和 Autoware 规划界面里路由的画法一样）"""
        stamp = self.get_clock().now().to_msg()
        clear = Marker(action=Marker.DELETEALL)
        surf = Marker(type=Marker.TRIANGLE_LIST, action=Marker.ADD, ns='route_lanes', id=0)
        bounds = Marker(type=Marker.LINE_LIST, action=Marker.ADD, ns='route_bounds', id=1)
        for mk, (r, g, b, a), w in ((surf, (0.6, 0.6, 0.6, 0.4), 1.0), (bounds, (0.1, 0.55, 1.0, 1.0), 0.12)):
            mk.header.frame_id, mk.header.stamp = self.map_frame, stamp
            mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = mk.scale.z = w
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = r, g, b, a
        for ll in lanes:
            for tri in hdmap.lane_triangles(ll):
                surf.points += [Point(x=x, y=y, z=0.02) for x, y in tri]
            for bound in (ll.leftBound, ll.rightBound):
                pts = [Point(x=p.x, y=p.y, z=0.03) for p in bound]
                for p, q in zip(pts, pts[1:]):
                    bounds.points += [p, q]
        self.route_pub.publish(MarkerArray(markers=[clear, surf, bounds]))

    def fail(self, reason):
        self.plan, self.legs, self.kind = None, [], None
        self.gen += 1
        if self.goal_handle is not None:                  # 新目标无效：停下当前的，不让车继续开向旧目标
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.set_status('failed: ' + reason)

    def send(self, pts, leg):
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in pts:
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x, p.pose.position.y = x, y
            p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            path.poses.append(p)
        self.path_pub.publish(path)
        if not self.follow.wait_for_server(timeout_sec=2.0):
            self.fail('controller_server not available')
            return False
        # 正在跟上一条路径时直接发新目标：controller_server 会抢占（preempt）并换成新路径，车不会先停下
        self.gen += 1
        gen, self.leg = self.gen, leg
        goal = FollowPath.Goal(path=path, controller_id='FollowPath', goal_checker_id=self.goal_checker)
        self.follow.send_goal_async(goal).add_done_callback(lambda f: self.on_accepted(f, gen))
        return True

    def on_accepted(self, future, gen):
        handle = future.result()
        if gen != self.gen:
            return
        if not handle.accepted:
            return self.fail('%s rejected' % self.kind)
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self.on_result(f, gen))

    def on_result(self, future, gen):
        if gen != self.gen:
            return
        self.goal_handle = None
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.fail('%s %d' % (self.kind, status))
        elif self.kind == 'nav2' or self.leg == 'final':
            self.next_leg()                          # 一段到了马上开始下一段（没有了就是 arrived）
        # 开到停止线前：继续等灯（状态已经是 waiting_light），变绿时 update_stop 发剩下的路径


def main():
    rclpy.init()
    node = HdMapNavigator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()

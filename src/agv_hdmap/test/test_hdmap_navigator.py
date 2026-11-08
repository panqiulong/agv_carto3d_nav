"""hdmap_navigator 节点的行为：动作客户端换成假的，车的位姿直接给，直接调回调。需要先 source install/setup.bash"""
import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))
rclpy = pytest.importorskip('rclpy')
msgs = pytest.importorskip('autoware_perception_msgs.msg')
from action_msgs.msg import GoalStatus  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
import hdmap_navigator  # noqa: E402

OSM = os.path.join(HERE, '..', '..', 'agv_gazebo', 'worlds', 'warehouse_lanelet2_map.osm')
ROAD = (-19.0, -26.75, 0.0)            # map 系：北段往东外侧车道（十字路口西边）
ROAD_GOAL = (12.0, -26.75)             # 同一方向、过了十字路口
HOME = (0.0, 0.0, 0.0)


class _Done:
    def __init__(self, value):
        self.value = value

    def result(self):
        return self.value

    def add_done_callback(self, cb):
        cb(self)


class _Pending:
    def __init__(self):
        self.cbs, self.value = [], None

    def result(self):
        return self.value

    def add_done_callback(self, cb):
        self.cbs.append(cb)


class _Handle:
    accepted = True

    def __init__(self):
        self.result_future = _Pending()
        self.canceled = False

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.canceled = True

    def finish(self, status):
        self.result_future.value = type('R', (), {'status': status})()
        for cb in self.result_future.cbs:
            cb(self.result_future)


class _Client:
    def __init__(self):
        self.goals, self.handles = [], []

    def wait_for_server(self, timeout_sec=None):
        return True

    def send_goal_async(self, goal):
        h = _Handle()
        self.goals.append(goal)
        self.handles.append(h)
        return _Done(h)


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class _Service:
    """取消服务：默认立刻回复；pending=True 时回复由测试自己触发"""
    def __init__(self, pending=False):
        self.requests, self.futures, self.pending = [], [], pending

    def service_is_ready(self):
        return True

    def call_async(self, req):
        self.requests.append(req)
        f = _Pending() if self.pending else _Done(None)
        self.futures.append(f)
        return f


@pytest.fixture
def nav():
    rclpy.init(args=['--ros-args', '-p', 'hdmap_file:=' + OSM])
    node = hdmap_navigator.HdMapNavigator()
    node.follow, node.nav, node.speed_pub, node.route_pub = _Client(), _Client(), _Pub(), _Pub()
    node.nav_cancel = _Service()
    node.pose = ROAD
    node.robot_pose = lambda: node.pose
    yield node
    # 先停掉 TF 监听线程：测试失败时 traceback 还引用着节点，它的 TransformListener 会在下一个测试里被回收，
    # __del__ 里 join 线程会和新线程的启动互相等住
    node.tf_listener.executor.shutdown()
    node.tf_listener.dedicated_listener_thread.join()
    node.destroy_node()
    rclpy.try_shutdown()


def goal(x, y):
    g = PoseStamped()
    g.pose.position.x, g.pose.position.y = x, y
    g.pose.orientation.w = 1.0
    return g


def path_end(client):
    p = client.goals[-1].path.poses[-1].pose.position
    return p.x, p.y


def lights(node, color):
    """十字路口西进口那个灯设成 color，其他绿"""
    west = node.m.by_name['ring_n_eb_out_1'].trafficLights()[0].id
    E = msgs.TrafficLightElement
    code = {'red': E.RED, 'yellow': E.AMBER, 'green': E.GREEN}
    arr = msgs.TrafficLightGroupArray()
    for i in hdmap_navigator.hdmap.traffic_lights(node.m):
        arr.traffic_light_groups.append(msgs.TrafficLightGroup(
            traffic_light_group_id=i, elements=[E(color=code[color if i == west else 'green'])]))
    node.on_lights(arr)


def test_road_goal_follows_lane_path_and_sends_speed_limit(nav):
    nav.on_goal(goal(*ROAD_GOAL))
    assert nav.status == 'driving' and nav.nav.goals == []
    g = nav.follow.goals[-1]
    assert (g.controller_id, g.goal_checker_id) == ('FollowPath', 'general_goal_checker')
    assert path_end(nav.follow) == pytest.approx(ROAD_GOAL, abs=0.1)
    assert [m.speed_limit for m in nav.speed_pub.msgs] == [pytest.approx(0.5)]
    assert not nav.speed_pub.msgs[0].percentage


def test_goal_off_road_is_rejected(nav):
    nav.on_goal(goal(-16.0, -34.0))
    assert nav.status == 'failed: goal off road'
    assert nav.follow.goals == [] and nav.nav.goals == []


def test_red_cuts_path_at_stop_line_then_green_resends(nav):
    nav.on_goal(goal(*ROAD_GOAL))
    lights(nav, 'red')
    assert nav.status == 'waiting_light'
    x, _ = path_end(nav.follow)
    assert -4.25 - x == pytest.approx(0.85, abs=0.05)               # 西进口停止线 x = -4.25（map）
    nav.follow.handles[-1].finish(GoalStatus.STATUS_SUCCEEDED)       # 开到了停止线前
    assert nav.status == 'waiting_light'                             # 不是 arrived，继续等
    lights(nav, 'green')
    assert nav.status == 'driving'
    assert path_end(nav.follow) == pytest.approx(ROAD_GOAL, abs=0.1)
    nav.follow.handles[-1].finish(GoalStatus.STATUS_SUCCEEDED)
    assert nav.status == 'arrived'


def test_same_light_state_does_not_resend(nav):
    nav.on_goal(goal(*ROAD_GOAL))
    lights(nav, 'red')
    n = len(nav.follow.goals)
    lights(nav, 'red')
    assert len(nav.follow.goals) == n


def test_superseded_goal_result_is_ignored(nav):
    nav.on_goal(goal(*ROAD_GOAL))
    first = nav.follow.handles[-1]
    nav.on_goal(goal(14.0, -26.75))                                  # 开着的时候换目标：直接抢占
    assert not first.canceled
    first.finish(GoalStatus.STATUS_ABORTED)                          # 被顶掉的旧目标
    assert nav.status == 'driving'


def test_indoor_to_road_runs_nav2_then_lane(nav):
    nav.pose = HOME
    nav.on_goal(goal(*ROAD_GOAL))
    assert nav.follow.goals == []
    p = nav.nav.goals[-1].pose.pose
    out = nav.m.by_name['gate_out'].centerline[0]
    assert (p.position.x, p.position.y) == pytest.approx((out.x, out.y))
    assert 2 * math.atan2(p.orientation.z, p.orientation.w) == pytest.approx(-math.pi / 2, abs=1e-6)   # 朝南
    nav.pose = (out.x, out.y, -math.pi / 2)
    nav.nav.handles[-1].finish(GoalStatus.STATUS_SUCCEEDED)
    assert nav.status == 'driving'
    assert path_end(nav.follow) == pytest.approx(ROAD_GOAL, abs=0.1)
    nav.follow.handles[-1].finish(GoalStatus.STATUS_SUCCEEDED)
    assert nav.status == 'arrived'


def test_nav2_stops_short_of_gate_out_still_starts_lane(nav):
    """Nav2 到点容差 0.375 m，车可能停在 gate_out 起点线之前，这时车不在任何车道内（实测停在线前 0.15 m）"""
    nav.pose = HOME
    nav.on_goal(goal(*ROAD_GOAL))
    out = nav.m.by_name['gate_out'].centerline[0]
    nav.pose = (out.x + 0.1, out.y + 0.15, math.radians(-105.7))
    assert hdmap_navigator.hdmap.locate(nav.m, *nav.pose) is None
    nav.nav.handles[-1].finish(GoalStatus.STATUS_SUCCEEDED)
    assert nav.status == 'driving'
    assert path_end(nav.follow) == pytest.approx(ROAD_GOAL, abs=0.1)


def test_road_to_indoor_clears_speed_limit_before_nav2(nav):
    nav.on_goal(goal(*HOME[:2]))
    end = nav.m.by_name['gate_in'].centerline[-1]
    assert path_end(nav.follow) == pytest.approx((end.x, end.y), abs=0.1)
    nav.pose = (end.x, end.y, math.pi / 2)
    nav.follow.handles[-1].finish(GoalStatus.STATUS_SUCCEEDED)
    assert nav.speed_pub.msgs[-1].speed_limit == 0.0                 # 0 = 不限速，交给语义区域
    p = nav.nav.goals[-1].pose.pose.position
    assert (p.x, p.y) == pytest.approx(HOME[:2])
    nav.nav.handles[-1].finish(GoalStatus.STATUS_SUCCEEDED)
    assert nav.status == 'arrived'


def test_switching_from_nav2_to_lane_cancels_nav2(nav):
    nav.pose = HOME
    nav.on_goal(goal(*ROAD_GOAL))
    nav2_handle = nav.nav.handles[-1]
    nav.pose = ROAD                                                  # 假设车已经在公路上了
    nav.on_goal(goal(14.0, -26.75))
    assert nav2_handle.canceled
    nav2_handle.finish(GoalStatus.STATUS_CANCELED)
    assert nav.status == 'driving'


def test_lane_leg_waits_until_all_nav2_goals_are_canceled(nav):
    # RViz 的 Nav2 Goal 发的 navigate_to_pose 不归本节点管，但它的行为树会不停重发 FollowPath 把车道路径顶掉：
    # 先取消所有 navigate_to_pose 目标（goal_id 全 0、时间 0），收到回复再发车道路径
    nav.nav_cancel = _Service(pending=True)
    nav.on_goal(goal(*ROAD_GOAL))
    req = nav.nav_cancel.requests[-1]
    assert list(req.goal_info.goal_id.uuid) == [0] * 16
    assert (req.goal_info.stamp.sec, req.goal_info.stamp.nanosec) == (0, 0)
    assert nav.follow.goals == []
    f = nav.nav_cancel.futures[-1]
    for cb in f.cbs:
        cb(f)
    assert path_end(nav.follow) == pytest.approx(ROAD_GOAL, abs=0.1)
    assert nav.status == 'driving'


def test_cancel_reply_of_superseded_goal_is_ignored(nav):
    nav.nav_cancel = _Service(pending=True)
    nav.on_goal(goal(*ROAD_GOAL))
    nav.on_goal(goal(14.0, -26.75))
    for f in nav.nav_cancel.futures:
        for cb in f.cbs:
            cb(f)
    assert len(nav.follow.goals) == 1
    assert path_end(nav.follow) == pytest.approx((14.0, -26.75), abs=0.1)


def test_controller_failure_reports_leg(nav):
    nav.on_goal(goal(*ROAD_GOAL))
    nav.follow.handles[-1].finish(GoalStatus.STATUS_ABORTED)
    assert nav.status == 'failed: lane %d' % GoalStatus.STATUS_ABORTED


def test_speed_limit_republished_every_half_second(nav):
    nav.on_goal(goal(*ROAD_GOAL))
    n = len(nav.speed_pub.msgs)
    nav.on_timer()                       # 刚发过：车道没变、不到 0.5 s
    nav.on_timer()
    m = len(nav.speed_pub.msgs)
    assert m - n <= 1                    # 第一次 on_timer 可能因为 current_lanelet 初始为空而发一次
    nav.limit_sent -= 1.0                # 过了 1 s
    nav.on_timer()
    assert len(nav.speed_pub.msgs) == m + 1


def test_route_shown_as_lane_surfaces_and_bounds(nav):
    from visualization_msgs.msg import Marker
    nav.on_goal(goal(*ROAD_GOAL))
    (arr,) = nav.route_pub.msgs
    kinds = {mk.ns: mk for mk in arr.markers if mk.action == Marker.ADD}
    surf, bounds = kinds['route_lanes'], kinds['route_bounds']
    assert surf.type == Marker.TRIANGLE_LIST and 0 < surf.color.a < 1          # 半透明的车道面
    assert bounds.type == Marker.LINE_LIST and bounds.color.b > bounds.color.r  # 蓝色边界
    route = hdmap_navigator.hdmap.route(nav.m, nav.m.by_name['ring_n_eb_out_0'], nav.m.by_name['ring_n_eb_out_2'],
                                        0.0, 1.0)
    assert len(surf.points) == 3 * sum(len(hdmap_navigator.hdmap.lane_triangles(ll)) for ll in route)
    assert arr.markers[0].action == Marker.DELETEALL                           # 先清掉上一次的路由

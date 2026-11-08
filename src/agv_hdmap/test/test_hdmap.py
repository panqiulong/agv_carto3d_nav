import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))
import hdmap  # noqa: E402

# 用源码里生成的地图，不依赖 install
OSM = os.path.join(HERE, '..', '..', 'agv_gazebo', 'worlds', 'warehouse_lanelet2_map.osm')
SPAWN = (2.0, -4.0)


def to_map(x, y):
    return x - SPAWN[0], y - SPAWN[1]


@pytest.fixture(scope='module')
def m():
    return hdmap.load(OSM)


def test_load_uses_local_xy(m):
    c = [(p.x, p.y) for p in m.by_name['gate_out'].centerline]
    assert c[0][0] == pytest.approx(3.0 - 1.25 - SPAWN[0], abs=1e-6)
    assert c[0][1] == pytest.approx(-9.3 - SPAWN[1], abs=1e-6)
    # 靠右行驶：往南开的出门车道在路的西半边（世界 x < 3）
    assert all(x < 3.0 - SPAWN[0] for x, _ in c)


def test_locate_on_ring_with_heading(m):
    x, y = to_map(-17.0, -27.0 - 3.75)            # 北段往东外侧车道的中心线上
    ll = hdmap.locate(m, x, y, 0.0)
    assert ll is not None and ll.attributes['name'].startswith('ring_n_eb_out_')
    assert hdmap.locate(m, x, y, math.pi) is None


def test_locate_without_yaw_takes_nearest_centerline(m):
    x, y = to_map(-17.0, -27.0 - 1.0)             # 北段往东内侧车道，离中心线 0.25 m
    assert hdmap.locate(m, x, y, None).attributes['name'].startswith('ring_n_eb_in_')


def test_locate_off_road_is_none(m):
    assert hdmap.locate(m, *to_map(-14.0, -38.0), None) is None     # 环路中间的空地


def _launch():
    import importlib.util
    spec = importlib.util.spec_from_file_location('hdmap_launch', os.path.join(HERE, '..', 'launch', 'hdmap.launch.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_projector_file_next_to_map(tmp_path):
    osm = tmp_path / 'wh_lanelet2_map.osm'
    (tmp_path / 'map_projector_info.yaml').write_text('projector_type: Local\n')
    assert _launch().projector_path(str(osm)) == str(tmp_path / 'map_projector_info.yaml')   # 生成目录里的名字
    (tmp_path / 'wh_map_projector_info.yaml').write_text('projector_type: Local\n')
    assert _launch().projector_path(str(osm)) == str(tmp_path / 'wh_map_projector_info.yaml')  # start.sh 复制后的名字


def test_launch_starts_autoware_loader_and_visualizer():
    from launch import LaunchContext
    ctx = LaunchContext()
    ctx.launch_configurations.update({'hdmap': OSM, 'use_sim_time': 'true'})
    nodes = _launch().hdmap_nodes(ctx)
    execs = [n._Node__node_executable for n in nodes]
    assert execs == ['autoware_map_projection_loader_node', 'autoware_lanelet2_map_loader',
                     'autoware_lanelet2_map_visualizer', 'hdmap_navigator.py', 'traffic_light_sim.py']


# ---------------- 路由和路径 ----------------
from lanelet2 import geometry  # noqa: E402
from lanelet2.core import BasicPoint2d  # noqa: E402


def names(route):
    return [ll.attributes['name'] for ll in route]


def lane_changes(m, route):
    """路由里相邻两条是左右关系（变道）的 (前, 后)"""
    out = []
    for a, b in zip(route, route[1:]):
        for side in (m.graph.left(a), m.graph.right(a)):
            if side is not None and side.id == b.id:
                out.append((a, b))
    return out


def circum_radius(a, b, c):
    cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    if abs(cross) < 1e-9:
        return math.inf
    return math.dist(a, b) * math.dist(b, c) * math.dist(a, c) / (2 * abs(cross))


def point_on(ll, s):
    c = [(p.x, p.y) for p in ll.centerline]
    for a, b in zip(c, c[1:]):
        d = math.dist(a, b)
        if s <= d:
            return a[0] + (b[0] - a[0]) * s / d, a[1] + (b[1] - a[1]) * s / d
        s -= d
    return c[-1]


@pytest.fixture(scope='module')
def left_turn(m):
    # 北段往西外侧（十字路口东边、虚线段）→ 十字路口左转进南支路：要先变道到内侧
    start, goal = m.by_name['ring_n_wb_out_0'], m.by_name['branch_sb']
    sxy, gxy = point_on(start, 1.0), point_on(goal, 3.0)
    route = hdmap.route(m, start, goal, 1.0, 3.0)
    return route, hdmap.path(m, route, sxy, gxy), gxy


def test_route_ring_far_side(m):
    start, goal = m.by_name['ring_n_eb_out_0'], m.by_name['ring_s_wb_out_2']
    route = hdmap.route(m, start, goal, 2.0, 2.0)
    assert route is not None
    assert route[0].id == start.id and route[-1].id == goal.id


def test_route_with_left_turn_changes_to_inner_lane(m, left_turn):
    route, _, _ = left_turn
    assert 'x_e2s_left' in names(route)
    (a, b), = lane_changes(m, route)
    assert a.attributes['name'].startswith('ring_n_wb_out_') and b.attributes['name'].startswith('ring_n_wb_in_')
    assert a.leftBound.attributes['subtype'] == 'dashed'


def test_path_continuous_and_smooth(left_turn):
    _, pts, _ = left_turn
    assert all(math.dist(p[:2], q[:2]) < 0.15 for p, q in zip(pts, pts[1:]))
    s = pts[::10]                                    # 隔 1 m 取样
    radii = [circum_radius(*(p[:2] for p in s[i:i + 3])) for i in range(len(s) - 2)]
    assert min(radii) >= 3.0


def test_lane_change_transition_inside_dashed(m, left_turn):
    route, pts, _ = left_turn
    (a, b), = lane_changes(m, route)
    on_centre = lambda p: min(hdmap.nearest(ll, *p[:2])[0] for ll in route) < 0.05
    moving = [p for p in pts if not on_centre(p)]    # 变道过渡段：不在任何一条中心线上
    assert len(moving) > 30
    for p in moving:
        q = BasicPoint2d(*p[:2])
        assert min(geometry.distance(a, q), geometry.distance(b, q)) < 1e-6


def test_path_ends_at_goal_projection(left_turn):
    _, pts, gxy = left_turn
    assert math.dist(pts[-1][:2], gxy) < 0.1


def test_goal_behind_on_same_lanelet_goes_around(m):
    ll = m.by_name['ring_n_eb_out_2']
    sxy, gxy = point_on(ll, 6.0), point_on(ll, 2.0)
    route = hdmap.route(m, ll, ll, 6.0, 2.0)
    assert route[0].id == ll.id and route[-1].id == ll.id and len(route) > 2
    pts = hdmap.path(m, route, sxy, gxy)
    # 最短的绕法是经南支路的小圈（T 口右转、十字路口右转回来），约 75 m
    assert sum(math.dist(p[:2], q[:2]) for p, q in zip(pts, pts[1:])) > 50.0
    assert 'branch_nb' in names(route)
    assert math.dist(pts[-1][:2], gxy) < 0.1


def test_goal_yaw_ignored_uses_lane_direction(m):
    gx, gy = to_map(-5.0, -49.0 + 3.75)              # 南段往西外侧车道（T 口西边）
    goal = hdmap.locate_goal(m, gx, gy)
    assert goal is not None and goal.attributes['name'].startswith('ring_s_wb_out_')
    start = m.by_name['ring_s_wb_out_0']
    sxy = point_on(start, 1.0)
    route = hdmap.route(m, start, goal, 1.0, hdmap.project(goal, gx, gy))
    pts = hdmap.path(m, route, sxy, (gx, gy))
    assert abs(math.remainder(pts[-1][2] - math.pi, 2 * math.pi)) < 0.05      # 车道方向：往西


def test_project_is_arc_length(m):
    ll = m.by_name['ring_n_eb_out_0']
    assert hdmap.project(ll, *point_on(ll, 4.0)) == pytest.approx(4.0, abs=1e-6)


def test_locate_in_intersection_uses_heading(m):
    x, y = to_map(4.25, -27.0 - 3.75)                 # 十字路口里：东西向外侧直行车道和南北向直行车道的交点
    assert hdmap.locate(m, x, y, 0.0).attributes['name'] == 'x_w2e_out'
    assert hdmap.locate(m, x, y, math.pi / 2).attributes['name'] == 'x_s2n'


# ---------------- 交通规则 ----------------
def test_speed_limit_units(m):
    assert hdmap.speed_limit(m.by_name['ring_n_eb_out_0']) == pytest.approx(0.5, abs=1e-9)
    assert hdmap.speed_limit(m.by_name['ring_ne_corner_cw_out']) == pytest.approx(0.3, abs=1e-9)
    assert hdmap.speed_limit(m.by_name['x_e2s_left']) == pytest.approx(0.3, abs=1e-9)


def test_light_cycle():
    c = hdmap.light_color
    assert [c(t, 'ew') for t in (0.0, 20.5, 23.5, 46.5)] == ['green', 'yellow', 'red', 'green']
    assert [c(t, 'ns') for t in (0.0, 23.5, 43.5)] == ['red', 'green', 'yellow']


@pytest.fixture(scope='module')
def west_approach(m):
    """往东开过十字路口西进口的路径，和西进口那个红绿灯"""
    start, goal = m.by_name['ring_n_eb_out_0'], m.by_name['ring_n_eb_out_2']
    lanes = hdmap.route(m, start, goal, 1.0, 5.0)
    pts = hdmap.path(m, lanes, point_on(start, 1.0), point_on(goal, 5.0))
    (tl,) = m.by_name['ring_n_eb_out_1'].trafficLights()
    group, line = hdmap.traffic_lights(m)[tl.id]
    # 路径越过停止线的下标（停止线在 x = 常数上，路径往东）
    cross = next(i for i, p in enumerate(pts) if p[0] >= line[0][0])
    return lanes, pts, tl.id, group, line, cross


def test_traffic_lights_table(m, west_approach):
    _, _, tl_id, group, line, _ = west_approach
    lights = hdmap.traffic_lights(m)
    assert len(lights) == 4 and group == 'ew'
    assert line[0][0] == pytest.approx(line[1][0]) and line[0][0] == pytest.approx(-2.25 - SPAWN[0])


def test_red_stops_before_line(m, west_approach):
    lanes, pts, tl_id, _, line, _ = west_approach
    k = hdmap.stop_index(m, lanes, pts, 0, {tl_id: 'red'})
    assert k is not None
    assert line[0][0] - pts[k][0] == pytest.approx(0.85, abs=0.05)     # 车头（base_link 前 0.6）在线前 0.25


def test_green_no_stop(m, west_approach):
    lanes, pts, tl_id, *_ = west_approach
    assert hdmap.stop_index(m, lanes, pts, 0, {tl_id: 'green'}) is None
    assert hdmap.stop_index(m, lanes, pts, 0, {}) is None               # 没收到灯的状态：不停


def test_no_stop_when_past_line(m, west_approach):
    lanes, pts, tl_id, _, _, cross = west_approach
    assert hdmap.stop_index(m, lanes, pts, cross - 5, {tl_id: 'red'}) is None      # 车头已经过线 0.1 m


def test_yellow_close_to_line_passes(m, west_approach):
    lanes, pts, tl_id, _, line, cross = west_approach
    assert hdmap.stop_index(m, lanes, pts, cross - 10, {tl_id: 'yellow'}) is None   # 车头离线 0.4 m
    k = hdmap.stop_index(m, lanes, pts, cross - 26, {tl_id: 'yellow'})             # 车头离线 2 m
    assert k is not None and line[0][0] - pts[k][0] == pytest.approx(0.85, abs=0.05)


def test_red_while_waiting_at_line_keeps_stopping(m, west_approach):
    lanes, pts, tl_id, _, _, cross = west_approach
    k = hdmap.stop_index(m, lanes, pts, 0, {tl_id: 'red'})
    assert hdmap.stop_index(m, lanes, pts, k, {tl_id: 'red'}) == k                # 停在线前等红灯时不会往前挪


# ---------------- 仓库 ⇄ 公路交接 ----------------
# 仓库墙内表面 x ∈ [-25, 25]、y ∈ [-15, 15]（世界）换到 map 坐标
INDOOR = [to_map(-25.0, -15.0), to_map(25.0, -15.0), to_map(25.0, 15.0), to_map(-25.0, 15.0)]
ROAD = (*to_map(-17.0, -30.75), 0.0)               # 北段往东外侧车道上
HOME = (0.0, 0.0, 0.0)


def lane_end(ll, end):
    c = [(p.x, p.y) for p in ll.centerline]
    a, b = (c[0], c[1]) if end == 0 else (c[-2], c[-1])
    p = c[0] if end == 0 else c[-1]
    return pytest.approx((p[0], p[1], math.atan2(b[1] - a[1], b[0] - a[0])), abs=1e-6)


def test_road_to_road(m):
    goal = (*to_map(20.0, -49.0 + 3.75), math.pi)
    assert hdmap.plan_legs(m, ROAD, goal, INDOOR) == [('lane', goal)]


def test_indoor_to_road(m):
    legs = hdmap.plan_legs(m, HOME, ROAD, INDOOR)
    assert [k for k, _ in legs] == ['nav2', 'lane']
    assert legs[0][1] == lane_end(m.by_name['gate_out'], 0)        # 出门交接点：出门车道起点，朝南
    assert legs[1][1] == ROAD


def test_road_to_indoor(m):
    legs = hdmap.plan_legs(m, ROAD, HOME, INDOOR)
    assert [k for k, _ in legs] == ['lane', 'nav2']
    assert legs[0][1] == lane_end(m.by_name['gate_in'], -1)        # 进门交接点：进门车道终点，朝北
    assert legs[1][1] == HOME


def test_indoor_to_indoor(m):
    goal = (*to_map(-16.0, 1.45), 0.0)
    assert hdmap.plan_legs(m, HOME, goal, INDOOR) == [('nav2', goal)]


def test_goal_off_road_is_rejected(m):
    assert hdmap.plan_legs(m, HOME, (*to_map(-14.0, -38.0), 0.0), INDOOR) == 'goal off road'
    assert hdmap.plan_legs(m, ROAD, (*to_map(-14.0, -38.0), 0.0), INDOOR) == 'goal off road'


def test_robot_off_road_is_rejected(m):
    assert hdmap.plan_legs(m, (*to_map(-14.0, -38.0), 0.0), ROAD, INDOOR) == 'robot off road'


def test_robot_on_gate_lane_counts_as_road(m):
    robot = (*to_map(1.75, -12.0), -math.pi / 2)    # 交接车道在仓库里的那段上，朝南
    assert hdmap.plan_legs(m, robot, ROAD, INDOOR) == [('lane', ROAD)]


# ---------------- 路由显示 ----------------
def _area(poly):
    return abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(poly, poly[1:] + poly[:1]))) / 2


@pytest.mark.parametrize('name', ['ring_n_eb_out_0', 'ring_ne_corner_cw_in', 'x_e2s_left', 'gate_out'])
def test_lane_triangles_cover_the_lanelet(m, name):
    ll = m.by_name[name]
    tris = hdmap.lane_triangles(ll)
    area = sum(_area(list(t)) for t in tris)
    poly = [(p.x, p.y) for p in ll.leftBound] + [(p.x, p.y) for p in reversed(list(ll.rightBound))]
    assert area == pytest.approx(_area(poly), rel=0.02)

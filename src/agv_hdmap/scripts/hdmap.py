"""高精地图的纯计算部分（不依赖 ROS，单元测试直接 import）：读 Lanelet2 地图、定位到车道。

坐标都是 map 系：.osm 里每个点的 local_x / local_y（和 Autoware 的 Local 投影一样），
等于世界坐标减去出生点。
"""
import math
from dataclasses import dataclass

import lanelet2
from lanelet2 import geometry, routing, traffic_rules
from lanelet2.core import BasicPoint2d
from lanelet2.io import Origin
from lanelet2.projection import UtmProjector


@dataclass
class HdMap:
    lmap: object                 # lanelet2.core.LaneletMap
    rules: object                # 交通规则：Germany / Vehicle（Autoware 也用它）
    graph: object                # 路由图：lanelet 之间的前后、左右（能否变道）关系
    by_name: dict                # lanelet 的 name 属性 → Lanelet


def load(osm_path):
    # .osm 里的经纬度是占位的 UTM 投影，真正的坐标在 local_x / local_y：读完逐点覆盖
    lmap = lanelet2.io.load(osm_path, UtmProjector(Origin(35.0, 139.0)))
    for p in lmap.pointLayer:
        if 'local_x' in p.attributes:
            p.x, p.y = float(p.attributes['local_x']), float(p.attributes['local_y'])
    rules = traffic_rules.create(traffic_rules.Locations.Germany, traffic_rules.Participants.Vehicle)
    graph = routing.RoutingGraph(lmap, rules)
    by_name = {ll.attributes['name']: ll for ll in lmap.laneletLayer if 'name' in ll.attributes}
    return HdMap(lmap, rules, graph, by_name)


def _angle_diff(a, b):
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def nearest(ll, x, y):
    """点到 ll 中心线最近处：(距离, 该处车道方向, 弧长)"""
    c = [(p.x, p.y) for p in ll.centerline]
    best, s0 = None, 0.0
    for (ax, ay), (bx, by) in zip(c, c[1:]):
        dx, dy = bx - ax, by - ay
        seg = math.hypot(dx, dy)
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / (seg * seg))) if seg > 0 else 0.0
        d = math.hypot(x - ax - t * dx, y - ay - t * dy)
        if best is None or d < best[0]:
            best = (d, math.atan2(dy, dx), s0 + t * seg)
        s0 += seg
    return best


def locate(m, x, y, yaw):
    """车在哪条车道上：包含 (x, y) 的 lanelet 里，车道方向和 yaw 相差 < 90° 的（yaw 为 None 时不看方向），
    取离中心线最近的；都不满足时返回 None"""
    p = BasicPoint2d(x, y)
    best = None
    for ll in m.lmap.laneletLayer:
        if not geometry.inside(ll, p):
            continue
        d, heading, _ = nearest(ll, x, y)
        if yaw is not None and _angle_diff(heading, yaw) >= math.pi / 2:
            continue
        if best is None or d < best[0]:
            best = (d, ll)
    return best[1] if best else None


def locate_goal(m, x, y):
    """目标在哪条车道上：只看位置（RViz 里点目标时箭头方向常常点得不准），到达时的朝向取车道方向"""
    return locate(m, x, y, None)


def project(ll, x, y):
    """点在 ll 中心线上的弧长"""
    return nearest(ll, x, y)[2]


# ---------------- 路由 ----------------
# 路由图里每条 lanelet 是一个节点，"开完接着开"（following）和"变道到左右相邻"（left / right，
# 只有两条车道之间是虚线时才有）是边。shortestPath 在这张图上按长度找最短的 lanelet 序列。
def route(m, start, goal, start_s, goal_s):
    """start 车道上弧长 start_s 处 → goal 车道上 goal_s 处的 lanelet 序列，到不了返回 None。
    目标和车在同一条车道、但在车后面时，从这条车道的后继出发绕一圈回来"""
    if start.id == goal.id:
        if goal_s >= start_s:
            return [start]
        best = None
        for f in m.graph.following(start):
            p = m.graph.shortestPath(f, goal)
            if p is not None and (best is None or _length(p) < _length(best)):
                best = p
        return [start] + list(best) if best is not None else None
    p = m.graph.shortestPath(start, goal)
    return list(p) if p is not None else None


def _length(lanelets):
    return sum(geometry.length2d(ll) for ll in lanelets)


# ---------------- 路径 ----------------
# 路由只是 lanelet 序列，车要走的是一条线：沿每条 lanelet 的中心线走；序列里相邻两条是左右关系时是变道，
# 在前一条的最后 LANE_CHANGE 米里用余弦曲线从前一条的中心线移到后一条的中心线上。
LANE_CHANGE = 8.0
STEP = 0.1


def _centre(ll):
    return [(p.x, p.y) for p in ll.centerline]


def _at(c, s):
    """折线 c 上弧长 s 处的点"""
    for a, b in zip(c, c[1:]):
        d = math.dist(a, b)
        if s <= d or (a, b) == (c[-2], c[-1]):
            t = min(1.0, s / d) if d > 0 else 0.0
            return a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
        s -= d
    return c[-1]


def _samples(a, b):
    n = max(1, math.ceil((b - a) / STEP))
    return [a + (b - a) * k / n for k in range(n + 1)]


def _is_lane_change(m, a, b):
    return any(x is not None and x.id == b.id for x in (m.graph.left(a), m.graph.right(a)))


def path(m, lanelets, start_xy, goal_xy):
    """路由 → map 系 (x, y, yaw) 序列，间隔约 0.1 m。从起点在第一条中心线上的投影开始，到终点在最后一条上的投影结束"""
    n = len(lanelets)
    s_start = project(lanelets[0], *start_xy)
    s_goal = project(lanelets[-1], *goal_xy)
    pts = []
    i = 0
    while i < n:
        ll = lanelets[i]
        c = _centre(ll)
        length = geometry.length2d(ll)
        a = s_start if i == 0 else 0.0
        if i + 1 < n and _is_lane_change(m, ll, lanelets[i + 1]):
            nxt = lanelets[i + 1]
            cn, ln = _centre(nxt), geometry.length2d(nxt)
            b = s_goal * length / ln if i + 1 == n - 1 else length          # 变道进最后一条：过渡在终点处结束
            t0 = max(a, b - LANE_CHANGE)
            for s in _samples(a, b):
                u = 0.0 if s <= t0 else (s - t0) / (b - t0)
                w = (1 - math.cos(math.pi * u)) / 2
                p, q = _at(c, s), _at(cn, s * ln / length)
                pts.append((p[0] + (q[0] - p[0]) * w, p[1] + (q[1] - p[1]) * w))
            i += 2                                                           # 后一条已经开到头了
            continue
        b = s_goal if i == n - 1 else length
        pts += [_at(c, s) for s in _samples(a, b)]
        i += 1
    pts = _dedupe(pts)
    out = [(x, y, math.atan2(q[1] - y, q[0] - x)) for (x, y), q in zip(pts, pts[1:])]
    gx, gy = pts[-1]
    _, heading, _ = nearest(lanelets[-1], gx, gy)
    out.append((gx, gy, heading))
    return out


def _dedupe(pts):
    out = [pts[0]]
    for p in pts[1:]:
        if math.dist(p, out[-1]) > 1e-3:
            out.append(p)
    return out


# ---------------- 交通规则 ----------------
# 限速是 lanelet 自己的属性；红绿灯是挂在进口车道上的 regulatory element，引用灯和停止线。
# 车沿路由开，前方第一条挂了红绿灯的车道的停止线就是要看的线。
HEAD = 0.6            # 车头在 base_link 前 0.6 m
STOP_GAP = 0.25       # 停车时车头离停止线
YELLOW_PASS = 0.5     # 黄灯时车头离线不到这么远就直接通过（0.5 m/s 下来不及平稳停住）
COLORS = ('green', 'yellow', 'red')


def speed_limit(ll):
    """车道限速，m/s（地图里按 Autoware 约定写 km/h）"""
    return float(ll.attributes['speed_limit']) / 3.6


def light_color(t, group, green=20.0, yellow=3.0, red=23.0):
    """仿真红绿灯：ew 组 t = 0 变绿，ns 组在 ew 变红时变绿"""
    tt = (t - (0.0 if group == 'ew' else green + yellow)) % (green + yellow + red)
    return 'green' if tt < green else 'yellow' if tt < green + yellow else 'red'


def traffic_lights(m):
    """红绿灯 regulatory element 的 id → (signal_group, 停止线两端点)"""
    return {r.id: (r.attributes['signal_group'], [(p.x, p.y) for p in r.stopLine])
            for r in m.lmap.regulatoryElementLayer
            if 'subtype' in r.attributes and r.attributes['subtype'] == 'traffic_light'}


def _arc_lengths(pts):
    s = [0.0]
    for p, q in zip(pts, pts[1:]):
        s.append(s[-1] + math.dist(p[:2], q[:2]))
    return s


def _crossing(p, q, a, b):
    """线段 pq 和 ab 相交时返回在 pq 上的比例 t，否则 None"""
    d = (q[0] - p[0]) * (b[1] - a[1]) - (q[1] - p[1]) * (b[0] - a[0])
    if abs(d) < 1e-12:
        return None
    t = ((a[0] - p[0]) * (b[1] - a[1]) - (a[1] - p[1]) * (b[0] - a[0])) / d
    u = ((a[0] - p[0]) * (q[1] - p[1]) - (a[1] - p[1]) * (q[0] - p[0])) / d
    return t if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0 else None


def next_light(m, lanelets, pts, robot_i):
    """路径前方第一条车头还没过的停止线：(红绿灯 id, 停止线在路径上的弧长, 车头当前的弧长)，没有就 None"""
    s = _arc_lengths(pts)
    head = s[robot_i] + HEAD
    best = None
    seen = set()
    for ll in lanelets:
        for tl in ll.trafficLights():
            if tl.id in seen:
                continue
            seen.add(tl.id)
            a, b = [(p.x, p.y) for p in tl.stopLine][:2]
            for i in range(len(pts) - 1):
                t = _crossing(pts[i], pts[i + 1], a, b)
                if t is not None and s[i] + t * (s[i + 1] - s[i]) > head:
                    sc = s[i] + t * (s[i + 1] - s[i])
                    if best is None or sc < best[1]:
                        best = (tl.id, sc, head)
                    break
    return best


def stop_index(m, lanelets, pts, robot_i, colors):
    """红灯，或黄灯且车头离线还有 ≥ YELLOW_PASS 时，返回 base_link 该停的路径下标（车头在线前 STOP_GAP）；
    车头已经过线、灯是绿的、或没收到灯的状态时返回 None。colors：红绿灯 id → 'green' / 'yellow' / 'red'"""
    nl = next_light(m, lanelets, pts, robot_i)
    if nl is None:
        return None
    tl_id, s_cross, head = nl
    color = colors.get(tl_id, 'green')
    if color == 'green' or (color == 'yellow' and s_cross - head < YELLOW_PASS):
        return None
    s = _arc_lengths(pts)
    target = s_cross - HEAD - STOP_GAP
    k = min(range(len(s)), key=lambda i: abs(s[i] - target))
    return max(k, robot_i)


# ---------------- 仓库 ⇄ 公路交接 ----------------
# 公路上按车道开，仓库里交给 Nav2。交接车道 gate_out / gate_in 有一段伸进仓库：出门时 Nav2 把车送到 gate_out
# 的起点（车头朝南），回来时车沿车道开到 gate_in 的终点，再由 Nav2 开到目标。
def inside(poly, x, y):
    """点在多边形里（射线法）"""
    hit = False
    for (ax, ay), (bx, by) in zip(poly, poly[1:] + poly[:1]):
        if (ay > y) != (by > y) and x < ax + (y - ay) * (bx - ax) / (by - ay):
            hit = not hit
    return hit


def _end_pose(ll, end):
    c = _centre(ll)
    a, b = (c[0], c[1]) if end == 0 else (c[-2], c[-1])
    x, y = c[0] if end == 0 else c[-1]
    return x, y, math.atan2(b[1] - a[1], b[0] - a[0])


def plan_legs(m, robot, goal, indoor):
    """把一个目标拆成几段：[('nav2' | 'lane', (x, y, yaw)), ...]；不能去时返回原因字符串。
    robot / goal 是 map 系 (x, y, yaw)，indoor 是仓库内部的多边形"""
    robot_on_road = locate(m, *robot) is not None
    goal_on_road = locate_goal(m, *goal[:2]) is not None
    if not goal_on_road and not inside(indoor, *goal[:2]):
        return 'goal off road'
    if not robot_on_road and not inside(indoor, *robot[:2]):
        return 'robot off road'
    if robot_on_road and goal_on_road:
        return [('lane', goal)]
    if robot_on_road:
        return [('lane', _end_pose(m.by_name['gate_in'], -1)), ('nav2', goal)]
    if goal_on_road:
        return [('nav2', _end_pose(m.by_name['gate_out'], 0)), ('lane', goal)]
    return [('nav2', goal)]


# ---------------- 路由显示 ----------------
def _resample(c, n):
    """折线 c 按弧长等分成 n 个点"""
    total = sum(math.dist(a, b) for a, b in zip(c, c[1:]))
    return [_at(c, total * k / (n - 1)) for k in range(n)]


def lane_triangles(ll):
    """把 lanelet 铺成三角形（左右边界按弧长对齐后两两连成四边形，再切成两个三角形），RViz 画车道面用"""
    left, right = [(p.x, p.y) for p in ll.leftBound], [(p.x, p.y) for p in ll.rightBound]
    n = max(len(left), len(right), 2)
    left, right = _resample(left, n), _resample(right, n)
    tris = []
    for i in range(n - 1):
        tris += [(left[i], right[i], right[i + 1]), (left[i], right[i + 1], left[i + 1])]
    return tris

#!/usr/bin/env python3
"""仓库南墙外的公路：几何 + Autoware 格式的 Lanelet2 高精地图。

gen_warehouse_world.py 用同一份几何画 Gazebo 里的路面和标线（build((0, 0))，世界坐标），
再写出 .osm（build(出生点)，map 坐标）。设计见 docs/superpowers/specs/2026-10-26-hd-map-road-design.md。

布局（世界坐标，x 向东，y 向北，单位 m）：
  - 环路：道路中线是矩形 x ∈ [-31.25, 37.25]、y ∈ [-49, -27]，四角圆弧半径 9；每向 2 条车道，靠右行驶
  - 门路：大门 x ∈ [0, 6] 往南到十字路口 (3, -27)；交接车道 gate_out / gate_in 伸进仓库到 y = -9.3
  - 南支路：十字路口往南到 T 形路口 (3, -49)
  - 路口前 10 m 是实线，再往外 10 m 是虚线（能变道）；弯道和东西两段短直路是实线

代码按 Lanelet2 的概念分节：1 点和线 → 2 路段的边界线 → 3 lanelet → 4 路口的转弯车道 → 5 交通规则 → 6 画图用的数据。
"""
import math
from dataclasses import dataclass

from lanelet2.core import AttributeMap, Lanelet, LaneletMap, LineString3d, Point3d, TrafficLight
from lanelet2.io import Origin, write
from lanelet2.projection import UtmProjector

LANE_W = 2.5
RING_X, RING_Y, RING_R = (-31.25, 37.25), (-49.0, -27.0), 9.0   # 环路道路中线的矩形和四角半径
ROAD_X = 3.0                                                      # 门路 / 南支路的道路中线
GATE_Y, HANDOFF_Y = -15.3, -9.3                                   # 南墙外表面；交接车道在仓库里的起点
BOX_HX, BOX_HY = 5.25, 7.75                                       # 路口到道路中心交点的距离（x 向 / y 向）
SOLID_BEFORE = 10.0                                               # 路口前不许变道的长度
ARC_STEP = 0.5                                                    # 弯道按道路中线弧长离散
TURN_STEP = 0.4
SPEED_STRAIGHT, SPEED_SLOW = '1.8', '1.08'                        # km/h（Autoware 约定）= 0.5 / 0.3 m/s
LIGHT_Z, LIGHT_W, POLE_GAP = 2.0, 0.6, 1.2                        # 灯离地高度、灯宽；灯杆离路口角的距离

PROJECTOR_INFO = {'projector_type': 'Local'}

# 横断面：边界线的横向偏移（道路中线左侧为正）和车道 (方向, 右边界偏移, 左边界偏移)。
# fwd 沿路段定义方向行驶，bwd 反向。in 是靠中线的车道，out 是靠路边的车道
RING_LINES = (-2 * LANE_W, -LANE_W, 0.0, LANE_W, 2 * LANE_W)
RING_LANES = {'fwd_out': ('fwd', -2 * LANE_W, -LANE_W), 'fwd_in': ('fwd', -LANE_W, 0.0),
              'bwd_in': ('bwd', 0.0, LANE_W), 'bwd_out': ('bwd', LANE_W, 2 * LANE_W)}
TWO_LINES = (-LANE_W, 0.0, LANE_W)
TWO_LANES = {'fwd': ('fwd', -LANE_W, 0.0), 'bwd': ('bwd', 0.0, LANE_W)}


@dataclass
class Road:
    lmap: LaneletMap
    markings: list      # (kind, [(x, y), ...])，kind ∈ solid / dashed / stop_line
    surface: list       # (cx, cy, yaw, 长, 宽)
    lights: list        # (signal_group, x, y, 灯朝向 yaw)


class _Builder:
    def __init__(self, offset):
        self.ox, self.oy = offset
        self.next_id = 0
        self.lmap = LaneletMap()
        self.lanelets = {}
        self.cache = {}          # (路名, 站号, 偏移) → Point3d：相邻路段在端点共用同一个点对象

    # ---------- 1. 点和线 ----------
    # Lanelet2 的地图只有三种基本元素：点（Point3d）、线（LineString3d，一串点）、以及由线组成的 lanelet 和规则。
    # 线的 type / subtype 决定它是什么：line_thin + solid / dashed 是车道线，virtual 是路口里看不见的边界，
    # stop_line 是停止线，traffic_light 是灯。Autoware 的 Local 投影直接读每个点的 local_x / local_y。
    def _id(self):
        self.next_id += 1
        return self.next_id

    def pt(self, x, y, z=0.0):
        x, y = x - self.ox, y - self.oy
        return Point3d(self._id(), x, y, z, AttributeMap({'local_x': f'{x:.6f}', 'local_y': f'{y:.6f}'}))

    def line(self, pts, type_, subtype=None, **extra):
        attrs = {'type': type_}
        if subtype:
            attrs['subtype'] = subtype
        attrs.update(extra)
        return LineString3d(self._id(), pts, AttributeMap(attrs))

    def station_pt(self, road, i, d):
        """路 road 的第 i 个站点往左偏 d 处的点（站点 = 道路中线上的 (x, y, 朝向)）。"""
        key = (road.name, i, d)
        if key not in self.cache:
            x, y, h = road.stations[i]
            self.cache[key] = self.pt(x - math.sin(h) * d, y + math.cos(h) * d)
        return self.cache[key]

    # ---------- 3. lanelet ----------
    # lanelet = 左边界 + 右边界（相对行驶方向），中心线由两条边界决定，这里直接给出同一份几何算的中心线。
    # 前后相接的 lanelet 共用端点，路由图就是靠这个知道"开完这条接哪条"；左右相邻的 lanelet 共用一条边界线，
    # 这条线是 dashed 时路由图允许变道，solid / virtual 不允许。反方向的车道用 line.invert() 引用同一条线。
    def lanelet(self, name, left, right, centre, slow):
        attrs = {'type': 'lanelet', 'subtype': 'road', 'location': 'urban', 'one_way': 'yes',
                 'participant:vehicle': 'yes', 'speed_limit': SPEED_SLOW if slow else SPEED_STRAIGHT, 'name': name}
        ll = Lanelet(self._id(), left, right, AttributeMap(attrs))
        ll.centerline = LineString3d(self._id(), centre)
        self.lmap.add(ll)
        self.lanelets[name] = ll
        return ll


class _Path:
    """一条路的道路中线：站点 (x, y, 朝向) 和路段 (kind, 站号列表, 名字信息)。"""

    def __init__(self, name, lines, lanes):
        self.name, self.lines, self.lanes = name, lines, lanes
        self.stations, self.sections = [], []

    def add(self, pts, kinds, info):
        """pts：这一段的站点（第一个和上一段的最后一个重合时不重复加）；kinds[k] 是第 k 个间隔的路段类型。"""
        if self.stations and math.dist(self.stations[-1][:2], pts[0][:2]) < 1e-9:
            pts = pts[1:]
            start = len(self.stations) - 1
        else:
            start = len(self.stations)
        self.stations += pts
        idx = list(range(start, len(self.stations)))
        if len(kinds) == 1:
            self.sections.append((kinds[0], idx, info))
        else:
            for k, kind in enumerate(kinds):
                self.sections.append((kind, idx[k:k + 2], info))

    def close(self):
        """首尾站点重合时把最后一个站点换成第 0 个，环路就闭合了。"""
        last = len(self.stations) - 1
        self.stations.pop()
        self.sections = [(k, [0 if i == last else i for i in idx], info) for k, idx, info in self.sections]

    def index(self, x, y):
        return next(i for i, s in enumerate(self.stations) if math.dist(s[:2], (x, y)) < 1e-9)


def _straight(x0, y0, x1, y1, cuts):
    h = math.atan2(y1 - y0, x1 - x0)
    return [(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, h) for t in cuts]


def _arc(cx, cy, a0, a1):
    n = math.ceil(RING_R * abs(a1 - a0) / ARC_STEP)
    sgn = 1.0 if a1 > a0 else -1.0
    return [(cx + RING_R * math.cos(a), cy + RING_R * math.sin(a), a + sgn * math.pi / 2)
            for a in (a0 + (a1 - a0) * k / n for k in range(n + 1))]


def _ring():
    """环路，按顺时针（cw）定义：fwd 车道 = 环路内侧半幅（北段往东），bwd 车道 = 外侧半幅（北段往西）。"""
    (x0, x1), (y0, y1), r = RING_X, RING_Y, RING_R
    xw, xe = ROAD_X - BOX_HX, ROAD_X + BOX_HX
    along = [x0 + r, xw - SOLID_BEFORE, xw, xe, xe + SOLID_BEFORE, x1 - r]    # 北段 / 南段的切分点（西 → 东）
    ring = _Path('ring', RING_LINES, RING_LANES)
    seg = lambda s, cw, ccw: {'seg': s, 'cw': cw, 'ccw': ccw}
    ring.add([(x, y1, 0.0) for x in along], ['straight', 'straight', 'x', 'straight', 'straight'],
             {'seg': 'n', 'cw': 'eb', 'ccw': 'wb', 'x_cw': 'x_w2e', 'x_ccw': 'x_e2w'})
    ring.add(_arc(x1 - r, y1 - r, math.pi / 2, 0.0), ['corner'], {'corner': 'ne', 'c': (x1 - r, y1 - r)})
    ring.add(_straight(x1, y1 - r, x1, y0 + r, (0.0, 1.0)), ['straight'], seg('e', 'sb', 'nb'))
    ring.add(_arc(x1 - r, y0 + r, 0.0, -math.pi / 2), ['corner'], {'corner': 'se', 'c': (x1 - r, y0 + r)})
    ring.add([(x, y0, math.pi) for x in reversed(along)], ['straight', 'straight', 't', 'straight', 'straight'],
             {'seg': 's', 'cw': 'wb', 'ccw': 'eb', 'x_cw': 't_e2w', 'x_ccw': 't_w2e'})
    ring.add(_arc(x0 + r, y0 + r, -math.pi / 2, -math.pi), ['corner'], {'corner': 'sw', 'c': (x0 + r, y0 + r)})
    ring.add(_straight(x0, y0 + r, x0, y1 - r, (0.0, 1.0)), ['straight'], seg('w', 'nb', 'sb'))
    ring.add(_arc(x0 + r, y1 - r, math.pi, math.pi / 2), ['corner'], {'corner': 'nw', 'c': (x0 + r, y1 - r)})
    ring.close()
    return ring


def _gate_road():
    """门路 + 南支路，从仓库里往南定义：fwd = 往南（西半幅，x = 1.75），bwd = 往北（东半幅）。"""
    y1, y0 = RING_Y[1], RING_Y[0]
    road = _Path('gate', TWO_LINES, TWO_LANES)
    h = -math.pi / 2
    road.add([(ROAD_X, HANDOFF_Y, h), (ROAD_X, GATE_Y, h)], ['handoff'], {'fwd': 'gate_out', 'bwd': 'gate_in'})
    road.add([(ROAD_X, GATE_Y, h), (ROAD_X, y1 + BOX_HY, h)], ['plain'], {'fwd': 'gate_sb', 'bwd': 'gate_nb'})
    road.add([(ROAD_X, y1 + BOX_HY, h), (ROAD_X, y1 - BOX_HY, h)], ['x'], {'fwd': 'x_n2s', 'bwd': 'x_s2n'})
    road.add([(ROAD_X, y1 - BOX_HY, h), (ROAD_X, y0 + BOX_HY, h)], ['plain'], {'fwd': 'branch_sb', 'bwd': 'branch_nb'})
    return road


def _section_length(road, idx):
    return sum(math.dist(road.stations[a][:2], road.stations[b][:2]) for a, b in zip(idx, idx[1:]))


# ---------- 2. 路段的边界线 ----------
# 每个路段按横断面偏移出 5 条（环路）或 3 条（门路）边界线。直路上同向两条车道之间的线：
# 路口前 SOLID_BEFORE 是实线（快到路口了，不许再变道），其余是虚线；弯道、短直路（< 10 m）全是实线；
# 路口里的线是 virtual（路面上没有画，只是 lanelet 的边）。线型变化的地方 lanelet 必须切开，
# 因为路由图按"整条 lanelet 的边界线"判断能不能变道。
def _ring_line_types(road, k):
    sections = road.sections
    kind = sections[k][0]
    if kind in ('x', 't'):
        return {d: ('virtual', None) for d in RING_LINES}
    types = {d: ('line_thin', 'solid') for d in RING_LINES}
    if kind == 'straight' and _section_length(road, sections[k][1]) >= 10.0:
        nxt, prev = sections[(k + 1) % len(sections)][0], sections[k - 1][0]
        types[-LANE_W] = ('line_thin', 'solid' if nxt in ('x', 't') else 'dashed')     # cw：下游是路口就实线
        types[LANE_W] = ('line_thin', 'solid' if prev in ('x', 't') else 'dashed')      # ccw 反向走，看上游
    return types


def _lane_end(b, road, i, lane):
    """road 第 i 站上 lane 车道的 (左边界点, 右边界点, 中心点, 行驶朝向)。"""
    sense, lo, hi = road.lanes[lane]
    x, y, h = road.stations[i]
    c = b.station_pt(road, i, (lo + hi) / 2)
    if sense == 'fwd':
        return b.station_pt(road, i, hi), b.station_pt(road, i, lo), c, h
    return b.station_pt(road, i, lo), b.station_pt(road, i, hi), c, h + math.pi


def _build_path(b, road, line_types, names):
    """按路段生成边界线和 lanelet。names(k, lane) 返回 (名字, 是否限速慢行)。"""
    for k, (kind, idx, info) in enumerate(road.sections):
        types = line_types(road, k)
        lines = {d: b.line([b.station_pt(road, i, d) for i in idx], *types[d]) for d in road.lines}
        for lane, (sense, lo, hi) in road.lanes.items():
            name, slow = names(k, lane)
            centre = [b.station_pt(road, i, (lo + hi) / 2) for i in idx]
            if sense == 'fwd':
                b.lanelet(name, lines[hi], lines[lo], centre, slow)
            else:
                b.lanelet(name, lines[lo].invert(), lines[hi].invert(), centre[::-1], slow)


def _ring_names(ring):
    straight = {}
    for k, (kind, _, info) in enumerate(ring.sections):
        if kind == 'straight':
            straight.setdefault(info['seg'], []).append(k)

    def names(k, lane):
        kind, _, info = ring.sections[k]
        sense, side = lane.split('_')
        rot = 'cw' if sense == 'fwd' else 'ccw'
        if kind == 'corner':
            return f"ring_{info['corner']}_corner_{rot}_{side}", True
        if kind in ('x', 't'):
            return f"{info['x_' + rot]}_{side}", True
        ks = straight[info['seg']]
        n = ks.index(k) if sense == 'fwd' else len(ks) - 1 - ks.index(k)
        return f"ring_{info['seg']}_{info[rot]}_{side}_{n}", False
    return names


def _gate_line_types(road, k):
    kind = road.sections[k][0]
    t = ('virtual', None) if kind in ('handoff', 'x') else ('line_thin', 'solid')
    return {d: t for d in TWO_LINES}


# ---------- 4. 路口的转弯车道 ----------
# 转弯车道也是 lanelet：起点共用进口车道终点的两个边界点，终点共用出口车道起点的两个边界点，路由图就把
# 进口 → 转弯 → 出口连起来了。中间用三次 Hermite 曲线（两端切线 = 车道方向）连，边界是 virtual 线。
def _hermite(p0, h0, p1, h1, n):
    k = 1.172 * math.dist(p0, p1)        # 两端垂直、位移相等时接近四分之一圆
    d0, d1 = (math.cos(h0), math.sin(h0)), (math.cos(h1), math.sin(h1))
    out = []
    for j in range(1, n):
        t = j / n
        a, bb, c, d = 2 * t ** 3 - 3 * t ** 2 + 1, t ** 3 - 2 * t ** 2 + t, -2 * t ** 3 + 3 * t ** 2, t ** 3 - t ** 2
        out.append(tuple(a * p0[i] + bb * k * d0[i] + c * p1[i] + d * k * d1[i] for i in (0, 1)))
    return out


def _turn(b, name, a, z):
    """从进口车道端 a 到出口车道端 z 的转弯车道（a、z 是 _lane_end 的返回值）。"""
    w = lambda p: (p.x + b.ox, p.y + b.oy)      # 回到世界坐标再插值，点再由 b.pt 减偏移
    n = max(6, math.ceil(math.dist(w(a[2]), w(z[2])) / TURN_STEP))
    curve = lambda p, q: [p] + [b.pt(*xy) for xy in _hermite(w(p), a[3], w(q), z[3], n)] + [q]
    left = b.line(curve(a[0], z[0]), 'virtual')
    right = b.line(curve(a[1], z[1]), 'virtual')
    return b.lanelet(name, left, right, curve(a[2], z[2]), True)


# ---------- 5. 交通规则 ----------
# 规则是独立的 regulatory element，再"挂"到 lanelet 上：这里是红绿灯（traffic_light），它引用灯（一条
# traffic_light 线，标出灯的位置和高度）和停止线。挂在进口车道上，车沿这条车道开就知道前面有灯、停在哪条线。
# signal_group 是自定义属性：ew 一组、ns 一组，交替放行（traffic_light_sim 按它切换）。
def _traffic_light(b, group, approach, stop, heading, ext_f, ext_r):
    fx, fy = math.cos(heading), math.sin(heading)
    rx, ry = math.sin(heading), -math.cos(heading)
    cx, cy = ROAD_X, RING_Y[1]                      # 十字路口中心
    px = cx + fx * (ext_f + POLE_GAP) + rx * (ext_r + POLE_GAP)     # 路口对面、右侧角外
    py = cy + fy * (ext_f + POLE_GAP) + ry * (ext_r + POLE_GAP)
    hw = LIGHT_W / 2
    lamp = b.line([b.pt(px - rx * hw, py - ry * hw, LIGHT_Z), b.pt(px + rx * hw, py + ry * hw, LIGHT_Z)],
                  'traffic_light', 'red_yellow_green', height='0.5')
    stop_line = b.line(stop, 'stop_line')
    tl = TrafficLight(b._id(), AttributeMap({'type': 'regulatory_element', 'subtype': 'traffic_light',
                                             'signal_group': group}), [lamp], stop_line)
    for name in approach:
        b.lanelets[name].addRegulatoryElement(tl)
    b.lmap.add(tl)
    return group, px - b.ox, py - b.oy, heading + math.pi


def build(offset=(0.0, 0.0)) -> Road:
    b = _Builder(offset)
    ring, gate = _ring(), _gate_road()
    _build_path(b, ring, _ring_line_types, _ring_names(ring))
    _build_path(b, gate, _gate_line_types,
                lambda k, lane: (gate.sections[k][2][lane], gate.sections[k][0] == 'x'))

    (y0, y1), xw, xe = RING_Y, ROAD_X - BOX_HX, ROAD_X + BOX_HX
    rw, re_ = ring.index(xw, y1), ring.index(xe, y1)          # 十字路口西、东边缘
    tw, te = ring.index(xw, y0), ring.index(xe, y0)           # T 形路口西、东边缘
    g_top, g_bot, t_top = gate.index(ROAD_X, y1 + BOX_HY), gate.index(ROAD_X, y1 - BOX_HY), gate.index(ROAD_X, y0 + BOX_HY)
    R = lambda i, lane: _lane_end(b, ring, i, lane)
    G = lambda i, lane: _lane_end(b, gate, i, lane)
    turns = [
        # 十字路口：左转只从内侧车道，右转只从外侧车道；进 2 车道的路时左转进内侧、右转进外侧
        ('x_w2n_left', R(rw, 'fwd_in'), G(g_top, 'bwd')), ('x_w2s_right', R(rw, 'fwd_out'), G(g_bot, 'fwd')),
        ('x_e2s_left', R(re_, 'bwd_in'), G(g_bot, 'fwd')), ('x_e2n_right', R(re_, 'bwd_out'), G(g_top, 'bwd')),
        ('x_n2e_left', G(g_top, 'fwd'), R(re_, 'fwd_in')), ('x_n2w_right', G(g_top, 'fwd'), R(rw, 'bwd_out')),
        ('x_s2w_left', G(g_bot, 'bwd'), R(rw, 'bwd_in')), ('x_s2e_right', G(g_bot, 'bwd'), R(re_, 'fwd_out')),
        # T 形路口（南段 cw = 往西）
        ('t_e2n_right', R(te, 'fwd_out'), G(t_top, 'bwd')), ('t_w2n_left', R(tw, 'bwd_in'), G(t_top, 'bwd')),
        ('t_n2e_left', G(t_top, 'fwd'), R(te, 'bwd_in')), ('t_n2w_right', G(t_top, 'fwd'), R(tw, 'fwd_out')),
    ]
    for name, a, z in turns:
        _turn(b, name, a, z)

    P, Q = (lambda i, d: b.station_pt(ring, i, d)), (lambda i, d: b.station_pt(gate, i, d))
    lights = [
        _traffic_light(b, 'ew', ['ring_n_eb_in_1', 'ring_n_eb_out_1'], [P(rw, -2 * LANE_W), P(rw, 0.0)],
                       0.0, BOX_HX, BOX_HY),
        _traffic_light(b, 'ew', ['ring_n_wb_in_1', 'ring_n_wb_out_1'], [P(re_, 2 * LANE_W), P(re_, 0.0)],
                       math.pi, BOX_HX, BOX_HY),
        _traffic_light(b, 'ns', ['gate_sb'], [Q(g_top, -LANE_W), Q(g_top, 0.0)], -math.pi / 2, BOX_HY, BOX_HX),
        _traffic_light(b, 'ns', ['branch_nb'], [Q(g_bot, LANE_W), Q(g_bot, 0.0)], math.pi / 2, BOX_HY, BOX_HX),
    ]

    # ---------- 6. 画图用的数据 ----------
    # Gazebo 里的标线直接取地图里的线对象，路面矩形按同一份站点算，保证看到的和地图一致。
    markings = []
    for ls in sorted(b.lmap.lineStringLayer, key=lambda ls: ls.id):
        t = ls.attributes['type'] if 'type' in ls.attributes else None
        if t == 'line_thin':
            markings.append((ls.attributes['subtype'], [(p.x, p.y) for p in ls]))
        elif t == 'stop_line':
            markings.append(('stop_line', [(p.x, p.y) for p in ls]))
    return Road(b.lmap, markings, _surface(ring, gate, b.ox, b.oy), lights)


def _surface(ring, gate, ox, oy):
    rects = []

    def rect(x0, y0, x1, y1, width):
        rects.append(((x0 + x1) / 2 - ox, (y0 + y1) / 2 - oy, math.atan2(y1 - y0, x1 - x0),
                      math.dist((x0, y0), (x1, y1)), width))
    for kind, idx, info in ring.sections:
        s = [ring.stations[i] for i in idx]
        if kind == 'straight':
            rect(*s[0][:2], *s[-1][:2], 4 * LANE_W)
        elif kind == 'corner':
            # 每 1 m 弧长一块；矩形长按外边缘的弦长算，外侧不留缝
            n = math.ceil(RING_R * math.pi / 2)
            cx, cy = info['c']
            a0 = math.atan2(s[0][1] - cy, s[0][0] - cx)
            a1 = math.atan2(s[-1][1] - cy, s[-1][0] - cx)
            da = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
            ln = 2 * (RING_R + 2 * LANE_W) * math.tan(abs(da) / n / 2)
            for j in range(n):
                a = a0 + da * (j + 0.5) / n
                rects.append((cx + RING_R * math.cos(a) - ox, cy + RING_R * math.sin(a) - oy,
                              a - math.pi / 2, ln, 4 * LANE_W))
    for kind, idx, _ in gate.sections:
        if kind == 'plain':
            rect(*gate.stations[idx[0]][:2], *gate.stations[idx[-1]][:2], 2 * LANE_W)
    (y0, y1) = RING_Y
    rect(ROAD_X - BOX_HX, y1, ROAD_X + BOX_HX, y1, 2 * BOX_HY)                         # 十字路口
    yb, yt = y0 - 2 * LANE_W, y0 + BOX_HY                                               # T 形路口：南边到路边
    rect(ROAD_X - BOX_HX, (yb + yt) / 2, ROAD_X + BOX_HX, (yb + yt) / 2, yt - yb)
    return rects


def write_osm(road: Road, path: str) -> None:
    """写 Autoware 格式的 .osm。UTM 原点只是占位：Autoware 的 Local 投影只读 local_x / local_y。"""
    write(path, road.lmap, UtmProjector(Origin(35.0, 139.0)))

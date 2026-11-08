"""进电梯的纯计算部分（不依赖 ROS）：从点云提取轿厢位姿、滤波，以及沿轿厢中心线进出的控制。

坐标约定：所有位姿都是同一个平面坐标系里的 2D 量（节点里用 odom 系）。
轿厢 yaw 是轴线指向轿厢里面的方向，也就是车头朝里停好时车的朝向；"左"是沿轴线往里看的左手边。
"""
import collections
import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Cabin:            # 轿厢先验或提取结果
    x: float            # 轿厢中心
    y: float
    yaw: float          # 轴线指向轿厢里面的方向 = 车头朝里停好时车的朝向


@dataclass(frozen=True)
class Line:
    point: np.ndarray       # (2,) 内点质心
    direction: np.ndarray   # (2,) 单位向量
    inliers: int
    length: float           # 内点沿 direction 的跨度（最长的连续段）


@dataclass(frozen=True)
class FitParams:
    width: float = 2.0                  # 轿厢内净宽（垂直于轴线）
    depth: float = 2.6                  # 轿厢内净深（沿轴线）
    roi_margin: float = 0.5             # 先验矩形四周外扩
    z_min: float = 0.2                  # 离地高度范围：去掉地面残留和顶板
    z_max: float = 2.0
    inlier_tol: float = 0.03
    min_inliers: int = 30
    min_length: float = 0.6             # 短于这个的线不参与分类（门洞侧面只有 0.3 m）
    max_lines: int = 4
    iterations: int = 200
    parallel_deg: float = 10.0
    perpendicular_deg: float = 80.0
    width_tol: float = 0.1
    sides_parallel_deg: float = 3.0


LINE_GAP = 0.2          # 内点沿线方向相邻间隔超过这个就算断开：只用最长的连续段算质心和长度，散点不会把线"拉长"


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _frame(c):
    """轿厢系的轴线方向 u 和左手法向 v"""
    u = np.array([math.cos(c.yaw), math.sin(c.yaw)])
    return u, np.array([-u[1], u[0]])


def roi_points(points, prior, ground_z, p=FitParams()):
    """points (N,3) -> (M,2)：落在先验矩形外扩 roi_margin 内、离地 (z - ground_z) ∈ [z_min, z_max] 的点的 xy"""
    pts = np.asarray(points, dtype=np.float64)
    u, v = _frame(prior)
    d = pts[:, :2] - (prior.x, prior.y)
    a, b, h = d @ u, d @ v, pts[:, 2] - ground_z
    keep = ((np.abs(a) <= p.depth / 2 + p.roi_margin) & (np.abs(b) <= p.width / 2 + p.roi_margin)
            & (h >= p.z_min) & (h <= p.z_max))
    return pts[keep, :2]


def _line_from(xy):
    """PCA 拟合：返回 (质心, 单位方向)"""
    c = xy.mean(axis=0)
    w, vecs = np.linalg.eigh(np.cov((xy - c).T))
    return c, vecs[:, np.argmax(w)]


def _distance(xy, point, direction):
    d = xy - point
    return np.abs(d[:, 0] * direction[1] - d[:, 1] * direction[0])


def fit_lines(xy, p=FitParams(), rng=None):
    """依次做 RANSAC：每次取内点最多的线，PCA 重新拟合后去掉它的内点，再拟合下一条，最多 max_lines 条"""
    rng = np.random.default_rng(0) if rng is None else rng
    rest = np.asarray(xy, dtype=np.float64)
    lines = []
    while len(lines) < p.max_lines and len(rest) >= p.min_inliers:
        best = None
        for _ in range(p.iterations):
            i, j = rng.choice(len(rest), 2, replace=False)
            d = rest[j] - rest[i]
            n = np.hypot(*d)
            if n < 1e-6:
                continue
            mask = _distance(rest, rest[i], d / n) < p.inlier_tol
            if best is None or mask.sum() > best.sum():
                best = mask
        if best is None or best.sum() < p.min_inliers:
            break
        c, d = _line_from(rest[best])
        mask = _distance(rest, c, d) < p.inlier_tol
        if mask.sum() < p.min_inliers:
            break
        # 最长的连续段
        pts = rest[mask]
        s = (pts - c) @ d
        order = np.argsort(s)
        breaks = np.flatnonzero(np.diff(s[order]) > LINE_GAP)
        bounds = np.r_[0, breaks + 1, len(s)]
        k = np.argmax(np.diff(bounds))
        seg = pts[order[bounds[k]:bounds[k + 1]]]
        if len(seg) >= p.min_inliers:
            c, d = _line_from(seg)
            ss = (seg - c) @ d
            lines.append(Line(c, d, len(seg), float(ss.max() - ss.min())))
        rest = rest[~mask]
    return lines


def cabin_from_lines(lines, prior, last, p=FitParams()):
    """按与先验轴线的夹角把线分成左右侧墙和后墙，算轿厢位姿；看不出来时返回 None"""
    u, v = _frame(prior)
    cos_par, cos_perp = math.cos(math.radians(p.parallel_deg)), math.cos(math.radians(p.perpendicular_deg))
    left = right = back = None
    for ln in lines:
        if ln.length < p.min_length:
            continue
        c = abs(float(ln.direction @ u))
        rel = ln.point - (prior.x, prior.y)
        if c > cos_par:
            if rel @ v > 0:
                if left is None or ln.inliers > left.inliers:
                    left = ln
            elif right is None or ln.inliers > right.inliers:
                right = ln
        elif c < cos_perp and abs(rel @ u - p.depth / 2) < p.depth / 4:
            # 后墙要在先验后墙附近：门洞两侧的外墙面也是垂直线，后墙没看到时不能拿它当后墙
            if back is None or rel @ u > (back.point - (prior.x, prior.y)) @ u:
                back = ln

    def along(ln):      # 方向统一成和先验轴线同向
        return ln.direction if ln.direction @ u >= 0 else -ln.direction

    if left is not None and right is not None:
        dl, dr = along(left), along(right)
        if math.degrees(math.acos(min(1.0, float(dl @ dr)))) >= p.sides_parallel_deg:
            return None
        spacing = (_distance(right.point[None], left.point, dl)[0] + _distance(left.point[None], right.point, dr)[0]) / 2
        if abs(spacing - p.width) > p.width_tol:
            return None
        axis = dl + dr
        axis /= np.hypot(*axis)
        n = np.array([-axis[1], axis[0]])
        mid = (left.point @ n + right.point @ n) / 2
    elif left is not None or right is not None:
        side = left if left is not None else right
        axis = along(side)
        n = np.array([-axis[1], axis[0]])
        mid = side.point @ n + (-p.width / 2 if side is left else p.width / 2)
    else:
        return None
    if back is not None:
        s = back.point @ axis - p.depth / 2
    elif last is not None:
        s = np.array([last.x, last.y]) @ axis
    else:
        return None
    center = s * axis + mid * n
    return Cabin(float(center[0]), float(center[1]), math.atan2(axis[1], axis[0]))


class CabinFilter:
    """最近 window 帧的滑动平均；和均值相差太多的帧丢掉，连续丢 reset_after 帧说明均值本身错了，清空重来"""

    def __init__(self, window=5, max_dev=0.10, max_dev_yaw=math.radians(3.0), reset_after=5):
        self.frames = collections.deque(maxlen=window)
        self.max_dev, self.max_dev_yaw, self.reset_after = max_dev, max_dev_yaw, reset_after
        self.rejected = 0

    @property
    def count(self):
        return len(self.frames)

    def mean(self):
        if not self.frames:
            return None
        xs = np.array([(c.x, c.y, math.cos(c.yaw), math.sin(c.yaw)) for c in self.frames]).mean(axis=0)
        return Cabin(float(xs[0]), float(xs[1]), math.atan2(xs[3], xs[2]))

    def update(self, c):
        if c is not None:
            m = self.mean()
            if m is not None and (math.hypot(c.x - m.x, c.y - m.y) > self.max_dev
                                  or abs(_wrap(c.yaw - m.yaw)) > self.max_dev_yaw):
                self.rejected += 1
                if self.rejected >= self.reset_after:
                    self.frames.clear()
                    self.frames.append(c)
                    self.rejected = 0
            else:
                self.frames.append(c)
                self.rejected = 0
        return self.mean()


# ---------------------------------------------------------------- 控制
def to_cabin_frame(x, y, yaw, cabin):
    """base_link 位姿 -> (depth 沿轴线、相对轿厢中心，往里为正; lateral 左为正; yaw_err 归到 (-π, π])"""
    u, v = _frame(cabin)
    d = np.array([x - cabin.x, y - cabin.y])
    return float(d @ u), float(d @ v), _wrap(yaw - cabin.yaw)


def pursuit_curvature(x, y, yaw, cabin, direction, lookahead=1.0, rear_offset=0.432, max_curvature=1 / 2.3):
    """以后轴为参考点、沿轿厢中心线的纯跟踪。direction +1 进 / -1 出。
    指令 = Twist(linear.x = direction * v, angular.z = k * v)：倒车时把车头方向转 180° 再算（同 harness 的 route.py）"""
    rx, ry = x - rear_offset * math.cos(yaw), y - rear_offset * math.sin(yaw)
    u, _ = _frame(cabin)
    s = (np.array([rx - cabin.x, ry - cabin.y]) @ u) + direction * lookahead
    gx, gy = cabin.x + s * u[0], cabin.y + s * u[1]
    h = yaw if direction > 0 else yaw + math.pi
    dx, dy = gx - rx, gy - ry
    lx = math.cos(h) * dx + math.sin(h) * dy
    ly = -math.sin(h) * dx + math.cos(h) * dy
    k = 2.0 * ly / max(lx * lx + ly * ly, 1e-6)
    return max(-max_curvature, min(max_curvature, k))


def approach_speed(remaining, v_max, slow_dist=0.3, v_min=0.03):
    """remaining <= 0 -> 0；最后 slow_dist 线性减速，但不低于 v_min（太慢底盘动不了，车会停在终点前）"""
    if remaining <= 0.0:
        return 0.0
    if remaining >= slow_dist:
        return v_max
    return max(v_min, v_max * remaining / slow_dist)


def cabin_ahead(x, y, yaw, dist):
    """示教用的粗先验：车头大致对着门停在门外时，轿厢中心在车正前方 dist 处"""
    return Cabin(x + dist * math.cos(yaw), y + dist * math.sin(yaw), yaw)


def search_cabin(points, ground_z, x, y, yaw, dist, p=FitParams(), rng=None, yaw_range_deg=25.0, step_deg=5.0):
    """示教时还没有提取结果：车头航向可能偏得多，把车正前方 dist 的先验绕车转 ±yaw_range_deg 逐个试。
    某个先验提取出轿厢后，再拿结果当先验提取一次，两次一致（5 cm、1°）才算数：
    框偏了时一面墙、门洞外墙之类凑出来的结果，换成以它为中心框选后对不上。都不行返回 None"""
    n = int(round(yaw_range_deg / step_deg))
    for k in sorted(range(-n, n + 1), key=abs):
        prior = cabin_ahead(x, y, yaw + math.radians(k * step_deg), dist)
        c = cabin_from_lines(fit_lines(roi_points(points, prior, ground_z, p), p, rng), prior, None, p)
        if c is None:
            continue
        c2 = cabin_from_lines(fit_lines(roi_points(points, c, ground_z, p), p, rng), c, None, p)
        if c2 is not None and math.hypot(c2.x - c.x, c2.y - c.y) < 0.05 and abs(_wrap(c2.yaw - c.yaw)) < math.radians(1):
            return c2
    return None


def wait_point(cabin, dist):
    """轿厢中心线上、门外离轿厢中心 dist 的点"""
    return cabin.x - dist * math.cos(cabin.yaw), cabin.y - dist * math.sin(cabin.yaw)


def inside_cabin(x, y, cabin, p=FitParams()):
    """base_link 在轿厢内净矩形里"""
    depth, lat, _ = to_cabin_frame(x, y, cabin.yaw, cabin)
    return abs(depth) <= p.depth / 2 and abs(lat) <= p.width / 2


def footprint_hits(xy_base, length=1.2, width=1.08, pad=0.05):
    """base_link 系下的点 (N,2)，落在外扩 pad 的车体矩形里的个数"""
    xy = np.asarray(xy_base, dtype=np.float64).reshape(-1, 2)
    return int(np.count_nonzero((np.abs(xy[:, 0]) <= length / 2 + pad) & (np.abs(xy[:, 1]) <= width / 2 + pad)))

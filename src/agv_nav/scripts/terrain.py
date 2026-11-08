"""2.5D 地形计算（纯 numpy / scipy，不依赖 ROS，便于单元测试）

网格约定：二维数组下标 [row, col]，row 对应 y、col 对应 x；origin 是 [0, 0] 格左下角的 (x, y)。
代价：-1 未知，0..100，100 = 不可通行。
"""
from dataclasses import dataclass

import numpy as np
from scipy.ndimage import (binary_dilation, distance_transform_edt, label, maximum, maximum_filter, minimum_filter,
                           uniform_filter)

UNKNOWN = -1
LETHAL = 100


@dataclass
class TerrainParams:
    resolution: float = 0.05
    min_points: int = 3                  # 格子里至少这么多点才有地面高度
    ground_tolerance: float = 0.05       # 离地面这么近的点算地面点
    obstacle_min_height: float = 0.08    # 高出地面 0.08~1.2 m 的点算障碍点（底盘离地约 0.19 m）
    obstacle_max_height: float = 1.2
    min_occupied_hits: int = 3
    min_obstacle_ground_ratio: float = 0.3
    min_peak_ratio: float = 0.2
    min_obstacle_cells: int = 2          # 障碍连通块至少这么多格（去掉孤立单格）
    slope_window: int = 5                # 求坡度前平滑地面高度的窗口（格）
    step_radius: float = 0.15            # 台阶：与这个半径内各格的最大高差
    max_step: float = 0.05               # 车轮半径 0.187 m 的约 27%（放大前定的，偏保守）
    max_slope_deg: float = 15.0
    flat_slope_deg: float = 2.0
    max_slope_cost: int = 60


@dataclass
class TerrainLayers:
    elevation: np.ndarray    # float32，NaN = 未知
    obstacle: np.ndarray     # bool
    slope_deg: np.ndarray    # float32，NaN = 未知
    step: np.ndarray         # float32，NaN = 未知
    cost: np.ndarray         # int8：-1 未知，0..100


def cell_index(xy, origin, shape, resolution):
    col = np.floor((xy[:, 0] - origin[0]) / resolution).astype(np.int64)
    row = np.floor((xy[:, 1] - origin[1]) / resolution).astype(np.int64)
    inside = (row >= 0) & (row < shape[0]) & (col >= 0) & (col < shape[1])
    return row, col, inside


def _nan_mean(a, size):
    """NaN 感知的均值滤波，返回 (均值, 窗口内有效格数)"""
    valid = np.isfinite(a)
    area = size * size
    total = uniform_filter(np.where(valid, a, 0.0), size=size, mode='constant') * area
    count = uniform_filter(valid.astype(np.float64), size=size, mode='constant') * area
    with np.errstate(invalid='ignore', divide='ignore'):
        return np.where(count > 0.5, total / count, np.nan), count


def _remove_unreachable(cost, origin, resolution, start_xy):
    """起点所在连通域（未知格也算连通）以外的已知格改为不可通行"""
    labels, _ = label(cost != LETHAL, structure=np.ones((3, 3)))
    r0 = int((start_xy[1] - origin[1]) / resolution)
    c0 = int((start_xy[0] - origin[0]) / resolution)
    rad = int(round(1.0 / resolution))
    window = labels[max(r0 - rad, 0):r0 + rad + 1, max(c0 - rad, 0):c0 + rad + 1]
    ids = window[window > 0]
    if ids.size == 0:
        return cost
    keep = np.bincount(ids).argmax()      # 起点 1 m 内占比最多的连通域
    out = cost.copy()
    out[(labels != keep) & (cost >= 0)] = LETHAL
    return out


def compute_layers(points, origin, shape, params=None, start_xy=None):
    p = params or TerrainParams()
    n = shape[0] * shape[1]
    row, col, inside = cell_index(points, origin, shape, p.resolution)
    flat = (row * shape[1] + col)[inside]
    z = np.asarray(points, dtype=np.float64)[inside, 2]

    # 1. 地面高度：格内最低点；3x3 邻域里至少 3 格有值时填补空洞
    counts = np.bincount(flat, minlength=n)
    zmin = np.full(n, np.inf)
    np.minimum.at(zmin, flat, z)
    elevation = np.where(counts >= p.min_points, zmin, np.nan).reshape(shape)
    mean3, count3 = _nan_mean(elevation, 3)
    elevation = np.where(np.isnan(elevation) & (count3 >= 3), mean3, elevation)

    # 2. 障碍：高出本格地面的点，沿用建图节点原有的判定规则
    h = z - elevation.ravel()[flat]
    known = np.isfinite(h)
    ground_hits = np.bincount(flat[known & (np.abs(h) < p.ground_tolerance)], minlength=n).reshape(shape)
    obstacle_hits = np.bincount(
        flat[known & (h > p.obstacle_min_height) & (h < p.obstacle_max_height)], minlength=n).reshape(shape)
    obstacle = ((obstacle_hits >= p.min_occupied_hits)
                & (obstacle_hits >= p.min_obstacle_ground_ratio * ground_hits)
                & (obstacle_hits >= p.min_peak_ratio * maximum_filter(obstacle_hits, size=5)))
    labels, _ = label(obstacle, structure=np.ones((3, 3)))
    obstacle &= np.bincount(labels.ravel())[labels] >= p.min_obstacle_cells

    # 3. 坡度：去掉障碍格后平滑，再求梯度
    smooth, _ = _nan_mean(np.where(obstacle, np.nan, elevation), p.slope_window)
    with np.errstate(invalid='ignore'):
        gy, gx = np.gradient(smooth, p.resolution)
    slope = np.degrees(np.arctan(np.hypot(gx, gy))).astype(np.float32)

    # 4. 台阶：与半径内各格的最大高差
    r = int(round(p.step_radius / p.resolution))
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    disk = xx ** 2 + yy ** 2 <= r * r
    finite = np.isfinite(elevation)
    hi = maximum_filter(np.where(finite, elevation, -np.inf), footprint=disk, mode='constant', cval=-np.inf)
    lo = minimum_filter(np.where(finite, elevation, np.inf), footprint=disk, mode='constant', cval=np.inf)
    step = np.where(finite, np.maximum(hi - elevation, elevation - lo), np.nan).astype(np.float32)

    # 5. 代价
    s = np.nan_to_num(slope, nan=0.0)
    cost = np.where(s < p.flat_slope_deg, 0.0,
                    np.round(p.max_slope_cost * np.minimum(s, p.max_slope_deg) / p.max_slope_deg))
    lethal = obstacle | (np.nan_to_num(step, nan=0.0) > p.max_step) | (s > p.max_slope_deg)
    out = np.full(shape, UNKNOWN, np.int8)
    out[finite] = cost[finite].astype(np.int8)
    out[finite & lethal] = LETHAL
    out[obstacle] = LETHAL
    if start_xy is not None:
        out = _remove_unreachable(out, origin, p.resolution, start_xy)
    return TerrainLayers(elevation.astype(np.float32), obstacle, slope, step, out)


def to_grid_map_data(layer):
    """grid_map 的数据顺序：矩阵行沿 x 递减、列沿 y 递减，列主序存储。
    对 [row=y, col=x] 的数组，这等价于上下左右都翻转后按行展开。"""
    return np.ascontiguousarray(np.asarray(layer, dtype=np.float32)[::-1, ::-1]).ravel()


# 单帧分类：只在一帧点云内部判断高度关系，不受 SLAM 高度漂移影响（spec 修订 1）
# 单帧格子 0.1 m：一根细杆（人腿、立柱）只占一格，不能按“孤立单格”去掉；单格噪声由全局投票过滤
FRAME_PARAMS = TerrainParams(resolution=0.1, min_points=2, min_obstacle_cells=1)
FRAME_RANGE = 5.0
SELF_RANGE = 0.6        # 离雷达这么近的点打在自己的车身 / 车轮上，不用（雷达 2026-11-01 架高 0.2 m 后实测最近的点在 0.8 m，车身轮廓内没有点）


def frame_layers(local_points, frame_range=FRAME_RANGE, params=FRAME_PARAMS):
    """local_points：重力对齐、朝向同 map、原点在车上的一帧点。只用水平 frame_range 内、离原点 SELF_RANGE 以上的点。
    返回 (TerrainLayers, 网格 origin, 用到的点的掩码)"""
    pts = np.asarray(local_points, dtype=np.float64)
    keep = (np.hypot(pts[:, 0], pts[:, 1]) <= frame_range) & (np.linalg.norm(pts, axis=1) > SELF_RANGE)
    n = int(np.ceil(2 * frame_range / params.resolution))
    origin = (-frame_range, -frame_range)
    return compute_layers(pts[keep], origin, (n, n), params), origin, keep


def obstacle_point_mask(local_points, frame_range=FRAME_RANGE, params=FRAME_PARAMS):
    """一帧里哪些点是障碍：落在单帧障碍格、且高出本格地面 obstacle_min_height~obstacle_max_height"""
    pts = np.asarray(local_points, dtype=np.float64)
    layers, origin, keep = frame_layers(pts, frame_range, params)
    row, col, inside = cell_index(pts, origin, layers.cost.shape, params.resolution)
    idx = np.flatnonzero(keep & inside)
    r, c = row[idx], col[idx]
    with np.errstate(invalid='ignore'):
        h = pts[idx, 2] - layers.elevation[r, c]
        ok = layers.obstacle[r, c] & (h > params.obstacle_min_height) & (h < params.obstacle_max_height)
    out = np.zeros(len(pts), bool)
    out[idx[ok]] = True
    return out


class VoteGrid:
    """全局网格上累积各帧的单帧判断：只用每帧位姿的水平位置，不跨帧累积高度值（spec 修订 1）"""

    def __init__(self, origin, shape, resolution=0.05):
        self.origin, self.shape, self.resolution = origin, shape, resolution
        self.known = np.zeros(shape, np.int32)       # 多少帧对这个格子给出了判断
        self.lethal = np.zeros(shape, np.int32)      # 其中多少帧判为不可通行
        self.slope_sum = np.zeros(shape)             # 判为可通行的帧的坡度和
        self.slope_n = np.zeros(shape, np.int32)
        self.elev_sum = np.zeros(shape)              # 仅用于显示
        self.elev_n = np.zeros(shape, np.int32)

    def add_frame(self, layers, frame_origin, frame_resolution, center_xyz):
        fh, fw = layers.cost.shape
        x0, y0 = center_xyz[0] + frame_origin[0], center_xyz[1] + frame_origin[1]
        res = self.resolution
        c0 = max(int(np.floor((x0 - self.origin[0]) / res)), 0)
        r0 = max(int(np.floor((y0 - self.origin[1]) / res)), 0)
        c1 = min(int(np.ceil((x0 + fw * frame_resolution - self.origin[0]) / res)), self.shape[1])
        r1 = min(int(np.ceil((y0 + fh * frame_resolution - self.origin[1]) / res)), self.shape[0])
        if r1 <= r0 or c1 <= c0:
            return
        # 全局格子中心落在单帧的哪一格
        fc = np.floor((self.origin[0] + (np.arange(c0, c1) + 0.5) * res - x0) / frame_resolution).astype(np.int64)
        fr = np.floor((self.origin[1] + (np.arange(r0, r1) + 0.5) * res - y0) / frame_resolution).astype(np.int64)
        vc, vr = (fc >= 0) & (fc < fw), (fr >= 0) & (fr < fh)
        sub = np.ix_(np.flatnonzero(vr), np.flatnonzero(vc))
        src = np.ix_(fr[vr], fc[vc])
        cost = np.full((r1 - r0, c1 - c0), UNKNOWN, np.int16)
        slope = np.full(cost.shape, np.nan)
        elev = np.full(cost.shape, np.nan)
        cost[sub] = layers.cost[src]
        slope[sub] = layers.slope_deg[src]
        elev[sub] = layers.elevation[src] + center_xyz[2]
        win = (slice(r0, r1), slice(c0, c1))
        known = cost >= 0
        free = known & (cost != LETHAL)
        self.known[win] += known
        self.lethal[win] += cost == LETHAL
        self.slope_sum[win] += np.where(free, np.nan_to_num(slope), 0.0)
        self.slope_n[win] += free
        finite = np.isfinite(elev)
        self.elev_sum[win] += np.where(finite, elev, 0.0)
        self.elev_n[win] += finite

    def finalize(self, params=None, min_lethal_votes=2, lethal_ratio=0.3, seal_min_area=0.5, seal_width=0.1,
                 seal_hole_window=0.35, seal_min_half_width=0.3, start_xy=None):
        p = params or TerrainParams()
        with np.errstate(invalid='ignore', divide='ignore'):
            slope = np.where(self.slope_n > 0, self.slope_sum / self.slope_n, np.nan)
            elev = np.where(self.elev_n > 0, self.elev_sum / self.elev_n, np.nan)
        lethal = (self.lethal >= min_lethal_votes) & (self.lethal >= lethal_ratio * self.known)
        s = np.nan_to_num(slope, nan=0.0)
        c = np.where(s < p.flat_slope_deg, 0.0,
                     np.round(p.max_slope_cost * np.minimum(s, p.max_slope_deg) / p.max_slope_deg))
        cost = np.full(self.shape, UNKNOWN, np.int8)
        known = self.known > 0
        cost[known] = c[known].astype(np.int8)
        cost[known & ((s > p.max_slope_deg) | lethal)] = LETHAL
        # 只被判为不可通行、票数又不够：说不清，算未知
        cost[known & (self.slope_n == 0) & ~lethal] = UNKNOWN
        # 大片未知区（例：没从下方看过的平台开放边缘外侧）四周封成不可通行，不指望 Nav2 挡住未知区
        # （MPPI 不把未知格算碰撞）。已知区里零星的小洞（窗口内多半是已知格）先不算未知，
        # 否则一串小洞沿对角线连到外面，整片已知区都会被当成挨着大片未知区
        unknown = cost == UNKNOWN
        win = max(1, int(round(seal_hole_window / self.resolution)))
        hole = uniform_filter((~unknown).astype(np.float64), size=win, mode='constant') > 0.5
        labels, _ = label(unknown & ~hole, structure=np.ones((3, 3)))
        sizes = np.bincount(labels.ravel())
        sizes[0] = 0
        big = sizes[labels] * self.resolution ** 2 >= seal_min_area
        # 只封“有厚度”的未知区：两条行驶线之间漏看的窄带（最大内切半径 < seal_min_half_width）不是没去过的地方，
        # 封上再膨胀会把车道堵死；平台开放边缘外那种大片未知区很厚，照样封
        if labels.max() > 0:
            depth = distance_transform_edt(labels > 0) * self.resolution
            thick = np.zeros(labels.max() + 1, bool)
            thick[1:] = np.asarray(maximum(depth, labels, np.arange(1, labels.max() + 1))) >= seal_min_half_width
            big &= thick[labels]
        if start_xy is not None:
            # 雷达前倾，出生点周围建图时一直是盲区；碰到起点 0.75 m 内（车身所在处）的未知区不封，否则车一开始就被困住
            r0 = int((start_xy[1] - self.origin[1]) / self.resolution)
            c0 = int((start_xy[0] - self.origin[0]) / self.resolution)
            rad = int(round(0.75 / self.resolution))
            near = labels[max(r0 - rad, 0):r0 + rad + 1, max(c0 - rad, 0):c0 + rad + 1]
            big &= ~np.isin(labels, near[near > 0])
        # 补洞时大片未知区的边上最多有 win//2 格被当成了小洞，外扩时加上
        rim = binary_dilation(big, iterations=win // 2 + max(1, int(round(seal_width / self.resolution))))
        cost[rim & (cost >= 0)] = LETHAL
        if start_xy is not None:
            cost = _remove_unreachable(cost, self.origin, self.resolution, start_xy)
        return TerrainLayers(elev.astype(np.float32), lethal, slope.astype(np.float32),
                             np.full(self.shape, np.nan, np.float32), cost)

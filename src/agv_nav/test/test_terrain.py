import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import terrain  # noqa: E402

RES = 0.05
ORIGIN = (0.0, 0.0)
SHAPE = (80, 80)          # 4 m x 4 m


def surface(x0, x1, y0, y1, zfun, step=0.02):
    xs, ys = np.meshgrid(np.arange(x0, x1, step), np.arange(y0, y1, step))
    return np.c_[xs.ravel(), ys.ravel(), zfun(xs, ys).ravel()]


def wall_x(x, y0, y1, z0, z1, step=0.02):
    ys, zs = np.meshgrid(np.arange(y0, y1, step), np.arange(z0, z1, step))
    return np.c_[np.full(ys.size, x), ys.ravel(), zs.ravel()]


def wall_y(y, x0, x1, z0, z1, step=0.02):
    xs, zs = np.meshgrid(np.arange(x0, x1, step), np.arange(z0, z1, step))
    return np.c_[xs.ravel(), np.full(xs.size, y), zs.ravel()]


def flat(x0=0, x1=4, y0=0, y1=4, z=0.0):
    return surface(x0, x1, y0, y1, lambda x, y: np.full_like(x, z))


def cost_at(layers, x, y):
    return int(layers.cost[int(y / RES), int(x / RES)])


def test_flat_floor_is_free():
    layers = terrain.compute_layers(flat(), ORIGIN, SHAPE)
    assert cost_at(layers, 2.0, 2.0) == 0
    assert cost_at(layers, 0.5, 3.5) == 0
    assert abs(layers.elevation[40, 40]) < 0.01


@pytest.mark.parametrize('deg, lo, hi', [(5.0, 18, 22), (10.0, 38, 42)])
def test_ramp_is_traversable_with_slope_cost(deg, lo, hi):
    k = math.tan(math.radians(deg))
    layers = terrain.compute_layers(surface(0, 4, 0, 4, lambda x, y: k * x), ORIGIN, SHAPE)
    assert lo <= cost_at(layers, 2.0, 2.0) <= hi
    assert abs(layers.slope_deg[40, 40] - deg) < 0.5


def test_steep_slope_is_lethal():
    k = math.tan(math.radians(20.0))
    layers = terrain.compute_layers(surface(0, 4, 0, 4, lambda x, y: k * x), ORIGIN, SHAPE)
    assert cost_at(layers, 2.0, 2.0) == terrain.LETHAL


def test_platform_edge_is_lethal_without_gaps():
    pts = np.vstack([flat(0, 2, 0, 4, 0.0), flat(2, 4, 0, 4, 0.5), wall_x(2.0, 0, 4, 0, 0.5)])
    layers = terrain.compute_layers(pts, ORIGIN, SHAPE)
    assert cost_at(layers, 1.0, 2.0) == 0
    assert cost_at(layers, 3.0, 2.0) == 0
    assert cost_at(layers, 1.95, 2.0) == terrain.LETHAL
    assert cost_at(layers, 2.05, 2.0) == terrain.LETHAL
    # 边线上每一行都必须有不可通行格，否则车能从缺口开下去
    band = layers.cost[10:70, int(1.85 / RES):int(2.15 / RES) + 1]
    assert (band == terrain.LETHAL).any(axis=1).all()


def box(x0, x1, y0, y1, h):
    return np.vstack([flat(x0, x1, y0, y1, h), wall_x(x0, y0, y1, 0, h), wall_x(x1, y0, y1, 0, h),
                      wall_y(y0, x0, x1, 0, h), wall_y(y1, x0, x1, 0, h)])


def floor_around(x0, x1, y0, y1):
    """4 m x 4 m 地面，去掉箱子占地范围内的点：实心箱子底下的地面雷达看不到"""
    pts = flat()
    under = (pts[:, 0] > x0) & (pts[:, 0] < x1) & (pts[:, 1] > y0) & (pts[:, 1] < y1)
    return pts[~under]


def test_box_is_obstacle_and_floor_around_is_free():
    layers = terrain.compute_layers(np.vstack([floor_around(1.8, 2.2, 1.8, 2.2), box(1.8, 2.2, 1.8, 2.2, 0.6)]),
                                    ORIGIN, SHAPE)
    assert cost_at(layers, 1.8, 2.0) == terrain.LETHAL
    assert cost_at(layers, 2.2, 2.0) == terrain.LETHAL
    assert cost_at(layers, 1.0, 2.0) == 0


def test_unreachable_box_top_is_lethal_when_start_given():
    pts = np.vstack([floor_around(1.5, 2.5, 1.5, 2.5), box(1.5, 2.5, 1.5, 2.5, 0.6)])
    without_start = terrain.compute_layers(pts, ORIGIN, SHAPE)
    assert cost_at(without_start, 2.0, 2.0) == 0          # 平顶面本身是“可通行”的小岛
    with_start = terrain.compute_layers(pts, ORIGIN, SHAPE, start_xy=(0.5, 0.5))
    assert cost_at(with_start, 2.0, 2.0) == terrain.LETHAL
    assert cost_at(with_start, 0.5, 3.5) == 0


def test_start_on_unknown_cell_keeps_reachable_area():
    pts = flat()
    keep = ~((np.abs(pts[:, 0] - 0.5) < 0.3) & (np.abs(pts[:, 1] - 0.5) < 0.3))   # 起点周围没有点
    layers = terrain.compute_layers(pts[keep], ORIGIN, SHAPE, start_xy=(0.5, 0.5))
    assert cost_at(layers, 2.0, 2.0) == 0
    assert cost_at(layers, 3.5, 3.5) == 0


def test_sparse_cell_is_unknown():
    pts = np.vstack([flat(0, 2, 0, 2), [[3.52, 3.52, 0.0], [3.53, 3.53, 0.0]]])
    layers = terrain.compute_layers(pts, ORIGIN, SHAPE)
    assert cost_at(layers, 3.52, 3.52) == terrain.UNKNOWN


def test_single_missing_cell_is_filled():
    pts = flat()
    hole = (pts[:, 0] >= 2.0) & (pts[:, 0] < 2.05) & (pts[:, 1] >= 2.0) & (pts[:, 1] < 2.05)
    layers = terrain.compute_layers(pts[~hole], ORIGIN, SHAPE)
    assert cost_at(layers, 2.02, 2.02) == 0
    assert abs(layers.elevation[40, 40]) < 0.01


def test_to_grid_map_data_order():
    layer = np.arange(6, dtype=np.float32).reshape(2, 3)       # 2 行 (y) x 3 列 (x)
    # grid_map：索引 (0,0) 在 x、y 最大处，列主序 -> 等价于把数组上下、左右都翻转后按行展开
    assert terrain.to_grid_map_data(layer).tolist() == [5, 4, 3, 2, 1, 0]


def frame_scene_step():
    """8 m x 8 m：x<4 是地面 z=0，x>=4 是高 0.5 m 的平台，x=4 处是平台侧面"""
    return np.vstack([flat(0, 4, 0, 8, 0.0), flat(4, 8, 0, 8, 0.5), wall_x(4.0, 0, 8, 0, 0.5)])


def local_cost(layers, origin, x, y):
    row, col, _ = terrain.cell_index(np.array([[x, y]]), origin, layers.cost.shape, terrain.FRAME_PARAMS.resolution)
    return int(layers.cost[row[0], col[0]])


def test_frame_ramp_is_traversable_and_has_no_obstacle_points():
    k = math.tan(math.radians(10.0))
    world = surface(0, 8, 0, 8, lambda x, y: k * x)
    local = world - np.array([2.0, 4.0, 0.3 + k * 2.0])       # 车在坡上 (2, 4)，雷达离坡面 0.3 m
    layers, origin, _ = terrain.frame_layers(local)
    assert 35 <= local_cost(layers, origin, 1.0, 0.0) <= 45
    assert not terrain.obstacle_point_mask(local).any()


def test_frame_platform_edge_is_lethal():
    local = frame_scene_step() - np.array([3.0, 4.0, 0.3])
    layers, origin, _ = terrain.frame_layers(local)
    assert local_cost(layers, origin, 0.95, 0.0) == terrain.LETHAL      # 世界 x=3.95
    assert local_cost(layers, origin, 1.05, 0.0) == terrain.LETHAL      # 世界 x=4.05
    assert local_cost(layers, origin, -1.0, 0.0) == 0                    # 世界 x=2.0
    assert local_cost(layers, origin, 2.0, 0.0) == 0                     # 世界 x=5.0（平台顶面）


def test_frame_box_side_points_are_obstacles_floor_is_not():
    world = np.vstack([floor_around(3.8, 4.2, 3.8, 4.2), box(3.8, 4.2, 3.8, 4.2, 0.6)])
    local = world - np.array([3.0, 4.0, 0.3])
    mask = terrain.obstacle_point_mask(local)
    side = (np.abs(world[:, 0] - 3.8) < 1e-6) & (world[:, 2] > 0.1)
    far_floor = (world[:, 2] == 0.0) & (np.hypot(world[:, 0] - 4.0, world[:, 1] - 4.0) > 0.6)
    assert mask[side].mean() > 0.5
    assert not mask[far_floor].any()


def test_frame_ignores_points_beyond_range():
    local = np.array([[6.0, 0.0, 0.3], [0.0, 5.5, 0.3]])
    assert not terrain.obstacle_point_mask(local).any()


def vote(scene, centers, shape=(160, 160)):
    grid = terrain.VoteGrid((0.0, 0.0), shape, RES)
    for c in centers:
        layers, origin, _ = terrain.frame_layers(scene - np.asarray(c))
        grid.add_frame(layers, origin, terrain.FRAME_PARAMS.resolution, c)
    return grid.finalize()


CENTERS = [(2.5, 2.0, 0.3), (3.0, 4.0, 0.3), (2.5, 6.0, 0.3)]


def test_votes_mark_edge_lethal_without_gaps_and_floor_free():
    final = vote(frame_scene_step(), CENTERS)
    assert cost_at(final, 2.0, 4.0) == 0
    assert cost_at(final, 5.0, 4.0) == 0
    band = final.cost[int(1.0 / RES):int(7.0 / RES), int(3.85 / RES):int(4.15 / RES) + 1]
    assert (band == terrain.LETHAL).any(axis=1).all()


def test_votes_are_immune_to_frame_height_drift():
    scene = frame_scene_step()
    ref = vote(scene, CENTERS)
    # 模拟 SLAM 高度漂移：后两帧整体（点和位姿）抬高 0.15 m
    grid = terrain.VoteGrid((0.0, 0.0), (160, 160), RES)
    merged = []
    for i, c in enumerate(CENTERS):
        dz = 0.15 if i else 0.0
        pts, cen = scene + [0, 0, dz], np.asarray(c) + [0, 0, dz]
        layers, origin, _ = terrain.frame_layers(pts - cen)
        grid.add_frame(layers, origin, terrain.FRAME_PARAMS.resolution, cen)
        merged.append(pts)
    assert np.array_equal(grid.finalize().cost, ref.cost)
    # 对照：把漂移后的点合并起来直接算（修订前的做法），地面会大片变成不可通行
    old = terrain.compute_layers(np.vstack(merged), (0.0, 0.0), (160, 160))
    floor = old.cost[int(1.0 / RES):int(7.0 / RES), int(1.0 / RES):int(3.5 / RES)]
    assert (floor == terrain.LETHAL).mean() > 0.3


def fake_layers(cost, slope=0.0):
    cost = np.asarray(cost, np.int8)
    return terrain.TerrainLayers(np.zeros(cost.shape, np.float32), cost == terrain.LETHAL,
                                 np.full(cost.shape, slope, np.float32), np.zeros(cost.shape, np.float32), cost)


def test_single_lethal_vote_is_outvoted_two_are_not():
    free, bad = fake_layers(np.zeros((20, 20))), fake_layers(np.full((20, 20), terrain.LETHAL))
    for n_bad, expect in ((1, 0), (2, terrain.LETHAL)):
        grid = terrain.VoteGrid((0.0, 0.0), (40, 40), RES)
        for layers in [free] * 4 + [bad] * n_bad:
            grid.add_frame(layers, (-1.0, -1.0), 0.1, (1.0, 1.0, 0.0))
        assert cost_at(grid.finalize(), 1.0, 1.0) == expect


def test_cell_seen_once_as_lethal_only_is_unknown():
    grid = terrain.VoteGrid((0.0, 0.0), (40, 40), RES)
    grid.add_frame(fake_layers(np.full((20, 20), terrain.LETHAL)), (-1.0, -1.0), 0.1, (1.0, 1.0, 0.0))
    assert cost_at(grid.finalize(), 1.0, 1.0) == terrain.UNKNOWN


def test_votes_average_slope_into_cost():
    grid = terrain.VoteGrid((0.0, 0.0), (40, 40), RES)
    for slope in (4.0, 6.0):
        grid.add_frame(fake_layers(np.zeros((20, 20)), slope), (-1.0, -1.0), 0.1, (1.0, 1.0, 0.0))
    assert cost_at(grid.finalize(), 1.0, 1.0) == 20          # 平均 5° -> round(60*5/15)


def test_large_unknown_region_is_sealed_small_hole_is_not():
    free = fake_layers(np.zeros((80, 40)))                         # x 0~2 m、y 0~4 m
    grid = terrain.VoteGrid((0.0, 0.0), (80, 80), RES)             # 4 m x 4 m，右半边（x>=2）从没观测到
    grid.add_frame(free, (-1.0, -2.0), 0.05, (1.0, 2.0, 0.0))
    grid.known[60, 30] = 0                                         # 左半边里一个 1 格的小空洞 (1.5, 3.0)
    final = grid.finalize()
    row = final.cost[30, 20:]                                      # y=1.5，从 x=1 往右
    assert row[np.flatnonzero(row != 0)[0]] == terrain.LETHAL      # 进入大片未知区之前先碰到不可通行
    assert cost_at(final, 1.0, 2.0) == 0
    assert cost_at(final, 1.55, 3.05) == 0                         # 小空洞旁边不封


def ring_floor(center, radii, z=0.0, step_deg=0.5):
    """真实雷达一帧打在平地上是一圈一圈的点：圈与圈之间（远处约 1 m）没有点"""
    a = np.radians(np.arange(0.0, 360.0, step_deg))
    return np.vstack([np.column_stack([center[0] + r * np.cos(a), center[1] + r * np.sin(a),
                                       np.full(a.size, z)]) for r in radii])


def test_sparse_ring_frame_still_votes_floor_free():
    c = (4.0, 4.0, 0.4)
    grid = terrain.VoteGrid((0.0, 0.0), (160, 160), RES)
    for dx in (0.0, 0.05, 0.1):                                    # 几帧，位置略有不同
        cc = (c[0] + dx, c[1], c[2])
        layers, origin, _ = terrain.frame_layers(ring_floor(cc, (3.3, 3.9, 4.7)) - np.asarray(cc))
        grid.add_frame(layers, origin, terrain.FRAME_PARAMS.resolution, cc)
    final = grid.finalize(seal_min_area=1e9)
    assert cost_at(final, 4.0 + 3.3, 4.0) == 0                     # 圈上的地面格是可通行，不是未知


def test_speckled_holes_reaching_big_unknown_do_not_seal_interior():
    free = fake_layers(np.zeros((80, 40)))                         # x 0~2 m、y 0~4 m 已知，x>=2 未知
    grid = terrain.VoteGrid((0.0, 0.0), (80, 80), RES)
    grid.add_frame(free, (-1.0, -2.0), 0.05, (1.0, 2.0, 0.0))
    for i in range(40):                                            # 一条 1 格宽的对角线小洞，一直连到右边的大片未知区
        grid.known[20 + i, i] = 0
    final = grid.finalize()
    assert cost_at(final, 1.075, 2.025) == 0                       # 紧挨着小洞的格子不封
    assert cost_at(final, 0.5, 3.0) == 0
    row = final.cost[70, 20:]                                      # y=3.5，从 x=1 往右
    assert row[np.flatnonzero(row != 0)[0]] == terrain.LETHAL      # 大片未知区仍然封住


def test_unknown_blind_spot_around_start_is_not_sealed():
    # 雷达前倾，出生点周围建图时一直没被看到；封口不能把车困在原地
    free = fake_layers(np.zeros((80, 40)))                         # x 0~2 m、y 0~4 m 已知，x>=2 未知
    grid = terrain.VoteGrid((0.0, 0.0), (80, 80), RES)
    grid.add_frame(free, (-1.0, -2.0), 0.05, (1.0, 2.0, 0.0))
    grid.known[30:50, 10:30] = 0                                   # 起点 (1.0, 2.0) 周围 1 m x 1 m 从没观测到
    final = grid.finalize(start_xy=(1.0, 2.0))
    assert cost_at(final, 1.0, 2.6) == 0                           # 盲区旁边不封
    assert cost_at(final, 0.4, 2.0) == 0
    row = final.cost[70, 20:]                                      # y=3.5：远处的大片未知区仍然封住
    assert row[np.flatnonzero(row != 0)[0]] == terrain.LETHAL


def test_frame_thin_pole_in_single_cell_is_obstacle():
    # 直径 8 cm 的杆（人腿、货架立柱）正好落在一个 0.1 m 单帧格子里
    a = np.radians(np.arange(0.0, 360.0, 10.0))
    zs = np.arange(0.0, 1.0, 0.02)
    cx, cy = 1.05, 0.05
    pole = np.array([[cx + 0.04 * math.cos(t), cy + 0.04 * math.sin(t), z] for t in a for z in zs])
    floor = flat(-2.0, 4.0, -2.0, 2.0, 0.0)
    floor = floor[np.hypot(floor[:, 0] - cx, floor[:, 1] - cy) > 0.04]
    local = np.vstack([floor, pole]) - np.array([0.0, 0.0, 0.4])
    mask = terrain.obstacle_point_mask(local)
    is_pole = np.arange(len(local)) >= len(floor)
    assert mask[is_pole & (local[:, 2] > -0.25)].mean() > 0.5
    assert not mask[~is_pole].any()


def test_frame_ignores_own_body_and_wheels_within_self_range():
    # 车放大 1.5 倍后前轮离雷达约 0.53 m：车身、车轮上的点（SELF_RANGE 以内）不能被当成障碍
    floor = np.array([[x, y, -0.4] for x in np.arange(-3, 3, 0.05) for y in np.arange(-3, 3, 0.05)])
    wheel = np.array([[0.0, 0.45, z] for z in np.arange(-0.4, -0.1, 0.02)] * 4)   # 离原点约 0.5 m 的竖直“车轮”
    mask = terrain.obstacle_point_mask(np.vstack([floor, wheel]))
    assert not mask[len(floor):].any()
    assert terrain.SELF_RANGE >= 0.6


def test_narrow_unknown_strip_between_seen_floor_is_not_sealed():
    # 两条行驶线之间漏看的一条窄带（车放大后雷达更高，常见 0.3~0.5 m 宽）：不是“大片没去过的地方”，
    # 封成不可通行再加上膨胀，会把车道整个堵死。只封最窄处也有一定宽度的未知区
    free = fake_layers(np.zeros((80, 80)))                         # 4 m x 4 m 全部观测过
    grid = terrain.VoteGrid((0.0, 0.0), (80, 80), RES)
    grid.add_frame(free, (-2.0, -2.0), 0.05, (2.0, 2.0, 0.0))
    grid.known[10:70, 36:44] = 0                                   # x 1.8~2.2、y 0.5~3.5 的窄带没观测到
    final = grid.finalize()
    assert (final.cost != terrain.LETHAL).all()
    assert cost_at(final, 2.0, 2.0) == terrain.UNKNOWN
    assert cost_at(final, 1.6, 2.0) == 0 and cost_at(final, 2.4, 2.0) == 0

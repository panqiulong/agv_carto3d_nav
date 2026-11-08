import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import elevator_fit as ef  # noqa: E402

# 合成数据，几何取自原 industrial 世界（电梯在西墙，车头朝西进梯）：轿厢内净 x ∈ [-32.75, -30.15]、y ∈ [-12, -10]，外墙内表面 x = -29.85，
# 门洞 y ∈ [-11.9, -10.1]
PRIOR = ef.Cabin(-31.45, -11.0, math.pi)
PRIOR_OFF = ef.Cabin(-31.42, -10.97, math.pi + math.radians(1))     # 先验带 3 cm / 1° 的地图误差
ROI_Y = (-12.5, -9.5)
HEIGHTS = np.arange(0.2, 2.0, 0.1)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def wall_points_y(y, x0, x1):
    """y = 常数的一面墙，沿墙每 2 cm、每 0.1 m 高一个点，投影到 xy"""
    xs = np.arange(x0, x1, 0.02)
    return np.repeat(np.c_[xs, np.full(xs.size, y)], HEIGHTS.size, axis=0)


def wall_points_x(x, y0, y1):
    ys = np.arange(y0, y1, 0.02)
    return np.repeat(np.c_[np.full(ys.size, x), ys], HEIGHTS.size, axis=0)


def cabin_cloud(car_x, car_y, car_yaw, sides=('n', 's'), back=True, jambs=True, noise=0.01, clutter=200, seed=0):
    rng = np.random.default_rng(seed)
    parts = [wall_points_x(-29.85, ROI_Y[0], -11.9), wall_points_x(-29.85, -10.1, ROI_Y[1])]   # 门洞两侧外墙
    if 'n' in sides:
        parts.append(wall_points_y(-10.0, -32.75, -30.15))
    if 's' in sides:
        parts.append(wall_points_y(-12.0, -32.75, -30.15))
    if back:
        parts.append(wall_points_x(-32.75, -12.0, -10.0))
    if jambs:
        parts += [wall_points_y(-11.9, -30.15, -29.85), wall_points_y(-10.1, -30.15, -29.85)]
    xy = np.vstack(parts)
    xy = xy + rng.normal(0.0, noise, xy.shape)
    junk = np.c_[rng.uniform(-33.25, -29.65, clutter), rng.uniform(ROI_Y[0], ROI_Y[1], clutter)]
    xy = np.vstack([xy, junk])
    return xy[np.hypot(xy[:, 0] - car_x, xy[:, 1] - car_y) < 8.0]


def extract(xy, prior=PRIOR_OFF, last=None):
    return ef.cabin_from_lines(ef.fit_lines(xy, rng=np.random.default_rng(0)), prior, last)


@pytest.mark.parametrize('dy', [-0.375, 0.0, 0.375])
@pytest.mark.parametrize('dyaw', [-0.3, 0.0, 0.3])
def test_extracts_cabin_from_wait_point(dy, dyaw):
    c = extract(cabin_cloud(-26.0, -11.0 + dy, math.pi + dyaw, seed=int(10 * dy + 100 * dyaw) % 97))
    assert math.hypot(c.x + 31.45, c.y + 11.0) < 0.02
    assert abs(wrap(c.yaw - math.pi)) < math.radians(0.5)


def test_roi_drops_floor_roof_and_far_points():
    pts = np.array([[-31.0, -11.0, 0.05], [-31.0, -11.0, 2.3], [-31.0, -11.0, 1.0], [-20.0, -11.0, 1.0]])
    assert ef.roi_points(pts, PRIOR, ground_z=0.0).tolist() == [[-31.0, -11.0]]


def test_one_side_wall_is_completed():
    c = extract(cabin_cloud(-26, -11, math.pi, sides=('n',)))
    assert math.hypot(c.x + 31.45, c.y + 11.0) < 0.02


def test_missing_back_wall_keeps_last_depth():
    c = extract(cabin_cloud(-26, -11, math.pi, back=False), last=ef.Cabin(-31.40, -11.0, math.pi))
    assert c.x == pytest.approx(-31.40, abs=0.005) and c.y == pytest.approx(-11.0, abs=0.02)


def test_missing_back_wall_without_last_is_none():
    assert extract(cabin_cloud(-26, -11, math.pi, back=False)) is None


def test_wrong_wall_spacing_is_rejected():
    xy = cabin_cloud(-26, -11, math.pi)
    xy = xy[~((xy[:, 1] < -11.95) & (xy[:, 0] < -30.15))]           # 去掉南侧墙
    xy = np.vstack([xy, wall_points_y(-12.3, -32.75, -30.15)])      # 换成 2.3 m 外的一面
    assert extract(xy) is None


def test_door_jambs_are_not_side_walls():
    c = extract(cabin_cloud(-26, -11, math.pi, sides=('n',), jambs=True))
    assert abs(c.y + 11.0) < 0.02


def test_filter_averages_and_drops_outliers():
    f = ef.CabinFilter()
    for dx in (0.0, 0.01, -0.01, 0.0):
        f.update(ef.Cabin(-31.45 + dx, -11.0, math.pi))
    m = f.update(ef.Cabin(-31.10, -11.0, math.pi))               # 偏 35 cm，丢掉
    assert f.count == 4 and m.x == pytest.approx(-31.45, abs=0.005)
    assert f.update(None).x == pytest.approx(-31.45, abs=0.005)   # 没看到的帧不改变均值
    for _ in range(3):
        f.update(ef.Cabin(-31.45, -11.0, math.pi))
    assert f.count == 5                                          # 窗口最多 5 帧


def test_filter_recovers_from_bad_first_frame():
    f = ef.CabinFilter()
    f.update(ef.Cabin(-31.45, -10.5, math.pi))
    for _ in range(5):
        f.update(ef.Cabin(-31.45, -11.0, math.pi))
    m = f.update(ef.Cabin(-31.45, -11.0, math.pi))
    assert m.y == pytest.approx(-11.0, abs=1e-6)


def test_filter_yaw_mean_wraps_at_pi():
    f = ef.CabinFilter()
    f.update(ef.Cabin(-31.45, -11.0, math.pi - 0.01))
    m = f.update(ef.Cabin(-31.45, -11.0, -math.pi + 0.01))
    assert abs(wrap(m.yaw - math.pi)) < 1e-6


# ---------------------------------------------------------------- 控制
CAB = ef.Cabin(-31.45, -11.0, math.pi)
CAR_L, CAR_W, REAR = 1.2, 1.08, 0.432


def car_corners(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * a - s * b, y + s * a + c * b) for a in (CAR_L / 2, -CAR_L / 2) for b in (CAR_W / 2, -CAR_W / 2)]


def touches(x, y, yaw, cab):
    """车体碰到门框墙体 / 轿厢墙，或者门框内角点落进车体（都在轿厢系里判断）"""
    for px, py in car_corners(x, y, yaw):
        d, lat, _ = ef.to_cabin_frame(px, py, 0.0, cab)
        if (-1.6 <= d <= -1.3 and abs(lat) > 0.9) or (d > -1.3 and abs(lat) > 1.0) or d > 1.3:
            return True
    u = np.array([math.cos(cab.yaw), math.sin(cab.yaw)])
    v = np.array([-u[1], u[0]])
    h = np.array([math.cos(yaw), math.sin(yaw)])
    for d in (-1.6, -1.3):
        for lat in (0.9, -0.9):
            q = np.array([cab.x, cab.y]) + d * u + lat * v - (x, y)
            if abs(q @ h) < CAR_L / 2 and abs(q[0] * -h[1] + q[1] * h[0]) < CAR_W / 2:
                return True
    return False


def drive(x, y, yaw, cab, direction, target_depth, v_max, dt=0.05):
    """以后轴为参考点的独轮车运动学：后轴沿车头方向走，偏航角速度 = k * v"""
    hit = False
    for _ in range(4000):
        depth, _, _ = ef.to_cabin_frame(x, y, yaw, cab)
        remaining = target_depth - depth if direction > 0 else depth - target_depth
        v = ef.approach_speed(remaining, v_max)
        if v == 0.0:
            break
        k = ef.pursuit_curvature(x, y, yaw, cab, direction)
        rx, ry = x - REAR * math.cos(yaw), y - REAR * math.sin(yaw)
        yaw += k * v * dt
        rx += direction * v * dt * math.cos(yaw)
        ry += direction * v * dt * math.sin(yaw)
        x, y = rx + REAR * math.cos(yaw), ry + REAR * math.sin(yaw)
        hit = hit or touches(x, y, yaw, cab)
    return (x, y, yaw), hit


@pytest.mark.parametrize('dy', [-0.375, 0.0, 0.375])
@pytest.mark.parametrize('dyaw', [-0.3, 0.0, 0.3])
def test_enter_from_wait_point_centers_without_contact(dy, dyaw):
    (x, y, yaw), hit = drive(-26.0, -11.0 + dy, math.pi + dyaw, CAB, +1, target_depth=0.0, v_max=0.2)
    depth, lat, yerr = ef.to_cabin_frame(x, y, yaw, CAB)
    assert not hit
    assert abs(lat) < 0.03 and abs(yerr) < math.radians(1.0) and abs(depth) < 0.10


def test_exit_backs_out_along_centerline():
    (x, y, yaw), hit = drive(-31.45, -11.02, math.pi + 0.01, CAB, -1, target_depth=-5.45, v_max=0.15)
    depth, lat, _ = ef.to_cabin_frame(x, y, yaw, CAB)
    assert not hit and abs(depth + 5.45) < 0.10 and abs(lat) < 0.05


def test_to_cabin_frame():
    assert ef.to_cabin_frame(-26.0, -10.0, math.pi, CAB) == pytest.approx((-5.45, -1.0, 0.0))


def test_curvature_is_capped():
    assert abs(ef.pursuit_curvature(-26.0, -9.0, math.pi / 2, CAB, +1)) == pytest.approx(1 / 2.3)


def test_approach_speed_floor_until_target():
    assert ef.approach_speed(2.0, 0.2) == 0.2
    assert ef.approach_speed(0.15, 0.2) == pytest.approx(0.1)
    assert ef.approach_speed(0.001, 0.2) == 0.03
    assert ef.approach_speed(0.0, 0.2) == 0.0 and ef.approach_speed(-0.01, 0.2) == 0.0


def test_inside_cabin():
    assert ef.inside_cabin(-31.45, -11.0, CAB)
    assert not ef.inside_cabin(-26.0, -11.0, CAB)
    assert not ef.inside_cabin(-31.45, -12.2, CAB)


def test_footprint_hits():
    pts = np.array([[0.62, 0.0], [0.0, 0.58], [0.7, 0.0], [0.0, 0.6]])
    assert ef.footprint_hits(pts) == 2


@pytest.mark.parametrize('dy', [-0.3, 0.3])
@pytest.mark.parametrize('dyaw_deg', [-3.0, 3.0])
def test_teach_from_rough_pose_converges(dy, dyaw_deg):
    # 示教：车头大致对着门停在门外（横向偏 0.3 m、航向偏 3°），先验 = 车正前方 5.45 m；
    # 第一帧可能只框进一面侧墙，用提取结果重新框选，第二帧两面墙都在
    car_x, car_y, car_yaw = -26.0, -11.0 + dy, math.pi + math.radians(dyaw_deg)
    xy = cabin_cloud(car_x, car_y, car_yaw)
    pts = np.c_[xy, np.ones(len(xy))]
    prior = ef.cabin_ahead(car_x, car_y, car_yaw, 5.45)
    for _ in range(2):
        prior = extract(ef.roi_points(pts, prior, 0.0), prior=prior)
        assert prior is not None
    assert math.hypot(prior.x + 31.45, prior.y + 11.0) < 0.02
    assert abs(wrap(prior.yaw - math.pi)) < math.radians(0.5)


@pytest.mark.parametrize('dyaw_deg', [-20.0, -12.0, 12.0, 20.0])
def test_teach_search_finds_cabin_when_heading_is_far_off(dyaw_deg):
    # 导航 / 遥控停车航向偏 20° 时，正前方 5.45 m 的先验横向偏 1.9 m，框不到轿厢：绕车转着找
    car_x, car_y, car_yaw = -26.0, -11.0 + 0.2, math.pi + math.radians(dyaw_deg)
    xy = cabin_cloud(car_x, car_y, car_yaw)
    pts = np.c_[xy, np.ones(len(xy))]
    c = ef.search_cabin(pts, 0.0, car_x, car_y, car_yaw, 5.45, rng=np.random.default_rng(0))
    assert math.hypot(c.x + 31.45, c.y + 11.0) < 0.02
    assert abs(wrap(c.yaw - math.pi)) < math.radians(0.5)


def test_teach_search_rejects_scene_without_cabin():
    xy = cabin_cloud(-26.0, -11.0, math.pi, sides=(), back=False, jambs=False)
    pts = np.c_[xy, np.ones(len(xy))]
    assert ef.search_cabin(pts, 0.0, -26.0, -11.0, math.pi, 5.45, rng=np.random.default_rng(0)) is None


def test_wait_point_on_centerline_outside_door():
    assert ef.wait_point(PRIOR, 5.45) == pytest.approx((-26.0, -11.0))

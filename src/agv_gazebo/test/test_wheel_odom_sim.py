import importlib.util
import math
import os

import numpy as np
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(__file__)
spec = importlib.util.spec_from_file_location(
    'wheel_odom_sim', os.path.join(HERE, '..', 'scripts', 'wheel_odom_sim.py'))
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

Q0 = [0.0, 0.0, 0.0, 1.0]


def model(**kw):
    return mod.WheelOdomModel(**{'scale': 0.0, 'sigma_s': 0.0, 'curvature_bias': 0.0, 'sigma_yaw': 0.0, **kw})


def drive(m, poses, dt=0.01):
    out = None
    for i, (p, q) in enumerate(poses):
        out = m.update(np.array(p, float), q, i * dt)
    return out


def straight(length, step=0.01):
    return [([i * step, 0.0, 0.0], Q0) for i in range(int(round(length / step)) + 1)]


def test_first_message_only_initializes_at_truth():
    assert model().update(np.array([2.0, -4.0, 0.18]), Q0, 0.0) is None


def test_no_noise_reproduces_truth_on_a_pitched_arc():
    poses = []
    for i in range(200):
        yaw, pitch = i * 0.01, -0.1
        q = R.from_euler('ZY', [yaw, pitch]).as_quat()
        poses.append(([2.0 + math.sin(yaw), math.cos(yaw) - 1.0, 0.1 * i * 0.01], q))
    out = drive(model(), poses)
    assert np.allclose(out.position, poses[-1][0], atol=1e-9)
    assert (R.from_quat(out.orientation) * R.from_quat(poses[-1][1]).inv()).magnitude() < 1e-9


def test_scale_error_stretches_distance_and_speed():
    out = drive(model(scale=0.01), straight(10.0))
    assert math.isclose(out.position[0], 10.1, abs_tol=1e-6)
    assert math.isclose(out.linear[0], 1.01, abs_tol=1e-6)


def test_curvature_bias_accumulates_with_distance_and_reverses_backward():
    forward = drive(model(curvature_bias=0.002), straight(10.0))
    assert math.isclose(R.from_quat(forward.orientation).as_euler('ZYX')[0], 0.02, abs_tol=1e-6)

    there_and_back = straight(10.0) + straight(10.0)[::-1]
    back = drive(model(curvature_bias=0.002), there_and_back)
    assert abs(R.from_quat(back.orientation).as_euler('ZYX')[0]) < 1e-6


def test_lateral_velocity_is_kept():
    """base_link 在后轴前面，转弯时本来就有横向速度，模型不能把它抹掉"""
    poses = [([0.0, i * 0.001, 0.0], Q0) for i in range(5)]
    assert math.isclose(drive(model(), poses).linear[1], 0.1, abs_tol=1e-9)


def test_standing_still_does_not_drift_even_with_noise():
    m = model(scale=0.01, sigma_s=0.01, curvature_bias=0.002, sigma_yaw=0.003)
    out = drive(m, [([2.0, -4.0, 0.18], Q0)] * 1000)
    assert np.allclose(out.position, [2.0, -4.0, 0.18])
    assert R.from_quat(out.orientation).magnitude() < 1e-9      # q 和 -q 是同一个旋转，不能逐分量比


def test_random_walk_does_not_depend_on_message_rate():
    """同一段路用不同频率采样，累积的噪声大小应该相同"""
    def spread(step):
        ends = [drive(model(sigma_s=0.01, seed=s), straight(25.0, step)).position[0] for s in range(40)]
        return np.std(ends)
    expected = 0.01 * math.sqrt(25.0)
    assert 0.6 * expected < spread(0.05) < 1.5 * expected
    assert 0.6 * expected < spread(0.01) < 1.5 * expected


def test_repeated_timestamp_is_skipped():
    m = model()
    m.update(np.zeros(3), Q0, 1.0)
    assert m.update(np.array([0.1, 0.0, 0.0]), Q0, 1.0) is None

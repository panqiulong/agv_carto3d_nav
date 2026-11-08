import importlib.util
import os

import pytest
import rclpy
from geometry_msgs.msg import Twist

HERE = os.path.dirname(__file__)
spec = importlib.util.spec_from_file_location(
    'cmd_vel_to_ackermann', os.path.join(HERE, '..', 'scripts', 'cmd_vel_to_ackermann.py'))
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


@pytest.fixture
def node():
    rclpy.init()
    n = mod.CmdVelToAckermann()
    out = []
    n.pub.publish = out.append
    n.out = out
    yield n
    n.destroy_node()
    rclpy.shutdown()


def send(node, v, wz, dt):
    msg = Twist()
    msg.linear.x, msg.angular.z = v, wz
    node.on_cmd(msg, dt)
    return node.out[-1].angular.z


def test_steer_change_is_rate_limited(node):
    # 0.5 m/s、0.25 rad/s 对应转角 atan(0.855 * 0.25 / 0.5) ≈ 0.40 rad，一步只能转 max_steer_rate * dt
    step = node.max_steer_rate * 0.05
    assert send(node, 0.5, 0.25, 0.05) == pytest.approx(step)
    assert send(node, 0.5, 0.25, 0.05) == pytest.approx(2 * step)


def test_small_change_passes_through(node):
    send(node, 0.5, 0.0, 0.05)
    target = mod.math.atan(node.wheelbase * 0.01 / 0.5)
    assert send(node, 0.5, 0.01, 0.05) == pytest.approx(target)


def test_reversing_does_not_flip_the_wheels(node):
    # 插件按 sign(v) 翻转转角：前进时实际转角 +s，换成倒车时指令要变成 -s，实际转角才保持 +s 不跳变
    for _ in range(100):
        send(node, 0.5, 0.2, 0.05)
    s = node.last_steer
    assert send(node, -0.3, 0.0, 0.05) == pytest.approx(-(s - node.max_steer_rate * 0.05))


def test_long_gap_is_capped(node):
    assert send(node, 0.5, 0.25, 5.0) == pytest.approx(node.max_steer_rate * 0.2)

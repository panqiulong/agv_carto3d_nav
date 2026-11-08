import os
import stat
import sys
import threading
import time

import pytest
from action_msgs.msg import GoalStatus

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
rclpy = pytest.importorskip('rclpy')
PlaceEdit = pytest.importorskip('agv_semantic.srv').PlaceEdit     # 需要先 source install/setup.bash
import semantic_map as S  # noqa: E402
import semantic_server  # noqa: E402


@pytest.fixture
def server(tmp_path):
    path = str(tmp_path / 'ro' / 'm_semantic.yaml')
    S.save(S.SemanticMap(places=[S.Place('a', 0, 0, 0)]), path)
    rclpy.init(args=['--ros-args', '-p', 'semantic_file:=' + path])
    node = semantic_server.SemanticServer()
    node.robot_pose = lambda: (1.0, 2.0, 0.5)
    os.chmod(tmp_path / 'ro', stat.S_IRUSR | stat.S_IXUSR)       # 目录只读：保存会失败
    yield node, path
    os.chmod(tmp_path / 'ro', stat.S_IRWXU)
    node.destroy_node()
    rclpy.shutdown()


def test_add_fails_cleanly_when_file_cannot_be_saved(server):
    node, path = server
    res = node.on_add(PlaceEdit.Request(name='b'), PlaceEdit.Response())
    assert not res.success and '保存失败' in res.message
    assert [p.name for p in node.sm.places] == ['a']                 # 内存与文件保持一致
    assert [p.name for p in S.load(path).places] == ['a']


def test_remove_fails_cleanly_when_file_cannot_be_saved(server):
    node, path = server
    res = node.on_remove(PlaceEdit.Request(name='a'), PlaceEdit.Response())
    assert not res.success and '保存失败' in res.message
    assert [p.name for p in node.sm.places] == ['a']


@pytest.fixture
def goto_server(tmp_path):
    path = str(tmp_path / 'm_semantic.yaml')
    S.save(S.SemanticMap(places=[S.Place('a', 0, 0, 0)]), path)
    rclpy.init(args=['--ros-args', '-p', 'semantic_file:=' + path])
    node = semantic_server.SemanticServer()
    yield node
    node.destroy_node()
    rclpy.try_shutdown()


class _Future:
    def __init__(self, value=None, done=False):
        self.value, self.is_done = value, done

    def done(self):
        return self.is_done

    def result(self):
        return self.value


class _NavHandle:
    accepted = True

    def __init__(self):
        self.result_future = _Future()

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        # Nav2 过一会儿才真正结束目标
        def finish():
            self.result_future.value = type('R', (), {'status': GoalStatus.STATUS_CANCELED})()
            self.result_future.is_done = True
        threading.Timer(0.3, finish).start()
        return _Future(done=True)


class _Nav:
    def __init__(self):
        self.handle = _NavHandle()

    def wait_for_server(self, timeout_sec=None):
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        return _Future(self.handle, done=True)


class _GoalHandle:
    def __init__(self, name):
        self.request = type('Req', (), {'name': name})()
        self.calls = []

    @property
    def is_cancel_requested(self):
        return True

    def canceled(self):
        self.calls.append('canceled')

    def abort(self):
        self.calls.append('abort')

    def succeed(self):
        self.calls.append('succeed')

    def publish_feedback(self, fb):
        pass


def test_goto_cancel_waits_for_nav2_goal_to_finish(goto_server):
    node = goto_server
    node.nav = _Nav()
    gh = _GoalHandle('a')
    res = node.on_goto(gh)
    assert gh.calls == ['canceled'] and not res.success
    assert node.nav.handle.result_future.done()                      # 返回前 Nav2 目标已真正结束


@pytest.fixture
def zone_server(tmp_path):
    path = str(tmp_path / 'm_semantic.yaml')
    S.save(S.SemanticMap(zones=[S.Zone('坡道', [[0, 0], [10, 0], [10, 2], [0, 2]], speed_limit=0.3),
                                S.Zone('房间', [[20, 0], [30, 0], [30, 2], [20, 2]])]), path)
    rclpy.init(args=['--ros-args', '-p', 'semantic_file:=' + path])
    node = semantic_server.SemanticServer()
    sent = []

    class Pub:
        def publish(self, msg):
            sent.append((msg.speed_limit, msg.percentage))
    node.speed_pub = Pub()
    yield node, sent
    node.destroy_node()
    rclpy.shutdown()


def test_speed_limit_is_republished_while_inside_a_speed_zone(zone_server):
    # MPPI 每收到新的跟踪目标都会把速度上限重置成 vx_max，而限速过滤器只在变化时发一次 /speed_limit；
    # 车在限速区里触发恢复动作后限速就丢了，所以在区内要周期性重发
    node, sent = zone_server
    node.robot_pose = lambda: (5.0, 1.0, 0.0)
    node.on_zone_timer()
    node.on_zone_timer()
    assert sent == [(0.3, False), (0.3, False)]


def test_leaving_a_speed_zone_publishes_no_limit_once(zone_server):
    node, sent = zone_server
    node.robot_pose = lambda: (5.0, 1.0, 0.0)
    node.on_zone_timer()
    node.robot_pose = lambda: (25.0, 1.0, 0.0)              # 普通区域，没有限速
    node.on_zone_timer()
    node.on_zone_timer()
    assert sent == [(0.3, False), (0.0, False)]             # 0 = 不限速，只发一次


class _GoalHandleNoCancel(_GoalHandle):
    @property
    def is_cancel_requested(self):
        return False


def test_goto_returns_when_node_shuts_down(goto_server):
    # launch 关闭时如果 goto 正在等 Nav2 的结果，结果永远不会来：等待循环必须在节点关闭后退出，
    # 否则执行器线程一直卡着，进程收到 SIGTERM 也退不出去（成了孤儿进程）
    node = goto_server
    node.nav = _Nav()
    gh = _GoalHandleNoCancel('a')
    threading.Timer(0.3, rclpy.shutdown).start()
    t0 = time.time()
    res = node.on_goto(gh)
    assert time.time() - t0 < 5.0
    assert not res.success and gh.calls == ['abort']

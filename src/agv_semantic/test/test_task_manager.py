import os
import stat
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
rclpy = pytest.importorskip('rclpy')
srv = pytest.importorskip('agv_semantic.srv')            # 需要先 source install/setup.bash
from std_srvs.srv import Trigger  # noqa: E402
import semantic_map as S  # noqa: E402
import tasks as T  # noqa: E402
import task_manager  # noqa: E402


class FakeGoto:
    """记录去过的地点；fail 里的地点总是失败；hook(name) 在每次调用时先执行（用来模拟中途取消/提交）"""

    def __init__(self, fail=(), hook=None):
        self.calls, self.fail, self.hook = [], set(fail), hook

    def __call__(self, name, should_cancel):
        self.calls.append(name)
        if self.hook:
            self.hook(name)
        if should_cancel():
            return False, task_manager.CANCELED
        if name in self.fail:
            return False, '规划失败'
        return True, '已到达 %s' % name


@pytest.fixture
def make(tmp_path):
    nodes = []

    def _make(dwell=0.0, pose=(0.0, 0.0, 0.0), folder='d'):
        d = tmp_path / folder
        sem = str(d / 'm_semantic.yaml')
        S.save(S.SemanticMap(places=[S.Place(n, x, 0.0, 0.0) for n, x in
                                     (('HOME', 0), ('R1', 5), ('S1', 10), ('A1', 15), ('A2', 20))]), sem)
        path = str(d / 'm_tasks.yaml')
        T.save(T.TaskConfig('HOME', dwell, ['R1'], ['S1'], {'A1': False, 'A2': True}), path)
        rclpy.init(args=['--ros-args', '-p', 'tasks_file:=' + path, '-p', 'semantic_file:=' + sem])
        node = task_manager.TaskManager()
        node.robot_pose = lambda: pose
        node.goto = FakeGoto()
        nodes.append(node)
        return node, path

    yield _make
    for n in nodes:
        n.destroy_node()
    rclpy.try_shutdown()


def submit(node, type_, source='', target=''):
    return node.on_submit(srv.TaskSubmit.Request(type=type_, source=source, target=target), srv.TaskSubmit.Response())


def test_inbound_runs_and_saves_stock(make):
    node, path = make()
    res = submit(node, 'inbound')
    assert res.success and res.id == 1 and 'R1' in res.message and 'A1' in res.message
    assert node.step() is True
    assert node.goto.calls == ['R1', 'A1']
    assert node.board.recent[-1].state == 'done'
    assert T.load(path, {'HOME', 'R1', 'S1', 'A1', 'A2'}).slots == {'A1': True, 'A2': True}


def test_submit_error_is_returned(make):
    node, _ = make()
    res = submit(node, 'outbound', 'A1', '')
    assert not res.success and 'A1 是空的' in res.message


def test_goto_failure_retries_once_then_pauses(make):
    node, path = make()
    node.goto = FakeGoto(fail={'A1'})
    submit(node, 'inbound', 'R1', 'A1')
    submit(node, 'outbound', 'A2', 'S1')
    node.step()
    assert node.goto.calls == ['R1', 'A1', 'A1']
    t = node.board.recent[-1]
    assert t.state == 'failed' and '去 A1 失败' in t.message and '货在车上' in t.message
    assert node.board.paused and node.step() is False                # 暂停时不执行下一单，也不回 HOME
    res = node.on_resume(Trigger.Request(), Trigger.Response())
    assert res.success
    node.goto = FakeGoto()
    node.step()
    assert node.goto.calls == ['A2', 'S1'] and node.board.recent[-1].state == 'done'


def test_cancel_active_during_goto(make):
    node, _ = make()
    node.goto = FakeGoto(hook=lambda name: name == 'A1' and node.on_cancel(
        srv.TaskId.Request(id=1), srv.TaskId.Response()))
    submit(node, 'inbound', 'R1', 'A1')
    node.step()
    t = node.board.recent[-1]
    assert t.state == 'canceled' and node.board.paused and node.board.reserved == set()


def test_cancel_during_dwell_stops_immediately(make):
    import threading
    import time
    node, _ = make(dwell=3600.0)                       # 不中断的话测试会卡一小时
    submit(node, 'inbound', 'R1', 'A1')
    # 到 R1 是瞬间完成的；0.3 s 后车正在装货停留，这时取消
    threading.Timer(0.3, lambda: node.on_cancel(srv.TaskId.Request(id=1), srv.TaskId.Response())).start()
    t0 = time.time()
    node.step()
    assert time.time() - t0 < 5.0
    assert node.goto.calls == ['R1'] and node.board.recent[-1].state == 'canceled'


def test_cancel_queued_and_unknown(make):
    node, _ = make()
    submit(node, 'inbound', 'R1', 'A1')
    res = node.on_cancel(srv.TaskId.Request(id=1), srv.TaskId.Response())
    assert res.success and node.board.queue == [] and not node.board.paused
    res = node.on_cancel(srv.TaskId.Request(id=7), srv.TaskId.Response())
    assert not res.success and '#7' in res.message


def test_returns_home_when_idle_only_once(make):
    node, _ = make(pose=(5.0, 0.0, 0.0))
    assert node.step() is True and node.goto.calls == ['HOME']
    assert node.step() is False and node.goto.calls == ['HOME']


def test_already_home_does_not_move(make):
    node, _ = make(pose=(0.5, 0.0, 0.0))
    node.step()
    assert node.goto.calls == []


def test_new_task_interrupts_going_home(make):
    node, _ = make(pose=(5.0, 0.0, 0.0))
    node.goto = FakeGoto(hook=lambda name: name == 'HOME' and submit(node, 'inbound', 'R1', 'A1'))
    node.step()                                            # 回家途中来了新任务：取消回家
    node.step()                                            # 下一步直接执行任务
    assert node.goto.calls == ['HOME', 'R1', 'A1']


def test_set_stock_and_list(make):
    node, path = make()
    res = node.on_set_stock(srv.SlotSet.Request(slot='A1', full=True), srv.SlotSet.Response())
    assert res.success and T.load(path, {'HOME', 'R1', 'S1', 'A1', 'A2'}).slots['A1'] is True
    res = node.on_set_stock(srv.SlotSet.Request(slot='R1', full=True), srv.SlotSet.Response())
    assert not res.success and '不是库位' in res.message
    res = node.on_list(Trigger.Request(), Trigger.Response())
    assert res.success and 'A1 有货' in res.message


def test_save_failure_keeps_running(make, tmp_path):
    node, path = make(folder='ro')
    submit(node, 'inbound', 'R1', 'A1')
    os.chmod(tmp_path / 'ro', stat.S_IRUSR | stat.S_IXUSR)    # 目录只读：保存会失败
    try:
        node.step()
    finally:
        os.chmod(tmp_path / 'ro', stat.S_IRWXU)
    assert node.board.recent[-1].state == 'done' and node.board.cfg.slots['A1'] is True
    assert T.load(path, {'HOME', 'R1', 'S1', 'A1', 'A2'}).slots['A1'] is False


def test_status_published_as_json(make):
    import json
    node, _ = make()
    submit(node, 'inbound', 'R1', 'A1')
    s = json.loads(node.last_status)
    assert s['queue'][0]['target'] == 'A1' and s['reserved'] == ['A1']


def test_unexpected_error_fails_task_and_pauses(make):
    node, _ = make()

    def boom(name):
        if name == 'A1':
            raise RuntimeError('意外')
    node.goto = FakeGoto(hook=boom)
    submit(node, 'inbound', 'R1', 'A1')
    node.step()                                            # 不能把异常抛给工作线程、让任务卡在执行中
    t = node.board.recent[-1]
    assert t.state == 'failed' and '意外' in t.message
    assert node.board.active is None and node.board.paused and node.board.reserved == set()


def test_status_compare_and_publish_hold_the_lock(make):
    node, _ = make()
    seen = []

    class Pub:
        def publish(self, msg):
            seen.append(node.lock._is_owned())             # 比较 last_status 和发布必须在同一段锁内
    node.status_pub = Pub()
    submit(node, 'inbound', 'R1', 'A1')
    assert seen and all(seen)


def test_cancel_after_unloading_dwell_still_cancels(make):
    node, path = make()
    submit(node, 'inbound', 'R1', 'A1')
    real_dwell = node.dwell

    def dwell():                                           # 取消恰好在卸货停留结束、库存更新之前到达
        real_dwell()
        if node.board.active.state == 'unloading':
            node.on_cancel(srv.TaskId.Request(id=1), srv.TaskId.Response())
    node.dwell = dwell
    node.step()
    t = node.board.recent[-1]
    assert t.state == 'canceled' and node.board.paused      # 与取消服务的回复（“之后队列暂停”）一致
    assert node.board.cfg.slots['A1'] is False


def test_go_home_skips_when_a_task_is_already_queued(make):
    node, _ = make(pose=(5.0, 0.0, 0.0))
    submit(node, 'inbound', 'R1', 'A1')                    # 判定空闲之后、发回家目标之前来了任务
    assert node.go_home() is True
    assert node.goto.calls == [] and node.home_done is False


def test_dwell_returns_when_node_shuts_down(make):
    import threading
    import time
    node, _ = make(dwell=3600.0)
    submit(node, 'inbound', 'R1', 'A1')
    threading.Timer(0.3, rclpy.shutdown).start()
    t0 = time.time()
    node.step()
    assert time.time() - t0 < 5.0
    assert node.board.recent[-1].state == 'canceled'

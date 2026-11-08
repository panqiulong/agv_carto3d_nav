import importlib.util
import os
import pathlib
import stat
import subprocess
from unittest import mock

import pytest
from launch import LaunchContext

LAUNCH = pathlib.Path(__file__).resolve().parent.parent / 'launch' / 'gazebo_sim.launch.py'


@pytest.fixture(scope='module')
def script():
    """launch 文件里相机检查实际执行的 bash 脚本"""
    spec = importlib.util.spec_from_file_location('gazebo_sim_launch', LAUNCH)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    processes = []

    def record(**kwargs):
        processes.append(kwargs)
        return mock.MagicMock()

    context = LaunchContext()
    context.launch_configurations['world'] = 'warehouse'
    with mock.patch.object(m, 'ExecuteProcess', record), mock.patch.object(m, 'RegisterEventHandler'), \
            mock.patch.object(m, 'OnProcessExit'), mock.patch.object(m, 'OnProcessStart'), mock.patch.object(m, 'Node'):
        m.launch_setup(context, pkg_gazebo='/tmp')
    return next(p['cmd'][2] for p in processes if p['cmd'][0] == 'bash')


def run(script, tmp_path, ready_after, display=':0'):
    """假的 ros2：第 ready_after 次查询起才有发布者（None 表示一直没有）；sleep 不真等。返回调用记录"""
    calls = tmp_path / 'calls'
    fake = tmp_path / 'ros2'
    fake.write_text(f'''#!/bin/bash
echo "$@" >> {calls}
n=$(grep -c "topic info" {calls})
if [ "$1" = topic ]; then
  if [ -n "{ready_after or ''}" ] && [ "$n" -ge "{ready_after or 0}" ]; then echo "Publisher count: 1"
  else echo "Publisher count: 0"; fi
fi
''')
    nap = tmp_path / 'sleep'
    nap.write_text('#!/bin/bash\n')
    for f in (fake, nap):
        f.chmod(f.stat().st_mode | stat.S_IEXEC)
    env = {**os.environ, 'PATH': f'{tmp_path}:{os.environ["PATH"]}', 'DISPLAY': display}
    out = subprocess.run(['bash', '-c', script], env=env, capture_output=True, text=True, timeout=20).stdout
    return (calls.read_text() if calls.exists() else ''), out


def test_camera_ready_at_first_check_is_left_alone(script, tmp_path):
    calls, _ = run(script, tmp_path, ready_after=1)
    assert 'delete_entity' not in calls and 'spawn_entity' not in calls


def test_one_empty_answer_does_not_delete_the_robot(script, tmp_path):
    # 只查一次时，一次没查到就删车重建；删车后渲染场景和模型对不上，No link visual 刷了 132 万条（2026-11-07）
    calls, _ = run(script, tmp_path, ready_after=5)
    assert 'delete_entity' not in calls and 'spawn_entity' not in calls


def test_camera_that_never_comes_up_is_respawned(script, tmp_path):
    calls, out = run(script, tmp_path, ready_after=None)
    assert calls.count('topic info') == 15          # 约 30 s
    assert 'delete_entity' in calls and 'spawn_entity' in calls
    assert 'Publisher count: 0' in out              # 失败时留下查询结果，才分得清真失败还是漏报


def test_no_display_skips_the_check(script, tmp_path):
    calls, _ = run(script, tmp_path, ready_after=None, display='')
    assert calls == ''

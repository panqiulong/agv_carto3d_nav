"""仿真启动时发布静态 TF world -> map（map 原点 = 出生点），RViz 才能把 /ground_truth（world 系）叠到地图上"""
import importlib.util
import pathlib
from unittest import mock

from launch import LaunchContext

LAUNCH = pathlib.Path(__file__).resolve().parent.parent / 'launch' / 'gazebo_sim.launch.py'


def test_world_to_map_static_tf_uses_spawn_pose():
    spec = importlib.util.spec_from_file_location('gazebo_sim_launch', LAUNCH)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    nodes = []

    def record(**kwargs):
        nodes.append(kwargs)
        return mock.MagicMock()

    context = LaunchContext()
    context.launch_configurations['world'] = 'warehouse'
    with mock.patch.object(m, 'Node', record), mock.patch.object(m, 'ExecuteProcess'), \
            mock.patch.object(m, 'RegisterEventHandler'), mock.patch.object(m, 'OnProcessExit'), \
            mock.patch.object(m, 'OnProcessStart'):
        m.launch_setup(context, pkg_gazebo='/tmp')
    tfs = [n for n in nodes if n.get('executable') == 'static_transform_publisher']
    assert len(tfs) == 1
    args = tfs[0]['arguments']
    x, y, yaw = m.WORLDS['warehouse']
    opt = dict(zip(args[::2], args[1::2]))
    assert (opt['--frame-id'], opt['--child-frame-id']) == ('world', 'map')
    assert (float(opt['--x']), float(opt['--y']), float(opt['--yaw'])) == (x, y, yaw)

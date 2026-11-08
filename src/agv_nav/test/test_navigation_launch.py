import importlib.util
import os

import yaml

HERE = os.path.dirname(__file__)


def load_launch():
    spec = importlib.util.spec_from_file_location('nav_launch', os.path.join(HERE, '..', 'launch', 'navigation.launch.py'))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def params():
    return yaml.safe_load(open(os.path.join(HERE, '..', 'config', 'nav2_params.yaml')))


def test_default_params_have_no_filters():
    p = params()
    assert 'filters' not in p['global_costmap']['global_costmap']['ros__parameters']
    assert 'filters' not in p['local_costmap']['local_costmap']['ros__parameters']


def test_add_costmap_filters_uses_configured_plugins():
    p = load_launch().add_costmap_filters(params())
    g = p['global_costmap']['global_costmap']['ros__parameters']
    loc = p['local_costmap']['local_costmap']['ros__parameters']
    assert g['filters'] == ['keepout_filter', 'speed_filter']
    assert loc['filters'] == ['keepout_filter']
    assert g['keepout_filter']['plugin'] == 'nav2_costmap_2d::KeepoutFilter'
    assert g['speed_filter']['plugin'] == 'nav2_costmap_2d::SpeedFilter'
    assert loc['keepout_filter']['filter_info_topic'] == '/keepout_filter_info'


def test_filter_nodes_types():
    masks = {'keepout_yaml': '/m/k.yaml', 'speed_yaml': '/m/s.yaml'}
    nodes = load_launch().filter_nodes(masks, True)
    names = [n._Node__node_name for n in nodes]      # node_name 属性要等 launch 执行后才能读
    assert names == ['keepout_mask_server', 'keepout_filter_info_server', 'speed_mask_server',
                     'speed_filter_info_server', 'lifecycle_manager_filters']


def test_semantic_speed_cap_matches_mppi_vx_max():
    import sys
    sys.path.insert(0, os.path.join(HERE, '..', '..', 'agv_semantic', 'scripts'))
    import semantic_map
    vx_max = params()['controller_server']['ros__parameters']['FollowPath']['vx_max']
    assert semantic_map.MAX_SPEED_LIMIT == vx_max


def test_task_manager_node():
    node = load_launch().task_manager_node('/m/w_tasks.yaml', '/m/w_semantic.yaml', True)
    assert node._Node__node_name == 'task_manager'      # 与 test_filter_nodes_types 相同的读法
    assert node._Node__node_executable == 'task_manager.py'
    # 参数在构造时被 launch_ros 规范化，不便在这里比较；参数是否传对由 Task 7 的仿真场景覆盖


def test_tasks_without_semantic_is_an_error():
    import pytest
    from launch import LaunchContext
    m = load_launch()
    ctx = LaunchContext()
    ctx.launch_configurations['tasks'] = '/m/w_tasks.yaml'
    with pytest.raises(RuntimeError, match='semantic'):
        m.require_semantic_for_tasks(ctx)
    ctx.launch_configurations['tasks'] = ''
    assert m.require_semantic_for_tasks(ctx) == []


def test_turning_radius_is_achievable_and_consistent():
    # 实测满舵后轴半径约 2.19 m（内侧前轮顶到 30° 关节限位，理论值 1.94 打不满）；
    # MPPI 的 vx / wz 是后轴半径。规划器 / 平滑器的路径点是 base_link（车身中心，在后轴前 0.432 m），
    # 它的半径 = hypot(后轴半径, 0.432)。规划半径小于车实际能转的半径时，终点前的倒车弧线转不过来，车会前后反复挪
    p = params()
    mppi = p['controller_server']['ros__parameters']['FollowPath']['AckermannConstraints']['min_turning_r']
    planner = p['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius']
    smoother = p['smoother_server']['ros__parameters']['constrained_smoother']['minimum_turning_radius']
    assert mppi >= 2.19
    assert planner >= (mppi ** 2 + 0.432 ** 2) ** 0.5
    assert smoother == planner


def test_mppi_does_not_penalize_reversing():
    # PreferForwardCritic 罚所有倒车速度，路径要求倒车时 MPPI 每帧在前进 / 倒车间翻转（实测 nav 指令 +0.4 / -0.3 交替）
    critics = params()['controller_server']['ros__parameters']['FollowPath']['critics']
    assert 'PreferForwardCritic' not in critics


def test_planner_penalizes_direction_changes():
    # change_penalty 为 0 时换向不计代价，加上 reverse_penalty 偏高，规划器会把倒车拆成一串 0.2 m 的碎段
    assert params()['planner_server']['ros__parameters']['GridBased']['change_penalty'] > 0

TEB_FILE = os.path.join(HERE, '..', 'config', 'teb_params.yaml')


def test_select_controller_mppi_keeps_params():
    p = load_launch().select_controller(params(), 'mppi', TEB_FILE)
    assert p['controller_server']['ros__parameters']['FollowPath']['plugin'] == 'nav2_mppi_controller::MPPIController'


def test_select_controller_teb_replaces_follow_path():
    p = load_launch().select_controller(params(), 'teb', TEB_FILE)
    teb = p['controller_server']['ros__parameters']['FollowPath']
    assert teb['plugin'] == 'teb_local_planner::TebLocalPlannerROS'
    assert 'vx_std' not in teb                      # 整段替换，不残留 MPPI 参数


def test_select_controller_rejects_unknown():
    import pytest
    with pytest.raises(RuntimeError):
        load_launch().select_controller(params(), 'dwb', TEB_FILE)


def test_teb_matches_vehicle_limits():
    # TEB 的 v / omega 与 MPPI 的 vx / wz 同为后轴半径；速度上限要与语义限速上限（= MPPI vx_max）一致
    mppi = params()['controller_server']['ros__parameters']['FollowPath']
    teb = yaml.safe_load(open(TEB_FILE))['FollowPath']
    local = params()['local_costmap']['local_costmap']['ros__parameters']
    assert teb['min_turning_radius'] == mppi['AckermannConstraints']['min_turning_r']
    assert teb['max_vel_x'] == mppi['vx_max']
    assert teb['max_vel_x_backwards'] == -mppi['vx_min']
    assert teb['footprint_model']['vertices'] == local['footprint']


def test_teb_param_types_match_teb_source():
    # TEB 按声明类型读参数，类型不对 controller_server 直接起不来（实测 teb_autosize 写成 true）
    import re
    import pytest
    root = os.path.join(HERE, '..', '..', 'third_party', 'teb_local_planner', 'teb_local_planner')
    if not os.path.isdir(root):
        pytest.skip('没有 TEB 源码')
    src = open(os.path.join(root, 'src', 'teb_config.cpp')).read()
    hdr = open(os.path.join(root, 'include', 'teb_local_planner', 'teb_config.h')).read()
    decl = dict(re.findall(r'name \+ "\." \+ "([a-z_.]+)", rclcpp::ParameterValue\(([a-z_.]+)\)', src))
    want = {'double': float, 'int': int, 'bool': bool, 'std::string': str}
    for k, v in yaml.safe_load(open(TEB_FILE))['FollowPath'].items():
        if k not in decl:
            continue
        m = re.search(r'\b(double|int|bool|std::string)\s+%s\s*;' % decl[k].split('.')[-1], hdr)
        if m:
            t = want[m.group(1)]
            assert type(v) is t, '%s = %r，TEB 要 %s' % (k, v, m.group(1))


def test_teb_look_ahead_needs_cusp_patch():
    # 前瞻窗口跨过换向点时前进和倒车的位移互相抵消，换向机动用时变成 3~5 倍；只有打了本地补丁才能用大于 1 的值
    import pytest
    src = os.path.join(HERE, '..', '..', 'third_party', 'teb_local_planner', 'teb_local_planner',
                       'src', 'optimal_planner.cpp')
    if not os.path.isfile(src):
        pytest.skip('没有 TEB 源码')
    if yaml.safe_load(open(TEB_FILE))['FollowPath'].get('control_look_ahead_poses', 1) > 1:
        assert '前瞻窗口不跨换向点' in open(src, encoding='utf-8').read()


def test_hdmap_off_by_default_and_included_when_given():
    from launch import LaunchContext
    from launch.actions import IncludeLaunchDescription
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    m = load_launch()
    ctx = LaunchContext()
    ctx.launch_configurations.update({'hdmap': '', 'use_sim_time': 'true'})
    assert m.hdmap_setup(ctx) == []                  # 不给 hdmap：和以前完全一样
    ctx.launch_configurations['hdmap'] = '/m/wh_lanelet2_map.osm'
    (inc,) = m.hdmap_setup(ctx)
    assert isinstance(inc, IncludeLaunchDescription)
    src = inc.launch_description_source
    src.get_launch_description(ctx)                  # 展开路径（装好的 agv_hdmap 里的文件要能加载）
    assert src.location.endswith(os.path.join('agv_hdmap', 'launch', 'hdmap.launch.py'))
    sub = lambda x: perform_substitutions(ctx, normalize_to_list_of_substitutions(x))
    args = {sub(k): sub(v) for k, v in inc.launch_arguments}
    assert args['hdmap'] == '/m/wh_lanelet2_map.osm'


def launch_text(name):
    return open(os.path.join(HERE, '..', 'launch', name)).read()


def test_cartographer_gets_raw_wheel_odometry_not_ekf_output():
    """EKF 已经用了 IMU，Cartographer 又直接订阅 /imu，接 EKF 输出等于同一份数据用两次"""
    for name in ('mapping.launch.py', 'navigation.launch.py'):
        text = launch_text(name)
        assert "('odom', '/wheel/odom')" in text
        assert "('odom', '/odom')" not in text
        assert 'odometry.launch.py' in text


def test_ekf_fuses_velocities_and_tilt_only():
    p = yaml.safe_load(open(os.path.join(HERE, '..', 'config', 'ekf.yaml')))['ekf_filter_node']['ros__parameters']
    assert p['two_d_mode'] is False                     # 场地有坡道
    assert p['odom0'] == '/wheel/odom' and p['imu0'] == '/imu/filtered'
    wheel, imu = p['odom0_config'], p['imu0_config']
    assert not any(wheel[:6])                           # 轮速的积分位姿会漂，只用速度
    assert wheel[6:9] == [True, True, True]             # base_link 不在后轴上，vy 不能当 0
    assert imu[3:6] == [True, True, False]              # 没有磁力计，不用 IMU 的绝对航向
    assert imu[9:12] == [True, True, True]


def test_both_costmaps_observe_front_and_rear_lidar():
    # 两路发到同一话题不行：observation_persistence 为 0 时每个观测源只留最新一帧，前后障碍会互相顶掉
    for costmap in ('local_costmap', 'global_costmap'):
        layer = params()[costmap][costmap]['ros__parameters']['obstacle_layer']
        assert layer['observation_sources'].split() == ['terrain', 'terrain_ground', 'terrain_rear', 'terrain_ground_rear']
        for front, rear in (('terrain', 'terrain_rear'), ('terrain_ground', 'terrain_ground_rear')):
            assert layer[rear]['topic'] == layer[front]['topic'] + '_rear'
            assert {k: v for k, v in layer[rear].items() if k != 'topic'} == {k: v for k, v in layer[front].items() if k != 'topic'}


def test_rear_terrain_filter_publishes_its_own_topics():
    text = launch_text('navigation.launch.py')
    assert "'cloud_topic': '/livox/lidar_rear_PointCloud2'" in text
    assert "('terrain_obstacles', '/terrain_obstacles_rear')" in text
    assert "('terrain_ground', '/terrain_ground_rear')" in text
    assert 'terrain_filter_rear_node,' in text


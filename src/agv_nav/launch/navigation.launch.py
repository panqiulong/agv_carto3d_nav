"""Cartographer 3D 纯定位 + Nav2（Smac Hybrid-A* 全局规划，局部控制 controller:=mppi|teb）。
先启动仿真（agv_gazebo gazebo_sim.launch.py），再启动本文件。"""
import os
import sys
import tempfile

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

CONTROLLERS = ('mppi', 'teb')


def select_controller(params, controller, teb_file):
    """mppi 用 nav2_params.yaml 里原有的 FollowPath；teb 用 teb_params.yaml 的 FollowPath 整段替换（插件名不变，行为树不用改）"""
    if controller not in CONTROLLERS:
        raise RuntimeError('controller 只能是 %s，收到 %r' % (' / '.join(CONTROLLERS), controller))
    if controller == 'teb':
        with open(teb_file) as f:
            params['controller_server']['ros__parameters']['FollowPath'] = yaml.safe_load(f)['FollowPath']
    return params


def nav2_params(context, pkg_dir):
    """读 params_file，注入行为树绝对路径（不能写死在 yaml 里），按 controller 换局部控制器"""
    with open(LaunchConfiguration('params_file').perform(context)) as f:
        params = yaml.safe_load(f)
    params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = os.path.join(
        pkg_dir, 'behavior_trees', 'navigate_to_pose_ackermann.xml')
    controller = LaunchConfiguration('controller').perform(context)
    if controller == 'teb':
        get_package_share_directory('teb_local_planner')    # 没编译 TEB 时在这里就报错，而不是 controller_server 起不来
    return select_controller(params, controller, os.path.join(pkg_dir, 'config', 'teb_params.yaml'))


def include_nav2(params, nav2_bringup_dir, use_sim_time):
    tmp = tempfile.NamedTemporaryFile('w', suffix='_nav2_params.yaml', delete=False)
    yaml.safe_dump(params, tmp)
    tmp.close()
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time, 'params_file': tmp.name, 'autostart': 'true'}.items())


def plain_setup(context, pkg_dir, nav2_bringup_dir):
    """不用语义地图时：只启动 Nav2"""
    return [include_nav2(nav2_params(context, pkg_dir), nav2_bringup_dir, LaunchConfiguration('use_sim_time'))]


def add_costmap_filters(params):
    """把语义地图的禁行 / 限速过滤器加进代价地图（参数段已在 nav2_params.yaml 里）"""
    params['global_costmap']['global_costmap']['ros__parameters']['filters'] = ['keepout_filter', 'speed_filter']
    params['local_costmap']['local_costmap']['ros__parameters']['filters'] = ['keepout_filter']
    return params


def filter_nodes(masks, use_sim_time):
    """掩码地图服务 + 过滤器信息服务。限速类型 2 = SPEED_FILTER_ABSOLUTE，限速 = 掩码值 x 0.01 m/s"""
    common = {'use_sim_time': use_sim_time}

    def node(executable, name, extra):
        return Node(package='nav2_map_server', executable=executable, name=name, output='screen',
                    parameters=[common, extra])
    return [
        node('map_server', 'keepout_mask_server',
             {'yaml_filename': masks['keepout_yaml'], 'topic_name': '/keepout_filter_mask', 'frame_id': 'map'}),
        node('costmap_filter_info_server', 'keepout_filter_info_server',
             {'type': 0, 'filter_info_topic': '/keepout_filter_info', 'mask_topic': '/keepout_filter_mask',
              'base': 0.0, 'multiplier': 1.0}),
        node('map_server', 'speed_mask_server',
             {'yaml_filename': masks['speed_yaml'], 'topic_name': '/speed_filter_mask', 'frame_id': 'map'}),
        node('costmap_filter_info_server', 'speed_filter_info_server',
             {'type': 2, 'filter_info_topic': '/speed_filter_info', 'mask_topic': '/speed_filter_mask',
              'base': 0.0, 'multiplier': 0.01}),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_filters',
             output='screen', parameters=[common, {'autostart': True, 'node_names': [
                 'keepout_mask_server', 'keepout_filter_info_server', 'speed_mask_server', 'speed_filter_info_server']}]),
    ]


def task_manager_node(tasks, semantic, use_sim_time):
    return Node(package='agv_semantic', executable='task_manager.py', name='task_manager', output='screen',
                parameters=[{'use_sim_time': use_sim_time, 'tasks_file': tasks, 'semantic_file': semantic}])


def require_semantic_for_tasks(context):
    """任务要按地点名导航，离不开 semantic_server"""
    if LaunchConfiguration('tasks').perform(context):
        raise RuntimeError('给了 tasks 必须同时给 semantic（任务按语义地点导航）')
    return []


def semantic_setup(context, pkg_dir, nav2_bringup_dir):
    """给了 semantic 时：semantic_server；掩码存在就加过滤器；Nav2 用加了 filters 的参数启动"""
    semantic = os.path.abspath(os.path.expanduser(LaunchConfiguration('semantic').perform(context)))
    use_sim_time = LaunchConfiguration('use_sim_time')
    sys.path.insert(0, os.path.join(get_package_prefix('agv_semantic'), 'lib', 'agv_semantic'))
    import semantic_map
    actions = [Node(package='agv_semantic', executable='semantic_server.py', name='semantic_server',
                    output='screen', parameters=[{'use_sim_time': use_sim_time, 'semantic_file': semantic}])]
    tasks = LaunchConfiguration('tasks').perform(context)
    if tasks:
        actions.append(task_manager_node(os.path.abspath(os.path.expanduser(tasks)), semantic, use_sim_time))
    params = nav2_params(context, pkg_dir)
    masks = semantic_map.mask_paths(semantic)
    if os.path.exists(masks['keepout_yaml']) and os.path.exists(masks['speed_yaml']):
        add_costmap_filters(params)
        actions += filter_nodes(masks, use_sim_time)
    else:
        actions.append(LogInfo(msg='没有找到区域掩码 %s，禁行和限速不生效；先运行 make_filter_masks.py'
                               % masks['keepout_yaml']))
    actions.append(include_nav2(params, nav2_bringup_dir, use_sim_time))
    return actions


def hdmap_setup(context):
    """给了 hdmap（.osm）时带上 agv_hdmap 的地图加载和显示；不给时什么都不加（也不要求编译了 agv_hdmap）"""
    osm = LaunchConfiguration('hdmap').perform(context)
    if not osm:
        return []
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('agv_hdmap'), 'launch', 'hdmap.launch.py')),
        launch_arguments={'hdmap': osm, 'use_sim_time': LaunchConfiguration('use_sim_time').perform(context)}.items())]


def generate_launch_description():
    pkg_dir = get_package_share_directory('agv_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    pbstream = LaunchConfiguration('pbstream')
    map_yaml = LaunchConfiguration('map')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', os.path.join(pkg_dir, 'config'),
            '-configuration_basename', 'carto_3d_localization.lua',
            '-load_state_filename', pbstream,
            '-load_frozen_state', 'true',
            # 定位轨迹由下面的 start_localization 带初始位姿启动
            '-start_trajectory_with_default_topics=false',      # gflags 布尔值必须写成 =false
        ],
        remappings=[
            ('points2_1', '/livox/lidar_PointCloud2'),
            ('points2_2', '/livox/lidar_rear_PointCloud2'),
            ('imu', '/imu'),
            ('odom', '/wheel/odom'),
        ],
    )

    # 不给初始位姿时，新轨迹要等全局匹配（抽样少、门槛高）才和地图挂上：工业场景实测 5 分钟匹配不上，
    # 只靠局部 SLAM 推算，之后还会误匹配。给定初始位姿后走 1.5 m 窗口的局部匹配。
    # 初始位姿相对于建图轨迹的第一个节点（即建图起点处的 imu_link，map 原点就在这里），默认 0 = 建图起点
    x, y, z = LaunchConfiguration('initial_x'), LaunchConfiguration('initial_y'), LaunchConfiguration('initial_z')
    yaw = LaunchConfiguration('initial_yaw')
    start_localization = ExecuteProcess(
        cmd=['bash', '-c', [
            # 等 cartographer 的服务最多 60 s；服务调用没返回 Success 就报错退出，免得 Nav2 在没有定位的情况下照常跑
            'for i in $(seq 60); do ros2 service list | grep -q "^/start_trajectory$" && break; sleep 1; done; '
            'ros2 service call /start_trajectory cartographer_ros_msgs/srv/StartTrajectory "{'
            "configuration_directory: '", os.path.join(pkg_dir, 'config'), "', "
            "configuration_basename: 'carto_3d_localization.lua', use_initial_pose: true, "
            'relative_to_trajectory_id: 0, initial_pose: {position: {x: ', x, ', y: ', y, ', z: ', z, '}, '
            'orientation: {z: ', PythonExpression(['__import__("math").sin((', yaw, ') / 2)']),
            ', w: ', PythonExpression(['__import__("math").cos((', yaw, ') / 2)']), '}}}" | tee /dev/stderr '
            '| grep -q "Success" || { echo "start_localization FAILED: no localization trajectory" >&2; exit 1; }']],
        output='screen',
    )

    # RViz 的 2D Pose Estimate（/initialpose）-> 结束当前定位轨迹、用点选的位姿重开一条：开机不在建图起点、或定位丢了时用
    initial_pose_node = Node(
        package='agv_nav',
        executable='initial_pose.py',
        name='initial_pose',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'configuration_directory': os.path.join(pkg_dir, 'config'),
                     'configuration_basename': 'carto_3d_localization.lua'}],
    )

    # 建图时由 cloud_to_grid_map 生成、map_saver 保存的 2D 栅格，与 pbstream 同一个 map 坐标系
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}],
    )

    map_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'node_names': ['map_server']}],
    )

    # 实时障碍：每帧点云在单帧内部区分地面和障碍（坡面不算障碍），只输出障碍点
    terrain_filter_node = Node(
        package='agv_nav',
        executable='terrain_obstacle_filter.py',
        name='terrain_obstacle_filter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 后雷达单独一个实例、单独的话题：观测源只保留最新一帧，两路发到同一话题时前后障碍会互相顶掉
    terrain_filter_rear_node = Node(
        package='agv_nav',
        executable='terrain_obstacle_filter.py',
        name='terrain_obstacle_filter_rear',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'cloud_topic': '/livox/lidar_rear_PointCloud2'}],
        remappings=[('terrain_obstacles', '/terrain_obstacles_rear'), ('terrain_ground', '/terrain_ground_rear')],
    )

    cmd_vel_node = Node(
        package='agv_nav',
        executable='cmd_vel_to_ackermann.py',
        name='cmd_vel_to_ackermann',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    nav2 = OpaqueFunction(
        function=plain_setup, kwargs={'pkg_dir': pkg_dir, 'nav2_bringup_dir': nav2_bringup_dir},
        condition=LaunchConfigurationEquals('semantic', ''))

    # 给了语义文件时由 semantic_setup 启动 Nav2（参数里加了过滤器）和语义节点
    semantic_actions = OpaqueFunction(
        function=semantic_setup, kwargs={'pkg_dir': pkg_dir, 'nav2_bringup_dir': nav2_bringup_dir},
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('semantic'), "' != ''"])))

    tasks_check = OpaqueFunction(function=require_semantic_for_tasks,
                                 condition=LaunchConfigurationEquals('semantic', ''))

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'agv.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        # pip 装的 numpy 自带 OpenBLAS，默认每个进程开几十个线程忙等：实测 terrain_obstacle_filter 一个节点占 17 个核，
        # 把仿真和 MPPI 都拖慢。这些节点只做小矩阵运算，单线程足够
        SetEnvironmentVariable('OPENBLAS_NUM_THREADS', '1'),
        SetEnvironmentVariable('OMP_NUM_THREADS', '1'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('pbstream', description='Cartographer 地图，如 maps/wh/wh.pbstream'),
        DeclareLaunchArgument('map', description='Nav2 栅格地图，如 maps/wh/wh.yaml'),
        DeclareLaunchArgument('use_ekf', default_value='true',
                              description='false = 不启动融合里程计，/odom 和 TF odom->base_link 要另外提供（对比测试用）'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('controller', default_value='mppi', description='局部控制器：mppi | teb'),
        DeclareLaunchArgument('initial_x', default_value='0.0', description='定位初始位姿（map 系，默认建图起点）'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_z', default_value='0.0', description='在平台上启动时填平台高度'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0', description='弧度'),
        DeclareLaunchArgument('semantic', default_value='',
                              description='语义文件（地点与区域），如 maps/map_semantic.yaml；空 = 不用语义地图'),
        DeclareLaunchArgument('tasks', default_value='',
                              description='任务配置与库存（出入库任务），如 maps/map_tasks.yaml；需要同时给 semantic'),
        DeclareLaunchArgument('hdmap', default_value='',
                              description='公路的 Lanelet2 高精地图（.osm），如 maps/wh/wh_lanelet2_map.osm；空 = 不用'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'odometry.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(LaunchConfiguration('use_ekf'))),
        cartographer_node,
        start_localization,
        initial_pose_node,
        map_server_node,
        map_lifecycle_node,
        terrain_filter_node,
        terrain_filter_rear_node,
        cmd_vel_node,
        nav2,
        semantic_actions,
        tasks_check,
        OpaqueFunction(function=hdmap_setup),
        rviz_node,
    ])

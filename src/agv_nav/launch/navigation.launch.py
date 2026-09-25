"""Cartographer 3D 纯定位 + Nav2（Smac Hybrid-A* 全局规划，MPPI 局部控制）。
先启动仿真（agv_urdf gazebo_sim.launch.py），再启动本文件。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_dir = get_package_share_directory('agv_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    pbstream = LaunchConfiguration('pbstream')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

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
        ],
        remappings=[
            ('points2', '/livox/lidar_PointCloud2'),
            ('imu', '/imu'),
            ('odom', '/odom'),
        ],
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

    # 从 3D 点云按车体高度切一层 2D 激光给代价地图。base_link 离地约 0.12 m，
    # min_height=-0.02 即离地 10 cm 以上：雷达前倾，车前地面点极多，每个角度取最近点时
    # 地面噪声离群点会变成假障碍，离地 5 cm 不够。range_min 滤掉雷达自身附近的无效点
    pointcloud_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': 'base_link',
            'transform_tolerance': 0.05,
            'min_height': -0.02,
            'max_height': 1.2,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.45,
            'range_max': 20.0,
            'use_inf': True,
        }],
        remappings=[
            ('cloud_in', '/livox/lidar_PointCloud2'),
            ('scan', '/scan'),
        ],
    )

    cmd_vel_node = Node(
        package='agv_nav',
        executable='cmd_vel_to_ackermann.py',
        name='cmd_vel_to_ackermann',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 行为树文件需要绝对路径，不能写死在 yaml 里，这里注入
    nav2_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'default_nav_to_pose_bt_xml': os.path.join(
            pkg_dir, 'behavior_trees', 'navigate_to_pose_ackermann.xml')},
        convert_types=True,
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'autostart': 'true',
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('pbstream', default_value=os.path.join(pkg_dir, 'maps', 'map.pbstream')),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg_dir, 'maps', 'map.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg_dir, 'params', 'nav2_params.yaml')),
        cartographer_node,
        map_server_node,
        map_lifecycle_node,
        pointcloud_to_scan_node,
        cmd_vel_node,
        nav2,
        rviz_node,
    ])

"""Cartographer 3D 建图。先启动仿真（agv_urdf gazebo_sim.launch.py），再启动本文件，
用键盘遥控（/cmd_vel）绕场景走一圈，最后 `ros2 run agv_nav save_map.sh <输出前缀>` 保存。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('agv_nav')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', os.path.join(pkg_dir, 'config'),
            '-configuration_basename', 'carto_3d_mapping.lua',
        ],
        remappings=[
            ('points2', '/livox/lidar_PointCloud2'),
            ('imu', '/imu'),
            ('odom', '/odom'),
        ],
    )

    # 3D 点云 -> 2D 栅格 /map，给 Nav2 全局规划用
    grid_map_node = Node(
        package='agv_nav',
        executable='cloud_to_grid_map.py',
        name='cloud_to_grid_map',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    cmd_vel_node = Node(
        package='agv_nav',
        executable='cmd_vel_to_ackermann.py',
        name='cmd_vel_to_ackermann',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', os.path.join(
            get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        cartographer_node,
        grid_map_node,
        cmd_vel_node,
        rviz_node,
    ])

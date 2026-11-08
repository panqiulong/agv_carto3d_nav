"""融合里程计：Madgwick 从 /imu 解算姿态，EKF 融合轮速和 IMU，发布 /odom 和 TF odom->base_link。
Nav2 / TEB 读 /odom；Cartographer 读原始的 /wheel/odom（EKF 已经用了 IMU，不能再喂给直接订阅 /imu 的 Cartographer）。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'use_mag': False, 'publish_tf': False,
                         'world_frame': 'enu', 'orientation_stddev': 0.02}],
            remappings=[('imu/data_raw', '/imu'), ('imu/data', '/imu/filtered')],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('agv_nav'), 'config', 'ekf.yaml'),
                        {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', '/odom')],
        ),
    ])

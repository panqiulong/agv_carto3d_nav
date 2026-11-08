"""高精地图：Autoware 的地图投影加载器、Lanelet2 地图加载器和地图显示。

    ros2 launch agv_hdmap hdmap.launch.py hdmap:=<.osm>

一般不单独启动，由 agv_nav navigation.launch.py hdmap:=<.osm> 带起来。
另外起 hdmap_navigator（/hdmap/goal → 沿车道开）。Autoware 自带的 lanelet2_map_visualizer.launch.xml 里写的可执行名和装好的不一致，这里直接起节点。
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def projector_path(osm):
    """地图旁边的投影文件：start.sh 复制过去的 <前缀>_map_projector_info.yaml，否则同目录的 map_projector_info.yaml
    （agv_gazebo/worlds 里生成的名字）"""
    d, name = os.path.split(osm)
    prefixed = os.path.join(d, name.replace('lanelet2_map.osm', 'map_projector_info.yaml'))
    return prefixed if name.endswith('lanelet2_map.osm') and os.path.isfile(prefixed) \
        else os.path.join(d, 'map_projector_info.yaml')


def hdmap_nodes(context):
    osm = os.path.abspath(os.path.expanduser(LaunchConfiguration('hdmap').perform(context)))
    sim = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    return [
        # 发 /map/map_projector_info（Local：直接用点的 local_x / local_y）
        Node(package='autoware_map_projection_loader', executable='autoware_map_projection_loader_node',
             name='map_projection_loader', output='screen',
             parameters=[{'use_sim_time': sim, 'map_projector_info_path': projector_path(osm),
                          'lanelet2_map_path': osm}]),
        # 收到投影信息后读 .osm，发 /map/vector_map（autoware_map_msgs/LaneletMapBin，transient_local）
        Node(package='autoware_map_loader', executable='autoware_lanelet2_map_loader', name='lanelet2_map_loader',
             output='screen', remappings=[('output/lanelet2_map', '/map/vector_map')],
             parameters=[{'use_sim_time': sim, 'lanelet2_map_path': osm, 'allow_unsupported_version': True,
                          'center_line_resolution': 5.0, 'use_waypoints': True}]),
        # 把地图画成 MarkerArray：车道、边界线、停止线、红绿灯
        Node(package='autoware_lanelet2_map_visualizer', executable='autoware_lanelet2_map_visualizer',
             name='lanelet2_map_visualization', output='screen',
             remappings=[('input/lanelet2_map', '/map/vector_map'),
                         ('output/lanelet2_map_marker', '/map/vector_map_marker')],
             parameters=[{'use_sim_time': sim}]),
        # 按车道路由、生成路径，交给 Nav2 的 follow_path
        Node(package='agv_hdmap', executable='hdmap_navigator.py', name='hdmap_navigator', output='screen',
             parameters=[os.path.join(get_package_share_directory('agv_hdmap'), 'config', 'hdmap_params.yaml'),
                         {'use_sim_time': sim, 'hdmap_file': osm}]),
        # 仿真红绿灯（真车上换成感知）
        Node(package='agv_hdmap', executable='traffic_light_sim.py', name='traffic_light_sim', output='screen',
             parameters=[os.path.join(get_package_share_directory('agv_hdmap'), 'config', 'hdmap_params.yaml'),
                         {'use_sim_time': sim, 'hdmap_file': osm}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hdmap', description='Lanelet2 地图（.osm），如 maps/wh/wh_lanelet2_map.osm'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=hdmap_nodes),
    ])

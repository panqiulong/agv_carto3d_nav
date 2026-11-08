import os

import yaml

HERE = os.path.dirname(__file__)
PKG = os.path.join(HERE, '..')


def displays(items):
    """展开 Group，返回所有显示项"""
    for d in items:
        yield d
        yield from displays(d.get('Displays', []))


def enabled_topics():
    cfg = yaml.safe_load(open(os.path.join(PKG, 'rviz', 'agv.rviz')))
    out = set()
    for d in displays(cfg['Visualization Manager']['Displays']):
        t = d.get('Topic')
        if d.get('Enabled') and isinstance(t, dict):
            out.add(t['Value'])
    return out


def test_rviz_shows_lidar_obstacles_semantic_and_elevator():
    assert {'/livox/lidar_PointCloud2', '/livox/lidar_rear_PointCloud2', '/terrain_obstacles', '/semantic_markers',
            '/elevator/pose', '/elevator/walls', '/map', '/ground_truth'} <= enabled_topics()


def test_rviz_shows_hd_map():
    assert {'/map/vector_map_marker', '/hdmap/path', '/hdmap/route_markers', '/hdmap/traffic_light_markers'} <= enabled_topics()


def test_rviz_has_hd_map_goal_tool():
    cfg = yaml.safe_load(open(os.path.join(PKG, 'rviz', 'agv.rviz')))
    tools = [t for t in cfg['Visualization Manager']['Tools'] if t['Class'] == 'rviz_default_plugins/SetGoal']
    assert [(t['Topic']['Value'], t.get('Name')) for t in tools] == [('/hdmap/goal', 'HD Map Goal')]


def test_rviz_drops_nav2_demo_robot_topics():
    # Nav2 默认配置里这些是示例机器人（Turtlebot）的话题，我们的车没有
    assert not {'/scan', '/mobile_base/sensors/bumper_pointcloud', '/particle_cloud'} & enabled_topics()


def test_mapping_and_navigation_launch_use_agv_rviz():
    for name in ('mapping.launch.py', 'navigation.launch.py'):
        src = open(os.path.join(PKG, 'launch', name)).read()
        assert "'rviz', 'agv.rviz'" in src, name
        assert 'nav2_default_view.rviz' not in src, name

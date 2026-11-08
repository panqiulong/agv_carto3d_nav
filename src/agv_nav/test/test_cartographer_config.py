import os
import re

HERE = os.path.dirname(__file__)


def lua(name):
    return open(os.path.join(HERE, '..', 'config', name)).read()


def setting(text, key):
    m = re.search(r'^%s\s*=\s*([^\s-]+)' % re.escape(key), text, re.M)
    return float(m.group(1)) if m else None


def test_localization_never_drops_to_global_search():
    # 定位轨迹开头 10 s 没和冻结地图建立约束，Cartographer 就只做全局搜索、再也不做局部匹配；全局门槛 0.7 实测到不了，
    # 全局搜索又把约束队列堆到几千条，CPU 占满（2026-11-01：开一圈 0 条约束）。启动时总给初始位姿，不需要全局重定位
    t = lua('carto_3d_localization.lua')
    assert setting(t, 'POSE_GRAPH.global_constraint_search_after_n_seconds') >= 1e5


def test_cartographer_merges_front_and_rear_lidar():
    # 两路点云没有逐点时间，RangeDataCollator 每次只放出一颗雷达的一帧，所以一个节点要累积 2 次才同时含前后两颗；
    # 不累积的话节点在前后视野之间来回切换。定位配置 include 建图配置，一起生效
    t = lua('carto_3d_mapping.lua')
    assert re.search(r'^\s*num_point_clouds\s*=\s*2,', t, re.M)
    assert setting(t, 'TRAJECTORY_BUILDER_3D.num_accumulated_range_data') == 2
    assert 'carto_3d_mapping.lua' in lua('carto_3d_localization.lua')


def test_both_launch_files_feed_both_lidars_to_cartographer():
    for name in ('mapping.launch.py', 'navigation.launch.py'):
        t = open(os.path.join(HERE, '..', 'launch', name)).read()
        assert "('points2_1', '/livox/lidar_PointCloud2')" in t
        assert "('points2_2', '/livox/lidar_rear_PointCloud2')" in t

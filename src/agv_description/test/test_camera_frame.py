import math
import pathlib
import xml.etree.ElementTree as ET

import pytest
import xacro

URDF = pathlib.Path(__file__).resolve().parent.parent / 'urdf' / 'r2.xacro'


@pytest.fixture(scope='module')
def robot():
    return ET.fromstring(xacro.process_file(str(URDF)).toxml())


def test_camera_has_optical_frame(robot):
    # ROS 相机图像约定光学坐标系（z 朝前、x 朝右、y 朝下）；RViz Camera 显示、深度点云都按这个约定解释
    j = next(j for j in robot.iter('joint') if j.find('child').get('link') == 'camera_optical_frame')
    assert j.get('type') == 'fixed' and j.find('parent').get('link') == 'camera_Link'
    assert [float(v) for v in j.find('origin').get('rpy').split()] == pytest.approx([-math.pi / 2, 0, -math.pi / 2])


def test_camera_plugin_publishes_in_optical_frame(robot):
    plugin = next(p for p in robot.iter('plugin') if p.get('name') == 'camera_controller')
    assert plugin.find('frame_name').text.strip() == 'camera_optical_frame'


def test_lidar_mounted_above_the_body(robot):
    # 雷达前倾 47°。装在车顶上 0.25 m 时，正下方的车身挡掉约 3 成射线（实测每帧 40000 条里 11500 条打在 0.1~0.45 m），
    # 两侧以外几乎看不到结构，Cartographer 定位和地图的匹配得分一直到不了门槛；抬高 0.2 m 后车身遮挡为 0
    j = next(j for j in robot.iter('joint') if j.find('child').get('link') == 'lidar_Link')
    xyz = [float(v) for v in j.find('origin').get('xyz').split()]
    assert xyz[2] >= 0.45


def test_lidar_mast_has_no_collision(robot):
    # 支架只有外观：有碰撞体的话，前倾雷达朝下的射线又会打在它上面
    mast = next(l for l in robot.iter('link') if l.get('name') == 'lidar_mast_Link')
    assert mast.find('visual') is not None and mast.find('collision') is None
    assert mast.find('inertial') is not None          # 没有惯量的 fixed link 在转成 SDF 时会被丢掉


def test_rear_lidar_mirrors_front_lidar(robot):
    # 后雷达和前雷达关于车身中心对称：同样架高、同样倾角，但朝后，才能看到车尾的地面
    def origin(child):
        o = next(j for j in robot.iter('joint') if j.find('child').get('link') == child).find('origin')
        return [float(v) for v in o.get('xyz').split()], [float(v) for v in o.get('rpy').split()]
    front_xyz, front_rpy = origin('lidar_Link')
    rear_xyz, rear_rpy = origin('lidar_rear_Link')
    assert rear_xyz == pytest.approx([-front_xyz[0], 0, front_xyz[2]])
    assert rear_rpy == pytest.approx([0, front_rpy[1], math.pi])


def test_rear_lidar_has_its_own_sensor_and_topic(robot):
    plugins = {p.get('name'): p for p in robot.iter('plugin') if p.get('filename') == 'libros2_livox.so'}
    assert plugins['livox_sensor_plugin'].find('topic').text == 'livox/lidar'
    assert plugins['livox_sensor_rear_plugin'].find('topic').text == 'livox/lidar_rear'


def test_rear_lidar_mast_has_no_collision(robot):
    mast = next(l for l in robot.iter('link') if l.get('name') == 'lidar_rear_mast_Link')
    assert mast.find('visual') is not None and mast.find('collision') is None
    assert mast.find('inertial') is not None

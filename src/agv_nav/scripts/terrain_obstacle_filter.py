#!/usr/bin/env python3
"""导航时的实时障碍：每帧点云在单帧内部区分地面和障碍，只输出障碍点（spec 修订 1）。

用 map->雷达 的旋转把点转到重力对齐的局部坐标（不用绝对高度，不受定位高度误差影响），
坡面、平台顶面不算障碍。输出点仍在雷达坐标系，代价地图射线清除以雷达真实位置为起点。
平台开放边缘这类固定落差由静态地图负责。
"""
import numpy as np
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from terrain import SELF_RANGE, obstacle_point_mask


class TerrainObstacleFilter(Node):
    def __init__(self):
        super().__init__('terrain_obstacle_filter')
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.frame_range = self.declare_parameter('frame_range', 5.0).value
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        self.pub = self.create_publisher(PointCloud2, 'terrain_obstacles', 10)
        # 单帧范围内的非障碍点（抽稀），代价地图只用来做射线清除
        self.ground_pub = self.create_publisher(PointCloud2, 'terrain_ground', 10)
        self.ground_step = self.declare_parameter('ground_decimation', 4).value
        cloud_topic = self.declare_parameter('cloud_topic', '/livox/lidar_PointCloud2').value
        self.create_subscription(PointCloud2, cloud_topic, self.on_cloud, 10)

    def rotation(self, frame, stamp):
        for when, timeout in ((stamp, 0.05), (Time(), 0.0)):   # 点云时刻的 TF 还没到时用最新的
            try:
                q = self.tf_buffer.lookup_transform(self.map_frame, frame, when, Duration(seconds=timeout)).transform.rotation
                if when is not stamp:
                    self.get_logger().warn(f'TF {self.map_frame}->{frame} at cloud stamp not available, using latest',
                                           throttle_duration_sec=10.0)
                return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            except tf2_ros.TransformException as e:
                err = e
        self.get_logger().warn(f'no TF {self.map_frame}->{frame}: {err}', throttle_duration_sec=5.0)
        return None

    def on_cloud(self, msg):
        R = self.rotation(msg.header.frame_id, Time.from_msg(msg.header.stamp))
        if R is None:
            return
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        local = pts @ R.T
        mask = obstacle_point_mask(local, self.frame_range)
        self.pub.publish(point_cloud2.create_cloud_xyz32(msg.header, pts[mask].astype(np.float32)))
        ground = ~mask & (np.hypot(local[:, 0], local[:, 1]) <= self.frame_range) & (np.linalg.norm(local, axis=1) > SELF_RANGE)
        self.ground_pub.publish(point_cloud2.create_cloud_xyz32(
            msg.header, pts[ground][::self.ground_step].astype(np.float32)))


def main():
    rclpy.init()
    rclpy.spin(TerrainObstacleFilter())


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""把 Cartographer 3D 的 /scan_matched_points2（map 坐标系）变成 2.5D 地形：高程、障碍、坡度、台阶、可通行代价。

计算在 terrain.py 里：每帧点云在单帧内部判断障碍、台阶、坡度（高度关系只在一帧内比较，不受 SLAM 高度漂移影响），
再按每帧位姿的水平位置投票合成全局可通行代价（spec 修订 1）。
建图过程中每 preview_period 秒用一部分帧预览一次 /map；每帧点云连同当时的跟踪系位姿都会保存下来。
建图结束后调用 ~/rebuild_from_trajectory，用 Cartographer 回环优化后的轨迹（/trajectory_query）
把每帧重新投影一遍再算各层；参数 elevation_path 非空时同时把各层存成 npz（高程只用于显示和评估）。
/map 发布的是连续代价（-1 未知，0..100），要用 map_saver_cli --mode raw 保存。
"""
from dataclasses import fields

import numpy as np
import rclpy
import tf2_ros
from cartographer_ros_msgs.srv import TrajectoryQuery
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from scipy.spatial.transform import Rotation, Slerp
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from std_srvs.srv import Trigger

from terrain import FRAME_PARAMS, UNKNOWN, TerrainParams, VoteGrid, frame_layers, to_grid_map_data


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class CloudToGridMap(Node):
    def __init__(self):
        super().__init__('cloud_to_grid_map')
        # 以 map 原点（出生点）为中心
        size = self.declare_parameter('size', 130.0).value          # 盖住仓库南边的公路和围栏（至少 120 m）
        # Cartographer 的 tracking_frame，/trajectory_query 返回的就是它在 map 系下的位姿
        self.tracking_frame = self.declare_parameter('tracking_frame', 'imu_link').value
        self.trajectory_id = self.declare_parameter('trajectory_id', 0).value
        # 建图时预览：每隔 preview_period 秒，用每 preview_decimation 帧取一帧算一次
        self.preview_decimation = self.declare_parameter('preview_decimation', 5).value
        self.declare_parameter('elevation_path', '')
        self.terrain = TerrainParams(**{f.name: self.declare_parameter(f.name, f.default).value
                                        for f in fields(TerrainParams)})
        # 单帧只用水平这么远的点（姿态误差 0.5° 在 5 m 处约 4 cm，低于障碍阈值 0.08 m）
        self.frame_range = self.declare_parameter('frame_range', 5.0).value
        res = self.terrain.resolution
        self.shape = (int(round(size / res)),) * 2
        self.origin = (-size / 2.0, -size / 2.0)
        # 每帧：(时间, 当时的 map->tracking 4x4 位姿, map 系下的点 float32)
        self.frames = []
        self.previewed = 0
        # 预览用的投票网格只累加新帧，避免每次把所有帧重算一遍（帧多了会占满执行器、丢点云）
        self.preview_grid = VoteGrid(self.origin, self.shape, self.terrain.resolution)
        self.rebuilt = False

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', qos)
        self.grid_map_pub = self.create_publisher(GridMap, 'elevation_map', qos)
        self.create_subscription(PointCloud2, 'scan_matched_points2', self.on_cloud, 50)
        self.create_service(Trigger, '~/rebuild_from_trajectory', self.on_rebuild)
        self.create_timer(self.declare_parameter('preview_period', 5.0).value, self.on_preview)
        self.frame_id = 'map'
        self.tf_buffer = tf2_ros.Buffer()
        # TF 单独线程接收，否则点云回调占满单线程执行器，TF 缓存落后导致查询失败、整帧被丢
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        # 在服务回调里同步调用 /trajectory_query，用单独的节点 spin，避免阻塞本节点的执行器
        self.client_node = rclpy.create_node('cloud_to_grid_map_trajectory_client')
        self.trajectory_client = self.client_node.create_client(TrajectoryQuery, 'trajectory_query')

    def lookup_tracking_pose(self, frame_id, stamp):
        # 必须是点云时刻的位姿：rebuild 时用它和优化后的位姿算修正量，拿别的时刻会把整帧点投歪
        try:
            tf = self.tf_buffer.lookup_transform(frame_id, self.tracking_frame, stamp, Duration(seconds=0.1))
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'no TF {frame_id}->{self.tracking_frame}, dropping cloud: {e}',
                                   throttle_duration_sec=5.0)
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        pose[:3, 3] = [t.x, t.y, t.z]
        return pose

    def on_cloud(self, msg):
        self.frame_id = msg.header.frame_id
        pose = self.lookup_tracking_pose(msg.header.frame_id, Time.from_msg(msg.header.stamp))
        if pose is None:
            return
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float32)
        # 单帧分类只用水平 frame_range 内的点；重建时位姿修正实测最大约 2.4 m，留 3 m 余量，其余不存（省内存）
        d = np.hypot(pts[:, 0] - pose[0, 3], pts[:, 1] - pose[1, 3])
        pts = pts[d <= self.frame_range + 3.0]
        if pts.size:
            self.frames.append((stamp_to_sec(msg.header.stamp), pose, pts))

    def on_preview(self):
        if self.rebuilt or len(self.frames) == self.previewed:
            return
        for i in range(self.previewed, len(self.frames)):
            if i % self.preview_decimation == 0:
                self.add_to(self.preview_grid, self.frames[i][2], self.frames[i][1][:3, 3])
        self.previewed = len(self.frames)
        self.publish(self.preview_grid.finalize(self.terrain))

    def vote(self, frames, start_xy=None):
        """每帧：点减去该帧跟踪系位置得到局部坐标（重力对齐、朝向同 map），单帧分类后按水平位置投票"""
        grid = VoteGrid(self.origin, self.shape, self.terrain.resolution)
        for pts, center in frames:
            self.add_to(grid, pts, center)
        return grid.finalize(self.terrain, start_xy=start_xy)

    def add_to(self, grid, pts, center):
        layers, origin, _ = frame_layers(pts - center.astype(np.float32), self.frame_range, FRAME_PARAMS)
        grid.add_frame(layers, origin, FRAME_PARAMS.resolution, center)

    def on_rebuild(self, request, response):
        if not self.frames:
            response.success, response.message = False, 'no point clouds received'
            return response
        if not self.trajectory_client.wait_for_service(timeout_sec=5.0):
            response.success, response.message = False, 'trajectory_query service not available'
            return response
        future = self.trajectory_client.call_async(TrajectoryQuery.Request(trajectory_id=self.trajectory_id))
        rclpy.spin_until_future_complete(self.client_node, future, timeout_sec=10.0)
        result = future.result()
        if result is None or len(result.trajectory) < 2:
            response.success, response.message = False, 'trajectory_query returned no trajectory'
            return response

        # 优化后的节点位姿，按时间插值到每帧点云的时刻
        nodes = sorted(result.trajectory, key=lambda p: stamp_to_sec(p.header.stamp))
        times = np.array([stamp_to_sec(p.header.stamp) for p in nodes])
        times, unique = np.unique(times, return_index=True)
        nodes = [nodes[i] for i in unique]
        positions = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in nodes])
        rotations = Rotation.from_quat([[p.pose.orientation.x, p.pose.orientation.y,
                                         p.pose.orientation.z, p.pose.orientation.w] for p in nodes])
        frame_times = np.clip([f[0] for f in self.frames], times[0], times[-1])
        new_rot = Slerp(times, rotations)(frame_times).as_matrix()
        new_pos = np.stack([np.interp(frame_times, times, positions[:, k]) for k in range(3)], axis=1)

        corrected, shift = [], []
        for (_, old, pts), rot, pos in zip(self.frames, new_rot, new_pos):
            new = np.eye(4)
            new[:3, :3], new[:3, 3] = rot, pos
            correction = new @ np.linalg.inv(old)
            corrected.append((pts @ correction[:3, :3].T.astype(np.float32) + correction[:3, 3].astype(np.float32), pos))
            shift.append(np.linalg.norm(correction[:2, 3]))
        # 以建图起点为种子去掉够不着的“小岛”（箱子、货车的平顶面）
        layers = self.vote(corrected, start_xy=tuple(new_pos[0][:2]))
        self.rebuilt = True
        self.publish(layers, with_grid_map=True)
        path = self.get_parameter('elevation_path').value
        if path:
            self.save_npz(path, layers)
        response.success = True
        response.message = (f'rebuilt from {len(self.frames)} clouds and {len(nodes)} optimized nodes, '
                            f'pose correction mean {np.mean(shift):.3f} m max {np.max(shift):.3f} m'
                            + (f'; elevation saved to {path}' if path else ''))
        self.get_logger().info(response.message)
        return response

    def crop(self, layers):
        """只保留有观测的包围盒，返回 (行切片, 列切片, 裁剪后的 origin)"""
        known = layers.cost != UNKNOWN
        rows, cols = np.flatnonzero(known.any(axis=1)), np.flatnonzero(known.any(axis=0))
        if rows.size == 0:
            return None
        r, c = slice(int(rows[0]), int(rows[-1]) + 1), slice(int(cols[0]), int(cols[-1]) + 1)
        res = self.terrain.resolution
        return r, c, (self.origin[0] + c.start * res, self.origin[1] + r.start * res)

    def publish(self, layers, with_grid_map=False):
        cropped = self.crop(layers)
        if cropped is None:
            return
        r, c, origin = cropped
        res = self.terrain.resolution
        stamp = self.get_clock().now().to_msg()
        cost = layers.cost[r, c]
        msg = OccupancyGrid()
        msg.header.frame_id, msg.header.stamp = self.frame_id, stamp
        msg.info.resolution = res
        msg.info.height, msg.info.width = cost.shape
        msg.info.origin.position.x, msg.info.origin.position.y = origin
        msg.info.origin.orientation.w = 1.0
        msg.data = cost.ravel().tolist()
        self.map_pub.publish(msg)
        # GridMap 消息很大（上百万个 float），只在 rebuild 后或有人订阅时发
        if not (with_grid_map or self.grid_map_pub.get_subscription_count() > 0):
            return
        rows, cols = cost.shape
        gm = GridMap()
        gm.header.frame_id, gm.header.stamp = self.frame_id, stamp
        gm.info.resolution = res
        gm.info.length_x, gm.info.length_y = cols * res, rows * res
        gm.info.pose.position.x = origin[0] + cols * res / 2
        gm.info.pose.position.y = origin[1] + rows * res / 2
        gm.info.pose.orientation.w = 1.0
        gm.basic_layers = ['elevation']
        for name, arr in (('elevation', layers.elevation[r, c]), ('slope', layers.slope_deg[r, c]),
                          ('step', layers.step[r, c]), ('cost', np.where(cost < 0, np.nan, cost))):
            data = Float32MultiArray()
            data.layout.dim = [MultiArrayDimension(label='column_index', size=rows, stride=rows * cols),
                               MultiArrayDimension(label='row_index', size=cols, stride=cols)]
            data.data = to_grid_map_data(arr).tolist()
            gm.layers.append(name)
            gm.data.append(data)
        self.grid_map_pub.publish(gm)

    def save_npz(self, path, layers):
        r, c, origin = self.crop(layers)
        np.savez_compressed(path, elevation=layers.elevation[r, c], slope_deg=layers.slope_deg[r, c],
                            step=layers.step[r, c], cost=layers.cost[r, c], obstacle=layers.obstacle[r, c],
                            origin=np.array(origin), resolution=self.terrain.resolution)


def main():
    rclpy.init()
    rclpy.spin(CloudToGridMap())


if __name__ == '__main__':
    main()

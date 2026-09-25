#!/usr/bin/env python3
"""把 Cartographer 3D 的 /scan_matched_points2（map 坐标系，已重力对齐）投影成 Nav2 用的 2D 栅格。

Cartographer 3D 子图自带的 2D 投影会把地面也算成占用，不能直接给 Nav2 用，所以按高度自己分：
  - 接近地面的点   -> 该格子被观测为空闲
  - 车体高度内的点 -> 该格子被观测为障碍（累计次数 >= min_occupied_hits、不少于地面点数的 0.3 倍、
                      不低于 5x5 邻域峰值的 min_peak_ratio 倍，且不是孤立单格，才算占用）
  - 都没有         -> 未知
map 系 z 原点不一定在地面上，地面高度 = 该帧跟踪系（imu_link，与 base_link 重合）的 z - base_height。

建图过程中实时发布 /map 供查看。每帧点云连同当时的跟踪系位姿都会保存下来；
建图结束后调用 ~/rebuild_from_trajectory，用 Cartographer 回环优化后的轨迹（/trajectory_query）
把每帧重新投影一遍再生成栅格，消掉建图过程中漂移造成的墙体变厚，再用 map_saver_cli 保存 /map。
"""
import numpy as np
import rclpy
import tf2_ros
from cartographer_ros_msgs.srv import TrajectoryQuery
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from scipy.ndimage import label, maximum_filter
from scipy.spatial.transform import Rotation, Slerp
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class CloudToGridMap(Node):
    def __init__(self):
        super().__init__('cloud_to_grid_map')
        self.resolution = self.declare_parameter('resolution', 0.05).value
        size = self.declare_parameter('size', 40.0).value
        # Cartographer 的 tracking_frame，/trajectory_query 返回的就是它在 map 系下的位姿
        self.tracking_frame = self.declare_parameter('tracking_frame', 'imu_link').value
        self.trajectory_id = self.declare_parameter('trajectory_id', 0).value
        # 跟踪系（与 base_link 重合）离地高度（车轮中心 z 0.0025 - 轮半径 0.1245）
        self.base_height = self.declare_parameter('base_height', 0.122).value
        self.ground_tolerance = self.declare_parameter('ground_tolerance', 0.05).value
        self.obstacle_min_height = self.declare_parameter('obstacle_min_height', 0.08).value
        self.obstacle_max_height = self.declare_parameter('obstacle_max_height', 1.2).value
        self.min_occupied_hits = self.declare_parameter('min_occupied_hits', 3).value
        # 建图中位姿瞬时误差会把墙点投到地面格子上，这类格子同时有大量地面点；
        # 真实障碍格子几乎看不到地面，所以要求障碍点数不少于地面点数的这个倍数
        self.min_obstacle_ground_ratio = self.declare_parameter('min_obstacle_ground_ratio', 0.3).value
        # 位姿误差会让墙面点散到相邻格子里，只按“命中 >= 3”判占用会取到误差分布的尾部，墙向内变厚约 10 cm。
        # 要求命中数不低于 5x5 邻域峰值的这个比例，只保留靠近真实表面的格子（仿真实测：
        # 远离障碍的误占用格 192 -> 36，墙面向内偏差 p99 0.225 -> 0.125 m），并去掉孤立的单个占用格
        self.min_peak_ratio = self.declare_parameter('min_peak_ratio', 0.2).value

        self.cells = int(round(size / self.resolution))
        self.origin = -size / 2.0
        self.ground_hits = np.zeros((self.cells, self.cells), np.int32)
        self.obstacle_hits = np.zeros((self.cells, self.cells), np.int32)
        # 每帧：(时间, 当时的 map->tracking 4x4 位姿, map 系下的点 float32)
        self.frames = []

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', qos)
        self.create_subscription(PointCloud2, 'scan_matched_points2', self.on_cloud, 10)
        self.create_service(Trigger, '~/rebuild_from_trajectory', self.on_rebuild)
        self.create_timer(1.0, self.publish_map)
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
        if pts.size == 0:
            return
        self.frames.append((stamp_to_sec(msg.header.stamp), pose, pts))
        self.accumulate(pts, pose[2, 3] - self.base_height, self.ground_hits, self.obstacle_hits)

    def accumulate(self, pts, ground_z, ground_hits, obstacle_hits):
        ix = np.floor((pts[:, 0] - self.origin) / self.resolution).astype(np.int64)
        iy = np.floor((pts[:, 1] - self.origin) / self.resolution).astype(np.int64)
        inside = (ix >= 0) & (ix < self.cells) & (iy >= 0) & (iy < self.cells)
        h = pts[:, 2] - ground_z
        ground = inside & (np.abs(h) < self.ground_tolerance)
        obstacle = inside & (h > self.obstacle_min_height) & (h < self.obstacle_max_height)
        flat = iy * self.cells + ix
        size = self.cells * self.cells
        ground_hits += np.bincount(flat[ground], minlength=size).reshape(self.cells, self.cells).astype(np.int32)
        obstacle_hits += np.bincount(flat[obstacle], minlength=size).reshape(self.cells, self.cells).astype(np.int32)

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

        ground_hits = np.zeros_like(self.ground_hits)
        obstacle_hits = np.zeros_like(self.obstacle_hits)
        shift = []
        for (_, old, pts), rot, pos in zip(self.frames, new_rot, new_pos):
            new = np.eye(4)
            new[:3, :3], new[:3, 3] = rot, pos
            correction = new @ np.linalg.inv(old)
            corrected = pts @ correction[:3, :3].T.astype(np.float32) + correction[:3, 3].astype(np.float32)
            self.accumulate(corrected, pos[2] - self.base_height, ground_hits, obstacle_hits)
            shift.append(np.linalg.norm(correction[:2, 3]))
        self.ground_hits, self.obstacle_hits = ground_hits, obstacle_hits
        self.publish_map()
        response.success = True
        response.message = (f'rebuilt from {len(self.frames)} clouds and {len(nodes)} optimized nodes, '
                            f'pose correction mean {np.mean(shift):.3f} m max {np.max(shift):.3f} m')
        self.get_logger().info(response.message)
        return response

    def publish_map(self):
        occupied = ((self.obstacle_hits >= self.min_occupied_hits) &
                    (self.obstacle_hits >= self.min_obstacle_ground_ratio * self.ground_hits) &
                    (self.obstacle_hits >= self.min_peak_ratio * maximum_filter(self.obstacle_hits, size=5)))
        labels, _ = label(occupied, structure=np.ones((3, 3)))
        occupied &= np.bincount(labels.ravel())[labels] >= 2
        free = (self.ground_hits > 0) & ~occupied
        observed_rows = np.flatnonzero((occupied | free).any(axis=1))
        observed_cols = np.flatnonzero((occupied | free).any(axis=0))
        if observed_rows.size == 0:
            return
        # 只发布有观测的包围盒，保存出来的 pgm 不会是一大片未知
        r0, r1 = int(observed_rows[0]), int(observed_rows[-1]) + 1
        c0, c1 = int(observed_cols[0]), int(observed_cols[-1]) + 1
        grid = np.full((r1 - r0, c1 - c0), -1, np.int8)
        grid[free[r0:r1, c0:c1]] = 0
        grid[occupied[r0:r1, c0:c1]] = 100

        msg = OccupancyGrid()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.resolution
        msg.info.width = c1 - c0
        msg.info.height = r1 - r0
        msg.info.origin.position.x = self.origin + c0 * self.resolution
        msg.info.origin.position.y = self.origin + r0 * self.resolution
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.ravel().tolist()
        self.map_pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(CloudToGridMap())


if __name__ == '__main__':
    main()

-- Cartographer 3D 建图：Mid-360 点云 + 仿真 IMU + Gazebo 轮式里程计
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  -- 3D 必须以 IMU 为跟踪坐标系（imu_link 与 base_link 原点重合）
  tracking_frame = "imu_link",
  -- odom -> base_link 由 Gazebo ackermann 插件发布，Cartographer 只发 map -> odom
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 4

-- 仿真插件每帧就是完整的一圈 360° 点云（10 Hz），不需要累积多帧
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
-- 滤掉打到自身车体上的点；房间只有 8x6 m，远处无意义
TRAJECTORY_BUILDER_3D.min_range = 0.4
TRAJECTORY_BUILDER_3D.max_range = 20.
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.05
-- 默认参数下每个节点只剩约 165 个点参与匹配，小房间里精度不够，保留更多点
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 1.0
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 300
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.05
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 15.
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.25
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 100
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 5.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 4e2

POSE_GRAPH.optimize_every_n_nodes = 60
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
POSE_GRAPH.optimization_problem.huber_scale = 5e2

return options

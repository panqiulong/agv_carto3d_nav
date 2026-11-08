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
  num_point_clouds = 2,   -- 前后两颗 Mid-360，话题 points2_1、points2_2
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

-- 时间对齐由 Cartographer 的 RangeDataCollator 完成：保留每颗雷达各自的原点，每个点再按自己的时间做运动补偿。
-- 点云没有逐点时间，它每次只放出一颗雷达的一帧，所以累积 2 次，一个节点才同时含前后两颗雷达
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 2
-- 滤掉打到自身车体上的点；工业场景 60 x 40 m，测距放到 30 m
TRAJECTORY_BUILDER_3D.min_range = 0.6
TRAJECTORY_BUILDER_3D.max_range = 30.
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.05
-- 默认参数下每个节点只剩约 165 个点参与匹配，小房间里精度不够，保留更多点
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 1.0
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 300
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.05
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 20.
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.25
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 100
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 5.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 4e2

POSE_GRAPH.optimize_every_n_nodes = 60
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
-- 坡道沿线各段看起来一样，回环在 5 m 外也能“对上”（仿真实测：优化后轨迹沿坡道偏 5 m，而只用局部 SLAM
-- 时最大偏 0.31 m）。局部漂移远小于 1.5 m，把搜索窗口从默认 5 m 缩到 1.5 m 排除错位匹配；
-- Huber 用默认 1e1，偶尔漏进来的错误约束影响有限（5e2 几乎不抗差）
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 1.5
POSE_GRAPH.optimization_problem.huber_scale = 1e1

return options

-- Cartographer 3D 纯定位：加载建图得到的 .pbstream（frozen），只维护少量新子图用于匹配
include "carto_3d_mapping.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
-- 定位时更频繁地做全局优化，让新轨迹尽快与冻结地图建立约束
POSE_GRAPH.optimize_every_n_nodes = 10
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.global_sampling_ratio = 0.1
-- 仿真实测：房间结构重复，冻结地图节点与新子图之间会出现偏差 4~5 m、得分 60~62% 的误匹配
-- （正确匹配得分 75~85%），被接受后定位跳变 1.3 m / 90°。提高得分门槛，缩小局部搜索窗口，
-- Huber 尺度用默认值让偶尔漏进来的离群约束影响有限（backpack_3d 的 5e2 几乎等于不抗差）
-- 0.65 是为上面房间里 4~5 m 外的错位匹配设的；局部搜索窗口已缩到 1.5 m，那种错位进不了窗口，
-- 局部门槛与建图一致即可（全局匹配仍用 0.7）
POSE_GRAPH.constraint_builder.min_score = 0.55
-- 不退回全局搜索：Cartographer 在定位轨迹连续 global_constraint_search_after_n_seconds 没和冻结地图建立约束后，
-- 就只做全局搜索、不再做局部匹配。仓储场景里正确的局部匹配得分 55~70%，到不了全局门槛 0.7；启动阶段 10 s 内
-- 没配上一次，就永远配不上，全局搜索还把约束队列堆到几千条、CPU 占满（2026-11-01 实测：开一圈 0 条约束）。
-- 启动时总给初始位姿（start_localization），局部窗口 1.5 m 内能找回来；代价是车被挪走后不能自动重定位
POSE_GRAPH.global_constraint_search_after_n_seconds = 1e6
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 1.5
POSE_GRAPH.optimization_problem.huber_scale = 1e1
-- 定位轨迹匹配上冻结地图之前，没有任何东西固定它；在线估计 IMU 外参时，整条轨迹的横滚/俯仰可以和外参
-- 一起转而代价不变，实测静止时姿态漂到 40°~80°，歪了的点云再也匹配不上。仿真 IMU 安装是精确的，不在线估计
POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d = false

return options

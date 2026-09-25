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
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 1.5
POSE_GRAPH.optimization_problem.huber_scale = 1e1

return options

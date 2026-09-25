#!/bin/bash
# 保存建图结果：<前缀>.pbstream（Cartographer 定位用）+ <前缀>.pgm/.yaml（Nav2 全局规划用）
# 用法: ros2 run agv_nav save_map.sh ~/AGV2/src/agv_nav/maps/map
set -e
if [ -z "$1" ]; then
  echo "usage: save_map.sh <output_prefix>   e.g. ~/AGV2/src/agv_nav/maps/map" >&2
  exit 1
fi
PREFIX=$(realpath -m "$1")
mkdir -p "$(dirname "$PREFIX")"

ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
# 用回环优化后的轨迹重新投影所有点云，再保存栅格，否则建图过程中的漂移会让墙变厚
ros2 service call /cloud_to_grid_map/rebuild_from_trajectory std_srvs/srv/Trigger
ros2 run nav2_map_server map_saver_cli -f "$PREFIX" --ros-args -p use_sim_time:=true
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '${PREFIX}.pbstream', include_unfinished_submaps: true}"
echo "saved: ${PREFIX}.pbstream ${PREFIX}.pgm ${PREFIX}.yaml"

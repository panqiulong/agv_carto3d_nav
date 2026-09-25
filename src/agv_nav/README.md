# agv_nav：Cartographer 3D 建图 / 定位 + Nav2 导航

在 Gazebo 仿真（`agv_urdf`，Ackermann 小车 + 前倾安装的 Mid-360）上实现：

- **建图**：Cartographer 3D（点云 + IMU + 轮式里程计）
- **定位**：Cartographer 3D 纯定位（加载建图得到的 `.pbstream`）
- **全局规划**：Nav2 Smac Hybrid-A\*（Reeds-Shepp，允许倒车）
- **局部控制**：Nav2 MPPI（Ackermann 运动模型）

环境：ROS 2 Humble、Gazebo Classic 11。

---

## 1. 系统结构

```
Gazebo (agv_urdf)
 ├─ Mid-360  ──> /livox/lidar_PointCloud2 ─┬─> Cartographer 3D ──> TF map->odom
 ├─ IMU      ──> /imu ─────────────────────┤        │
 ├─ 底盘插件  ──> /odom, TF odom->base_link ─┘        └─> /scan_matched_points2 ──> cloud_to_grid_map ──> /map（建图时）
 │                                         
 │   /livox/lidar_PointCloud2 ──> pointcloud_to_laserscan ──> /scan ──> Nav2 代价地图（导航时）
 │
 └─ 底盘插件 <── /cmd_vel_ackermann <── cmd_vel_to_ackermann <── /cmd_vel <── Nav2 / 键盘遥控
```

| 节点 / 文件 | 作用 |
|---|---|
| `cartographer_node` | 建图：`config/carto_3d_mapping.lua`；定位：`config/carto_3d_localization.lua` |
| `scripts/cloud_to_grid_map.py` | 把 3D 点云按高度分成地面/障碍，生成 Nav2 用的 2D 栅格 `/map`。建图结束时用回环优化后的轨迹重新投影所有点云，再输出地图 |
| `scripts/cmd_vel_to_ackermann.py` | 把标准 `/cmd_vel`（线速度 + **角速度**）换算成底盘插件要的（线速度 + **前轮转角**），并补偿插件只用右后轮做速度反馈的问题 |
| `scripts/save_map.sh` | 一次保存 `.pbstream`（定位用）和 `.pgm/.yaml`（规划用） |
| `pointcloud_to_laserscan` | 从 3D 点云里切出离地 0.10 ~ 1.32 m 的一层，生成 `/scan` 给代价地图 |
| `params/nav2_params.yaml` | Nav2 全部参数（Smac Hybrid-A\*、MPPI、代价地图、速度平滑） |
| `behavior_trees/navigate_to_pose_ackermann.xml` | 导航行为树：把默认恢复动作里的原地旋转换成“向前直行 0.3 m” |

TF 树：`map ─(Cartographer)→ odom ─(底盘插件)→ base_link ─(robot_state_publisher)→ imu_link / lidar_Link / livox_sensor / camera_Link …`

---

## 2. 安装与编译

```bash
sudo apt install ros-humble-cartographer-ros ros-humble-navigation2 ros-humble-nav2-bringup \
                 ros-humble-pointcloud-to-laserscan python3-scipy
cd ~/AGV2
colcon build
source install/setup.bash
```

---

## 3. 建图

**终端 1：启动仿真**

```bash
ros2 launch agv_urdf gazebo_sim.launch.py
```

**终端 2：启动建图**（会同时打开 RViz）

```bash
ros2 launch agv_nav mapping.launch.py
```

**终端 3：遥控小车绕场景走**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

开车建图时注意：

- `/cmd_vel` 是标准含义（`angular.z` 是角速度），换算节点会转成前轮转角。**车不能原地转**，只按 `j`/`l`（纯旋转）不会动，要用 `u`/`o`/`m`/`.` 这类边走边转的键。
- 最小转弯半径约 **1.3 m**，房间小的地方需要倒车掉头。
- 速度 0.3 ~ 0.5 m/s 即可，太快会增加漂移。
- 雷达朝前下方倾斜约 47°，**车身后方低处是盲区**，只能靠车头方向看清环境。所以要让车头朝向各个方向都扫一遍。
- 最后**回到起点附近再走一段**，让 Cartographer 做回环闭合。
- RViz 里 `/map` 会实时更新，这只是预览。保存时会用优化后的轨迹重新生成一次。

**保存地图**（在终端 3 停止遥控后执行）：

```bash
ros2 run agv_nav save_map.sh ~/AGV2/src/agv_nav/maps/map
```

这个脚本依次执行：结束轨迹 → 用优化后的轨迹重建栅格 → 保存 `map.pgm/map.yaml` → 保存 `map.pbstream`。完成后输出 `saved: ...`。

保存后**重新编译一次**，地图才会被安装到导航 launch 默认读取的位置：

```bash
colcon build --packages-select agv_nav
```

> 也可以不重新编译，启动导航时用 `pbstream:=` 和 `map:=` 直接指定文件路径，见下一节。

---

## 4. 定位 + 导航

**终端 1：启动仿真**（和建图时一样）

```bash
ros2 launch agv_urdf gazebo_sim.launch.py
```

**终端 2：启动定位和导航**

```bash
ros2 launch agv_nav navigation.launch.py
# 或者指定地图文件：
ros2 launch agv_nav navigation.launch.py \
    pbstream:=$HOME/AGV2/src/agv_nav/maps/map.pbstream \
    map:=$HOME/AGV2/src/agv_nav/maps/map.yaml
```

在 RViz 里用 **Nav2 Goal** 工具点击目标点、拖动方向即可。

> **重要：启动导航时，小车必须位于建图时的出发位置和朝向。** 纯定位默认从地图原点开始，目前不支持用 RViz 的 “2D Pose Estimate” 重新设置初始位姿。仿真每次启动都会把车放在同一个位置，只要建图也从这个位置出发就没有问题。

launch 参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `pbstream` | `share/agv_nav/maps/map.pbstream` | Cartographer 地图 |
| `map` | `share/agv_nav/maps/map.yaml` | Nav2 栅格地图 |
| `params_file` | `share/agv_nav/params/nav2_params.yaml` | Nav2 参数 |
| `use_rviz` | `true` | 是否打开 RViz（建图 launch 也有这个参数） |
| `use_sim_time` | `true` | 仿真时间 |

---

## 5. 主要参数

| 位置 | 参数 | 当前值 | 说明 |
|---|---|---|---|
| `nav2_params.yaml` → `planner_server` | `minimum_turning_radius` | 1.3 | 必须和车辆实际最小转弯半径一致 |
| 同上 | `motion_model_for_search` | `REEDS_SHEPP` | 允许倒车；改成 `DUBIN` 则只能前进 |
| `controller_server` → `FollowPath` | `AckermannConstraints.min_turning_r` | 1.3 | 同上 |
| 同上 | `vx_max` / `vx_min` | 0.5 / -0.3 | 前进 / 倒车最大速度 |
| 同上 | `controller_frequency`, `model_dt`, `time_steps` | 10 Hz, 0.1, 30 | 预测 3 s |
| `local_costmap` / `global_costmap` | `footprint` | 0.8 × 0.72 m | 车体轮廓 |
| 同上 | `footprint_padding` | 0.05 | 规划和控制都按外扩 5 cm 的轮廓，防止贴墙 |
| `velocity_smoother` | `max_velocity` / `max_accel` | 0.5 m/s / 1.0 m/s² | |
| `carto_3d_localization.lua` | `min_score` | 0.65 | 过低会接受误匹配，定位跳变 |
| `navigation.launch.py` → `pointcloud_to_laserscan` | `min_height` / `max_height` | -0.02 / 1.2（base_link 下） | 即离地 0.10 ~ 1.32 m 的障碍会进代价地图 |
| `cmd_vel_to_ackermann` | `wheelbase` / `max_steer` / `rear_track` | 0.57 / 0.414 / 0.62 | 必须和 URDF 一致 |

车辆几何：轴距 0.57 m，后轮距 0.62 m；内侧前轮最大转角 30°，对应车辆中心等效转角 0.414 rad，后轴中心最小转弯半径约 1.30 m。

---

## 6. 已知限制

- **初始位姿**：导航启动时车必须在建图起点（见第 4 节）。
- **后方盲区**：雷达前倾，车后方低处看不到。局部代价地图里加了静态层，地图上已有的障碍在倒车时仍然会被避开；但**建图之后新出现在车后方的障碍，倒车时看不到**。
- **不能原地旋转**：恢复行为用“向前直行 0.3 m”和“后退 0.3 m”代替原地旋转。
- **地图精度**：栅格分辨率为 5 cm，墙面在地图里会比实际向内多出约 0.05 ~ 0.1 m，所以规划会离墙稍远一些。
- **速度跟踪**：底盘插件的速度环只用右后轮做反馈，换算节点已经按 Ackermann 几何做了补偿；转弯时实际车速和横摆角速度约为指令的 85 ~ 93%，由 MPPI 闭环修正。
- **语义地图**：尚未实现。车前有深度相机 `camera_Link`，`map` 坐标系由 Cartographer 统一提供，可以在此基础上叠加。

---

## 7. 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| Cartographer、Nav2 一直等待，没有 TF 或时间不走 | 没有 `/clock`。必须用 `agv_urdf` 的 `gazebo_sim.launch.py` 启动仿真，它会加载 `libgazebo_ros_init.so` 并以 200 Hz 发布 `/clock`。自己写启动命令时，要带 `-s libgazebo_ros_init.so --ros-args --params-file .../agv_urdf/config/gazebo_params.yaml` |
| 雷达没有点云 | 检查 `mid360.xacro` 里的 `<always_on>true</always_on>` 是否还在。没有这个标签时，只有打开 gzclient 窗口雷达才出数据 |
| 键盘遥控车不动 | 必须先启动建图或导航的 launch（里面有 `cmd_vel_to_ackermann` 换算节点）；原地旋转的键无效，见第 3 节 |
| 规划报 `Starting point in lethal space` | 车的轮廓压到了地图障碍上（车太贴墙，或定位偏差较大）。行为树会自动尝试前进 / 后退 0.3 m 脱困；如果一直出现，先确认定位是否正常 |
| 导航时定位突然跳变 | 通常是启动时车不在建图起点，或地图质量差（建图时走得太快、没回到起点）。重新建图 |
| `Control loop missed its desired rate` 大量出现 | CPU 不够或 `/clock` 频率太低，见第一条 |

---

## 8. 仿真验证结果

在 `simple_room` 场景里做了一轮测试：先用运动学底盘沿固定路线建图，再用实际的 Ackermann 底盘连续跑 6 个导航目标（3 个目标各跑两遍，其中一个需要在东北角掉头）。结果：

- 6 个目标全部到达
- 定位误差最大 0.19 m / 4.2°
- 全程没有出现 `Starting point in lethal space`，车体离障碍始终大于 0.1 m
- 控制周期超时 1 次

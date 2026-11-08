# 工作区架构说明：功能、实现与数据链路

本文说明整个 `AGV2` 工作区里每个部分做什么、怎么实现、数据怎么流动。使用步骤和参数细节见 [src/agv_nav/README.md](src/agv_nav/README.md)。

---

## 1. 这个工作区做什么

一辆 Ackermann 小车（前轮转向，车体 1.2 × 1.08 m，最小转弯半径实测约 2.2 m）在 Gazebo 仿真里，用一颗 Livox Mid-360 激光雷达、一个 IMU 和轮式里程计完成：

1. **建图**：Cartographer 3D 建出三维地图（`.pbstream`），同时生成 2.5D 地形可通行代价图（`.pgm/.yaml`）；
2. **定位**：Cartographer 3D 纯定位，在已有地图上实时估计车的位置；
3. **导航**：Nav2 全局规划（Smac Hybrid-A\*）+ 局部控制（MPPI），能自主上下坡道，不会从平台边缘开下去；
4. **语义地图**：有名字的地点（按名字导航）和区域（限速 / 禁行）；
5. **出入库任务**：入库单 / 出库单排队执行，维护库存和库位预留（仓储场景 `warehouse`）。

一共三种运行方式，都要先启动仿真：

| 运行方式 | 启动命令 | 用途 |
|---|---|---|
| 仿真 | `ros2 launch agv_gazebo gazebo_sim.launch.py` | 提供车、传感器、场景 |
| 建图 | `ros2 launch agv_nav mapping.launch.py` | 遥控小车走一遍，保存地图 |
| 定位 + 导航 | `ros2 launch agv_nav navigation.launch.py [semantic:=... tasks:=...]` | 在 RViz 里点目标，或按地点名导航、提交出入库任务，车自己开过去 |

---

## 2. 目录与功能

```
AGV2/
├── README.md                      仓库简介、编译方法、第三方代码说明
├── ARCHITECTURE.md                本文
├── docs/superpowers/              设计文档（specs）和实施计划（plans）
├── maps/                          建好的地图，每张一个文件夹（不进 git）
└── src/
    ├── agv_description/           小车模型（URDF/xacro、网格）
    ├── agv_gazebo/                Gazebo 仓储场景 + 仿真启动
    ├── agv_nav/                   建图、定位、导航、2.5D 地形
    ├── agv_semantic/              语义地图：地点与区域（限速 / 禁行），按名字导航；出入库任务
    ├── agv_hdmap/                 公路的 Lanelet2 高精地图：车道路由、沿车道行驶、交通规则、和 Nav2 交接
    └── third_party/               第三方包（见第 6 节）
```

### 2.1 `agv_description` / `agv_gazebo`：仿真

小车模型放在 `agv_description`，场景和仿真启动放在 `agv_gazebo`。

| 文件 | 功能 |
|---|---|
| `agv_description/urdf/r2.xacro` | 小车模型：车身、4 个轮子、转向关节；挂载 Mid-360（`lidar_Link`，前倾约 47°，装在车头一根只有外观的立杆上，比车身高 0.2 m：原来装在车身上时约 29% 的射线打在车身上）、IMU（`imu_link`）、深度相机（`camera_Link`）；Gazebo 插件：Ackermann 底盘、IMU、深度相机。车轮外观是圆柱，碰撞体是同半径的球（圆柱边缘会在坡脚折线处卡住带转向角的前轮） |
| `agv_gazebo/launch/gazebo_sim.launch.py` | 启动 gzserver / gzclient、robot_state_publisher，把车生成在所选世界的出生点；`world` 参数选择场景。设置 `GAZEBO_MODEL_PATH` 和 `GAZEBO_RESOURCE_PATH`（AWS 模型的网格是相对路径 `file://models/...`，要靠后者解析） |
| `agv_gazebo/config/gazebo_params.yaml` | `/clock` 发布频率提到 200 Hz（默认 10 Hz 会让 TF 和 Nav2 控制周期不稳） |
| `agv_gazebo/scripts/gen_warehouse_world.py` | 生成仓储场景的脚本：尺寸、坡道、夹层、物体位置都在脚本开头定义，同时输出 map 坐标系下的语义文件和任务配置模板；带布局检查（间距、巷道宽度、每个地点的车体矩形） |
| `agv_gazebo/worlds/warehouse.world`、`warehouse_semantic.yaml`、`warehouse_tasks.yaml`、`warehouse_elevator.yaml` | 50 × 30 m 仓储场景：北墙收货月台 R1/R2、发货月台 S1/S2；西边 4 排货架（库位 A1..A8，在可穿行的巷道里）；东南角 1.0 m 高夹层（库位 M1..M4），经 5°、3.5 m 宽的坡道上下；东墙北段有一部货梯（`warehouse_elevator.yaml` 是电梯的世界真值；进梯程序用的是部署时示教出的 `maps/<前缀>/<前缀>_elevator.yaml`）；出生点即 `HOME` |

### 2.2 `agv_nav`：建图、定位、导航

| 文件 | 功能 |
|---|---|
| `launch/mapping.launch.py` | 建图：Cartographer（建图配置）+ `cloud_to_grid_map` + `cmd_vel_to_ackermann` + RViz |
| `launch/navigation.launch.py` | 导航：Cartographer（纯定位配置，加载 `.pbstream`）+ 带初始位姿启动定位 + map_server + `terrain_obstacle_filter` + `cmd_vel_to_ackermann` + Nav2 + RViz |
| `config/carto_3d_mapping.lua` | Cartographer 3D 建图参数 |
| `config/carto_3d_localization.lua` | 纯定位参数（在建图参数基础上修改） |
| `config/nav2_params.yaml` | Nav2 全部参数：规划器、控制器、两个代价地图、速度平滑、恢复行为 |
| `behavior_trees/navigate_to_pose_ackermann.xml` | 导航行为树：恢复动作不用原地旋转（Ackermann 车做不到），改成向前直行 / 后退 0.3 m |
| `scripts/terrain.py` | 2.5D 地形的纯计算模块（不依赖 ROS），下面两个节点都调用它 |
| `scripts/cloud_to_grid_map.py` | 建图节点：把点云变成可通行代价地图 `/map` 和高程图 `/elevation_map` |
| `scripts/terrain_obstacle_filter.py` | 导航节点：从每帧点云里挑出真正的障碍点，给代价地图用 |
| `scripts/cmd_vel_to_ackermann.py` | 把标准速度指令换算成底盘插件要的格式 |
| `scripts/save_map.sh` | 一次保存建图的全部结果 |
| `test/test_terrain.py` | `terrain.py` 的单元测试（25 项） |

### 2.3 `agv_semantic`：语义地图（地点与区域）与出入库任务

| 文件 | 功能 |
|---|---|
| `scripts/semantic_map.py` | 纯计算模块（不依赖 ROS）：语义文件读写与校验、点在多边形内判断、区域查询、区域哈希、生成禁行 / 限速掩码 |
| `scripts/semantic_server.py` | 节点：地点的增删查服务、按名字导航动作、`/current_zone`、RViz 标记 |
| `scripts/semantic.py` | 命令行：`add` / `remove` / `list` / `goto` / `where` |
| `scripts/make_filter_masks.py` | 由语义文件的区域生成 Nav2 禁行、限速掩码 |
| `scripts/tasks.py` | 纯计算模块（不依赖 ROS）：任务配置 / 库存文件读写与校验、任务队列、库位预留、状态流转 |
| `scripts/task_manager.py` | 节点：提交 / 取消 / 继续 / 库存 / 列表服务，`/task_status`；工作线程逐个执行任务 |
| `scripts/task.py` | 命令行：`submit` / `list` / `cancel` / `resume` / `stock` |
| `srv/PlaceEdit.srv`、`srv/TaskSubmit.srv`、`srv/TaskId.srv`、`srv/SlotSet.srv`、`action/GoToPlace.action` | 接口定义 |
| `test/` | 单元测试：`test_semantic_map.py`、`test_semantic_server.py`、`test_tasks.py`、`test_task_manager.py` |

### 2.4 `agv_hdmap`：公路的高精地图

地图本身由 `agv_gazebo/scripts/road_map.py` 生成（和 Gazebo 里的路面标线同一份几何），输出 `worlds/warehouse_lanelet2_map.osm`。

| 文件 | 功能 |
|---|---|
| `scripts/hdmap.py` | 纯计算模块（不依赖 ROS）：读地图（用 `local_x` / `local_y`）、定位到车道、路由、生成路径（含变道）、限速、红绿灯停车点、仓库 ⇄ 公路分段 |
| `scripts/hdmap_navigator.py` | 节点：收 `/hdmap/goal`，分段交给 Nav2 的 `navigate_to_pose`（仓库里）或 `follow_path`（公路上），发限速、按红绿灯截路径 |
| `scripts/traffic_light_sim.py` | 节点：按周期切换两组仿真红绿灯，用 Autoware 的消息发出去 |
| `scripts/hdmap_cli.py` | 命令行：`goto <x> <y>` |
| `launch/hdmap.launch.py` | Autoware 的地图投影加载器、Lanelet2 地图加载器、地图显示，加上面两个节点；由 `navigation.launch.py hdmap:=<.osm>` 带起来 |
| `README.md` | 学习指南：对照这张地图讲车道、连接与路由、交通规则、坐标与定位 |
| `test/test_hdmap.py` | 单元测试：读地图、定位、路由、路径、交通规则、分段，launch 的节点列表 |

---

## 3. 数据链路

### 3.1 总览

```
                 ┌───────────────────────── Gazebo 仿真 ─────────────────────────┐
                 │  Mid-360 插件×2      IMU 插件        Ackermann 底盘插件         │
                 └─────┬──────────────────┬──────────────────┬─────────▲─────────┘
  /livox/lidar[_rear]_PointCloud2      /imu          /odom + TF odom→base_link │ /cmd_vel_ackermann
                       │                  │                  │                   │
          ┌────────────┼──────────────────┴──────────────────┤                   │
          ▼            ▼                                     ▼                   │
   terrain_obstacle  Cartographer 3D ────────────────▶ TF map→odom               │
   _filter（导航时） │  建图：/scan_matched_points2                              │
          │          │         + /trajectory_query（优化后的轨迹）               │
          │          ▼                                                           │
          │    cloud_to_grid_map（建图时）──▶ /map、/elevation_map               │
          │          │                                                           │
          │          └─ save_map.sh ─▶ map.pbstream / map.pgm+yaml / map_elevation.npz
          │                                  │                                   │
          │                     导航时：pbstream ─▶ Cartographer 纯定位           │
          │                             pgm/yaml ─▶ map_server ─▶ /map           │
          ▼                                          ▼                           │
   /terrain_obstacles、/terrain_ground ─▶ Nav2 代价地图（静态层 + 障碍层 + 膨胀层） │
                                                     │                           │
                                 Smac Hybrid-A* 全局路径 ─▶ MPPI 局部控制         │
                                                     │ /cmd_vel                  │
                                                     ▼                           │
                                         cmd_vel_to_ackermann ───────────────────┘
```

### 3.2 主要话题

| 话题 | 类型 | 发布者 → 订阅者 | 内容 |
|---|---|---|---|
| `/livox/lidar_PointCloud2` | PointCloud2 | 前 Mid-360 插件 → Cartographer（`points2_1`）、`terrain_obstacle_filter`、`elevator` | 前雷达点云，每帧 2 万点，坐标系 `livox_sensor`；仿真里两颗雷达共用射线算力，实测 6~10 Hz（仿真时间） |
| `/livox/lidar_rear_PointCloud2` | PointCloud2 | 后 Mid-360 插件 → Cartographer（`points2_2`）、`terrain_obstacle_filter_rear` | 后雷达点云，坐标系 `livox_sensor_rear`。Cartographer 的 `RangeDataCollator` 做时间对齐，`num_accumulated_range_data = 2` 让一个节点同时含前后两帧 |
| `/imu` | Imu | IMU 插件 → Cartographer | 200 Hz，坐标系 `imu_link` |
| `/odom` | Odometry | 底盘插件 → Cartographer | 轮式里程计；插件的 `odom` 坐标系就是 Gazebo 世界坐标系 |
| `/clock` | Clock | Gazebo → 所有节点 | 仿真时间，200 Hz |
| `/scan_matched_points2` | PointCloud2 | Cartographer → `cloud_to_grid_map` | 建图时，每帧对齐到 `map` 坐标系后的点云 |
| `/map` | OccupancyGrid | 建图：`cloud_to_grid_map`；导航：map_server → 代价地图静态层 | 可通行代价：-1 未知，0~100，100 不可通行 |
| `/elevation_map` | grid_map_msgs/GridMap | `cloud_to_grid_map` → RViz | 高程、坡度、台阶、代价四层，供查看 |
| `/terrain_obstacles` | PointCloud2 | `terrain_obstacle_filter` → 代价地图障碍层 | 只含障碍点（用于标记障碍） |
| `/terrain_ground` | PointCloud2 | `terrain_obstacle_filter` → 代价地图障碍层 | 抽稀的地面点（只用于射线清除，擦掉已经离开的障碍） |
| `/terrain_obstacles_rear`、`/terrain_ground_rear` | PointCloud2 | `terrain_obstacle_filter_rear` → 代价地图障碍层 | 后雷达的障碍点和地面点。单独成话题：观测源只留最新一帧，和前雷达发到同一话题会互相顶掉 |
| `/cmd_vel` | Twist | Nav2 或键盘遥控 → `cmd_vel_to_ackermann` | 标准含义：线速度 + 角速度 |
| `/cmd_vel_ackermann` | Twist | `cmd_vel_to_ackermann` → 底盘插件 | 线速度 + **前轮转角**（插件的约定） |
| `/current_zone` | String | `semantic_server` → 命令行 / 其他程序 | 车当前所在区域的名字（多个用逗号分隔），变化时发布 |
| `/semantic_markers` | MarkerArray | `semantic_server` → RViz | 地点和区域的显示 |
| `/task_status` | String（JSON） | `task_manager` → 命令行 / 以后的 Web 界面 | `paused`、`active`、`queue`、`recent`、`slots`、`reserved`，变化时发布（transient local） |
| `/keepout_filter_info`、`/keepout_filter_mask` | CostmapFilterInfo、OccupancyGrid | 过滤器信息服务、掩码地图服务 → 代价地图禁行过滤器 | 禁行区掩码（100 = 禁行） |
| `/speed_filter_info`、`/speed_filter_mask` | CostmapFilterInfo、OccupancyGrid | 过滤器信息服务、掩码地图服务 → 全局代价地图限速过滤器 | 限速掩码（限速 = 像素值 × 0.01 m/s） |
| `/speed_limit` | nav2_msgs/SpeedLimit | 限速过滤器、`semantic_server`、`hdmap_navigator` → controller_server（MPPI） | 进入限速区时发限速值，离开时发 0（不限速）；`semantic_server` 在限速区内、`hdmap_navigator` 沿车道开时每 0.5 s 重发（MPPI 每收到新的跟踪目标都会重置限速） |
| `/map/vector_map` | autoware_map_msgs/LaneletMapBin | Autoware 地图加载器 → 地图显示 | Lanelet2 地图（transient local） |
| `/map/vector_map_marker` | MarkerArray | Autoware 地图显示 → RViz | 车道、边界线、停止线、红绿灯 |
| `/hdmap/goal` | PoseStamped | RViz "HD Map Goal"、`hdmap_cli.py` → `hdmap_navigator` | 高精地图导航的目标（map 系） |
| `/hdmap/path`、`/hdmap/current_lanelet`、`/hdmap/status` | Path、String、String | `hdmap_navigator` → RViz / 命令行 | 当前发给控制器的路径、车所在车道、状态（idle / driving / waiting_light / arrived / failed: 原因） |
| `/hdmap/route_markers` | MarkerArray | `hdmap_navigator` → RViz | 当前路由上的车道：半透明车道面 + 蓝色边界 |
| `/hdmap/traffic_light_states`、`/hdmap/traffic_light_markers` | autoware_perception_msgs/TrafficLightGroupArray、MarkerArray | `traffic_light_sim` → `hdmap_navigator` / RViz | 红绿灯状态（group id = 地图里红绿灯 regulatory element 的 id）和显示 |

### 3.3 TF 树

```
map ──(Cartographer)──▶ odom ──(底盘插件)──▶ base_link ──(robot_state_publisher)──▶ imu_link
                                                                                ├─▶ lidar_Link ─▶ livox_sensor
                                                                                ├─▶ camera_Link
                                                                                └─▶ 各车轮
```

- `odom → base_link`：底盘插件发布（仿真里等于车的真实位姿）；
- `map → odom`：Cartographer 发布，修正里程计漂移；
- `map` 坐标系的原点就是建图起点（仿真里即出生点），z=0 在地面高度。

---

## 4. 各部分怎么实现

### 4.1 建图：Cartographer 3D

- 跟踪坐标系是 `imu_link`（3D 模式的要求）；Cartographer 只发布 `map→odom`，`odom→base_link` 由底盘插件提供；
- 雷达测距用到 30 m；
- 回环检测的搜索窗口缩小到 1.5 m：坡道沿线各段看起来很像，默认 5 m 窗口会“对错位置”，曾导致轨迹沿坡道偏 5 m；
- 建图结束时调用 `/finish_trajectory`，Cartographer 做一次全局优化。

### 4.2 2.5D 地形：`terrain.py` + `cloud_to_grid_map.py`

**为什么不直接把所有点合在一起算高度**：Cartographer 3D 的高度估计有 0.1~0.3 m 误差，姿态有约 ±0.5° 抖动。同一块地面在不同时刻建出的高度能差十几厘米，比障碍判定阈值（0.08 m）和台阶阈值（0.05 m）还大，合在一起算，地图几乎全是“不可通行”。

**做法：高度只在单帧内比较，跨帧只投票。**

1. **单帧分类**（`frame_layers`）：取一帧点云，减去该帧的位置，得到“重力对齐、原点在车上”的局部坐标；只用水平 5 m 以内的点；在 0.1 m 网格上计算：
   - 地面高度：每格最低点；
   - 障碍：高出本格地面 0.08~1.2 m 的点足够多；
   - 坡度：地面高度平滑后求梯度；
   - 台阶：0.15 m 范围内的最大高差；
   - 代价：障碍、台阶 > 0.05 m 或坡度 > 15° 记为 100（不可通行）；坡度 2°~15° 的代价从 0 线性增加到 60；更平的记为 0。
2. **全局投票**（`VoteGrid`）：把每帧的判断按水平位置投到全局 0.05 m 网格上。某格至少 2 帧、且不少于 30% 的帧判为不可通行，就记为 100；否则取各帧坡度的平均值来算代价。
3. **封住大片未知区**：面积 ≥ 0.5 m² 的未知区域，四周标为不可通行，这样规划不会穿过没观测到的地方（比如平台开放边缘外侧）。零星的小洞先补上再判断；出生点附近的盲区不封（雷达前倾，车身周围始终看不到）。
4. **去掉够不着的“小岛”**：以建图起点为种子做连通分析，箱子、货车顶面这类被台阶围住、车上不去的地方改为不可通行。
5. **重建**：建图过程中每 5 s 增量更新一次预览；保存时用 `/trajectory_query` 拿到回环优化后的轨迹，修正每帧的位姿后重新投票，得到最终地图。

保存下来的 `map.pgm` 用 raw 模式，像素值就是 0~100 的连续代价，所以规划器能区分平地和坡道，会倾向于走平缓的路线。

### 4.3 实时障碍：`terrain_obstacle_filter.py`

- 每帧点云用 `map→雷达` 的旋转转到重力对齐的局部坐标（只用旋转，不用绝对高度，所以不受定位高度误差影响）；
- 调用同一套单帧分类，只输出“落在障碍格、且高出本格地面 0.08~1.2 m”的点，发到 `/terrain_obstacles`，所以坡面不会被当成障碍；
- 同时把抽稀的地面点发到 `/terrain_ground`，代价地图只用它来做射线清除，这样人走开后不会留下“幽灵障碍”；
- 平台边缘这类固定落差由静态地图负责，实时层不处理向下的落差。

### 4.4 定位：Cartographer 3D 纯定位

- 加载冻结的 `.pbstream`，新开一条定位轨迹，与地图做匹配；
- launch 通过 `/start_trajectory` 服务带初始位姿启动定位（参数 `initial_x/y/z/yaw`，默认建图起点）。给了初始位姿之后只做 1.5 m 窗口的局部匹配（得分 ≥ 0.55），不退回全局搜索：Cartographer 默认连续 10 s 没和地图建立约束就只做全局搜索（得分 ≥ 0.7），而仓储场景正确的局部匹配得分只有 55～70%，启动阶段错过一次就永远配不上地图。代价是车被挪走 / 定位丢失后不能自动找回，要带初始位姿重启；
- 加载 `.pbstream` 要 1～2 分钟，定位就绪前 Nav2 收到的目标会失败；
- 关闭 IMU 外参在线估计：定位轨迹匹配上地图之前，在线估计会让姿态漂到倾斜几十度。

### 4.5 导航：Nav2

| 组件 | 配置 | 说明 |
|---|---|---|
| 全局规划 | Smac Hybrid-A\*，Reeds-Shepp，最小转弯半径 2.3 m（车身中心） | 规划出车能实际开出来的路径，允许倒车 |
| 局部控制 | MPPI，Ackermann 模型，10 Hz，预测 3 s | 采样大量候选轨迹，按代价选最优，跟踪全局路径 |
| 全局 / 局部代价地图 | 静态层（`trinary_costmap: false`，读连续代价）+ 障碍层（`terrain_obstacles` 标记、`terrain_ground` 清除）+ 膨胀层 | 局部代价地图也带静态层：雷达前倾，车后方是盲区，倒车时靠地图兜底 |
| 行为树 | 自定义 `navigate_to_pose_ackermann.xml` | 只在目标变化、路径被挡或路径用了 15 s 时重规划，规划后用约束平滑器平滑；恢复动作用前进 / 后退 0.3 m 代替原地旋转 |
| 速度平滑 | `velocity_smoother`（加减速 0.5 m/s²）+ `cmd_vel_to_ackermann` 转角速率限制 0.8 rad/s | 指令不突变，前轮不猛打 |
| 车体轮廓 | 1.2 × 1.08 m，外扩 5 cm；膨胀半径 1.0 m | 规划和控制用同一个轮廓 |

### 4.6 速度换算：`cmd_vel_to_ackermann.py`

Gazebo 的 Ackermann 插件把 `angular.z` 当作**前轮转角**，而 Nav2 和键盘遥控发的是**角速度**。节点按 `转角 = atan(轴距 × 角速度 / |线速度|)` 换算，并对最大转角 0.414 rad 限幅。插件的速度反馈只用右后轮，节点还按转弯几何对速度做了补偿。

### 4.7 保存地图：`save_map.sh`

依次执行：
1. `/finish_trajectory` 结束建图；
2. 设置 `elevation_path`，调用 `/cloud_to_grid_map/rebuild_from_trajectory` 用优化后的轨迹重建地形；
3. `map_saver_cli --mode raw` 保存 `map.pgm/map.yaml`；
4. `/write_state` 保存 `map.pbstream`。

| 输出文件 | 谁用 |
|---|---|
| `map.pbstream` | 导航时 Cartographer 纯定位加载 |
| `map.pgm` + `map.yaml` | 导航时 map_server 加载，给代价地图静态层 |
| `map_elevation.npz` | 各层数据（高程、坡度、代价等），供查看和评估，导航不读 |

### 4.8 语义地图：`agv_semantic`

- **数据**：语义文件 `<地图前缀>_semantic.yaml`，地点（名字、x、y、yaw）和区域（名字、多边形，可选限速或禁行），都在 `map` 坐标系下。写文件时先写临时文件再改名，中途出错不会损坏原文件。
- **按名字导航**：`semantic.py goto 名字` → `semantic_server` 的 `GoToPlace` 动作 → 查坐标、检查是否在禁行区 → Nav2 `NavigateToPose`，把剩余距离转发回命令行。
- **区域规则**：`make_filter_masks.py` 按 `map.yaml` 的尺寸、原点、分辨率把区域栅格化成两张 raw 模式的掩码。导航 launch 给了 `semantic` 且掩码存在时，启动掩码地图服务和过滤器信息服务，并把过滤器加进代价地图：
  - 禁行过滤器（类型 0）加在全局和局部代价地图上，规划和控制都绕开禁行区；
  - 限速过滤器（类型 2，绝对速度，`multiplier = 0.01`）加在全局代价地图上，车进入限速区时发 `/speed_limit`，MPPI 按比例缩小速度上限；
  - 限速重发：MPPI 每收到新的跟踪目标（包括恢复动作之后）都会把速度上限重置为 `vx_max`，而限速过滤器只在限速变化时发一次。所以 `semantic_server` 每 0.5 s 按车的位置查一次限速，在限速区内重发 `/speed_limit`，离开时发一次 0（不限速）。
- **不给 `semantic` 时**：不启动任何语义节点，代价地图不加过滤器，行为与原来相同。
- **出入库任务**：导航 launch 同时给了 `semantic` 和 `tasks` 时启动 `task_manager`。它把库存和配置放在 `<地图前缀>_tasks.yaml`（只有它写，语义文件仍只由 `semantic_server` 写）。服务回调只改内存里的队列（加锁）；一个工作线程逐个执行任务：调用 `semantic_server` 的 `goto` 去取货点 → 按仿真时间停留 → 装货完成（出库的库位变空）→ 去放货点 → 停留 → 卸货完成（入库的库位变为有货），每次库存变化都写文件。提交时预留库位，预留只在内存里。导航失败重试 1 次；任务失败或被取消后队列暂停，`resume` 才继续；队列空且没暂停时回 `HOME`，途中来了新任务就取消回家。`goto` 的动作名是参数，以后多车时可以换成带 namespace 的名字。

### 4.9 高精地图：`agv_hdmap`

- **地图**：`road_map.py` 按"画边界线 → 组 lanelet → 挂规则"的顺序生成：每个路段先算道路中线（直线或圆弧），按横断面偏移出边界线，
  相邻路段在端点共用点对象；同向车道之间路口前 10 m 是实线、再往外是虚线，线型变化处切开 lanelet；路口里的转弯车道用两端切线等于车道方向的
  三次 Hermite 曲线，边界是 virtual 线；每条 lanelet 带显式中心线（Lanelet2 自己算的中心线在弯道上有锯齿）。十字路口四个进口各挂一个
  `traffic_light` regulatory element（灯 + 停止线，自定义属性 `signal_group`）。`.osm` 用 map 坐标写进每个点的 `local_x` / `local_y`。
- **加载和显示**：Autoware 的投影加载器读 `map_projector_info.yaml`（Local）发 `/map/map_projector_info`，地图加载器读 `.osm` 发 `/map/vector_map`，
  显示节点画成 MarkerArray。不装整个 Autoware，规划和控制不用 Autoware 的。
- **路由和路径**：`hdmap.py` 用 Lanelet2 的路由图（Germany / Vehicle 交通规则）找最短 lanelet 序列；沿中心线采样，相邻两条是左右关系时在前一条的最后
  8 m 用余弦曲线过渡（只会出现在虚线段）。目标在车后面同一条车道上时从后继出发绕回来。
- **执行**：`hdmap_navigator` 用 `plan_legs` 把目标分段：仓库里交给 Nav2 `navigate_to_pose`，公路上把路径交给 controller_server 的 `follow_path`
  （和 Nav2 用同一个 MPPI / TEB 控制器）。交接点是交接车道 `gate_out` 的起点和 `gate_in` 的终点（都在仓库里）。
- **交通规则**：换车道时发这条车道的限速，沿车道开时每 0.5 s 重发；回到仓库、交给 Nav2 前发一次 0，让语义区域的限速过滤器接手。
  每收到红绿灯状态就算前方第一条停止线：红灯或黄灯（车头离线 ≥ 0.5 m）时把路径截到车头在线前 0.25 m 重发，变绿后发剩下的路径。
- **不给 `hdmap` 时**：不启动任何高精地图节点，也不需要编译 `agv_hdmap`，行为与原来相同。

---

## 5. 测试与验证

| 层次 | 方法 |
|---|---|
| 单元测试 | `cd src/agv_nav && python3 -s -m pytest test/ -q`：用人工构造的点云（平地、坡道、平台边缘、箱子、细杆、稀疏环形点云、帧间高度漂移等）检验 `terrain.py`，另有导航 launch 的参数检查（31 项）<br>`cd src/agv_semantic && python3 -s -m pytest test/ -q`：语义文件读写与校验、区域几何、掩码；任务配置、队列、预留、状态流转；`semantic_server` / `task_manager` 节点用假的导航动作测失败重试、取消、回 HOME、限速重发等（91 项，节点测试需要先 `source install/setup.bash`）<br>`python3 -s -m pytest src/agv_description/test src/agv_gazebo/test src/agv_hdmap/test -q`：世界布局、生成的文件和仓库里一致、Lanelet2 地图（命名、曲率、连通、只在虚线变道、限速、红绿灯、通过 `lanelet2_validate`）；定位、路由、路径、停车点、分段 |
| 仿真验证 | 在隔离环境（`ROS_DOMAIN_ID=42`、独立的 Gazebo 端口）里自动建图、导航，拿 Gazebo 真值评估：轨迹误差、坡度、边缘缺口、可达性、导航成功率、离开放边缘的最小距离、坡道上的真实速度、出入库任务的结果和库存 |

实测数据见 [src/agv_nav/README.md](src/agv_nav/README.md) 第 8 节。

---

## 6. 第三方代码

| 目录 | 作用 | 本仓库的修改 |
|---|---|---|
| `src/third_party/livox_laser_simulation_ros2` | Mid-360 的 Gazebo 仿真插件，按真实扫描模式发射射线，发布 `<topic>_PointCloud2` 和 Livox 自定义消息 | 测距补上射线起点距离（原来所有点近 0.1 m）；修复每个点重复输出；`<always_on>` 让不开 Gazebo 界面也有数据；只保留 Mid-360 的扫描模式和模型文件；每帧 2 万点（同真机）；等第一次 `worldUpdateBegin` 才处理扫描，否则多传感器时雷达先于模型初始化完成，gzserver 段错误 |
| `src/third_party/livox_ros_driver2` | Livox 官方驱动 | 无修改；仿真插件用到它的消息定义 |
| `src/third_party/teb_local_planner`、`costmap_converter` | TEB 局部规划器（`controller:=teb`）；不进 git，按根目录 `agv.repos` 拉取 | 转换全局路径时用最新的 TF（`src/third_party/patches/teb_local_planner.patch`） |

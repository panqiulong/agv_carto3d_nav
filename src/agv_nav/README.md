# agv_nav：Cartographer 3D 建图 / 定位 + Nav2 导航

在 Gazebo 仿真（场景 `agv_gazebo`，小车模型 `agv_description`：Ackermann 小车 + 前倾安装的 Mid-360）上实现：

- **建图**：Cartographer 3D（点云 + IMU + 轮式里程计）
- **定位**：Cartographer 3D 纯定位（加载建图得到的 `.pbstream`）
- **全局规划**：Nav2 Smac Hybrid-A\*（Reeds-Shepp，允许倒车）
- **局部控制**：Nav2 MPPI（Ackermann 运动模型）
- **2.5D 地形**：每帧点云在单帧内部判断障碍 / 台阶 / 坡度，按水平位置投票合成连续可通行代价（0~100），能自主上下坡道、不会从平台边缘开下去
- **语义地图与出入库任务**（包 `agv_semantic`，导航 launch 的可选参数 `semantic`、`tasks`）：按名字导航、限速 / 禁行区域、入库单 / 出库单排队执行并维护库存，见 4.1、4.2 节

环境：ROS 2 Humble、Gazebo Classic 11。

---

## 1. 系统结构

```
Gazebo (agv_gazebo)
 ├─ 前后 Mid-360 ──> 两路点云 ─────────────┬─> Cartographer 3D ──> TF map->odom
 ├─ IMU      ──> /imu ─────────────────────┤        │
 ├─ 底盘插件  ──> /odom, TF odom->base_link ─┘        └─> /scan_matched_points2 ──> cloud_to_grid_map ──> /map（建图时）
 │                                         
 │   前后两路点云 ──> terrain_obstacle_filter[_rear] ──> /terrain_obstacles[_rear] ──> Nav2 代价地图障碍层（导航时）
 │
 └─ 底盘插件 <── /cmd_vel_ackermann <── cmd_vel_to_ackermann <── /cmd_vel <── Nav2 / 键盘遥控
```

| 节点 / 文件 | 作用 |
|---|---|
| `cartographer_node` | 建图：`config/carto_3d_mapping.lua`；定位：`config/carto_3d_localization.lua` |
| `scripts/cloud_to_grid_map.py` | 每帧点云在单帧内部判断障碍 / 台阶 / 坡度（不跨帧比较高度，不受 SLAM 高度误差影响），按水平位置投票合成全局可通行代价 `/map`（-1 未知，0~100，100 不可通行），同时发布 `/elevation_map`（GridMap）。建图结束时用回环优化后的轨迹重建 |
| `scripts/terrain.py` | 纯计算模块（单帧分类、投票、封住大片未知区、去掉够不着的“小岛”），`test/test_terrain.py` 是它的单元测试 |
| `scripts/terrain_obstacle_filter.py` | 导航时每帧点云在单帧内部区分地面和障碍（坡面不算障碍），只输出障碍点给代价地图 |
| `scripts/cmd_vel_to_ackermann.py` | 把标准 `/cmd_vel`（线速度 + **角速度**）换算成底盘插件要的（线速度 + **前轮转角**），并补偿插件只用右后轮做速度反馈的问题 |
| `scripts/initial_pose.py` | RViz 的 2D Pose Estimate（`/initialpose`）→ 结束当前 Cartographer 定位轨迹、以点选的位姿重开一条 |
| `scripts/elevator.py` | 进电梯：部署时示教电梯位置；导航到门外等待点，用实时点云提取轿厢，沿轿厢中心线开进去居中停车 / 倒出（见 4.3 节）；纯计算部分在 `scripts/elevator_fit.py`，单元测试 `test/test_elevator_fit.py` |
| `scripts/save_map.sh` | 一次保存 `.pbstream`（定位用）、`.pgm/.yaml`（规划用，raw 模式的连续代价）和 `_elevation.npz`（各层数据） |
| `config/nav2_params.yaml` | Nav2 全部参数（Smac Hybrid-A\*、MPPI、代价地图、速度平滑） |
| `behavior_trees/navigate_to_pose_ackermann.xml` | 导航行为树：只在需要时重规划（目标变了 / 路径被挡 / 用了 15 s），规划后经约束平滑器平滑；恢复动作里的原地旋转换成“向前直行 0.3 m” |

TF 树：`map ─(Cartographer)→ odom ─(底盘插件)→ base_link ─(robot_state_publisher)→ imu_link / lidar_Link / livox_sensor / camera_Link …`

---

## 2. 安装与编译

```bash
sudo apt install ros-humble-cartographer-ros ros-humble-navigation2 ros-humble-nav2-bringup \
                 ros-humble-grid-map-msgs ros-humble-grid-map-rviz-plugin python3-scipy python3-pytest \
                 ros-humble-aws-robomaker-small-warehouse-world     # warehouse 里的货架、托盘模型
cd ~/AGV2
colcon build
source install/setup.bash
```

---

## 3. 建图

**终端 1：启动仿真**

```bash
ros2 launch agv_gazebo gazebo_sim.launch.py                    # 50 x 30 m 仓储场景
```

| `world` | 场景 | 出生点 (x, y, yaw) |
|---|---|---|
| `warehouse`（默认） | 50 × 30 m 仓储场景：北墙收货月台 R1/R2、发货月台 S1/S2，西边 4 排货架（库位 A1..A8），东南角 1.0 m 高夹层（库位 M1..M4），经 5°、3.5 m 宽的坡道上下；东墙北段有一部货梯；南墙大门外是一圈公路（环路、十字路口带红绿灯、T 形路口），四周围栏。见 4.2、4.3、4.4 节 | (2, -4, 0) |

`warehouse.world` 由 `agv_gazebo/scripts/gen_warehouse_world.py` 生成：尺寸、坡度、夹层高度、物体位置都在脚本开头，改完执行 `python3 src/agv_gazebo/scripts/gen_warehouse_world.py` 重新生成（脚本会检查物体之间、物体与坡道口之间的间距）。

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
- 最小转弯半径：后轴中心理论 1.94 m、实测满舵约 **2.19 m**，房间小的地方需要倒车掉头。
- 速度 0.3 ~ 0.5 m/s 即可，太快会增加漂移。
- 前后各一颗雷达，都朝外下方倾斜约 47°。前雷达看不到车尾后方的地面，后雷达看不到车头前方的地面，两颗合起来覆盖车身四周。
- 最后**回到起点附近再走一段**，让 Cartographer 做回环闭合。
- RViz 里 `/map` 会实时更新，这只是预览。保存时会用优化后的轨迹重新生成一次。

**保存地图**（在终端 3 停止遥控后执行）：

```bash
ros2 run agv_nav save_map.sh ~/AGV2/maps/map
```

这个脚本依次执行：结束轨迹 → 用优化后的轨迹重建地形 → 保存 `map_elevation.npz` 和 `map.pgm/map.yaml`（raw 模式，像素值就是 0~100 的连续代价）→ 保存 `map.pbstream`。完成后输出 `saved: ...`。

建图时要让车**在坡道上下都走一遍、平台顶面也开一圈**：没观测到的大片区域会被当成不可通行（四周封上），规划不会穿过它们。

**在 RViz 里看高程图**：Add → By topic → `/elevation_map`（`grid_map_rviz_plugin/GridMap`），Height layer 选 `elevation`，Color layer 选 `cost`。

保存后**重新编译一次**，地图才会被安装到导航 launch 默认读取的位置：

```bash
colcon build --packages-select agv_nav
```

> 也可以不重新编译，启动导航时用 `pbstream:=` 和 `map:=` 直接指定文件路径，见下一节。

---

## 4. 定位 + 导航

**终端 1：启动仿真**（和建图时一样）

```bash
ros2 launch agv_gazebo gazebo_sim.launch.py
```

**终端 2：启动定位和导航**

```bash
ros2 launch agv_nav navigation.launch.py \
    pbstream:=$HOME/AGV2/maps/map.pbstream \
    map:=$HOME/AGV2/maps/map.yaml
```

在 RViz 里用 **Nav2 Goal** 工具点击目标点、拖动方向即可。

> **初始位姿：** 定位启动时通过 `initial_x` / `initial_y` / `initial_yaw`（map 坐标系，默认 0 = 建图起点）告诉 Cartographer 车在哪里，之后只在 1.5 m 窗口内做局部匹配，**不做全图搜索**：车被挪走或定位丢失后不会自动找回（关掉全图搜索的原因见下面参数表）。仿真每次都把车放在出生点，建图也从出生点出发时用默认值即可。
> 车不在建图起点、或定位丢了时，在 RViz 里用 **2D Pose Estimate** 点出车的大致位置和朝向（误差 1.5 m 以内）：`initial_pose.py` 会结束当前定位轨迹、以点选的位姿重开一条，不用重启导航。点得不准的部分要车开起来后才被局部匹配纠正，静止时定位停在点选的位置，所以尽量点准，刚点完开慢一点。点选的 z 当作 0（地面），车在夹层平台上时仍要用 `initial_z` 启动。

> **等定位就绪再发目标：** Cartographer 加载 `.pbstream` 要 1～2 分钟（带公路的仓库地图实测约 75 s），这期间 `map→odom` 还没有，Nav2 会拒绝目标。看到 RViz 里车的位置出现、Cartographer 日志开始出现 `on submap (0, …)` 的约束后再发目标。

launch 参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `pbstream` | 必填 | Cartographer 地图 |
| `map` | 必填 | Nav2 栅格地图 |
| `params_file` | `share/agv_nav/config/nav2_params.yaml` | Nav2 参数 |
| `use_rviz` | `true` | 是否打开 RViz（建图 launch 也有这个参数） |
| `use_sim_time` | `true` | 仿真时间 |
| `initial_x` / `initial_y` / `initial_z` / `initial_yaw` | `0.0` | 定位初始位姿，相对建图起点（map 原点），yaw 为弧度；在平台上启动时 `initial_z` 填平台高度 |
| `semantic` | 空 | 语义文件（地点与区域），如 `maps/map_semantic.yaml`；空 = 不用语义地图 |
| `tasks` | 空 | 任务配置与库存，如 `maps/wh/wh_tasks.yaml`；给了就启动 `task_manager`，必须同时给 `semantic` |
| `hdmap` | 空 | 公路的 Lanelet2 高精地图，如 `maps/wh/wh_lanelet2_map.osm`；给了就启动 Autoware 的地图加载 / 显示和 `agv_hdmap` 的导航节点、仿真红绿灯 |

### 4.1 语义地图：地点与区域

包 `agv_semantic` 给地图标注**有名字的地点**和**区域**，可以按名字导航；区域可以限速或禁止进入。

**语义文件**：每张地图一份，命名为 `<地图前缀>_semantic.yaml`，坐标在 `map` 坐标系下：

```yaml
places:
  - {name: P2装卸口, x: 38.6, y: -18.8, yaw: 0.0}
zones:
  - name: P1坡道
    polygon: [[-14.5, -5.0], [-9.2, -5.0], [-9.2, 9.0], [-14.5, 9.0]]
    speed_limit: 0.3        # 可选，m/s，范围 (0, 0.5]（车的最高速度 vx_max）
  - name: 南门洞
    polygon: [[12.0, -10.0], [16.0, -10.0], [16.0, -2.0], [12.0, -2.0]]
    keepout: true           # 可选，禁止进入
```

规则：名字不能为空、不能重复；多边形至少 3 个点；一个区域不能同时设限速和禁行。格式不对时 `semantic_server` 启动失败并指出是哪一条。

**启动**：导航时带上语义文件（文件不存在也可以，第一次添加地点时会创建）：

```bash
M=$HOME/AGV2/maps
ros2 launch agv_nav navigation.launch.py pbstream:=$M/map.pbstream map:=$M/map.yaml semantic:=$M/map_semantic.yaml
```

**地点**（把车开到目标位置再标注，“开到哪、标到哪”）：

```bash
ros2 run agv_semantic semantic.py add P2装卸口        # 把车当前位置保存为地点（已存在时加 --overwrite）
ros2 run agv_semantic semantic.py goto P2装卸口       # 按名字导航，显示剩余距离，Ctrl+C 取消
ros2 run agv_semantic semantic.py list               # 列出地点和区域
ros2 run agv_semantic semantic.py remove P2装卸口
ros2 run agv_semantic semantic.py where              # 车当前所在区域（话题 /current_zone）
```

目标地点在禁行区里时，`goto` 直接拒绝并说明是哪个禁行区。

**区域**：编辑语义文件的 `zones`（坐标可以在 RViz 里把鼠标放到地图上读出），然后生成 Nav2 过滤器用的掩码，再重启导航：

```bash
ros2 run agv_semantic make_filter_masks.py <map.yaml> <map_semantic.yaml>
```

改了区域但没重新生成掩码时，`semantic_server` 启动会警告。没有掩码时禁行不生效；限速仍然有效：`semantic_server` 按车的位置在限速区内每 0.5 s 发一次 `/speed_limit`（MPPI 每收到新的跟踪目标都会把限速重置，限速过滤器又只在变化时发一次，不重发的话车在限速区里触发恢复动作后就全速了）。

**RViz**：Add → By topic → `/semantic_markers`：地点是蓝色箭头；禁行区红色、限速区黄色、其他区域灰色。

### 4.2 出入库任务

在语义地图之上，`agv_semantic` 的 `task_manager` 节点提供**入库单**（收货月台 → 库位）和**出库单**（库位 → 发货月台）：任务排队逐个执行，维护一份库存，排队中的任务会预留库位，互不冲突。执行过程：去取货点 → 停留（模拟装货）→ 去放货点 → 停留（模拟卸货）；队列空了回 `HOME`。

**仓储场景 `warehouse`**（`agv_gazebo/scripts/gen_warehouse_world.py` 生成）：北墙收货月台 R1/R2、发货月台 S1/S2；西边地面货架区（库位 A1..A8）；东南角 1.0 m 高的夹层（库位 M1..M4），经一条 5°、3.5 m 宽的坡道上下。生成器同时输出语义文件（17 个地点 + 坡道/夹层限速区）和任务配置模板，建完图不用手工标注。

建好仓储地图后（地图、语义文件 `wh_semantic.yaml`、任务配置 `wh_tasks.yaml` 和区域掩码都放在 `maps/wh/` 文件夹里，统一用前缀 `wh`；`maps/` 里的文件不进 git，`.pbstream` 上百 MB），这样启动：

```bash
ros2 launch agv_gazebo gazebo_sim.launch.py world:=warehouse
M=$HOME/AGV2/maps/wh
ros2 launch agv_nav navigation.launch.py pbstream:=$M/wh.pbstream map:=$M/wh.yaml \
    semantic:=$M/wh_semantic.yaml tasks:=$M/wh_tasks.yaml
ros2 run agv_semantic task.py submit inbound --from R2 --to M1      # 提交一个入库单，车会自己去执行
```

建图方法：仿真开 `world:=warehouse`，按第 3 节建图并保存为 `maps/wh/wh`，再把生成器输出的语义文件和任务模板放到地图旁边、生成区域掩码：

```bash
W=$(ros2 pkg prefix agv_gazebo)/share/agv_gazebo/worlds
cp $W/warehouse_semantic.yaml maps/wh/wh_semantic.yaml
cp $W/warehouse_tasks.yaml maps/wh/wh_tasks.yaml
ros2 run agv_semantic make_filter_masks.py maps/wh/wh.yaml maps/wh/wh_semantic.yaml
```

`wh_tasks.yaml` 会随任务执行不断改写（库存），想恢复初始库存就重新从生成器复制一份。RViz 里加 `/semantic_markers`（MarkerArray）可以看到地点和区域。

生成器里 `map = 世界坐标 − 出生点`：建图必须从 `warehouse` 的默认出生点开始。`tasks` 必须和 `semantic` 一起给，否则 launch 报错。

**建图要走遍全场**：雷达外倾，只看得清车前后几米的地面，没走过的巷道和大片区域会被封成不可通行，落在里面的地点就到不了。每条货架巷道都要穿过，开阔区按 3～4 m 间距来回走，夹层上也要来回走几趟。

**地点朝向**：顺着车自然开过来的方向，Ackermann 车在窄处倒车换向很难对准。货架巷道单向通行（东口进、西口出），A 库位和收货月台 R 朝西，发货月台 S 朝东；夹层库位沿逆时针绕圈方向（M1、M2 朝东，M3 朝北，M4 朝西），不设车头朝南的库位：车头冲着南墙停下时，规划器常常算不出掉头再对准坡道口的路径。

**任务配置**（`<地图前缀>_tasks.yaml`，只由 `task_manager` 写；运行中手工改会被覆盖，改库存用 `task.py stock`）：

```yaml
home: HOME
dwell: 5.0                 # 取货、放货各停留的时间，秒（仿真时间）
receiving: [R1, R2]        # 收货月台
shipping: [S1, S2]         # 发货月台
slots:                     # 库位：true = 有货，false = 空；顺序就是自动选择的优先顺序
  A1: false
  A2: true
  M1: false
```

**命令**：

```bash
ros2 run agv_semantic task.py submit inbound                 # 入库：默认第一个收货月台 → 第一个空库位
ros2 run agv_semantic task.py submit inbound --from R2 --to M1
ros2 run agv_semantic task.py submit outbound --from A2      # 出库：A2 → 默认第一个发货月台
ros2 run agv_semantic task.py list                           # 当前任务、队列、最近结束的任务、库存
ros2 run agv_semantic task.py cancel 3
ros2 run agv_semantic task.py resume
ros2 run agv_semantic task.py stock A5 full                  # 手工改库存（被任务预留的库位不能改）
```

入库只能放到空库位，出库只能从有货的库位取；已被排队任务预留的库位不能再用。导航失败会自动重试 1 次；当前任务**失败或被取消后队列暂停**（装货后失败时车上还有货），人检查后用 `resume` 继续。状态以 JSON 发布在 `/task_status`。

### 4.3 电梯：进梯对齐 / 出梯

warehouse 世界东墙 y = 11.2（世界坐标，发货月台车道中线）处有一部货梯：门洞宽 1.8 m，轿厢内净 2.0 × 2.6 m（只比车宽 0.9 m），门一直开着、不升降。
导航的到点容差和定位误差都比这个余量大，所以进梯这一段以**实时点云看到的轿厢**为准：车先导航到门外 3.85 m 的等待点，
从原始点云里拟合出两侧墙和后墙，再以后轴为参考点沿轿厢中心线纯跟踪开进去，车体中心停在轿厢中心。

前提：wh 地图是加了电梯之后重建的（建图时沿北边车道往东一直开到电梯门前）；仿真和 `navigation.launch.py` 已经起来。

电梯在地图里的位置不从仿真世界拿，和真实部署一样**每张地图示教一次**：把车（遥控或导航）停在门外约 5.45 m、
车头大致对着门（航向偏 25° 以内都能找到），跑 `teach`。它用实时点云提取轿厢，把轿厢位姿和中心线上的等待点
按当前定位换到 map 系，写进电梯文件。文件和地图放在一起，地图重建后要重新示教。

```bash
E=maps/wh/wh_elevator.yaml
ros2 run agv_nav elevator.py teach  $E   # 示教：提取轿厢，写电梯文件（已有就覆盖）
ros2 run agv_nav elevator.py enter  $E   # 导航到等待点 -> 提取 -> 进梯居中；成功返回 0，失败返回 1
ros2 run agv_nav elevator.py exit   $E   # 沿中心线倒出到等待点
ros2 run agv_nav elevator.py detect $E   # 只提取、每秒打印结果，不动车（调试用）
```

- 完成标准：停车后车体中心相对轿厢中心横向 < 5 cm、航向 < 2°、纵向 < 10 cm，`enter` 最后打印一行 `ELEVATOR_ENTER … OK|FAIL`；
- 速度发到 `cmd_vel_nav`（velocity_smoother 的输入）；车体外扩 5 cm 内有障碍点、10 s 没有前进、看不到轿厢时都会停车并返回 1；
- 车已经在轿厢里时 `enter` 直接拒绝，车不在轿厢里时 `exit` 直接拒绝；
- 电梯文件只用来框选区域（四周外扩 0.5 m）；轿厢尺寸（电梯规格）、等待距离等参数在 `config/elevator_params.yaml`；
- `agv_gazebo/worlds/warehouse_elevator.yaml` 是仿真世界的真值（世界坐标减出生点），进梯程序不再读它；
- RViz 里加 `/elevator/pose`（PoseStamped，提取出的轿厢中心）和 `/elevator/walls`（Marker，拟合出的墙线）可以看提取结果。

### 4.4 公路与高精地图

warehouse 世界南墙 x ∈ [0, 6]（世界坐标）开了大门，外面是一圈公路，配一张 Autoware 格式的 Lanelet2 地图（`agv_gazebo/worlds/warehouse_lanelet2_map.osm`）。
带上 `hdmap:=<.osm>`（`start.sh nav warehouse` 在 `maps/wh/wh_lanelet2_map.osm` 存在时自动带）后，在 RViz 里用 "HD Map Goal" 点目标，
车在公路上按车道路由、沿车道开，遵守限速、停止线、红绿灯；目标或车在仓库里时，仓库里那段自动交给 Nav2。

```bash
ros2 run agv_hdmap hdmap_cli.py goto <x> <y>     # map 坐标；到了返回 0，失败返回 1
```

前提：wh 地图是加了公路之后重建的（建图路线要把公路每条车道都开到，`cloud_to_grid_map` 的 `size` 默认 130 m，盖住整个围栏）。
用法、地图里的概念和怎么改地图见 [agv_hdmap/README.md](../agv_hdmap/README.md)。

---

## 5. 主要参数

| 位置 | 参数 | 当前值 | 说明 |
|---|---|---|---|
| `nav2_params.yaml` → `planner_server` | `minimum_turning_radius` | 2.3 | base_link（车身中心）的半径，不能小于车实际能转的半径（后轴实测 2.19 m 换算到车身中心约 2.23 m）；规划得比车紧时，终点前的倒车弧线转不过来，车会前后反复挪 |
| 同上 | `motion_model_for_search` | `REEDS_SHEPP` | 允许倒车；改成 `DUBIN` 则只能前进 |
| `controller_server` → `FollowPath` | `AckermannConstraints.min_turning_r` | 2.2 | 后轴半径（vx / wz），不小于实测满舵 2.19 m |
| 同上 | `vx_max` / `vx_min` | 0.5 / -0.3 | 前进 / 倒车最大速度 |
| 同上 | `controller_frequency`, `model_dt`, `time_steps` | 10 Hz, 0.1, 30 | 预测 3 s |
| `local_costmap` / `global_costmap` | `footprint` | 1.2 × 1.08 m | 车体轮廓 |
| 同上 | `inflation_radius` | 1.0 m | 大于车体外接圆半径（0.8 m） |
| `local_costmap` | `width` / `height` | 7 m | |
| 同上 | `footprint_padding` | 0.05 | 规划和控制都按外扩 5 cm 的轮廓，防止贴墙 |
| `velocity_smoother` | `max_velocity` / `max_accel` | 0.5 m/s / 0.5 m/s² | |
| `smoother_server` → `constrained_smoother` | `minimum_turning_radius` / `w_cost` | 2.3 / 0.015 | 全局路径平滑：满足最小转弯半径、远离障碍；平滑失败时沿用原路径 |
| 行为树 `PathExpiringTimer` | `seconds` | 15 | 路径有效且目标没变时最多 15 s 重规划一次；1 Hz 无条件重规划时路径方案来回跳，车会来回摆 |
| `cmd_vel_to_ackermann` | `max_steer_rate` | 0.8 rad/s | 前轮转角变化速率上限（低速时 wz 小变化会让转角大幅跳变） |
| `carto_3d_localization.lua` | `min_score` / `global_localization_min_score` | 0.55 / 0.7 | 局部 / 全局匹配门槛；局部搜索窗口 1.5 m；`global_constraint_search_after_n_seconds` 1e6 s（等于不退回全局搜索）：原来 10 s 内没配上地图就只做全局搜索，仓储场景正确匹配得分 55～70% 过不了全局门槛 0.7，启动阶段一旦错过就永远配不上（2026-11-01 实测开一圈 0 条约束） |
| 同上 | `use_online_imu_extrinsics_in_3d` | `false` | 在线估计 IMU 外参时，没匹配上地图的定位轨迹姿态会漂到几十度 |
| `cloud_to_grid_map` | `max_step` / `max_slope_deg` / `max_slope_cost` | 0.05 m / 15° / 60 | 台阶超过 5 cm 或坡度超过 15° 不可通行；2°~15° 的坡代价线性 0~60 |
| `cloud_to_grid_map` / `terrain_obstacle_filter` | `frame_range` | 5 m | 单帧只用水平 5 m 内的点（姿态误差 0.5° 在 5 m 处约 4 cm） |
| `local_costmap` / `global_costmap` → `static_layer` | `trinary_costmap` | `false` | 读连续代价，坡道代价能传给规划器 |
| `cmd_vel_to_ackermann` | `wheelbase` / `max_steer` / `rear_track` | 0.855 / 0.414 / 0.93 | 必须和 URDF 一致 |
| `terrain.py` / `carto_3d_mapping.lua` | `SELF_RANGE` / `min_range` | 0.6 m | 离雷达这么近的点打在自己的车身、车轮上，丢掉 |

车辆几何（2026-10-10 整车放大 1.5 倍）：车身 1.125 × 0.75 m，车轮半径 0.187 m，轴距 0.855 m，后轮距 0.93 m；内侧前轮最大转角 30°，对应车辆中心等效转角 0.414 rad，后轴中心最小转弯半径理论 1.94 m、实测满舵约 2.19 m（内侧前轮顶到关节限位，打不满）。放大时质量不变、转动惯量 ×2.25，底盘插件的速度 / 转向 PID 和关节力矩上限 ×2.25。

---

## 6. 已知限制

- **初始位姿**：车不在建图起点时要用 RViz 的 2D Pose Estimate 点出大致位姿（或用 launch 参数 `initial_*` 启动）；全图搜索关掉了，车被挪走后不会自动找回。
- **可通行地图是离线的**：建图之后新增的坡道或台阶只能靠实时障碍层发现，需要重新建图。建图时没观测到的大片区域不可通行。
- **平台开放边缘**：边缘外侧没被观测到时靠“封住未知区”挡住；如果建图时在开放边缘下方开过、而平台顶上的帧又多得多，边缘有可能被投票成可通行（负障碍检测尚未实现）。
- **窄坡道入口**：`industrial` 的 P1 坡道（宽 3 m，内侧约 2.8 m）是按放大前的车（0.8 × 0.72 m、转弯半径 1.3 m）设计的。2026-10-10 用有货架的世界重建地图后，从出生点去 P1 顶面每次都在坡脚切弯、蹭进护栏膨胀区后失败（第 8 节）；和车轮形状、坡道宽度无关，需要单独调 MPPI / 规划参数。`warehouse` 的坡道两端留了 4 m 以上的直线进出段，没有这个问题。
- **终点附近对不准**：Ackermann 车在终点附近要倒车修正时，MPPI 偶尔收敛不了，反复“没有进展”直到超时（`warehouse` 的 A8、M1 各出现过）。任务层会重试 1 次。
- **倒车避障**：后雷达经 `terrain_obstacle_filter_rear` 进局部和全局代价地图，建图之后新出现在车后方的障碍也能标出来。仿真实测车倒向方块时在 0.62 m 外停住（只测过一次）。
- **倒车效率低**：目标在车后方时，规划器通常绕圈掉头而不是直接倒车，纯倒车路径 MPPI 也可能中止。
- **不能原地旋转**：恢复行为用“向前直行 0.3 m”和“后退 0.3 m”代替原地旋转。
- **地图精度**：栅格分辨率为 5 cm，墙面在地图里会比实际向内多出约 0.05 ~ 0.1 m，所以规划会离墙稍远一些。
- **速度跟踪**：底盘插件的速度环只用右后轮做反馈，换算节点已经按 Ackermann 几何做了补偿；转弯时实际车速和横摆角速度约为指令的 85 ~ 93%，由 MPPI 闭环修正。
- **语义地图：限速在下坡时不是硬上限**：限速区限制的是控制器的指令速度（实测 ≤ 0.31 m/s），真实速度 90% 分位约 0.30 m/s，下坡时偶有 0.46 m/s 左右的瞬时峰值。
- **语义地图：区域改动要重启**：改了区域要重新生成掩码并重启导航；地点可以随时增删。目前只能用命令行标注地点、手工编辑区域坐标，还没有 RViz 点选和网页界面。
- **物体级语义地图、语义避障**：尚未实现（车前有深度相机 `camera_Link`）。

---

## 7. 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| Cartographer、Nav2 一直等待，没有 TF 或时间不走 | 没有 `/clock`。必须用 `agv_gazebo` 的 `gazebo_sim.launch.py` 启动仿真，它会加载 `libgazebo_ros_init.so` 并以 200 Hz 发布 `/clock`。自己写启动命令时，要带 `-s libgazebo_ros_init.so --ros-args --params-file .../agv_gazebo/config/gazebo_params.yaml` |
| 雷达没有点云 | 检查 `mid360.xacro` 里的 `<always_on>true</always_on>` 是否还在。没有这个标签时，只有打开 gzclient 窗口雷达才出数据 |
| 键盘遥控车不动 | 必须先启动建图或导航的 launch（里面有 `cmd_vel_to_ackermann` 换算节点）；原地旋转的键无效，见第 3 节 |
| 规划报 `Starting point in lethal space` | 车的轮廓压到了地图障碍上（车太贴墙，或定位偏差较大）。行为树会自动尝试前进 / 后退 0.3 m 脱困；如果一直出现，先确认定位是否正常 |
| 导航时定位突然跳变 / 车的位置姿态明显不对 | 通常是初始位姿不对（车不在建图起点又没用 2D Pose Estimate 或 `initial_*` 给出位姿），或地图质量差（建图时走得太快、没回到起点）。重新建图 |
| `Control loop missed its desired rate` 大量出现 | CPU 不够或 `/clock` 频率太低，见第一条 |
| Gazebo 里看不到货架、托盘，车能直接穿过去 | AWS 模型的网格路径写成 `file://models/...`，要靠 `GAZEBO_RESOURCE_PATH` 解析。用 `gazebo_sim.launch.py` 启动会自动设置；自己启动 gzserver 时要把 `aws_robomaker_small_warehouse_world` 的 share 目录加进 `GAZEBO_RESOURCE_PATH` |
| 车在坡脚停住不动，指令是前进 | 早期版本车轮碰撞体是圆柱，前轮带转向角压在坡脚折线上起步时会被卡住；现在碰撞体是球（`r2.xacro`）。如果自己改回圆柱会复现 |
| 车在限速区里突然全速 | MPPI 每收到新的跟踪目标都会重置限速；`semantic_server` 在限速区内每 0.5 s 重发 `/speed_limit` 来兜底，确认导航 launch 带了 `semantic` |

---

## 8. 仿真验证结果

以下是 2026-09-26 在仿真里实测的数据（真实 Ackermann 底盘，真值取自 Gazebo）。

> 注意：2026-10-09 之前 AWS 货架 / 托盘模型的网格一直没加载，`industrial` 里实际上没有这些物体；下面 09-26、09-27 的 `industrial` 结果都是在没有货架的世界里测的。2026-10-10 用有货架的世界重测见本节末尾。

**`industrial` 建图 + 地图质量**（对照 `gen_industrial_world.py` 里的真实几何）

- 建图路线跑完；Cartographer 航向误差最大 0.60°（平移误差均值 0.42 m，地图整体相对真值有平移，但定位与地图一致，不影响导航）
- 坡道坡度：P1（设计 5°）中位数 4.43°，P2（设计 10°）中位数 8.98°；坡道上没有不可通行格
- 有护栏的平台边缘：不可通行带连续，没有缺口；开放边缘和没观测到的区域由“封住大片未知区”挡住
- 从出生点经坡道能到两个平台顶面；把坡道堵住后两个平台都到不了（不存在从边缘直接上下平台的路径）

**`industrial` 端到端导航**：出生点 → P1 顶面 → 厂区 B 地面 → P2 顶面（车头朝开放边缘）→ 出生点

- 4 个目标全部到达（用时 140 / 195 / 85 / 225 s，到点真值误差 0.14 ~ 0.80 m），平台上的高度 1.12 m / 0.62 m
- 全程没有掉落，车体离 P2 开放边缘最近 0.60 m
- 没有出现 `Starting point in lethal space`
- 此前的几轮测试中出现过：从侧面拐进 P1 坡道时贴近护栏、卡住后失败 1 次；从 P2 回出生点（约 55 m）超过 400 s 的测试时限 1 次

**`simple_room` 回归**：3 个目标各跑两遍，6 个全部到达，到点真值误差 0.11 ~ 0.30 m。西墙边车头朝西的目标需要在小房间里掉头，其中一次触发多次恢复动作，用了 574 s（另一次 96 s）。

单元测试：`cd src/agv_nav && python3 -s -m pytest test/test_terrain.py -q`（25 项）。

**语义地图**（`industrial` 场景，限速区设在 P1 坡道 0.3 m/s，禁行区设在南门洞）：

- 标注地点：普通导航开到 P1 平台、厂区 B、P2 装卸口，3 个 `add` 全部成功
- 按名字导航：到 P1 平台成功 4 次（到点真值误差 0.46 ~ 0.49 m，平台上 z = 1.12 m）；到 P2 装卸口成功（误差 0.35 m，z = 0.62 m）；到厂区 B 成功 1 次、失败 2 次（1 次超过测试时限、1 次 Nav2 放弃，都发生在从 P1 平台沿坡道下来的途中，见第 6 节“窄坡道入口”）
- 禁行区：每次去厂区 B 都绕开南门洞，从北门洞（y = 6.75）穿过隔墙；目标地点设在禁行区里时 `goto` 被拒绝
- 限速区：控制器输出的速度最大 0.31 m/s；上坡时真实速度最大 0.33 m/s；下坡时真实速度短暂冲到 0.49 ~ 0.52 m/s（90% 分位 0.30 ~ 0.32 m/s）
- 当前区域：进出 P1 坡道时 `/current_zone` 在 `P1坡道` 和空之间正确切换
- 不带 `semantic` 时：没有任何语义节点和过滤器，代价地图配置与之前相同（只多了三段未被引用的过滤器参数）。同一张地图上的对照测试在 P1 坡脚卡住（与第 6 节“窄坡道入口”相同的现象），4 个目标未完成

单元测试：`cd src/agv_semantic && python3 -s -m pytest test/ -q`（27 项）；`cd src/agv_nav && python3 -s -m pytest test/ -q`（28 项）。

**仓储场景与出入库任务**（2026-09-30，`warehouse` 世界，真值取自 Gazebo）

- 建图（覆盖式路线，约 25 分钟）：轨迹误差均值 0.047 m、最大 0.26 m，航向最大 0.30°；4 排货架、暂存托盘、柱子、夹层都在地图上
- 按名字导航 17 个地点：15 个成功，到点真值误差 0.09 ~ 0.31 m，夹层库位 z = 1.12 m。失败 2 个：A8（从 A7 倒车 6 m 过去，终点附近没收敛；同样几何的 A2、A4、A6 都成功，属于偶发）、R1（当时朝东，车要在西墙前约 5 m 的空间里转弯，已改为朝西，后面的任务测试里 R1 用了 5 次都成功）
- 出入库任务（一次运行，9 个任务）：
  - 正常：地面入库 R1→A1、夹层入库 R2→M1、夹层出库 M2→S1、地面出库 A4→S2，4 个全部完成，共 1320 s；库存（内存与文件）与预期一致
  - 失败：R1→A6（A6 临时放进禁行区），装货后去 A6 两次被拒 → `failed`，提示货在车上，A6 不变，队列暂停，后面的任务不动；`resume` 后 R1→A5 完成
  - 取消：排队中的出库单取消后库位预留立即释放、队列不暂停；执行中（已装货）取消后 10 s 内车移动 0.018 m，任务 `canceled`、提示货在车上、队列暂停；`resume` 后下一单完成
  - 全程没有跌落
- 坡道限速（2026-10-09 修复）：原来坡道上测到 0.66 ~ 0.73 m/s（限速 0.3 m/s），原因是 MPPI 在每次新的跟踪目标时重置限速、而限速过滤器只在变化时发一次。改为 `semantic_server` 在限速区内周期性重发，并把坡道限速区延到坡脚外 2 m 后，控制器输出最高 0.31 m/s，真实速度 90% 分位 0.30 m/s（下坡偶有 0.46 m/s 的瞬时峰值）
- 坡脚卡车（2026-10-09 修复）：车轮碰撞体原来是圆柱，前轮带转向角压在坡脚折线上起步时圆柱边缘顶住坡面，车在坡脚一卡几分钟。碰撞体改成同半径的球后，HOME → M1 ~ M4 → HOME 全部到达，坡脚停顿从 150 ~ 560 s 降到 33 s
- `industrial` 回归（2026-10-10，修好 AWS 模型后重建地图）：4 个目标全部失败，车每次都在 P1 坡脚切弯蹭进护栏膨胀区（MPPI 报 Optimizer fail，随后规划器报起点在致命代价区）。对照：换回圆柱车轮、把坡道加宽到 3.5 m 都一样失败，09-27 的测试里也出现过，属于第 6 节“窄坡道入口”这个老问题，需要单独调 MPPI / 规划参数，这次没有处理
- 修复：AWS 仓库模型（货架、托盘）的网格此前一直没加载（`gazebo_sim.launch.py` 缺 `GAZEBO_RESOURCE_PATH`），仿真里这些物体不存在。修复后 `industrial` 世界里也有了货架，**之前保存的 `industrial` 地图需要重新建**

单元测试：`cd src/agv_semantic && python3 -s -m pytest test/ -q`（91 项）；`cd src/agv_nav && python3 -s -m pytest test/ -q`（31 项）。

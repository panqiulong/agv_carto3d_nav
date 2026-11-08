# agv_carto3d_nav

Ackermann 小车 + 前后两颗 Livox Mid-360 的 ROS 2 Humble / Gazebo Classic 仿真，包含：

- **Cartographer 3D** 建图与纯定位（点云 + IMU + 轮式里程计）
- **Nav2**：全局规划用 Smac Hybrid-A\*（Reeds-Shepp，可倒车），局部控制用 MPPI（Ackermann 运动模型）
- **2.5D 地形导航**：从 3D 点云生成连续可通行代价地图（坡度、台阶、障碍），能自主上下坡道、不会从平台边缘开下去；导航时实时区分地面和障碍
- **语义地图**：有名字的地点（按名字导航）、区域（限速 / 禁行，用 Nav2 代价地图过滤器实现）
- **出入库任务**：入库单（收货月台 → 库位）、出库单（库位 → 发货月台），任务排队执行，维护库存和库位预留
- **高精地图（学习用）**：仓库外一圈公路配 Autoware 格式的 Lanelet2 地图，按车道路由、沿车道开、遵守限速 / 停止线 / 红绿灯，
  和仓库里的 Nav2 自动交接；学习指南见 [src/agv_hdmap/README.md](src/agv_hdmap/README.md)
- 仿真世界：50 × 30 m 仓储场景（收发货月台、货架区、经 5° 坡道上下的夹层存储区、东墙外的货梯），南墙大门外是公路（环路、十字路口、T 形路口）

## 目录结构

```
maps/                                建好的地图，每张一个文件夹（不进 git，导航时用绝对路径传入，不装进 install/）
src/
├── agv_nav/                         建图、定位、导航、2.5D 地形（launch、Cartographer/Nav2 配置、节点与脚本）
├── agv_semantic/                    语义地图（地点与区域）、按名字导航、出入库任务
├── agv_hdmap/                       公路的 Lanelet2 高精地图：读地图、车道路由、沿车道行驶、交通规则、和 Nav2 交接
├── agv_description/                 小车模型（URDF/xacro、网格）
├── agv_gazebo/                      Gazebo 仓储场景（世界生成器，以及同一份几何生成的语义、任务、电梯、Lanelet2 地图）、仿真启动文件
└── third_party/                     第三方包（见下文「第三方代码」）
```

## 环境与编译

需要 Ubuntu 22.04 + ROS 2 Humble + Gazebo 11。

**1. 安装依赖**

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-xacro ros-humble-robot-state-publisher \
                 ros-humble-cartographer-ros ros-humble-navigation2 ros-humble-nav2-bringup \
                 ros-humble-grid-map-msgs ros-humble-grid-map-rviz-plugin ros-humble-teleop-twist-keyboard \
                 python3-scipy python3-pytest python3-vcstool \
                 ros-humble-aws-robomaker-small-warehouse-world \
                 ros-humble-lanelet2 ros-humble-autoware-map-loader ros-humble-autoware-map-projection-loader \
                 ros-humble-autoware-lanelet2-map-visualizer ros-humble-autoware-lanelet2-extension-python
```

**2. 安装 Livox-SDK2**（`livox_ros_driver2` 编译需要）

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir build && cd build
cmake .. && make -j && sudo make install
```

**3. 克隆并编译**（这个仓库本身就是一个 colcon 工作区）

```bash
git clone https://github.com/panqiulong/agv_carto3d_nav.git
cd agv_carto3d_nav
vcs import src/third_party < agv.repos     # TEB（controller:=teb 才用到），版本固定在 agv.repos
git -C src/third_party/teb_local_planner apply ../patches/teb_local_planner.patch
cp src/third_party/livox_ros_driver2/package_ROS2.xml src/third_party/livox_ros_driver2/package.xml
colcon build --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
source install/setup.bash
```

之后再改代码，在工作区根目录直接 `colcon build` 即可。

跑全部单元测试：

```bash
colcon test --packages-select agv_description agv_gazebo agv_hdmap agv_nav agv_semantic
colcon test-result --verbose
```

## 快速开始

一键启动（交互菜单选择建图 / 导航和世界；也可以直接带参数）：

```bash
./start.sh                      # 菜单
./start.sh map warehouse        # 建图：键盘遥控开车，Ctrl+C 退出遥控后选择是否保存到 maps/wh/
./start.sh nav warehouse        # 导航：自动带上 maps/wh/wh_semantic.yaml、wh_tasks.yaml、wh_lanelet2_map.osm（存在时）
```

带上高精地图时，在 RViz 里用 "HD Map Goal" 点公路上的目标，或者 `ros2 run agv_hdmap hdmap_cli.py goto <x> <y>`（map 坐标）。

Ctrl+C 关闭全部，日志在 `log/start/`。也可以像下面这样分终端手动启动：

```bash
# 终端 1：仿真（50x30 m 仓储场景）
ros2 launch agv_gazebo gazebo_sim.launch.py

# 终端 2：建图（建完后 ros2 run agv_nav save_map.sh ~/agv_carto3d_nav/maps/map）
ros2 launch agv_nav mapping.launch.py

# 或者：定位 + 导航（需要先建过图）
ros2 launch agv_nav navigation.launch.py
```

仓储场景 + 出入库任务（先按 [src/agv_nav/README.md](src/agv_nav/README.md) 4.2 节建好仓储地图 `maps/wh/`；地图文件 `.pbstream` 上百 MB，不放进 git）：

```bash
ros2 launch agv_gazebo gazebo_sim.launch.py world:=warehouse
M=~/agv_carto3d_nav/maps
ros2 launch agv_nav navigation.launch.py pbstream:=$M/wh.pbstream map:=$M/wh.yaml \
    semantic:=$M/wh_semantic.yaml tasks:=$M/wh_tasks.yaml
ros2 run agv_semantic task.py submit inbound --from R2 --to M1     # 入库：收货月台 R2 → 夹层库位 M1
ros2 run agv_semantic task.py list
```

完整的使用说明、参数说明、已知限制和常见问题见 **[src/agv_nav/README.md](src/agv_nav/README.md)**；各部分的实现方式和数据链路见 **[ARCHITECTURE.md](ARCHITECTURE.md)**。

## 第三方代码

| 目录 | 来源 | 许可证 | 本仓库的修改 |
|---|---|---|---|
| `src/third_party/livox_ros_driver2` | [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | MIT（见目录内 `LICENSE.txt`） | 无 |
| `src/third_party/livox_laser_simulation_ros2` | [LihanChen2004/livox_laser_simulation_ros2](https://github.com/LihanChen2004/livox_laser_simulation_ros2)（Ricardo Casimiro、Lihan Chen；移植自 [Livox-SDK/livox_laser_simulation](https://github.com/Livox-SDK/livox_laser_simulation)） | MIT（见目录内 `LICENSE`） | 1. 测距补上光线起点距离 `minDist`（原来所有点比实际近 0.1 m）<br>2. 修复每个点被重复输出两次<br>3. `mid360.xacro` 加 `<always_on>`，不开 Gazebo 界面也能出数据<br>4. 删除了 Mid-360 以外型号的扫描模式和模型文件（约 150 MB，上游仓库里有） |
| `src/third_party/agv_ackermann_drive` | [ros-simulation/gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs) 3.9.0 的 `gazebo_ros_ackermann_drive.cpp` | Apache-2.0（见文件头） | 速度反馈改为两个后轮里转得慢的那个（原版只读右后轮，右后轮悬空时动力塌陷）；只放了这一个源文件，头文件用系统的 `gazebo_plugins` |
| `src/third_party/teb_local_planner`、`costmap_converter`（不进 git，按 `agv.repos` 拉取） | [rst-tu-dortmund/teb_local_planner](https://github.com/rst-tu-dortmund/teb_local_planner)、[costmap_converter](https://github.com/rst-tu-dortmund/costmap_converter) | BSD | TEB 转换全局路径时用最新的 TF，不按路径时间戳查（`src/third_party/patches/teb_local_planner.patch`） |

## 许可证

除上表中的第三方目录外，本仓库代码采用 [Apache License 2.0](LICENSE)。

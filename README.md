# agv_carto3d_nav

Ackermann 小车 + Livox Mid-360 的 ROS 2 Humble / Gazebo Classic 仿真，包含：

- **Cartographer 3D** 建图与纯定位（点云 + IMU + 轮式里程计）
- **Nav2**：全局规划用 Smac Hybrid-A\*（Reeds-Shepp，可倒车），局部控制用 MPPI（Ackermann 运动模型）
- 从 3D 点云生成 Nav2 可用的 2D 栅格地图，保存时用回环优化后的轨迹重新投影

## 目录结构

```
src/
├── agv_nav/                         建图、定位、导航（launch、Cartographer/Nav2 配置、工具脚本）
├── AGV_SIM/src/agv_urdf/            小车模型、Gazebo 场景、仿真启动文件
├── livox_laser_simulation_ros2/     Mid-360 仿真插件（第三方，有修改，见下文）
└── livox_ros_driver2/               Livox 驱动，仿真插件依赖它的消息定义（第三方）
```

## 环境与编译

需要 Ubuntu 22.04 + ROS 2 Humble + Gazebo 11。

**1. 安装依赖**

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-xacro ros-humble-robot-state-publisher \
                 ros-humble-cartographer-ros ros-humble-navigation2 ros-humble-nav2-bringup \
                 ros-humble-pointcloud-to-laserscan ros-humble-teleop-twist-keyboard python3-scipy
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
cd agv_carto3d_nav/src/livox_ros_driver2
./build.sh humble          # 生成 ROS 2 版 package.xml 并编译整个工作区
cd ../..
source install/setup.bash
```

之后再改代码，在工作区根目录直接 `colcon build` 即可。

## 快速开始

```bash
# 终端 1：仿真
ros2 launch agv_urdf gazebo_sim.launch.py

# 终端 2：建图（建完后 ros2 run agv_nav save_map.sh ~/agv_carto3d_nav/src/agv_nav/maps/map）
ros2 launch agv_nav mapping.launch.py

# 或者：定位 + 导航（需要先建过图）
ros2 launch agv_nav navigation.launch.py
```

完整的使用说明、参数说明、已知限制和常见问题见 **[src/agv_nav/README.md](src/agv_nav/README.md)**。

## 第三方代码

| 目录 | 来源 | 许可证 | 本仓库的修改 |
|---|---|---|---|
| `src/livox_ros_driver2` | [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | MIT（见目录内 `LICENSE.txt`） | 无 |
| `src/livox_laser_simulation_ros2` | [LihanChen2004/livox_laser_simulation_ros2](https://github.com/LihanChen2004/livox_laser_simulation_ros2)（Ricardo Casimiro、Lihan Chen；移植自 [Livox-SDK/livox_laser_simulation](https://github.com/Livox-SDK/livox_laser_simulation)） | MIT（见目录内 `LICENSE`） | 1. 测距补上光线起点距离 `minDist`（原来所有点比实际近 0.1 m）<br>2. 修复每个点被重复输出两次<br>3. `mid360.xacro` 加 `<always_on>`，不开 Gazebo 界面也能出数据<br>4. 只保留 Mid-360 的扫描模式文件和 xacro（其余型号的 csv 共 156 MB，本项目用不到） |

## 许可证

除上表中的第三方目录外，本仓库代码采用 [Apache License 2.0](LICENSE)。

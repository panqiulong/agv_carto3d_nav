# agv_hdmap：公路的 Lanelet2 高精地图

仓库南墙外有一圈公路（环路、十字路口、T 形路口、门路、南支路），配一张 Autoware 格式的 Lanelet2 地图。
车在公路上按车道路由、沿车道开，遵守限速、停止线和红绿灯；进了大门交给 Nav2。

这份文档是**学习指南**：对照这张地图里具体的车道、线和规则，讲清高精地图的四个基本概念，每节都有一段能直接粘贴运行的 Python。

- 地图怎么来的：`agv_gazebo/scripts/road_map.py` 生成几何和地图，`gen_warehouse_world.py` 用同一份几何画 Gazebo 里的路面标线，
  输出 `worlds/warehouse_lanelet2_map.osm`（map 坐标 = 世界坐标 − 出生点 (2, −4)）和 `map_projector_info.yaml`。
- 加载和显示：Autoware 的 `autoware_map_projection_loader`、`autoware_map_loader`、`autoware_lanelet2_map_visualizer`。
- 路由、沿车道开、交通规则：本包，`scripts/hdmap.py`（纯计算，不依赖 ROS）+ `hdmap_navigator.py` + `traffic_light_sim.py`。

## 安装和启动

```bash
sudo apt install ros-humble-lanelet2 ros-humble-autoware-map-loader ros-humble-autoware-map-projection-loader \
    ros-humble-autoware-lanelet2-map-visualizer ros-humble-autoware-lanelet2-extension-python
colcon build --packages-select agv_description agv_gazebo agv_nav agv_hdmap && source install/setup.bash

./start.sh nav warehouse          # maps/wh/ 里有 wh_lanelet2_map.osm 时自动带上高精地图
```

建图（`./start.sh map warehouse`）保存时会把生成的 `.osm` 和投影文件复制到 `maps/wh/`，名字是 `wh_lanelet2_map.osm`、
`wh_map_projector_info.yaml`；已经有了就不覆盖（可能是你手改过的）。

启动后先等定位就绪（Cartographer 加载带公路的地图约 75 s，RViz 里车的位置出现之后）再发目标，不然 Nav2 会拒绝。
雷达位置改过（抬高 0.2 m），这之前建的 `maps/wh/` 要重新建图。

在 RViz 里用工具栏的 **2D Goal Pose** 点公路上（或仓库里）的目标：配置里把它的话题改成了 `/hdmap/goal`
（配置里起的名字 "HD Map Goal" rviz2 不显示，按钮上仍是 2D Goal Pose）；"Nav2 Goal" 仍然直接给 Nav2、不走车道。
两者共用一个控制器：发高精地图目标时会先取消所有正在跑的 Nav2 目标（否则 Nav2 行为树不停重发路径，把车道路径顶掉）；
反过来，车道上开着时再点 Nav2 Goal，高精地图这边会报 `failed` 让给 Nav2。命令行：

```bash
ros2 run agv_hdmap hdmap_cli.py goto <x> <y>     # map 坐标；到了返回 0，失败返回 1
ros2 topic echo /hdmap/status                    # idle / driving / waiting_light / arrived / failed: <原因>
ros2 topic echo /hdmap/current_lanelet           # 车现在在哪条车道上
```

下面的 Python 都在工作空间根目录运行，先做一次准备：

```python
import sys; sys.path.insert(0, 'src/agv_hdmap/scripts')
import hdmap
m = hdmap.load('src/agv_gazebo/worlds/warehouse_lanelet2_map.osm')
name = lambda ll: ll.attributes['name']
attr = lambda o, k: o.attributes[k] if k in o.attributes else None
```

车道的名字：`ring_<段>_<方向>_<in|out>_<序号>`，段是 n / e / s / w（环路北、东、南、西段），方向是 eb / wb / nb / sb（往东、西、北、南），
`in` 是靠中线的车道、`out` 是靠路边的车道，序号按行驶顺序；弯道是 `ring_<ne|se|sw|nw>_corner_<cw|ccw>_<in|out>`（cw = 顺时针 = 环路内侧半幅）；
路口里是 `x_<从哪边>2<到哪边>_<in|out|left|right>`（十字路口）和 `t_...`（T 形路口）；交接车道 `gate_out` / `gate_in`。

## 1. 车道（lanelet）：左右两条边界线

lanelet 是地图的基本单位：**左边界 + 右边界**（相对行驶方向），中心线由两条边界决定。边界线的类型决定它是什么线：
`line_thin` + `solid` 是实线、`dashed` 是虚线，`virtual` 是路口里地面上没画的边。

例子：北段往东外侧的第一段 `ring_n_eb_out_0`（id 18）左边界是虚线（id 6，和内侧车道之间），右边界是实线（id 3，路边）。

```python
ll = m.by_name['ring_n_eb_out_0']
for side, b in (('左', ll.leftBound), ('右', ll.rightBound)):
    print(side, b.id, attr(b, 'type'), attr(b, 'subtype'))
print('中心线', [(round(p.x, 2), round(p.y, 2)) for p in ll.centerline])
print('限速', attr(ll, 'speed_limit'), 'km/h')
```

同一条线被左右两条车道共用：`ring_n_eb_in_0` 的右边界就是 id 6 这条虚线。反方向的车道用 `line.invert()` 引用同一条线。

**RViz**："高精地图"显示里白色实线 / 虚线就是这些边界线，和 Gazebo 里画的标线是同一份几何；每条车道中间有它的 id。

## 2. 车道之间的连接和路由

- **前后相接**：前一条的终点和后一条的起点**共用同一个点对象**。`ring_n_eb_out_1`（id 43）开完可以接直行的 `x_w2e_out`，也可以接右转的 `x_w2s_right`；
- **左右相邻**：两条车道共用一条边界线。这条线是虚线时路由图允许变道（`graph.left()` / `graph.right()` 有结果），实线和 virtual 不允许；
- **路口的转弯车道**：也是 lanelet，起点共用进口车道终点的两个点，终点共用出口车道起点的两个点。

线型变化的地方 lanelet 要切开：路由图按"整条 lanelet 的边界线"判断能不能变道，所以北段往东外侧在路口前 10 m 切成 `_0`（虚线）和 `_1`（实线）。

```python
g = m.graph
for n in ('ring_n_eb_out_0', 'ring_n_eb_out_1'):
    ll = m.by_name[n]
    print(n, '左边可变道到', [name(x) for x in [g.left(ll)] if x], '接着开', [name(f) for f in g.following(ll)])

# 北段往西外侧（十字路口东边）→ 南支路：要先变到内侧车道，再左转
route = g.shortestPath(m.by_name['ring_n_wb_out_0'], m.by_name['branch_sb'])
print(' → '.join(name(ll) for ll in route))
# ring_n_wb_out_0 → ring_n_wb_in_0 → ring_n_wb_in_1 → x_e2s_left → branch_sb
```

路由只是 lanelet 序列，`hdmap.path()` 把它变成车走的线：沿中心线走，变道时在前一条的最后 8 m 用余弦曲线移到旁边那条的中心线上。

```python
start, goal = m.by_name['ring_n_wb_out_0'], m.by_name['branch_sb']
lanes = hdmap.route(m, start, goal, 1.0, 3.0)
pts = hdmap.path(m, lanes, (25.0, -19.25), (-0.25, -34.0))
print(len(pts), '个点，前 3 个', pts[:3])
```

**RViz**："路由"显示把这次路由选中的车道画成半透明灰色面 + 蓝色边界（和 Autoware 规划界面里的路由一样），
"高精地图路径"（绿色宽带）是导航节点当前发给控制器的路径，变道的 S 弯能看得出来。
"高精地图"显示里的车道面是 Autoware 显示节点画的不透明深灰色，在 RViz 默认的深色背景上不明显，可以把背景调浅（Global Options → Background Color）。

## 3. 交通规则：regulatory element

规则是独立的对象，再**挂**到 lanelet 上。限速是例外，它直接是 lanelet 的属性 `speed_limit`（km/h，Autoware 约定）：
直路 1.8（0.5 m/s），弯道和路口里 1.08（0.3 m/s）。

红绿灯是 `traffic_light` 类型的 regulatory element，引用**灯**（一条 `traffic_light` 线，带高度）和**停止线**（`stop_line`），挂在路口每个进口的车道上。
十字路口有 4 个：

| id | 组 | 停止线 id | 挂在 |
|---|---|---|---|
| 2204 | ew | 2203 | `ring_n_eb_out_1`、`ring_n_eb_in_1`（西进口） |
| 2209 | ew | 2208 | `ring_n_wb_out_1`、`ring_n_wb_in_1`（东进口） |
| 2214 | ns | 2213 | `gate_sb`（北进口，门路） |
| 2219 | ns | 2218 | `branch_nb`（南进口，南支路） |

```python
ll = m.by_name['ring_n_eb_out_1']
for tl in ll.trafficLights():
    print('红绿灯', tl.id, '组', attr(tl, 'signal_group'),
          '停止线', tl.stopLine.id, [(round(p.x, 2), round(p.y, 2)) for p in tl.stopLine],
          '灯', [(lamp.id, attr(lamp, 'height')) for lamp in tl.trafficLights])
print('限速 m/s', hdmap.speed_limit(ll), hdmap.speed_limit(m.by_name['ring_ne_corner_cw_out']))
```

`signal_group`（ew / ns）是本项目自定义的属性，`traffic_light_sim` 按它切换：绿 20 s → 黄 3 s → 红 23 s，两组交替。
灯的状态用 Autoware 的消息 `autoware_perception_msgs/TrafficLightGroupArray` 发到 `/hdmap/traffic_light_states`，group id 就是上表的 id。

导航节点沿路径找前方第一条车头还没过的停止线：红灯，或黄灯且车头离线还有 0.5 m 以上时，把路径截到车头在线前 0.25 m 处（`hdmap.stop_index`）；
变绿后发剩下的路径。黄灯时离线已经不到 0.5 m 的直接通过，之后变红也不再停。换车道时往 `/speed_limit` 发这条车道的限速。

```python
pts = hdmap.path(m, hdmap.route(m, m.by_name['ring_n_eb_out_0'], m.by_name['ring_n_eb_out_2'], 1.0, 5.0),
                 (-19.0, -26.75), (12.0, -26.75))
k = hdmap.stop_index(m, hdmap.route(m, m.by_name['ring_n_eb_out_0'], m.by_name['ring_n_eb_out_2'], 1.0, 5.0),
                     pts, 0, {2204: 'red'})
print('红灯时停在', pts[k][:2], '（停止线 x = -4.25）')
```

**RViz**：停止线是红色粗线，红绿灯位置有当前颜色的球（"红绿灯"显示）；`ros2 topic echo /hdmap/status` 等红灯时是 `waiting_light`。

## 4. 地图坐标和车的定位

`.osm` 里每个点有 `local_x` / `local_y`（map 坐标，单位 m），经纬度只是 UTM 投影的占位。Autoware 的 Local 投影
（`map_projector_info.yaml` 里 `projector_type: Local`）和 `hdmap.load()` 都直接用 `local_x` / `local_y`，所以地图和 Cartographer 的 map 系对齐：
map 原点是出生点，世界 (x, y) 在地图里是 (x − 2, y + 4)。

车在哪条车道上（`hdmap.locate`）：包含车位置的 lanelet 里，车道方向和车头方向差 < 90° 的、离中心线最近的那条。路口里几条转弯车道重叠，靠车头方向区分。

```python
print(name(hdmap.locate(m, 2.25, -26.75, 0.0)))          # 十字路口里朝东 → x_w2e_out
print(name(hdmap.locate(m, 2.25, -26.75, 1.5708)))       # 同一点朝北 → x_s2n
print(hdmap.locate(m, -16.0, -34.0, None))               # 环路中间的空地 → None
p = m.by_name['gate_out'].centerline[0]
print('出门交接点', p.x, p.y, attr(p, 'local_x'), attr(p, 'local_y'))
```

**RViz**：`/hdmap/current_lanelet` 是车当前所在车道的名字；地图显示和激光点云、栅格地图叠在一起，能看出车道和路面对得上。

## 仓库 ⇄ 公路交接

`hdmap.plan_legs` 按车和目标是否在车道上分段：

| 车 | 目标 | 分段 |
|---|---|---|
| 公路上 | 公路上 | 沿车道 |
| 仓库里 | 公路上 | Nav2 到 `gate_out` 起点（仓库里、车头朝南）→ 沿车道 |
| 公路上 | 仓库里 | 沿车道到 `gate_in` 终点 → Nav2 |
| 仓库里 | 仓库里 | Nav2 |

目标既不在车道上、也不在仓库里（比如环路中间的空地）时直接拒绝：`failed: goal off road`。

## 用 JOSM 修改地图

1. 把地图**复制出去**再改（重新生成世界会覆盖 `worlds/` 里的文件），例如复制成 `maps/wh/wh_lanelet2_map.osm`；
2. 用 JOSM 打开 `.osm`（不需要底图）：节点的 `local_x` / `local_y` 是真正的坐标，挪点时要同时改这两个标签
   （JOSM 显示的位置是按经纬度画的，只用来看形状）。Autoware 的 Vector Map Builder（网页工具）可以直接编辑 local 坐标；
3. 改完可以先检查：`/opt/ros/humble/lib/x86_64-linux-gnu/lanelet2_validation/lanelet2_validate --map_file <osm> --lat 35 --lon 139`（只要没有 Error；`local_x` 等未知标签的 Warning 是正常的）；
4. `./start.sh nav warehouse` 用的就是 `maps/wh/wh_lanelet2_map.osm`；直接用 launch 时给 `hdmap:=<你的 .osm>`。

## 想加一条车道、改一个限速

都在 `agv_gazebo/scripts/road_map.py` 里，改完 `python3 src/agv_gazebo/scripts/gen_warehouse_world.py` 重新生成世界和地图，再跑 `src/agv_gazebo/test` 的测试：

- **改限速**：`SPEED_STRAIGHT` / `SPEED_SLOW`（km/h）；只改某一段的话在 `_Builder.lanelet()` 里按名字给不同的值；
- **改虚实线**：`_ring_line_types()` 决定每个路段同向车道之间是实线还是虚线（`SOLID_BEFORE` 是路口前实线的长度）；
- **加一条路**：照 `_gate_road()` 写一条 `_Path`（站点 + 路段 + 横断面），在 `build()` 里 `_build_path()` 生成边界线和 lanelet，
  路口的转弯车道加到 `turns` 列表（`_lane_end` 取进口 / 出口车道的端点）；要挂红绿灯就照 `_traffic_light()` 加一个；
- 世界那边（路面、标线）自动跟着变，`check_outdoor` 会检查路灯、建筑有没有压到新路面。

## 测试记录

（仿真验收结果见下，命令在 `.superpowers/harness/`，不进 git。）

- 单元测试：`python3 -s -m pytest src/agv_gazebo/test src/agv_hdmap/test -q`（节点测试需要先 `source install/setup.bash`）。
- 建图（2026-11-01，雷达抬高后，公路 0.8 m/s、仓库 0.6 m/s、坡道和夹层 0.4 m/s）：全部路段跑完；轨迹相对真值平均 0.171 m、最大 0.481 m，航向最大 0.53°。
  地图已装到 `maps/wh/`，旧图备份在 `maps/wh_old_20261011/`。
- 公路一圈纯定位：和地图的约束 1062 条（修改前 0 条），没有跳变（最大 0.17 m），每 10 m 一段里的抖动 0.01～0.03 m。
  相对真值平均 0.227 m、最大 0.432 m（横向平均 0.148 m、最大 0.350 m），未达到原定的 < 0.2 m：这是建出来的地图在公路部分相对世界有约 0.4° 的旋转和收缩，
  离仓库越远越大（大门约 0，东南角约 0.4 m），定位本身跟地图一致。高精地图按世界坐标生成，所以东南角的车道和点云地图差约 0.4 m；
  车道宽 2.5 m、车宽 1.08 m，每侧余量 0.71 m。实际项目里车道是在点云地图上画的，没有这项偏差。
- 仿真里看过车出仓库、沿车道行驶、变道和红绿灯，效果符合预期（用户确认）；自动验收脚本（`hdmap_drive.py` 全部场景 + 电梯回归）没有跑完全程。

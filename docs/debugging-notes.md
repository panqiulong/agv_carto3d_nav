# 调试问题总结（2026-10 导航调优、TEB）

按主题整理这一轮调试里遇到的问题（这里 14 条）。每条写清楚：现象、原因、怎么处理的、现在的状态。
割草机场景的条目已随代码移到 lawn_mower_ros 仓库的 `docs/debugging-notes.md`，这里只保留导航、TEB、构建相关的条目（编号沿用原来的，所以不连续）。
车辆参数：轴距 0.855 m，base_link 在车身中心（后轴前 0.432 m），实测满舵后轴转弯半径约 2.19 m。

---

## 一、导航（阿克曼 + Hybrid-A* + MPPI）

### 1. 车在去目标的路上来回摆，甚至越走越远

- **现象**：导航途中车头左右摆，有时离目标越来越远。
- **原因**：行为树每秒无条件重规划一次（`RateController 1 Hz`，按真实时间计，仿真里约 0.6 s 一次）。Hybrid-A* 每次的结果不完全一样，经常在"先倒车再前进"和"直接前进"之间切换。阿克曼车修正 0.2 m 横向偏差要走 1 m 多、约 4 s，远比重规划周期长，结果一直在追一条不断变化的路径。差速车能原地转，修正快，所以问题不大。
- **处理**：改成按需重规划：只在目标变化、路径被挡住（`IsPathValid`）或路径超过 15 s 时才重规划；同时开启 `always_reload_bt_xml`，避免不同目标之间沿用行为树的内部状态。另外加了约束平滑器和速度平滑。
- **状态**：已解决，已在仿真中确认能正常到达目标。

### 2. 终点前倒车来回倒（转弯半径不一致）

- **现象**：到终点前需要倒车调整时，车前后反复挪。
- **原因**：规划用的最小转弯半径 1.95 m 小于车实际能做到的半径。内侧前轮会先顶到关节限位，实测后轴半径约 2.19 m，而 base_link 的半径 = hypot(2.19, 0.432) ≈ 2.23 m。规划出的倒车弧线车跟不上。倒车时的指令换算本身是对的，已用 odom 验证。
- **处理**：MPPI `min_turning_r` 改为 2.2（MPPI 的 vx/wz 对应的是后轴半径）；Smac 和约束平滑器改为 2.3（路径点是 base_link，按 hypot 换算后再留余量）。加了测试 `test_turning_radius_is_achievable_and_consistent`。
- **状态**：已改，但单靠这一条没有解决倒车问题，见第 3、4 条。

### 3. MPPI 每个控制周期都在前进和倒车之间翻转

- **现象**：路径明明是一条纯倒车直线，MPPI 输出却 +0.4 / −0.3 交替，车原地前后挪。
- **原因**：`PreferForwardCritic` 对所有倒车速度都罚分，不管路径是不是要求倒车。它和跟踪路径的 critic 正好对着干，两边代价差不多，最优解就在两个方向之间跳。
- **处理**：从 critics 列表里删掉它，前进还是倒车只听路径（Smac 的 `reverse_penalty` 已经偏向前进）。加了测试 `test_mppi_does_not_penalize_reversing`。
- **状态**：纯倒车已能稳定跟踪。

### 4. MPPI 在换向点附近抖动、弯道倒车跟不住

- **现象**：路径是"先前进 0.4 m 再倒车"时，车停在换向点前只输出 ±0.03 的抖动；需要边倒边转时，倒约 1 m 又改成前进。
- **原因**：Humble 1.1.20 的 MPPI 只对最终输出做转弯半径限制，1000 条采样轨迹本身不受限。按 `wz_std 0.4 / wz_max 0.8` 采样，大部分轨迹车都转不出来，全靠 `ConstraintCritic` 罚分。这个罚分很大，而且对前进和倒车一样，压过了其他 critic，方向判断就不稳定。
- **处理**：运行时试过把 wz 收窄到 0.1 / 0.25，转向平滑了，但换向问题依旧，所以没有写进参数文件。割草机场景（已拆到 lawn_mower_ros 仓库）改为从规划上避开倒车。
- **对比（2026-10-25，仓库 HOME 出发，5 个倒车目标各去一次、回一次）**：

  | 控制器 | 到达 | 典型表现 |
  |---|---|---|
  | TEB | 10 / 10 | 实际换向次数基本等于规划的换向点数，终点误差 0.17～0.38 m，朝向误差在 17° 容差内 |
  | MPPI | 0 / 4（跑了 4 段就停了） | "正后 4 m"是一条纯倒车直线，150 s 内换向 81 次、开了 28 m，最后还在原地附近；"左平移"超时，朝向差 80° |

- **状态**：MPPI 这个问题**仍未解决**；带换向点的机动改用 TEB（`start.sh nav warehouse -c teb`）。彻底解决 MPPI 的做法是把参考点改到后轴中心，或者升级到对采样轨迹做阿克曼约束的新版 MPPI。

### 5. 为什么阿克曼车要以后轴中心为参考点

- 后轮不转向，后轴中心是车上唯一速度始终沿车头方向的点；瞬时转动中心在后轴延长线上，`R = L / tanδ`、`wz = v·tanδ / L` 都只对后轴中心成立。
- base_link 在后轴前 0.432 m：转弯半径更大（hypot），转弯时有横向速度 0.432·wz（odom 实测比值 0.43）；倒车时它相对运动方向落在后面，属于非最小相位，跟踪更难。
- 现在的做法是保持参考点不动，在参数里补偿（MPPI 用后轴半径，规划器和平滑器用 base_link 半径）。彻底的做法是把 `robot_base_frame` 改到后轴中心。

### 6. 掉头被拆成一串 0.2 m 的倒车碎段（换向点太多）

- **现象**：仓库 HOME 附近很空（前 7 m、后 25 m、左 16 m 都没有障碍），"原地掉头"却规划出 7～9 个换向点，形如"倒 0.2、前 0.6、倒 0.2、前 1.0……"。TEB 照着走，实际换了 8 次向，用了 40 s。
- **原因**：读了 Smac Hybrid 的 `getTraversalCost` 源码：
  - 换向本身不计代价；
  - `reverse_penalty 5` 让倒车每米的代价是前进的 5 倍；
  - 运动基元每段 0.2 m（转弯半径 2.3 m × 角度分辨率 5°）。

  所以倒车拆成多少段，总代价都一样，搜索很容易停在一串碎段上。
- **容易误解的地方**：`change_penalty` 不只罚左右切换。它看的是 6 种运动方向（前直/前左/前右/倒直/倒左/倒右）有没有变化，前左 → 倒右这种换向也会被罚。它默认是 0，等于没开。
- **处理**：`change_penalty` 从 0 改为 3，`reverse_penalty` 保持 5（它是为了窄处少倒车）。扫描 8 个倒车目标（规划合计）：

  | reverse : change | 换向点 | 总长 | 倒车 |
  |---|---|---|---|
  | 5 : 0（原来） | 31 | 51.1 m | 24.5 m |
  | 5 : 1 | 20 | 53.6 m | 24.4 m |
  | **5 : 3（现在）** | **15** | 56.3 m | 29.7 m |
  | 2.1 : 0 | 38 | 45.4 m | 24.3 m |
  | 2.1 : 1 | 20 | 45.9 m | 24.2 m |

  加了测试 `test_planner_penalizes_direction_changes`。
- **仿真验收（TEB，与改之前相同的 10 段）**：
  - 规划换向点 21 → 16，实际换向 27 → 21；
  - "原地掉头（去）"换向 8 → 3 次，用时 40 → 27 s；
  - 但总用时 249 → 290 s，多出来的主要在"左平移 回""左后转 回"：规划只有 1 个换向点，TEB 实际换了 5～6 次向。
- **状态**：规划碎段的问题已解决。TEB 在终点附近额外换向的问题**待查**。

### 7. 运行中改 Smac Hybrid 参数，整套导航被关掉

- **现象**：`ros2 param set /planner_server GridBased.change_penalty 3.0` 之后几秒，`lifecycle_manager_navigation` 报 `CRITICAL FAILURE: SERVER planner_server IS DOWN after not receiving a heartbeat for 4000 ms`，导航节点全部被关闭，之后所有规划请求都失败。
- **原因**：参数回调会重建 A*，重新生成 Reeds-Shepp 启发表（`lookup_table_size` 20 m × 72 个角度）。这一步在节点线程里跑了 4 s 以上，心跳就断了。`bond_timeout` 是 nav2_bringup 设的，参数文件里改不了。
- **处理**：调规划参数时，在 `/sweep` 命名空间另起一个 `planner_server`（参数相同，全局代价地图只留静态层和膨胀层，手动 `ros2 lifecycle set` 激活，不挂 lifecycle manager），在它上面改参数、做规划。主导航不受影响。

---

## 二、构建与环境

### 8. 编译 TEB 时报 Fast-CDR 的 `setState` 不存在

- **现象**：`costmap_converter_msgs` 编译报 `'class eprosima::fastcdr::Cdr' has no member named 'setState'`。
- **原因**：`~/.bashrc` 里 source 了 `~/dds_router_ws`，它把 Fast-CDR 2.x 放进了 `CMAKE_PREFIX_PATH`，而 Humble 用的是 1.x。
- **处理**：编译时加 `-Dfastcdr_DIR=/opt/ros/humble/lib/cmake/fastcdr`，必要时先删掉对应的 build / install 目录。

### 10. pip 装的 numpy 让每个 Python 节点占满十几个核

- **现象**：机器负载 34（32 核），仿真变慢；`terrain_obstacle_filter` 一个节点占 1731% CPU，`rtk_localization` 占 312%。
- **原因**：pip 装的 numpy 自带 OpenBLAS，默认每个进程开几十个线程忙等。
- **处理**：mapping / navigation 两个 launch（以及当时的 mowing launch）都加了 `OPENBLAS_NUM_THREADS=1`、`OMP_NUM_THREADS=1`，之后这两个节点降到约 23% 和 12%。
- **影响**：之前 MPPI 的"控制循环跟不上 / missed its desired rate"，很可能也是这个原因。

---

## 三、TEB

前三条是在割草机场景里发现的，问题本身在 TEB，仓储导航用 TEB 时同样适用。

### 25. TEB 启动失败：参数类型不对

- **现象**：`parameter 'FollowPath.teb_autosize' has invalid type`，Nav2 起不来。
- **原因**：TEB 把 `teb_autosize` 声明为 double，参数文件里写成了 `true`。
- **处理**：改成 `1.0`。加了测试 `test_teb_param_types_match_teb_source`，按 TEB 源码里的声明类型逐项核对参数文件。

### 26. TEB 一直报 extrapolation into the future，车开不动

- **现象**：每个控制周期都报 `Lookup would require extrapolation into the future`、`Could not transform the global plan`，车在原地前后挪，6 次失败后放弃了整段沿边。
- **原因**：TEB 按路径的时间戳（也就是"现在"）查 odom 到 map 的变换，而定位节点发布的 map→odom 时间戳总是略晚于"现在"。MPPI 和纯跟踪有变换容差，所以不受影响。
- **处理**：（割草机的）RTK 定位节点把 map→odom 的时间戳往后推 0.2 s（参数 `transform_tolerance`，和 AMCL 的做法一样）。之后报错归零，TEB 转场正常。
- **后续**：根本原因在 TEB 按路径时间戳查 TF，第 28 条改了 TEB 源码，彻底解决了这个问题（后推时间戳保留，没有坏处）。

### 28. TEB 回桩失败：extrapolation into the past

- **现象**：割完回充电桩时 TEB 一直报 `extrapolation into the past`，请求的时间固定不变，最后导航放弃。
- **原因**：TEB 按**路径的时间戳**查 TF。改成按需重规划后，一条路径能存活 15 s，而 TF 缓存只有 10 s，路径超过 10 s 就查不到了。Nav2 默认行为树每秒重规划一次，所以默认配置下不会遇到。第 26 条的 "into the future" 也是同一个原因。
- **处理**：改了 TEB 源码（`src/third_party/teb_local_planner/.../teb_local_planner_ros.cpp` 的 `transformGlobalPlan`，补丁存在 `src/third_party/patches/teb_local_planner.patch`），改为用最新的变换转换路径（map→odom 变化慢，最新值足够），有注释说明。**这是对第三方源码的本地修改**，以后更新 TEB 时要重新打上。这样仓储 + Cartographer + TEB 的组合也不会再报这两种错。

### 29. TEB 走直线时方向盘一直小幅摇摆（仓储场景，2026-11-06）

- **现象**：绝对直的路径上，0.5 m/s 时左前轮转角 std 0.073 rad，峰峰值约 21°，周期约 0.63 s。
- **排除**：转向 PID（开环固定转角时峰峰值 0.0002 rad）、转向执行链（指令到轮角延迟 0.04 s，r = 0.99）、规划路径（人工构造的直线一样摆）、`dt_ref`（0.3 改 0.2 周期不变）。
- **原因**：`getVelocityCommand` 默认只用轨迹第一段算控制量（`control_look_ahead_poses = 1`），`omega = (θ1 - θ0) / dt`，一段约 0.15 m，增益很高。周期性的波形说明很可能是闭环振荡，这一点没有直接证明。另外 Cartographer 的 map->odom 有锯齿（见下），也贡献了一部分：路径放到 odom 系后 std 降到 0.058。
- **处理**：`control_look_ahead_poses` 改为 5，同时改 TEB 源码（`optimal_planner.cpp`），让前瞻窗口遇到换向点就截断。不截断时窗口两端的位移互相抵消，12 段换向验收只到 10 段，掉头用时变成 3~5 倍。
- **验证**：

  | | 左轮 std（直线） | 弯道最大偏差 | 换向验收 | 总用时 | 实际换向 | 最大到点误差 |
  |---|---|---|---|---|---|---|
  | 前瞻 1 | 0.073 rad | 0.18 m | 12 / 12 | 215 s | 19 | 0.73 m |
  | 前瞻 5 + 截断 | 0.011 rad | 0.28 m | 12 / 12 | 233 s | 29 | 0.43 m |

  只有“原地掉头 回”这一段换向明显变多（2 -> 10 次，用时 15 -> 24 s），只跑过一次。窄道没测。
- **顺带修复**：`transformGlobalPlan` 和 `pruneGlobalPlan` 里车的位姿还在按“现在”查 TF（第 28 条漏了），报 extrapolation into the future，连续失败时整段导航被中止，改为用最新的变换。这两处和上面的截断都在 `src/third_party/patches/teb_local_planner.patch`，更新 TEB 时要重新打上。
- **未解决**：Cartographer 把位姿外推到现在，`published_to_tracking` 却取扫描时刻（`published_frame = odom` + 外部 EKF 时），车的运动被算两遍，map->odom 每约 0.1 s 一次锯齿。`use_pose_extrapolator = false` 在补 TF 之前会让 TEB 中止，补 TF 之后没有重测。

---

## 四、定位

### 31. 不能用 RViz 的 2D Pose Estimate、车被挪走后找不回来

- Cartographer 是图优化，不订阅 `/initialpose`；唯一的入口是开新轨迹时（`/start_trajectory`）带初始位姿。已加桥接节点 `scripts/initial_pose.py`：收到 `/initialpose` 就 finish 当前定位轨迹、以点选位姿 start 一条新的（2026-11-08）。
- 全图搜索仍然关着（`global_constraint_search_after_n_seconds = 1e6`）。曾改成 10 s，结果启动阶段 10 s 内没配上一次就永远只做全局搜索、不再局部匹配，而仓储里正确匹配得分 55~70% 到不了全局门槛 0.7，等于永远配不上（commit b0cb467）。车被挪走后要人用 2D Pose Estimate 点一下。

---

## 五、仿真与调试工具的坑

| 坑 | 说明 / 做法 |
|---|---|
| 杀掉 `ros2 action send_goal` 不会取消目标 | 目标在后台继续执行。要取消，调用 `/navigate_to_pose/_action/cancel_goal "{}"` |
| `pkill -f <模式>` 把自己的 shell 也杀了 | 模式出现在当前命令行里也会被匹配（exit 144）。按 PID 杀，或者确保模式不出现在命令行里 |
| 后台启动的 start.sh / Python 收不到 Ctrl+C | 非交互 shell 里用 `&` 启动的进程会忽略 SIGINT，而且这种信号无法再 trap。停止时用 TERM |
| 重启仿真后新 gzserver 被杀 | 旧的 start.sh 清理的最后一步是 `pkill gzserver`，要等旧的 start.sh 完全退出再启动新的 |
| 生命周期管理器重名 | 上一轮残留的同名 lifecycle_manager 会把新起的节点停掉，而且收到 SIGTERM 不一定退出，需要 kill -9 |
| 偶发：spawn 等不到 `/robot_description` | 仿真启动时序问题，重试一次就好 |
| 自己的 shell 被误杀，但它启动的脚本还在跑 | shell 收到 TERM 退出，子进程不会跟着退。结果后台的重启脚本继续跑，再手动起一次就有两套仿真、两个 mow.py（日志里有 `more than one action server`，行交错）。按进程名精确匹配杀进程（`ps -eo pid,args \| awk '$2=="python3" && $3 ~ /mow\.py$/'`），重启前确认只有一个 start.sh |
| 机器重启后 /tmp 被清空 | 临时目录里的辅助脚本、日志、轨迹都会丢，正在跑的验收结果也没了。要长期用的脚本别只放在 /tmp |

---

## 六、当前状态

- **已验证（2026-10-25 仓储导航 + Cartographer 定位 + TEB）**：从 HOME 出发，6 个需要倒车的目标（正后 4 m、左平移、左后转、后右斜、原地掉头、前方掉头）各去一次、回一次，12 段全部到达，终点误差 0.08～0.59 m。
- **未解决 / 未验证**：
  - MPPI 带换向点的倒车（第 4 条）：仍未解决。仓储导航带换向点的机动用 TEB。
  - TEB 在终点附近额外换向（第 6 条）：规划只有 1 个换向点，实际换了 5～6 次向。
  - 仓储出入库任务（窄库位倒车入位）在 TEB + `change_penalty 3` 下还没跑过。
  - 全局重定位（第 31 条）。`/initialpose` 桥接已做（仿真验证：点错位姿时定位跟着换过去；点偏 0.58 m / 5° 时静止不纠正，开 15 m 后剩 0.12 m / 2.4°）。

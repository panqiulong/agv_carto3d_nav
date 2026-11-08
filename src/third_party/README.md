# 第三方包

| 目录 | 来源 | 怎么管理 |
|---|---|---|
| `livox_ros_driver2` | [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | 源码放在 git 里 |
| `livox_laser_simulation_ros2` | ros2_livox_simulation（包名） | 源码放在 git 里，有本地修改（`git log -- src/third_party/livox_laser_simulation_ros2`） |
| `costmap_converter` | rst-tu-dortmund | 不进 git，版本见根目录 `agv.repos` |
| `teb_local_planner` | rst-tu-dortmund | 不进 git，版本见 `agv.repos`，本地修改在 `patches/teb_local_planner.patch` |

新机器上拉取后两个：

```bash
vcs import src/third_party < agv.repos
git -C src/third_party/teb_local_planner apply ../patches/teb_local_planner.patch
```

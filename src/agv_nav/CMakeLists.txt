cmake_minimum_required(VERSION 3.8)
project(agv_nav)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch behavior_trees rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/cmd_vel_to_ackermann.py
  scripts/cloud_to_grid_map.py
  scripts/save_map.sh
  scripts/terrain_obstacle_filter.py
  scripts/elevator.py
  scripts/initial_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

# 节点脚本 import 的纯计算模块，装到脚本同一目录
install(FILES scripts/terrain.py scripts/elevator_fit.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # 不加载 ~/.local 的包（同文档里的 python3 -s -m pytest）：那里的 anyio 带的 pytest 插件和 apt 的 pytest 6 不兼容，
  # 配置时 ament 的 pytest 检测就会崩，然后静默跳过测试，所以配置和运行两处都要设
  set(ENV{PYTHONNOUSERSITE} 1)
  ament_add_pytest_test(unit test ENV PYTHONNOUSERSITE=1)
endif()

ament_package()

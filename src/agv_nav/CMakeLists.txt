cmake_minimum_required(VERSION 3.8)
project(agv_nav)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch params maps behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/cmd_vel_to_ackermann.py
  scripts/cloud_to_grid_map.py
  scripts/save_map.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(agv_ackermann_drive)

set(CMAKE_CXX_STANDARD 17)

# Gazebo 用系统 Boost。本机 /usr/local 有个 1.83，CMake 会优先选中它，运行时又找不到，插件静默加载失败
set(Boost_DIR /usr/lib/x86_64-linux-gnu/cmake/Boost-1.74.0)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_plugins REQUIRED)   # 只用它的头文件，不链接原版插件
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(agv_ackermann_drive SHARED src/gazebo_ros_ackermann_drive.cpp)
target_include_directories(agv_ackermann_drive PRIVATE include ${gazebo_plugins_INCLUDE_DIRS})
ament_target_dependencies(agv_ackermann_drive
  gazebo_dev gazebo_ros geometry_msgs nav_msgs rclcpp std_msgs tf2 tf2_geometry_msgs tf2_ros)
install(TARGETS agv_ackermann_drive LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rear_wheel_speed test/test_rear_wheel_speed.cpp)
  target_include_directories(test_rear_wheel_speed PRIVATE include)
endif()

ament_package()

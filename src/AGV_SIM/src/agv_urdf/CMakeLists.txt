cmake_minimum_required(VERSION 3.8)
project(agv_urdf)

find_package(ament_cmake REQUIRED)

# ======================
# Install
# ======================

# 安装资源文件
install(
  DIRECTORY
    config
    launch
    urdf
    meshes
    worlds
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()

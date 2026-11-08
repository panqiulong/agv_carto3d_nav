cmake_minimum_required(VERSION 3.8)
project(agv_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # 不加载 ~/.local 的包（同文档里的 python3 -s -m pytest）：那里的 anyio 带的 pytest 插件和 apt 的 pytest 6 不兼容，
  # 配置时 ament 的 pytest 检测就会崩，然后静默跳过测试，所以配置和运行两处都要设
  set(ENV{PYTHONNOUSERSITE} 1)
  ament_add_pytest_test(unit test ENV PYTHONNOUSERSITE=1)
endif()

ament_package()

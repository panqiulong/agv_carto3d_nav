cmake_minimum_required(VERSION 3.8)
project(agv_hdmap)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/hdmap_navigator.py
  scripts/hdmap_cli.py
  scripts/traffic_light_sim.py
  DESTINATION lib/${PROJECT_NAME}
)
# 纯计算模块 hdmap.py 给节点 import，不直接运行
install(FILES scripts/hdmap.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # 不加载 ~/.local 的包（同文档里的 python3 -s -m pytest）：那里的 anyio 带的 pytest 插件和 apt 的 pytest 6 不兼容，
  # 配置时 ament 的 pytest 检测就会崩，然后静默跳过测试，所以配置和运行两处都要设
  set(ENV{PYTHONNOUSERSITE} 1)
  ament_add_pytest_test(unit test ENV PYTHONNOUSERSITE=1)
endif()

ament_package()

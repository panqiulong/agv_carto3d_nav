cmake_minimum_required(VERSION 3.8)
project(agv_semantic)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlaceEdit.srv"
  "srv/TaskSubmit.srv"
  "srv/TaskId.srv"
  "srv/SlotSet.srv"
  "action/GoToPlace.action"
)

# 节点和工具直接 import 同目录下的 semantic_map.py、tasks.py
install(PROGRAMS
  scripts/semantic_server.py
  scripts/semantic.py
  scripts/make_filter_masks.py
  scripts/task_manager.py
  scripts/task.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES scripts/semantic_map.py scripts/tasks.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # 不加载 ~/.local 的包（同文档里的 python3 -s -m pytest）：那里的 anyio 带的 pytest 插件和 apt 的 pytest 6 不兼容，
  # 配置时 ament 的 pytest 检测就会崩，然后静默跳过测试，所以配置和运行两处都要设
  set(ENV{PYTHONNOUSERSITE} 1)
  ament_add_pytest_test(unit test ENV PYTHONNOUSERSITE=1)
endif()

ament_package()

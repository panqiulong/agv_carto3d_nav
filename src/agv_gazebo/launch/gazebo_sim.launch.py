import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable, TimerAction)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

# 可选的世界及对应出生点 (x, y, yaw)。warehouse.world 由 scripts/gen_warehouse_world.py 生成
WORLDS = {
    'warehouse': (2.0, -4.0, 0.0),
}
ROBOT_NAME = 'r2'
SPAWN_Z = 0.19      # 轮子静止接地高度约 0.183（车按 1.5 倍放大后），留 7 mm 余量避免穿模


def launch_setup(context, pkg_gazebo):
    """gzserver、spawn、相机检查依赖所选世界（世界文件和出生点），在这里按 world 参数生成"""
    world = LaunchConfiguration('world').perform(context)
    x, y, yaw = WORLDS[world]

    # --- Gazebo Server ---
    gzserver_node = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            os.path.join(pkg_gazebo, 'worlds', world + '.world'),
            # init 插件负责发布 /clock，不加载的话 use_sim_time 的节点（Cartographer、Nav2）拿不到时间。
            # 发布频率在 config/gazebo_params.yaml 里提到 200 Hz（gzserver 会把命令行的 -p 当成回放文件，
            # 只能用 --params-file）
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            '--ros-args', '--params-file', os.path.join(pkg_gazebo, 'config', 'gazebo_params.yaml')
        ],
        output='screen'
    )

    # --- Gazebo GUI ---
    gzclient_node = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # --- spawn robot（出生点在文件开头的 WORLDS 里按世界配置）---
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_agv',
        arguments=[
            '-topic', '/robot_description',
            '-entity', ROBOT_NAME,
            '-x', str(x), '-y', str(y), '-z', str(SPAWN_Z),
            '-R', '0.0', '-P', '0.0', '-Y', str(yaw),
        ],
        output='screen'
    )

    # --- 静态 TF world -> map：map 原点就是出生点（建图从这里开始），RViz 才能把 /ground_truth（world 系）叠到地图上 ---
    world_to_map_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map',
        arguments=['--x', str(x), '--y', str(y), '--yaw', str(yaw), '--frame-id', 'world', '--child-frame-id', 'map'],
        output='screen'
    )

    # --- 延迟 spawn（防止 Gazebo 未完全启动）---
    spawn_after_gzserver = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gzserver_node,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[spawn_entity_node]
                )
            ]
        )
    )

    # --- Gazebo Classic 偶发问题：gzserver 里第一次通过 /spawn_entity 动态生成的模型，
    # 深度相机这类依赖渲染引擎的传感器有时初始化不出来（雷达这种基于物理射线的传感器不受影响），
    # 是个时序竞争，不是每次都发生。先检测相机话题有没有正常出数据，不行才删掉重新 spawn 一次，
    # 避免对一个其实已经正常的模型做无意义的 delete（会在 gzserver 里刷一堆无害但吓人的
    # "No link visual with id..." 错误）---
    check_and_fix_camera = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'sleep 2; '
            # 没有显示（无界面运行）时深度相机本来就初始化不了，重新 spawn 修不好，
            # 而无界面下 delete_entity 会让 gzserver 卡死，所以直接跳过
            'if [ -z "$DISPLAY" ]; then echo "no DISPLAY: depth camera needs rendering, skip respawn check"; '
            'else '
            # 查一次没查到不算数（2026-11-07 就这样误删过车）：每 2 s 查一次，约 30 s 还没有才认定失败。
            # 每次的查询结果都打印出来，下次才分得清是相机真没起来，还是查询漏报
            'ok=; for i in $(seq 15); do '
            'out=$(ros2 topic info /camera/image_raw 2>&1); '
            'if echo "$out" | grep -q "Publisher count: [1-9]"; then ok=1; break; fi; '
            'echo "camera check $i/15: $out"; sleep 2; '
            'done; '
            'if [ -n "$ok" ]; then echo "camera sensor already ok, no respawn needed"; else '
            'echo "camera sensor did not come up on first spawn, deleting and respawning..."; '
            "ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity \"{name: '%s'}\"; "
            'sleep 1; '
            'ros2 run gazebo_ros spawn_entity.py -topic /robot_description -entity %s '
            '-x %s -y %s -z %s -R 0.0 -P 0.0 -Y %s; '
            'fi; fi'
            % (ROBOT_NAME, ROBOT_NAME, x, y, SPAWN_Z, yaw)
        ],
        output='screen'
    )

    fix_camera_after_first_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[check_and_fix_camera]
        )
    )

    return [gzserver_node, gzclient_node, spawn_after_gzserver, fix_camera_after_first_spawn, world_to_map_node]


def generate_launch_description():

    pkg_gazebo = get_package_share_directory('agv_gazebo')
    pkg_description = get_package_share_directory('agv_description')

    # --- 模型路径 ---
    default_model_path = os.path.join(pkg_description, 'urdf', 'r2.xacro')

    # --- launch 参数 ---
    declare_model_path_cmd = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='URDF/XACRO 文件路径'
    )

    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value='warehouse',
        choices=list(WORLDS),
        description='仿真世界：warehouse（50x30 m 仓储，月台 + 货架 + 夹层）'
    )

    model_path = LaunchConfiguration('model')

    # --- robot_description ---
    robot_description = {
        'robot_description': Command(['xacro ', model_path])
    }

    # --- 改进后的 GAZEBO_MODEL_PATH（关键修复 mesh 找不到的问题）---
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')

    new_paths = [
        gazebo_model_path,
        os.path.expanduser('~/.gazebo/models'),
        pkg_description,                          # 包目录
        os.path.dirname(pkg_description),         # ← 重要：包的父目录（让 model://agv_description 生效）
        os.path.join(pkg_description, 'meshes'),  # ← 直接添加 meshes 目录（增强兼容性）
    ]
    # industrial / warehouse 世界里的货架、托盘等来自 AWS 仓库模型包（ros-humble-aws-robomaker-small-warehouse-world）。
    # 它的 SDF 里网格写成相对路径 file://models/...，Gazebo 按 GAZEBO_RESOURCE_PATH 解析，所以包目录也要加进去，
    # 否则模型能找到、网格加载失败，货架在仿真里没有碰撞体也看不见
    resource_paths = [os.environ.get('GAZEBO_RESOURCE_PATH', '') or '/usr/share/gazebo-11']
    try:
        aws_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
        new_paths.append(os.path.join(aws_dir, 'models'))
        resource_paths.append(aws_dir)
    except PackageNotFoundError:
        pass

    gazebo_model_path = os.pathsep.join([p for p in new_paths if p])

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path
    )
    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=os.pathsep.join(resource_paths)
    )

    # --- robot_state_publisher ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # --- 仿真的轮式里程计：给插件输出的真值加上轮速的误差 ---
    wheel_odom_node = Node(
        package='agv_gazebo',
        executable='wheel_odom_sim.py',
        name='wheel_odom_sim',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # --- livox 仿真插件需要 boost 1.83（本机 ~/.bashrc 的 CMAKE_PREFIX_PATH=/usr/local
    # 导致编译时链接到了 /usr/local/boost_1_83_0，而不是系统自带的 boost 1.74），
    # 系统默认库搜索路径里没有，运行时单独补上，否则插件加载失败、雷达不出数据 ---
    ld_library_path = os.environ.get('LD_LIBRARY_PATH', '')
    set_boost_library_path = SetEnvironmentVariable(
        name='LD_LIBRARY_PATH',
        value=os.pathsep.join(['/usr/local/boost_1_83_0/lib', ld_library_path])
    )

    # --- LaunchDescription ---
    return LaunchDescription([
        declare_model_path_cmd,
        declare_world_cmd,
        set_gazebo_model_path,
        set_gazebo_resource_path,
        set_boost_library_path,
        robot_state_publisher_node,
        wheel_odom_node,
        OpaqueFunction(function=launch_setup, kwargs={'pkg_gazebo': pkg_gazebo}),
    ])
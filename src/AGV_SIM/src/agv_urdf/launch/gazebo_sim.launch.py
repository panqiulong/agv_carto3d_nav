import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    robot_name_in_model = "r2"

    pkg_agv_urdf = get_package_share_directory('agv_urdf')

    # --- 模型路径 ---
    default_model_path = os.path.join(pkg_agv_urdf, 'urdf', 'r2.xacro')
    default_world_path = os.path.join(pkg_agv_urdf, 'worlds', 'simple_room.world')

    # --- launch 参数 ---
    declare_model_path_cmd = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='URDF/XACRO 文件路径'
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
        pkg_agv_urdf,                          # 包目录
        os.path.dirname(pkg_agv_urdf),         # ← 重要：包的父目录（让 model://agv_urdf 生效）
        os.path.join(pkg_agv_urdf, 'meshes')   # ← 直接添加 meshes 目录（增强兼容性）
    ]

    gazebo_model_path = os.pathsep.join([p for p in new_paths if p])

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path
    )

    # --- robot_state_publisher ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # --- Gazebo Server ---
    gzserver_node = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            default_world_path,
            # init 插件负责发布 /clock，不加载的话 use_sim_time 的节点（Cartographer、Nav2）拿不到时间。
            # 发布频率在 config/gazebo_params.yaml 里提到 200 Hz（gzserver 会把命令行的 -p 当成回放文件，
            # 只能用 --params-file）
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            '--ros-args', '--params-file', os.path.join(pkg_agv_urdf, 'config', 'gazebo_params.yaml')
        ],
        output='screen'
    )

    # --- Gazebo GUI ---
    gzclient_node = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # --- spawn robot ---
        # --- spawn robot（修改初始位置在这里）---
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_agv',
        arguments=[
            '-topic', '/robot_description',
            '-entity', robot_name_in_model,
            '-x', '0.0',       # 房间中心附近
            '-y', '0.0',
            '-z', '0.13',      # 轮子静止接地高度约0.122，留5mm余量避免穿模
            '-R', '0.0',      # Roll  （一般不动）
            '-P', '0.0',      # Pitch （一般不动）
            '-Y', '0.0'       # ← Yaw 朝向角度（弧度），1.57=90度，3.14=180度
        ],
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
            'if ! ros2 topic info /camera/image_raw 2>/dev/null | grep -q "Publisher count: [1-9]"; then '
            'echo "camera sensor did not come up on first spawn, deleting and respawning..."; '
            "ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity \"{name: '%s'}\"; "
            'sleep 1; '
            'ros2 run gazebo_ros spawn_entity.py -topic /robot_description -entity %s '
            '-x 0.0 -y 0.0 -z 0.13 -R 0.0 -P 0.0 -Y 0.0; '
            'else echo "camera sensor already ok, no respawn needed"; fi'
            % (robot_name_in_model, robot_name_in_model)
        ],
        output='screen'
    )

    fix_camera_after_first_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[check_and_fix_camera]
        )
    )
    # --- ⭐ Gazebo Plugin Path（关键） ---
    gazebo_plugin_path = os.environ.get('GAZEBO_PLUGIN_PATH', '')

    plugin_paths = [
        gazebo_plugin_path,
        os.path.join(pkg_agv_urdf, 'lib'),
        os.path.join(pkg_agv_urdf, 'lib', 'agv_urdf')  # ⭐ 核心路径
    ]

    gazebo_plugin_path = os.pathsep.join([p for p in plugin_paths if p])

    set_gazebo_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=gazebo_plugin_path
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
        set_gazebo_model_path,
        set_gazebo_plugin_path,
        set_boost_library_path,
        robot_state_publisher_node,
        gzserver_node,
        gzclient_node,
        spawn_after_gzserver,
        fix_camera_after_first_spawn,
    ])
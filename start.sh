#!/usr/bin/env bash
# 一键启动：仿真 + 建图，或 仿真 + 定位导航。
#
#   ./start.sh                         交互菜单
#   ./start.sh map <世界> [地图前缀]     建图：键盘遥控开车，Ctrl+C 退出遥控后可选择保存地图
#   ./start.sh nav <世界> [地图前缀]     导航：地图旁边有 <前缀>_semantic.yaml / <前缀>_tasks.yaml /
#                                      <前缀>_lanelet2_map.osm（公路高精地图）时自动带上
#
# 选项 -c / --controller mppi|teb：局部控制器（nav 模式），默认 mppi，可以写在任意位置。
# 世界：warehouse。地图前缀默认 warehouse -> wh，其他 -> 世界名，
# 每张地图一个文件夹 maps/<前缀>/（如 maps/wh/wh.pbstream、maps/wh/wh.yaml）。
# 日志在 log/start/，nav 模式下终端实时显示其中的告警和报错。Ctrl+C 退出时关掉本脚本启动的所有进程。
WS="$(cd "$(dirname "$0")" && pwd)"
MAPS="$WS/maps"
LOGS="$WS/log/start"
WORLDS="warehouse"
CONTROLLERS="mppi teb"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
mkdir -p "$LOGS"

pick() {   # pick <提示> <选项...>：打印选中的选项
    local prompt=$1; shift
    local i=1 opt
    echo "$prompt" >&2
    for opt in "$@"; do echo "  $i) $opt" >&2; i=$((i + 1)); done
    while true; do
        read -rp "> " i
        if [[ "$i" =~ ^[0-9]+$ ]] && (( i >= 1 && i <= $# )); then echo "${!i}"; return; fi
        echo "请输入 1~$#" >&2
    done
}

CONTROLLER=
POS=()
while [ $# -gt 0 ]; do
    case "$1" in
        -c|--controller) CONTROLLER=${2:-}; shift 2 ;;
        *) POS+=("$1"); shift ;;
    esac
done
MODE=${POS[0]:-}
WORLD=${POS[1]:-}
INTERACTIVE=; [ -z "$MODE" ] && INTERACTIVE=1
[ -z "$MODE" ] && MODE=$(pick "选择模式：" map nav)
[ -z "$WORLD" ] && WORLD=$(pick "选择世界：" $WORLDS)
case "$MODE" in map|nav) ;; *) echo "模式只能是 map 或 nav" >&2; exit 1 ;; esac
[[ " $WORLDS " == *" $WORLD "* ]] || { echo "世界只能是：$WORLDS" >&2; exit 1; }
if [ "$MODE" != map ] && [ -z "$CONTROLLER" ]; then
    if [ -n "$INTERACTIVE" ]; then CONTROLLER=$(pick "选择局部控制器：" $CONTROLLERS); else CONTROLLER=mppi; fi
fi
[ "$MODE" = map ] || [[ " $CONTROLLERS " == *" $CONTROLLER "* ]] || { echo "控制器只能是：$CONTROLLERS" >&2; exit 1; }
DEFAULT_PREFIX=$WORLD; [ "$WORLD" = warehouse ] && DEFAULT_PREFIX=wh
PREFIX=${POS[2]:-$DEFAULT_PREFIX}
MAP="$MAPS/$PREFIX/$PREFIX"   # 地图文件的路径前缀

PIDS=()
cleanup() {
    local rc=$?
    trap - INT TERM EXIT
    echo; echo "正在关闭……"
    for p in "${PIDS[@]}"; do kill -INT -- "-$p" 2>/dev/null; done      # 每个 launch 是独立的进程组
    sleep 5
    for p in "${PIDS[@]}"; do kill -KILL -- "-$p" 2>/dev/null; done
    pkill -KILL -f "gzserver .*/worlds/$WORLD.world" 2>/dev/null       # gzserver 偶尔不响应 SIGINT
    echo "已关闭。日志在 $LOGS"
    exit $rc          # 不退出的话，INT 处理完会回到原来等待的 read，终端看起来像卡住了
}
trap cleanup INT TERM EXIT

echo "启动仿真：$WORLD（日志 $LOGS/sim.log）"
setsid ros2 launch agv_gazebo gazebo_sim.launch.py world:="$WORLD" > "$LOGS/sim.log" 2>&1 < /dev/null &
PIDS+=($!)
for _ in $(seq 90); do grep -q "Spawn status" "$LOGS/sim.log" && break; sleep 2; done
grep -q "Successfully spawned" "$LOGS/sim.log" || { echo "仿真没有起来，看 $LOGS/sim.log" >&2; exit 1; }

if [ "$MODE" = map ]; then
    echo "启动建图（日志 $LOGS/mapping.log）"
    setsid ros2 launch agv_nav mapping.launch.py > "$LOGS/mapping.log" 2>&1 < /dev/null &
    PIDS+=($!)
    sleep 8
    cat <<EOF

================ 建图 ================
用下面的键盘遥控开车（车不能原地转，用 u/o/m/. 边走边转；最小转弯半径约 1.95 m）：
  · 每条通道、坡道上下、平台顶面都要走到，没走过的地方会被当成不可通行
  · 最后回到起点附近再走一段
开完后按 Ctrl+C 退出遥控，再选择是否保存地图（保存到 $MAPS/$PREFIX/）
======================================
EOF
    trap ':' INT          # 遥控里按 Ctrl+C 只退出遥控，脚本接着问是否保存地图（用空处理函数而不是忽略：子进程不会继承）
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    # 这里不能恢复 cleanup：提示处按 Ctrl+C 会杀掉 Cartographer，没保存的地图就没了。不保存要明确输入 n
    read -rp "保存地图到 $MAPS/$PREFIX/ ？[Y/n] " ans
    if [[ ! "$ans" =~ ^[Nn] ]]; then
        ros2 run agv_nav save_map.sh "$MAP" && echo "已保存。导航：./start.sh nav $WORLD $PREFIX"
        if [ "$WORLD" = warehouse ] && [ ! -f "${MAP}_semantic.yaml" ]; then
            W="$WS/install/agv_gazebo/share/agv_gazebo/worlds"
            cp "$W/warehouse_semantic.yaml" "${MAP}_semantic.yaml"
            cp "$W/warehouse_tasks.yaml" "${MAP}_tasks.yaml"
            echo "已放入 warehouse 的语义文件和任务配置模板"
        fi
        if [ "$WORLD" = warehouse ] && [ ! -f "${MAP}_lanelet2_map.osm" ]; then     # 已有就不覆盖（可能手改过）
            W="$WS/install/agv_gazebo/share/agv_gazebo/worlds"
            cp "$W/warehouse_lanelet2_map.osm" "${MAP}_lanelet2_map.osm"
            cp "$W/map_projector_info.yaml" "${MAP}_map_projector_info.yaml"
            echo "已放入公路的高精地图 ${MAP##*/}_lanelet2_map.osm"
        fi
        if [ -f "${MAP}_semantic.yaml" ]; then
            ros2 run agv_semantic make_filter_masks.py "$MAP.yaml" "${MAP}_semantic.yaml"
        fi
    fi
    exit 0
fi

# ---------------- 导航 ----------------
for f in "$MAP.pbstream" "$MAP.yaml"; do
    [ -f "$f" ] || { echo "没有地图 $f，先建图：./start.sh map $WORLD $PREFIX" >&2; exit 1; }
done
ARGS=(pbstream:="$MAP.pbstream" map:="$MAP.yaml" controller:="$CONTROLLER")
[ -f "${MAP}_semantic.yaml" ] && ARGS+=(semantic:="${MAP}_semantic.yaml")
[ -f "${MAP}_semantic.yaml" ] && [ -f "${MAP}_tasks.yaml" ] && ARGS+=(tasks:="${MAP}_tasks.yaml")
[ -f "${MAP}_lanelet2_map.osm" ] && ARGS+=(hdmap:="${MAP}_lanelet2_map.osm")
echo "启动定位 + 导航（日志 $LOGS/nav.log）：${ARGS[*]##*/}"
setsid ros2 launch agv_nav navigation.launch.py "${ARGS[@]}" > "$LOGS/nav.log" 2>&1 < /dev/null &
PIDS+=($!)
cat <<EOF

================ 导航 ================
RViz 里用 Nav2 Goal 点目标。$( [ -f "${MAP}_semantic.yaml" ] && echo "
按名字导航：ros2 run agv_semantic semantic.py goto <地点>（list 列出地点）")$( [ -f "${MAP}_tasks.yaml" ] && echo "
出入库任务：ros2 run agv_semantic task.py submit inbound --from R1 --to A1   （list / cancel / resume）
（任务执行时别在 RViz 里点目标，会把任务的导航顶掉）")
下面只显示告警和报错，完整日志在 $LOGS/
按 Ctrl+C 关闭全部
======================================
EOF
# 日志量很大（一次约 8 万行），终端只放告警和报错；-n +1 补上启动阶段已经写进日志的
setsid bash -c 'tail -q -n +1 -F "$@" | grep --line-buffered -aE "\[(WARN|ERROR|FATAL)\]|\[(Wrn|Err)\]|Traceback|[A-Za-z]Error:|what\(\)"' _ "$LOGS/sim.log" "$LOGS/nav.log" &
PIDS+=($!)
wait

#!/usr/bin/env python3
"""生成 warehouse.world：约 50 x 30 m 的仓储场景（收货月台 → 地面货架 / 夹层存储 → 发货月台），南墙外一圈公路，
并输出 map 坐标系下的语义文件 warehouse_semantic.yaml、任务配置模板 warehouse_tasks.yaml、进梯程序用的电梯先验
warehouse_elevator.yaml，以及公路的 Lanelet2 高精地图 warehouse_lanelet2_map.osm 和 map_projector_info.yaml。

    python3 gen_warehouse_world.py            # 在 ../worlds/ 生成六个文件

布局（世界坐标，x 向东，y 向北，单位 m；设计见 docs/superpowers/specs/2026-09-30-warehouse-tasks-design.md 第 2 节）：
  - 北墙：收货月台门 R1/R2（西）、发货月台门 S1/S2（东），门是关着的彩色墙板；门前是月台车道，门之间靠墙放暂存托盘
  - 西半部：4 排东西向货架，排间巷道约 4.2 m，两端都连着横向通道；库位 A1..A8 在巷道里和第一排北侧
  - 东南角：夹层 12 x 10 m，高 1.0 m，西、北两边有护栏；5° 坡道宽 3.5 m，从夹层北边向北下到地面
  - 中部开阔区：HOME（= 出生点）
  - 货梯：东墙 y=11.2（发货月台车道中线）处开 1.8 m 门洞，轿厢（内净 2.0 x 2.6 m，三面墙 + 顶板）在外墙外侧，
    车从 S2 沿车道往东直行就到等待点
  - 公路：南墙 x ∈ [0, 6] 开大门，外面是环路、十字路口、T 形路口（几何和高精地图见 road_map.py），四周围栏、
    路灯杆、红绿灯杆，环路中间几座方块建筑给激光定位当参照物。路面和标线只有外观、没有碰撞
地点全部是「可穿行位姿」：沿车道 / 巷道方向停车，停完可以继续往前开。
"""
import math
import os
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import road_map  # noqa: E402

WALL_H, WALL_T = 6.0, 0.3
GUARD_H, GUARD_T = 1.1, 0.1                      # 护栏高度 / 厚度
IN_X, IN_Y = 25.0, 15.0                          # 墙内表面
CAR = (1.2, 1.08)                                # 车体长、宽（2026-10-10 整车放大 1.5 倍）
CAR_MARGIN = 0.3                                 # 地点检查时车体外扩

MEZZ = dict(name='mezz', x=(13.0, IN_X), y=(-IN_Y, -5.0), h=1.0, ramp_x=16.5, ramp_deg=5.0, ramp_w=3.5,
            ramp_edge='north', guards=('west', 'north'))       # 东、南两边是墙

SPAWN = (2.0, -4.0, 0.0)

# 月台门：(名字, 中心 x, 材质)。只有外观，没有碰撞（激光看到的仍是墙）
DOCK_DOORS = [('R1', -20.0, 'Gazebo/Orange'), ('R2', -14.0, 'Gazebo/Orange'),
              ('S1', 4.0, 'Gazebo/Green'), ('S2', 10.0, 'Gazebo/Green')]
DOOR_W, DOOR_H = 3.0, 3.5

# 货梯：东墙门洞中心 y、门洞宽；轿厢内净宽（y 向）、深（x 向）、墙厚、墙高；等待点离墙内表面的距离
ELEVATOR = dict(door_y=11.2, door_w=1.8, width=2.0, depth=2.6, wall_t=0.1, height=2.5, wait_dist=3.85)
CABIN_X0 = IN_X + WALL_T                         # 轿厢内净 x 范围：从外墙外表面往东
CABIN_X1 = CABIN_X0 + ELEVATOR['depth']
DOOR_ZONE = (IN_X - 5.0, IN_X, ELEVATOR['door_y'] - 2.0, ELEVATOR['door_y'] + 2.0)   # 门前 5 m 不放东西

SHELF_ROWS_Y = (4.0, -1.1, -6.2, -11.3)          # 排间巷道 5.1 - 0.88 = 4.22 m
SHELF_X = (-16.84, -12.92, -9.0)                 # 每排 3 个首尾相接，x 从 -18.8 到 -7.04（车放大后整体东移 1 m，
                                                 # 西侧横通道加宽到 6.2 m：5.2 m 时大车出巷道西口右转会切进货架端头）
# 摆放的 AWS 模型（ros-humble-aws-robomaker-small-warehouse-world，MIT-0）：(模型名, x, y, yaw)；
# 尺寸来自各模型碰撞网格的包围盒（yaw=0 时的 x/y 尺寸）
AWS_SIZE = {
    'ShelfD_01': (3.92, 0.88), 'ShelfE_01': (3.92, 0.88), 'ClutteringA_01': (2.16, 2.0),
    'ClutteringC_01': (1.77, 2.06), 'ClutteringD_01': (1.02, 1.52), 'PalletJackB_01': (1.16, 0.54),
    'Bucket_01': (0.94, 1.22),
}
AWS = []
for i, y in enumerate(SHELF_ROWS_Y):
    AWS += [('ShelfD_01' if i % 2 == 0 else 'ShelfE_01', x, y, 0.0) for x in SHELF_X]
AWS += [   # 暂存区：靠北墙、在月台门之间
    ('PalletJackB_01', -23.0, 14.4, 0.0), ('ClutteringC_01', -17.0, 13.9, 0.0), ('ClutteringD_01', -10.5, 14.2, 0.0),
    ('ClutteringA_01', 1.0, 13.9, 0.0), ('ClutteringC_01', 7.0, 13.9, 0.0), ('Bucket_01', 13.0, 14.3, 0.0),
]
PILLARS = [(6.0, 3.0), (11.0, -1.0)]             # 第二根原在 (9, -10)，挡住了按 1.95 m 转弯半径来回扫的建图路线
PILLAR_SIZE = 0.5

AISLE_Y = (1.45, -3.65, -8.75)                   # 三条巷道中线
# 地点朝向顺着作业流程里车自然开过来的方向，大车在窄处倒车 / 掉头很难对准（仿真里控制器反复“没有进展”）：
#   巷道单向通行，西口进、东口出，货架库位都朝东：从收货月台出来继续往西，经西侧横通道进巷道；出库从东口出来往北、沿车道往东到发货月台
#   收货月台在西北角，西墙前只有约 5 m，车从东边沿月台车道开过来，所以 R 朝西；发货月台东边空间大，S 朝东，回 R 时在东边开阔区掉头
PLACES = {
    'HOME': (SPAWN[0], SPAWN[1], SPAWN[2]),
    # 月台库位在 y=11.2（建图车道中线 11.4 附近）：北墙前大部分没观测到、会被封边，11.5 时大车轮廓压到封边
    'R1': (-20.0, 11.2, math.pi), 'R2': (-14.0, 11.2, math.pi), 'S1': (4.0, 11.2, 0.0), 'S2': (10.0, 11.2, 0.0),
    'A1': (-16.0, AISLE_Y[0], 0.0), 'A2': (-10.0, AISLE_Y[0], 0.0),
    'A3': (-16.0, AISLE_Y[1], 0.0), 'A4': (-10.0, AISLE_Y[1], 0.0),
    'A5': (-16.0, AISLE_Y[2], 0.0), 'A6': (-10.0, AISLE_Y[2], 0.0),
    'A7': (-16.0, 6.1, 0.0), 'A8': (-10.0, 6.1, 0.0),   # 第一排货架北侧车道的中线（建图时来回各开一遍）
    # 夹层库位沿逆时针绕圈的方向（朝东 / 朝北 / 朝西），不设朝南的：车头冲着南墙停的话，Hybrid-A* 常常规划不出
    # 掉头再对准坡道口回去的路径（夹层南半部朝南的起点全部规划失败）
    'M1': (18.5, -13.0, 0.0), 'M2': (21.0, -13.0, 0.0), 'M3': (23.0, -10.0, math.pi / 2),
    'M4': (20.5, -6.1, math.pi),                     # 在夹层北边车道上：绕圈时圈内侧一直在雷达近处盲区里，原来的 (20.5, -7) 建图看不到
}
# 公路（世界坐标）：南墙大门、围栏、路灯杆、建筑
GATE_X = (0.0, 6.0)
FENCE = (-40.0, 46.0, -59.0, -IN_Y - WALL_T)     # 围栏外缘；北边就是仓库南墙（墙外的两段另补）
FENCE_H, FENCE_T = 3.0, 0.2
POST_SIZE, POST_STEP = 0.5, 5.0                  # 围栏内侧的立柱：长直的围栏只约束航向和横向，立柱给出沿围栏方向的特征
LAMP_SIZE, LAMP_H, LAMP_OFF, LAMP_STEP = 0.2, 4.0, 2.0, 10.0   # 路灯杆：离环路外边缘 LAMP_OFF，约每 LAMP_STEP 一根
SIGNAL_SIZE, SIGNAL_H = 0.15, 2.5                # 红绿灯杆
# 定位参照物。雷达前倾 47°，前方被车身挡住、后方朝天，只有两侧能扫到远处的结构；第一版只有路灯杆、2 m 围栏和环路中间
# 4 座建筑，建图时公路上一圈航向漂了 10°（Cartographer 局部匹配几乎只剩地面点）。所以公路两侧都要有立着的东西：
#   - 环路中间两块空地：中间一排（BUILDINGS），沿内侧路边各一排（ROADSIDE_ROWS 生成），
#   - 环路北边（和仓库墙之间）一排，
#   - 围栏加高到 3 m，内侧每 POST_STEP 一根立柱
BUILDINGS = [(-22.25, -38.0, 1.5, 1.5), (-18.0, -38.0, 4.0, 2.0), (-9.0, -38.0, 3.0, 2.0),  # (x, y, 长, 宽)，高 BUILDING_H
             (14.0, -38.0, 4.0, 2.0), (22.0, -38.0, 5.0, 2.0), (28.25, -38.0, 1.5, 1.5)]   # 两头的放在弯道圆心之间
BUILDING_H = 3.0
ROADSIDE_ROWS = [  # (y 中心, 进深, x 起, x 止)：每 6 m 一座，长度轮流 3 / 4 / 2.5 m；压路、挨着灯杆的由 roadside_buildings 去掉
    (-34.75, 1.5, -24.0, 30.0), (-41.25, 1.5, -24.0, 30.0),            # 环路中间，离内侧路边 2.25 m
    (-18.5, 1.5, -38.0, -5.0), (-18.5, 1.5, 11.0, 43.0),               # 环路北边（仓库墙和环路之间），大门、路口两边空出来
]
HANDOFF = (road_map.ROAD_X - road_map.LANE_W, road_map.ROAD_X + road_map.LANE_W, -IN_Y, road_map.HANDOFF_Y)

RAMP = (MEZZ['ramp_x'] - MEZZ['ramp_w'] / 2, MEZZ['ramp_x'] + MEZZ['ramp_w'] / 2,
        MEZZ['y'][1], MEZZ['y'][1] + MEZZ['h'] / math.tan(math.radians(MEZZ['ramp_deg'])))


def box(x0, x1, y0, y1):
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]


ZONES = [   # (名字, 多边形, 规则)
    # 限速区往坡脚外再延 2 m：限速过滤器按车的位置生效、MPPI 逐渐减速，从坡脚开始的话车会全速冲上坡道
    ('坡道', box(RAMP[0], RAMP[1], RAMP[2], RAMP[3] + 2.0), {'speed_limit': 0.3}),
    ('夹层', box(MEZZ['x'][0], MEZZ['x'][1], MEZZ['y'][0], MEZZ['y'][1]), {'speed_limit': 0.3}),
    ('入库暂存', box(-IN_X, -8.0, 10.0, IN_Y), {}),
    ('出库暂存', box(-2.0, 13.0, 10.0, IN_Y), {}),
    ('货架区', box(-IN_X, -6.0, -IN_Y, 7.5), {'speed_limit': 0.3}),   # 含第一排货架北侧的 A7、A8；限速：大车在横通道拐进巷道时
                                                                       # 车速快会切弯切进货架端头
]
SLOTS = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7', 'A8', 'M1', 'M2', 'M3', 'M4']
INITIAL_FULL = {'A2', 'A4', 'M2'}


# ---------------------------------------------------------------- SDF 片段
def box_model(name, pose, size, material, collide=True):
    x, y, z, roll, pitch, yaw = pose
    col = (f'<collision name="c"><geometry><box><size>{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}</size></box>'
           f'</geometry></collision>') if collide else ''
    return (f'    <model name="{name}"><static>true</static><pose>{x:.4f} {y:.4f} {z:.4f} {roll:.6f} {pitch:.6f} '
            f'{yaw:.6f}</pose><link name="link">{col}<visual name="v"><geometry><box><size>{size[0]:.4f} '
            f'{size[1]:.4f} {size[2]:.4f}</size></box></geometry><material><script>'
            f'<uri>file://media/materials/scripts/gazebo.material</uri><name>{material}</name></script>'
            f'</material></visual></link></model>\n')


def include(name, uri, x, y, yaw, z=0.0):
    return (f'    <include><name>{name}</name><uri>model://{uri}</uri>'
            f'<pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.6f}</pose><static>true</static></include>\n')


def slab_on_slope(y0, y1, h0, h1, x_center, width, thickness_below, height_above=0.0):
    """沿 y 方向的斜面上的长方体：顶面（height_above=0）或底面（height_above>0）贴着从 (y0,h0) 到 (y1,h1) 的斜线。
    返回 (pose, size)。thickness_below 是斜面以下的厚度，height_above 是斜面以上的高度。"""
    if y1 < y0:   # 统一成 y 递增，保证转角在 ±90° 内、局部 z 轴朝上；否则“斜面以上/以下”会反过来
        y0, y1, h0, h1 = y1, y0, h1, h0
    dy, dz = y1 - y0, h1 - h0
    length = math.hypot(dy, dz)
    roll = math.atan2(dz, dy)                     # 绕 x 轴转：局部 y 轴指向 (0, cos, sin)
    n = (-math.sin(roll), math.cos(roll))         # 局部 z 轴在 (y, z) 平面里的方向
    my, mz = (y0 + y1) / 2, (h0 + h1) / 2
    t = thickness_below + height_above
    off = (height_above - thickness_below) / 2    # 盒子中心相对斜线沿法向的偏移
    return (x_center, my + n[0] * off, mz + n[1] * off, roll, 0.0, 0.0), (width, length, t)


def platform_parts(p):
    """平台本体 + 护栏 + 实心坡道 + 坡道两侧护栏。返回 (sdf 列表, 平台/坡道在地面上的占地矩形列表)"""
    (x0, x1), (y0, y1), h = p['x'], p['y'], p['h']
    ramp_w = p['ramp_w']
    parts, footprint = [], []
    parts.append(box_model(f"{p['name']}_platform", ((x0 + x1) / 2, (y0 + y1) / 2, h / 2, 0, 0, 0),
                           (x1 - x0, y1 - y0, h), 'Gazebo/Grey'))
    footprint.append((x0, x1, y0, y1))
    rx0, rx1 = p['ramp_x'] - ramp_w / 2, p['ramp_x'] + ramp_w / 2
    run = h / math.tan(math.radians(p['ramp_deg']))
    if p['ramp_edge'] == 'south':
        top_y, bot_y = y0, y0 - run
    else:
        top_y, bot_y = y1, y1 + run
    footprint.append((rx0, rx1, min(top_y, bot_y), max(top_y, bot_y)))
    # 实心楔形坡道：顶面从 (bot_y, 0) 到 (top_y, h)，往下的厚度足够让底面整个埋在地面以下
    pose, size = slab_on_slope(bot_y, top_y, 0.0, h, p['ramp_x'], ramp_w, thickness_below=h + 0.3)
    parts.append(box_model(f"{p['name']}_ramp", pose, size, 'Gazebo/DarkGrey'))
    # 坡道两侧护栏（在坡道宽度以内，贴着两侧边缘），底面贴坡面
    for side, xc in (('w', rx0 + GUARD_T / 2), ('e', rx1 - GUARD_T / 2)):
        pose, size = slab_on_slope(bot_y, top_y, 0.0, h, xc, GUARD_T, thickness_below=0.0, height_above=GUARD_H)
        parts.append(box_model(f"{p['name']}_ramp_rail_{side}", pose, size, 'Gazebo/Yellow'))
    # 平台护栏：站在平台顶面上；坡道所在的那条边留出坡道口
    gz = h + GUARD_H / 2
    for edge in p['guards']:
        if edge in ('south', 'north'):
            yy = y0 + GUARD_T / 2 if edge == 'south' else y1 - GUARD_T / 2
            spans = [(x0, x1)]
            if edge == p['ramp_edge']:
                spans = [(x0, rx0), (rx1, x1)]
            for i, (a, b) in enumerate(spans):
                if b - a > 1e-3:
                    parts.append(box_model(f"{p['name']}_guard_{edge}_{i}", ((a + b) / 2, yy, gz, 0, 0, 0),
                                           (b - a, GUARD_T, GUARD_H), 'Gazebo/Yellow'))
        else:
            xx = x0 + GUARD_T / 2 if edge == 'west' else x1 - GUARD_T / 2
            parts.append(box_model(f"{p['name']}_guard_{edge}", (xx, (y0 + y1) / 2, gz, 0, 0, 0),
                                   (GUARD_T, y1 - y0, GUARD_H), 'Gazebo/Yellow'))
    return parts, footprint


def world_sdf(body, script):
    return f"""<?xml version="1.0"?>
<!-- 由 {script} 生成，不要手改；改布局请改脚本后重新生成 -->
<sdf version="1.6">
  <world name="default">
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>
    <scene><ambient>0.6 0.6 0.6 1</ambient><shadows>false</shadows></scene>
    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
{''.join(body)}  </world>
</sdf>
"""


def walls():
    out = []
    hx, hy = IN_X + WALL_T / 2, IN_Y + WALL_T / 2
    for name, x, y, sx, sy in (('wall_n', 0, hy, 2 * hx + WALL_T, WALL_T), ('wall_w', -hx, 0, WALL_T, 2 * hy + WALL_T)):
        out.append(box_model(name, (x, y, WALL_H / 2, 0, 0, 0), (sx, sy, WALL_H), 'Gazebo/Grey'))
    # 南墙：在大门处切成两段
    for i, (lo, hi) in enumerate(((-hx - WALL_T / 2, GATE_X[0]), (GATE_X[1], hx + WALL_T / 2))):
        out.append(box_model(f'wall_s_{i}', ((lo + hi) / 2, -hy, WALL_H / 2, 0, 0, 0), (hi - lo, WALL_T, WALL_H),
                             'Gazebo/Grey'))
    # 东墙：在电梯门洞处切成两段
    y0, y1 = -hy - WALL_T / 2, hy + WALL_T / 2
    a, b = ELEVATOR['door_y'] - ELEVATOR['door_w'] / 2, ELEVATOR['door_y'] + ELEVATOR['door_w'] / 2
    for i, (lo, hi) in enumerate(((y0, a), (b, y1))):
        out.append(box_model(f'wall_e_{i}', (hx, (lo + hi) / 2, WALL_H / 2, 0, 0, 0), (WALL_T, hi - lo, WALL_H),
                             'Gazebo/Grey'))
    for name, x, mat in DOCK_DOORS:
        out.append(box_model(f'door_{name}', (x, IN_Y - 0.03, DOOR_H / 2, 0, 0, 0), (DOOR_W, 0.05, DOOR_H), mat,
                               collide=False))
    return out


def elevator_parts():
    """轿厢：后墙、两侧墙、顶板，内净 x ∈ [CABIN_X0, CABIN_X1]，贴着东墙外表面"""
    e = ELEVATOR
    t, h, cy = e['wall_t'], e['height'], e['door_y']
    outer_w = e['width'] + 2 * t                     # 外包宽度
    x1 = CABIN_X1 + t                                # 后墙外表面
    side_len = x1 - CABIN_X0
    mat = 'Gazebo/Blue'                              # 和灰色的墙区分开
    return [
        box_model('elevator_back', (CABIN_X1 + t / 2, cy, h / 2, 0, 0, 0), (t, outer_w, h), mat),
        box_model('elevator_side_n', ((CABIN_X0 + x1) / 2, cy + e['width'] / 2 + t / 2, h / 2, 0, 0, 0),
                  (side_len, t, h), mat),
        box_model('elevator_side_s', ((CABIN_X0 + x1) / 2, cy - e['width'] / 2 - t / 2, h / 2, 0, 0, 0),
                  (side_len, t, h), mat),
        box_model('elevator_roof', ((CABIN_X0 + x1) / 2, cy, h + t / 2, 0, 0, 0), (side_len, outer_w, t), mat),
    ]


def elevator_prior():
    """进梯程序的先验（map 坐标系 = 世界坐标 - 出生点）"""
    e = ELEVATOR
    sx, sy = SPAWN[0], SPAWN[1]
    cx = (CABIN_X0 + CABIN_X1) / 2
    wx = IN_X - e['wait_dist']
    return {
        'frame': 'map',
        'cabin_center': [round(cx - sx, 6), round(e['door_y'] - sy, 6)],
        'cabin_yaw': 0.0,
        'cabin_width': e['width'],
        'cabin_depth': e['depth'],
        'door_width': e['door_w'],
        'wait_point': [round(wx - sx, 6), round(e['door_y'] - sy, 6)],
    }


def multi_visual_model(name, boxes, material):
    """一个 static model 里放多个 <visual>、没有 collision（路面、标线：雷达看不到，地形图里仍是平地）。
    boxes: [(x, y, z, yaw, 长, 宽, 高)]"""
    mat = (f'<material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>{material}</name>'
           f'</script></material>')
    vis = ''.join(f'<visual name="v{i}"><pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.6f}</pose><geometry><box><size>'
                  f'{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>{mat}</visual>'
                  for i, (x, y, z, yaw, sx, sy, sz) in enumerate(boxes))
    return f'    <model name="{name}"><static>true</static><link name="link">{vis}</link></model>\n'


def marking_boxes(kind, pts):
    """一条标线的贴地矩形：实线每段一块，虚线画 1 m 空 1 m，停止线宽 0.3"""
    width = 0.3 if kind == 'stop_line' else 0.12
    out = []
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        ln, yaw = math.dist((x0, y0), (x1, y1)), math.atan2(y1 - y0, x1 - x0)
        spans = [(a, min(a + 1.0, ln)) for a in [2.0 * k for k in range(math.ceil(ln / 2.0))]] \
            if kind == 'dashed' else [(0.0, ln)]
        for a, b in spans:
            m = (a + b) / 2 / ln
            out.append((x0 + (x1 - x0) * m, y0 + (y1 - y0) * m, 0.003, yaw, b - a, width, 0.002))
    return out


def lamp_positions(road):
    """沿环路外边缘外 LAMP_OFF 的圆角矩形每隔约 LAMP_STEP 取一个点，去掉离路面、红绿灯杆不到 1 m 的"""
    (x0, x1), (y0, y1), r = road_map.RING_X, road_map.RING_Y, road_map.RING_R
    rr = r + 2 * road_map.LANE_W + LAMP_OFF
    dense = []
    for (cx, cy), a0 in (((x1 - r, y1 - r), math.pi / 2), ((x1 - r, y0 + r), 0.0),
                         ((x0 + r, y0 + r), -math.pi / 2), ((x0 + r, y1 - r), math.pi)):
        n = math.ceil(rr * math.pi / 2 / 0.1)
        dense += [(cx + rr * math.cos(a0 - math.pi / 2 * k / n), cy + rr * math.sin(a0 - math.pi / 2 * k / n))
                  for k in range(n + 1)]
    pts = []                              # 直线段：相邻两个圆弧端点之间按 0.1 m 插值
    for p, q in zip(dense, dense[1:] + dense[:1]):
        n = max(1, math.ceil(math.dist(p, q) / 0.1))
        pts += [(p[0] + (q[0] - p[0]) * k / n, p[1] + (q[1] - p[1]) * k / n) for k in range(n)]
    out, run = [], LAMP_STEP
    for p, q in zip(pts, pts[1:]):
        run += math.dist(p, q)
        if run >= LAMP_STEP:
            r_ = rect(*q, LAMP_SIZE, LAMP_SIZE)
            if min(gap(aabb(s), r_) for s in road.surface) >= 1.0 and \
                    all(gap(rect(x, y, SIGNAL_SIZE, SIGNAL_SIZE), r_) >= 1.0 for _, x, y, _ in road.lights):
                out.append(q)
                run = 0.0
    return out


def fence_rects():
    (fx0, fx1, fy0, fy1), t = FENCE, FENCE_T
    wx = IN_X + WALL_T                       # 仓库外墙外表面 x
    return [('fence_s', (fx0, fx1, fy0, fy0 + t)), ('fence_w', (fx0, fx0 + t, fy0, fy1)),
            ('fence_e', (fx1 - t, fx1, fy0, fy1)),
            ('fence_n_w', (fx0, -wx, fy1 - t, fy1)), ('fence_n_e', (wx, fx1, fy1 - t, fy1))]


def fence_posts():
    """围栏内侧每 POST_STEP 一根立柱（贴着围栏）"""
    (fx0, fx1, fy0, fy1), t, h = FENCE, FENCE_T, POST_SIZE / 2
    wx = IN_X + WALL_T
    out = []
    for a, b, fixed, along_x, inward in ((fx0, fx1, fy0 + t + h, True, 1), (fy0, fy1, fx0 + t + h, False, 1),
                                         (fy0, fy1, fx1 - t - h, False, -1), (fx0, -wx, fy1 - t - h, True, -1),
                                         (wx, fx1, fy1 - t - h, True, -1)):
        n = int((b - a) // POST_STEP)
        for k in range(1, n + 1):
            v = a + (b - a) * k / (n + 1)
            out.append((v, fixed) if along_x else (fixed, v))
    return out


def roadside_buildings(road, taken):
    """按 ROADSIDE_ROWS 摆一排排方块建筑，去掉离路面 < 2 m、离灯杆 / 红绿灯杆 / 已有结构 < 1 m 的"""
    lengths = (3.0, 4.0, 2.5)
    out = []
    for y, depth, x0, x1 in ROADSIDE_ROWS:
        for k in range(int((x1 - x0) // 6.0) + 1):
            r = rect(x0 + 6.0 * k, y, lengths[k % 3], depth)
            if r[3] > FENCE[3] - 0.5 or min(gap(aabb(s), r) for s in road.surface) < 2.0:
                continue
            if any(gap(r, o) < 1.0 for o in taken + out):
                continue
            out.append(r)
    return out


def outdoor_structures(road):
    """墙外有碰撞的东西：[(名字, 占地矩形)]。road 用世界坐标（road_map.build()）"""
    out = fence_rects()
    out += [(f'fence_post_{i}', rect(x, y, POST_SIZE, POST_SIZE)) for i, (x, y) in enumerate(fence_posts())]
    out += [(f'lamp_{i}', rect(x, y, LAMP_SIZE, LAMP_SIZE)) for i, (x, y) in enumerate(lamp_positions(road))]
    out += [(f'signal_{i}', rect(x, y, SIGNAL_SIZE, SIGNAL_SIZE)) for i, (_, x, y, _) in enumerate(road.lights)]
    out += [(f'building_{i}', rect(x, y, sx, sy)) for i, (x, y, sx, sy) in enumerate(BUILDINGS)]
    rows = roadside_buildings(road, [r for _, r in out[len(fence_rects()):]])
    out += [(f'building_{len(BUILDINGS) + i}', r) for i, r in enumerate(rows)]
    e = ELEVATOR
    out.append(('elevator', (CABIN_X0, CABIN_X1 + e['wall_t'], e['door_y'] - e['width'] / 2 - e['wall_t'],
                             e['door_y'] + e['width'] / 2 + e['wall_t'])))
    return out


def road_parts(road):
    surface = [(x, y, 0.001, yaw, ln, w, 0.002) for x, y, yaw, ln, w in road.surface]
    marks = [b for kind, pts in road.markings for b in marking_boxes(kind, pts)]
    body = [multi_visual_model('road_surface', surface, 'Gazebo/DarkGrey'),
            multi_visual_model('road_markings', marks, 'Gazebo/White')]
    for name, (a, b, c, d) in outdoor_structures(road):
        kind = name.split('_')[0]
        if kind == 'elevator':
            continue
        h, mat = {'fence': (FENCE_H, 'Gazebo/Grey'), 'lamp': (LAMP_H, 'Gazebo/DarkGrey'),
                  'signal': (SIGNAL_H, 'Gazebo/Yellow'), 'building': (BUILDING_H, 'Gazebo/Bricks')}[kind]
        body.append(box_model(name, ((a + b) / 2, (c + d) / 2, h / 2, 0, 0, 0), (b - a, d - c, h), mat))
    return body


# ---------------------------------------------------------------- 布局检查
def rect(x, y, sx, sy, yaw=0.0):
    if abs(math.sin(yaw)) > 0.5:
        sx, sy = sy, sx
    return (x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2)


def gap(a, b):
    """两个轴对齐矩形之间的最小距离（重叠为负）"""
    dx = max(a[0] - b[1], b[0] - a[1])
    dy = max(a[2] - b[3], b[2] - a[3])
    if dx < 0 and dy < 0:
        return max(dx, dy)
    return math.hypot(max(dx, 0), max(dy, 0))


def aabb(s):
    """路面矩形 (cx, cy, yaw, 长, 宽) 的外接轴对齐矩形（宁严勿松）"""
    cx, cy, yaw, ln, w = s
    c, si = abs(math.cos(yaw)), abs(math.sin(yaw))
    hx, hy = (c * ln + si * w) / 2, (si * ln + c * w) / 2
    return (cx - hx, cx + hx, cy - hy, cy + hy)


def car_rect(x, y, yaw):
    return rect(x, y, CAR[0] + 2 * CAR_MARGIN, CAR[1] + 2 * CAR_MARGIN, yaw)


def inside(r, outer):
    return r[0] >= outer[0] and r[1] <= outer[1] and r[2] >= outer[2] and r[3] <= outer[3]


def check_layout(structures):
    """structures: [(名字, 占地矩形)]，夹层和坡道。返回问题列表"""
    items = [(f'{m}#{i}', rect(x, y, *AWS_SIZE[m], yaw)) for i, (m, x, y, yaw) in enumerate(AWS)]
    items += [(f'pillar#{i}', rect(x, y, PILLAR_SIZE, PILLAR_SIZE)) for i, (x, y) in enumerate(PILLARS)]
    problems = []
    walls_in = (-IN_X, IN_X, -IN_Y, IN_Y)
    for n, r in items:
        if not inside(r, walls_in):
            problems.append(f'{n} 超出墙内范围')
        for sn, s in structures:
            if gap(r, s) < 2.0:                      # 坡道口要能开直线上下，夹层旁要能掉头
                problems.append(f'{n} 离 {sn} 只有 {gap(r, s):.2f} m')
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            g = gap(items[i][1], items[j][1])
            if g < 0.3 and not (items[i][0].startswith('Shelf') and items[j][0].startswith('Shelf') and g > -1e-6):
                problems.append(f'{items[i][0]} 与 {items[j][0]} 间距 {g:.2f} m')
    for a, b in zip(SHELF_ROWS_Y, SHELF_ROWS_Y[1:]):
        if a - b - AWS_SIZE['ShelfD_01'][1] < 4.0:
            problems.append(f'y={a} 和 y={b} 两排货架之间的巷道不到 4 m')
    mezz_top = (MEZZ['x'][0] + GUARD_T, MEZZ['x'][1], MEZZ['y'][0], MEZZ['y'][1] - GUARD_T)
    for name, (x, y, yaw) in PLACES.items():
        r = car_rect(x, y, yaw)
        if not inside(r, walls_in):
            problems.append(f'地点 {name} 的车体超出墙内范围')
        for n, it in items:
            if gap(r, it) < 0:
                problems.append(f'地点 {name} 的车体和 {n} 重叠')
        on_top = inside(r, mezz_top)
        if not on_top and any(gap(r, s) < 0 for _, s in structures):
            problems.append(f'地点 {name} 的车体压在夹层或坡道边上（要么整个在夹层上，要么整个在地面上）')
    # 坡道两端各留 4 m 平直进出段（坡道朝北下：底端在 RAMP[3]，顶端在夹层上的 RAMP[2]）
    for end, r in (('底端', (RAMP[0], RAMP[1], RAMP[3], RAMP[3] + 4.0)), ('顶端', (RAMP[0], RAMP[1], RAMP[2] - 4.0, RAMP[2]))):
        for n, it in items:
            if gap(r, it) < 0:
                problems.append(f'{n} 挡在坡道{end}外 4 m 的平直段里')
    # 库位要落在所属区域里，按区域查询（semantic.py where）才查得到
    zones = {n: poly for n, poly, _ in ZONES}
    for name, (x, y, _) in PLACES.items():
        zone = '货架区' if name.startswith('A') else '夹层' if name.startswith('M') else None
        if zone and not (zones[zone][0][0] <= x <= zones[zone][1][0] and zones[zone][0][1] <= y <= zones[zone][2][1]):
            problems.append(f'库位 {name} 不在区域 {zone} 里')
    # 交接车道（仓库里那 6 m）要空着：Nav2 把车送到起点，再沿车道开出大门
    for n, r in items + structures:
        if gap(r, HANDOFF) < 0.5:
            problems.append(f'{n} 挡在交接车道上')
    sx, sy, _ = SPAWN
    for n, r in items + structures:
        if gap(r, (sx - 1.5, sx + 1.5, sy - 1.5, sy + 1.5)) < 0:
            problems.append(f'出生点 1.5 m 内有 {n}')
        # 电梯门前 5 m 要空着：车在等待点对正、提取轿厢，再沿中心线开进去
        if gap(r, DOOR_ZONE) < 0:
            problems.append(f'{n} 挡在电梯门前')
    return problems


def check_outdoor(road):
    """墙外：结构不压路面；路灯、红绿灯杆离路面 ≥ 1 m，建筑 ≥ 2 m；都在围栏里"""
    problems = []
    fence_in = (FENCE[0] + FENCE_T, FENCE[1] - FENCE_T, FENCE[2] + FENCE_T, FENCE[3])
    for n, r in outdoor_structures(road):
        g = min(gap(aabb(s), r) for s in road.surface)
        need = {'lamp': 1.0, 'signal': 1.0, 'building': 2.0}.get(n.split('_')[0], 0.0)
        if g < need - 1e-6:
            problems.append(f'{n} 离路面只有 {g:.2f} m')
        if need and not inside(r, fence_in):
            problems.append(f'{n} 不在围栏里')
    for s in road.surface:
        if not inside(aabb(s), (FENCE[0] + FENCE_T, FENCE[1] - FENCE_T, FENCE[2] + FENCE_T, FENCE[3] + 1e-6)):
            problems.append(f'路面 {s[:2]} 超出围栏')
    return problems


def semantic_dict():
    """世界坐标 -> map 坐标：Cartographer 的 map 原点是车启动时的位姿，出生点 yaw = 0，所以只需平移"""
    sx, sy, yaw0 = SPAWN
    assert yaw0 == 0.0, '出生点 yaw 不为 0 时还要做旋转换算，这里只做了平移'

    places = [{'name': n, 'x': round(x - sx, 3), 'y': round(y - sy, 3), 'yaw': round(yaw, 4)}
              for n, (x, y, yaw) in PLACES.items()]
    zones = [dict({'name': n, 'polygon': [[round(x - sx, 3), round(y - sy, 3)] for x, y in poly]}, **rule)
             for n, poly, rule in ZONES]
    return {'places': places, 'zones': zones}


def tasks_dict():
    return {'home': 'HOME', 'dwell': 5.0, 'receiving': ['R1', 'R2'], 'shipping': ['S1', 'S2'],
            'slots': {s: s in INITIAL_FULL for s in SLOTS}}


HEADERS = {'warehouse_semantic.yaml': '# 由 gen_warehouse_world.py 生成（map 坐标系 = 世界坐标 - 出生点）\n',
           'warehouse_tasks.yaml': '# 由 gen_warehouse_world.py 生成的任务配置模板（初始库存），复制到地图旁边使用\n',
           'warehouse_elevator.yaml': '# 由 gen_warehouse_world.py 生成（map 坐标系 = 世界坐标 - 出生点）；进梯程序的先验\n',
           'map_projector_info.yaml': '# 由 gen_warehouse_world.py 生成；Autoware 的地图投影：Local = 直接用 .osm 里的 local_x / local_y\n'}


def build():
    """返回 (布局问题列表, world 文本, {yaml 文件名: 内容 dict})"""
    parts, fps = platform_parts(MEZZ)
    structures = [('夹层', fps[0]), ('坡道', fps[1])]
    road = road_map.build()
    problems = check_layout(structures) + check_outdoor(road)
    body = parts + walls() + elevator_parts() + road_parts(road)
    for i, (x, y) in enumerate(PILLARS):
        body.append(box_model(f'pillar_{i}', (x, y, WALL_H / 2, 0, 0, 0), (PILLAR_SIZE, PILLAR_SIZE, WALL_H),
                                'Gazebo/Grey'))
    for i, (m, x, y, yaw) in enumerate(AWS):
        body.append(include(f'{m}_{i}', f'aws_robomaker_warehouse_{m}', x, y, yaw))
    files = {'warehouse_semantic.yaml': semantic_dict(), 'warehouse_tasks.yaml': tasks_dict(),
             'warehouse_elevator.yaml': elevator_prior(), 'map_projector_info.yaml': dict(road_map.PROJECTOR_INFO)}
    return problems, world_sdf(body, 'gen_warehouse_world.py'), files


def main():
    problems, world, files = build()
    if problems:
        print('布局检查未通过：\n  ' + '\n  '.join(problems))
        sys.exit(1)
    worlds = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'worlds')
    with open(os.path.join(worlds, 'warehouse.world'), 'w') as f:
        f.write(world)
    for name, data in files.items():
        with open(os.path.join(worlds, name), 'w', encoding='utf-8') as f:
            f.write(HEADERS[name])
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    road_map.write_osm(road_map.build(SPAWN[:2]), os.path.join(worlds, 'warehouse_lanelet2_map.osm'))
    run = MEZZ['h'] / math.tan(math.radians(MEZZ['ramp_deg']))
    elev = files['warehouse_elevator.yaml']
    print(f"已生成 warehouse.world：{world.count('<model ') + world.count('<include><name>')} 个模型；出生点 {SPAWN}；"
          f"坡道水平长度 {run:.2f} m，y 从 {RAMP[2]} 到 {RAMP[3]:.2f}；{len(PLACES)} 个地点，{len(ZONES)} 个区域")
    print(f"  电梯：轿厢中心（map）{elev['cabin_center']}，等待点（map）{elev['wait_point']}")


if __name__ == '__main__':
    main()

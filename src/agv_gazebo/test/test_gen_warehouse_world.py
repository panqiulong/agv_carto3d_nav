import importlib.util
import pathlib
import re
import xml.etree.ElementTree as ET

import pytest
import yaml

PKG = pathlib.Path(__file__).resolve().parent.parent
WORLDS = PKG / 'worlds'
spec = importlib.util.spec_from_file_location('gen_warehouse_world', PKG / 'scripts' / 'gen_warehouse_world.py')
gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gen)


def test_layout_ok():
    problems, _, _ = gen.build()
    assert problems == []


def test_generated_files_up_to_date():
    _, world, files = gen.build()
    assert (WORLDS / 'warehouse.world').read_text() == world
    for name, data in files.items():
        assert yaml.safe_load((WORLDS / name).read_text(encoding='utf-8')) == data


def test_elevator_prior_in_map_frame():
    _, _, files = gen.build()
    e = files['warehouse_elevator.yaml']
    # 世界坐标：轿厢中心 (26.6, 11.2)，等待点 (21.15, 11.2)；出生点 (2, -4)
    assert e['frame'] == 'map'
    assert e['cabin_center'] == pytest.approx([24.6, 15.2])
    assert e['wait_point'] == pytest.approx([19.15, 15.2])
    assert e['cabin_yaw'] == pytest.approx(0.0)
    assert (e['cabin_width'], e['cabin_depth'], e['door_width']) == (2.0, 2.6, 1.8)


def test_door_cut_in_east_wall():
    _, world, _ = gen.build()
    # wall_e 切成两段：门洞 y ∈ [10.3, 12.1] 里没有东墙
    assert '<model name="wall_e"' not in world
    assert world.count('<model name="wall_e_') == 2
    for n in ('elevator_back', 'elevator_side_n', 'elevator_side_s', 'elevator_roof'):
        assert f'<model name="{n}"' in world


def test_object_in_front_of_door_is_rejected(monkeypatch):
    monkeypatch.setattr(gen, 'AWS', gen.AWS + [('Bucket_01', 23.0, 11.2, 0.0)])
    problems, _, _ = gen.build()
    assert any('电梯门前' in p for p in problems)


def _models(world):
    return {m.get('name'): m for m in ET.fromstring(world).iter('model')}


def _pose_size(model):
    pose = [float(v) for v in model.find('pose').text.split()]
    size = [float(v) for v in model.find('.//box/size').text.split()]
    return pose, size


def test_gate_cut_in_south_wall():
    _, world, _ = gen.build()
    assert '<model name="wall_s"' not in world
    models = _models(world)
    walls = [models[n] for n in ('wall_s_0', 'wall_s_1')]
    assert len(re.findall(r'<model name="wall_s_', world)) == 2
    for m in walls:
        (x, *_), (sx, *_) = _pose_size(m)
        assert x + sx / 2 <= 0.0 + 1e-6 or x - sx / 2 >= 6.0 - 1e-6     # 门洞 x ∈ [0, 6] 里没有墙


def test_road_does_not_hit_structures():
    road = gen.road_map.build()
    for name, r in gen.outdoor_structures(road):
        for s in road.surface:
            assert gen.gap(gen.aabb(s), r) >= -1e-6, name


def test_lamps_and_buildings_off_road():
    road = gen.road_map.build()
    checked = 0
    for name, r in gen.outdoor_structures(road):
        if name.startswith(('lamp', 'signal', 'building')):
            checked += 1
            assert min(gen.gap(gen.aabb(s), r) for s in road.surface) >= 1.0, name
    assert checked > 20


def test_osm_up_to_date(tmp_path):
    p = tmp_path / 'm.osm'
    gen.road_map.write_osm(gen.road_map.build(gen.SPAWN[:2]), str(p))
    assert (WORLDS / 'warehouse_lanelet2_map.osm').read_bytes() == p.read_bytes()
    assert yaml.safe_load((WORLDS / 'map_projector_info.yaml').read_text()) == gen.road_map.PROJECTOR_INFO


def test_road_visuals_have_no_collision():
    _, world, _ = gen.build()
    models = _models(world)
    for n in ('road_surface', 'road_markings'):
        assert models[n].find('.//collision') is None
        assert 0 < len(models[n].findall('.//visual')) < 1500


def test_landmarks_along_every_lane():
    # 雷达前倾，只有两侧能扫到远处：公路上任何位置左右两边 20 m（Cartographer 高分辨率子图的范围）内各要有 2 个以上
    # 立着的结构（围栏段 / 立柱 / 建筑 / 仓库墙），
    # 否则 Cartographer 在空地上航向会漂（2026-10-31 建图实测：公路一圈漂 10°）
    road = gen.road_map.build()
    outdoor = [r for n, r in gen.outdoor_structures(road) if not n.startswith(('lamp', 'signal', 'elevator'))]
    wall_s = (-gen.IN_X - gen.WALL_T, gen.IN_X + gen.WALL_T, -gen.IN_Y - gen.WALL_T, -gen.IN_Y)
    rects = outdoor + [wall_s]
    sparse = []
    for ll in road.lmap.laneletLayer:
        if not ll.attributes['name'].startswith('ring_'):
            continue
        c = [(p.x, p.y) for p in ll.centerline]
        for (x, y), (x1, y1) in zip(c, c[1:]):
            hx, hy = x1 - x, y1 - y
            sides = [0, 0]                                   # 左、右各几个（按结构上离车最近的点算）
            for r in rects:
                if gen.gap((x, x, y, y), r) < 20.0:
                    nx, ny = min(max(x, r[0]), r[1]), min(max(y, r[2]), r[3])
                    sides[0 if hx * (ny - y) - hy * (nx - x) > 0 else 1] += 1
            if min(sides) < 2:
                sparse.append((ll.attributes['name'], round(x, 1), round(y, 1), sides))
    assert sparse == []

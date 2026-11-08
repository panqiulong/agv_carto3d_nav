import os
import sys

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import semantic_map as S  # noqa: E402

SQUARE = [[0, 0], [2, 0], [2, 2], [0, 2]]


def write(tmp_path, data, name='m_semantic.yaml'):
    p = tmp_path / name
    p.write_text(yaml.safe_dump(data, allow_unicode=True), encoding='utf-8')
    return str(p)


def test_missing_file_is_empty(tmp_path):
    sm = S.load(str(tmp_path / 'nope_semantic.yaml'))
    assert sm.places == [] and sm.zones == []


def test_roundtrip_keeps_chinese_names_and_rules(tmp_path):
    sm = S.SemanticMap(
        places=[S.Place('P2 装卸口', 38.6, -18.8, 1.5)],
        zones=[S.Zone('P1坡道', SQUARE, speed_limit=0.3), S.Zone('货车停靠区', SQUARE, keepout=True),
               S.Zone('厂区B', SQUARE)])
    path = str(tmp_path / 'm_semantic.yaml')
    S.save(sm, path)
    back = S.load(path)
    assert back == sm
    assert 'P2 装卸口' in open(path, encoding='utf-8').read()      # 中文原样写入，不转义


def test_save_creates_missing_directory(tmp_path):
    path = str(tmp_path / 'a' / 'b' / 'm_semantic.yaml')
    S.save(S.SemanticMap(places=[S.Place('x', 0, 0, 0)]), path)
    assert S.load(path).place('x') is not None


@pytest.mark.parametrize('data, words', [
    ({'places': [{'name': 'a', 'x': 0, 'y': 0}, {'name': 'a', 'x': 1, 'y': 1}]}, '重复'),
    ({'zones': [{'name': 'z', 'polygon': SQUARE}, {'name': 'z', 'polygon': SQUARE}]}, '重复'),
    ({'places': [{'name': ' ', 'x': 0, 'y': 0}]}, '不能为空'),
    ({'zones': [{'name': 'z', 'polygon': [[0, 0], [1, 1]]}]}, '至少要 3 个点'),
    ({'zones': [{'name': 'z', 'polygon': SQUARE, 'speed_limit': 0}]}, '(0, 0.5]'),
    ({'zones': [{'name': 'z', 'polygon': SQUARE, 'speed_limit': 1.5}]}, '(0, 0.5]'),
    ({'zones': [{'name': 'z', 'polygon': SQUARE, 'speed_limit': 0.3, 'keepout': True}]}, '不能同时'),
    ({'zones': [{'name': 'z', 'polygon': [[0, 0, 0], [1, 0, 0], [1, 1, 0]]}]}, 'zones 第 1 条'),
    ({'places': [{'name': 'a', 'y': 0}]}, 'places 第 1 条'),
])
def test_invalid_files_are_rejected_with_reason(tmp_path, data, words):
    with pytest.raises(S.SemanticMapError) as e:
        S.load(write(tmp_path, data))
    assert words in str(e.value)


def test_save_invalid_map_leaves_existing_file_untouched(tmp_path):
    path = str(tmp_path / 'm_semantic.yaml')
    S.save(S.SemanticMap(places=[S.Place('a', 0, 0, 0)]), path)
    before = open(path, encoding='utf-8').read()
    with pytest.raises(S.SemanticMapError):
        S.save(S.SemanticMap(places=[S.Place('a', 0, 0, 0), S.Place('a', 1, 1, 0)]), path)
    assert open(path, encoding='utf-8').read() == before
    assert [p for p in os.listdir(tmp_path) if p.endswith('.tmp')] == []


L_SHAPE = [[0, 0], [3, 0], [3, 1], [1, 1], [1, 3], [0, 3]]     # 凹多边形


@pytest.mark.parametrize('x, y, inside', [
    (1, 1, True), (3, 1, False), (-0.1, 1, False),
    (0, 1, True), (2, 2, True), (2, 0, True),                 # 边上、顶点算在内
])
def test_point_in_square(x, y, inside):
    assert S.point_in_polygon(x, y, SQUARE) is inside


def test_point_in_concave_polygon():
    assert S.point_in_polygon(0.5, 2.5, L_SHAPE)
    assert S.point_in_polygon(2.5, 0.5, L_SHAPE)
    assert not S.point_in_polygon(2, 2, L_SHAPE)               # 凹进去的缺口


def test_clockwise_and_counterclockwise_agree():
    cw = list(reversed(L_SHAPE))
    for x, y in ((0.5, 2.5), (2.5, 0.5), (2, 2), (5, 5)):
        assert S.point_in_polygon(x, y, cw) == S.point_in_polygon(x, y, L_SHAPE)


def test_zone_queries_and_overlapping_speed_takes_minimum():
    sm = S.SemanticMap(zones=[
        S.Zone('慢', SQUARE, speed_limit=0.5),
        S.Zone('更慢', [[1, 1], [3, 1], [3, 3], [1, 3]], speed_limit=0.3),
        S.Zone('禁行', [[10, 10], [11, 10], [11, 11]], keepout=True),
        S.Zone('厂区', [[-5, -5], [5, -5], [5, 5], [-5, 5]]),
    ])
    assert [z.name for z in S.zones_at(sm, 0.5, 0.5)] == ['慢', '厂区']
    assert S.speed_limit_at(sm, 0.5, 0.5) == 0.5
    assert S.speed_limit_at(sm, 1.5, 1.5) == 0.3
    assert S.speed_limit_at(sm, 4, 4) is None
    assert S.keepout_zone_at(sm, 10.5, 10.2).name == '禁行'
    assert S.keepout_zone_at(sm, 0.5, 0.5) is None


def test_zones_hash_depends_only_on_zones():
    a = S.SemanticMap(zones=[S.Zone('z', SQUARE, speed_limit=0.3)])
    b = S.SemanticMap(places=[S.Place('p', 1, 2, 0)], zones=[S.Zone('z', SQUARE, speed_limit=0.3)])
    c = S.SemanticMap(zones=[S.Zone('z', SQUARE, speed_limit=0.4)])
    assert S.zones_hash(a) == S.zones_hash(b)
    assert S.zones_hash(a) != S.zones_hash(c)
    assert len(S.zones_hash(a)) == 40


import numpy as np  # noqa: E402


def test_speed_encoding():
    assert S.encode_speed(0.3) == 30 and abs(S.decode_speed(30) - 0.3) < 1e-9
    assert S.encode_speed(1.0) == 100
    assert S.encode_speed(0.004) == 1                          # 最低编码 1（0 表示不限速）


def test_rasterize_values_and_pgm_row_order():
    # 地图 4 m x 4 m，原点 (-1, -1)，分辨率 0.5 -> 8 x 8；格子中心 x = -0.75, -0.25, ..., 2.75
    sm = S.SemanticMap(zones=[
        S.Zone('上方禁行', [[0, 2], [2, 2], [2, 3], [0, 3]], keepout=True),        # y 2~3：地图上方
        S.Zone('慢', [[-1, -1], [1, -1], [1, 1], [-1, 1]], speed_limit=0.5),       # 左下 2 m x 2 m
        S.Zone('更慢', [[0, 0], [1, 0], [1, 1], [0, 1]], speed_limit=0.3),         # 与上面重叠 1 m x 1 m
    ])
    keep, speed = S.rasterize(sm, (-1.0, -1.0), 0.5, 8, 8)
    assert keep.shape == (8, 8) and keep.dtype == np.uint8
    # pgm 第 0 行是 y 最大（中心 y=2.75），第 1 行 y=2.25；x 中心 0.25~1.75 是第 2~5 列
    assert (keep[0:2, 2:6] == 100).all()
    assert keep[0:2].sum() == 100 * 8 and keep[2:].sum() == 0
    # 左下角：pgm 最后一行 y=-0.75；x=-0.75 是第 0 列
    assert speed[7, 0] == 50
    assert speed[5, 2] == 30                                   # 中心 (0.25, 0.25)：两区重叠取 0.3
    assert speed[0, 7] == 0                                    # 右上：不限速


def make_map(tmp_path, width=8, height=6, res=0.5, origin=(-1.0, -1.0)):
    (tmp_path / 'maps').mkdir()
    pgm = tmp_path / 'maps' / 'map.pgm'
    pgm.write_bytes(b'P5\n# CREATOR: map_saver.cpp 0.500 m/pix\n%d %d\n255\n' % (width, height)
                    + bytes(width * height))
    (tmp_path / 'maps' / 'map.yaml').write_text(yaml.safe_dump({
        'image': 'map.pgm', 'mode': 'raw', 'resolution': res, 'origin': [origin[0], origin[1], 0.0],
        'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196}))
    return str(tmp_path / 'maps' / 'map.yaml')


def test_mask_paths_naming():
    p = S.mask_paths('/d/map_semantic.yaml')
    assert p['keepout_yaml'] == '/d/map_keepout_mask.yaml' and p['speed_pgm'] == '/d/map_speed_mask.pgm'
    assert p['sha1'] == '/d/map_masks.sha1'
    assert S.mask_paths('/d/zones.yaml')['sha1'] == '/d/zones_masks.sha1'


def test_write_masks_from_relative_image_with_comment_header(tmp_path):
    map_yaml = make_map(tmp_path)
    sem = str(tmp_path / 'maps' / 'map_semantic.yaml')
    sm = S.SemanticMap(zones=[S.Zone('禁行', [[-1, -1], [0, -1], [0, 0], [-1, 0]], keepout=True),
                              S.Zone('慢', [[2, 1], [3, 1], [3, 2], [2, 2]], speed_limit=0.3)])
    S.save(sm, sem)
    paths = S.write_masks(map_yaml, sem)
    assert S.read_pgm_size(paths['keepout_pgm']) == (8, 6)
    meta = yaml.safe_load(open(paths['speed_yaml']))
    assert meta['mode'] == 'raw' and meta['image'] == 'map_speed_mask.pgm'
    assert meta['origin'] == [-1.0, -1.0, 0.0] and meta['resolution'] == 0.5
    raw = open(paths['keepout_pgm'], 'rb').read()
    keep = np.frombuffer(raw[-48:], np.uint8).reshape(6, 8)
    assert keep[5, 0] == 100 and keep[5, 1] == 100 and keep[4, 0] == 100 and keep[3, 0] == 0
    assert open(paths['sha1']).read().strip() == S.zones_hash(sm)


def test_speed_limit_above_vehicle_max_is_rejected(tmp_path):
    # MPPI 的绝对限速按 limit / vx_max 缩放速度上限且不封顶：大于 vx_max 的“限速”会让车更快
    with pytest.raises(S.SemanticMapError) as e:
        S.load(write(tmp_path, {'zones': [{'name': 'z', 'polygon': SQUARE, 'speed_limit': 0.8}]}))
    assert '(0, 0.5]' in str(e.value)

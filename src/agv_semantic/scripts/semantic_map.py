"""语义地图：有名字的地点与区域（纯 Python，不依赖 ROS，便于单元测试）

坐标都在 map 坐标系下，单位米、弧度。文件格式见
docs/superpowers/specs/2026-09-27-semantic-places-zones-design.md 第 2 节。
"""
import hashlib
import os
import tempfile
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np
import yaml


# 限速上限 = nav2_params.yaml 里 MPPI 的 vx_max。MPPI 的绝对限速按 limit / vx_max 缩放速度上限且不封顶，
# 更大的“限速”会让车更快（agv_nav/test/test_navigation_launch.py 检查两者一致）
MAX_SPEED_LIMIT = 0.5


class SemanticMapError(ValueError):
    """语义文件内容不合法"""


@dataclass
class Place:
    name: str
    x: float
    y: float
    yaw: float


@dataclass
class Zone:
    name: str
    polygon: List[List[float]]
    speed_limit: Optional[float] = None     # m/s
    keepout: bool = False


@dataclass
class SemanticMap:
    places: List[Place] = field(default_factory=list)
    zones: List[Zone] = field(default_factory=list)

    def place(self, name):
        return next((p for p in self.places if p.name == name), None)


def from_dict(data):
    """YAML 内容 -> SemanticMap，格式不对抛 SemanticMapError"""
    data = data or {}
    if not isinstance(data, dict):
        raise SemanticMapError('语义文件顶层必须是字典（places / zones）')
    places, zones = [], []
    for i, p in enumerate(data.get('places') or []):
        try:
            places.append(Place(str(p['name']).strip(), float(p['x']), float(p['y']), float(p.get('yaw', 0.0))))
        except (KeyError, TypeError, ValueError, AttributeError) as e:
            raise SemanticMapError('places 第 %d 条格式错误：%r' % (i + 1, e))
    for i, z in enumerate(data.get('zones') or []):
        try:
            poly = z['polygon']
            if not all(len(v) == 2 for v in poly):
                raise ValueError('多边形的每个点必须是 [x, y]')
            speed = z.get('speed_limit')
            zones.append(Zone(str(z['name']).strip(), [[float(v[0]), float(v[1])] for v in poly],
                              None if speed is None else float(speed), bool(z.get('keepout', False))))
        except (KeyError, TypeError, ValueError, AttributeError) as e:
            raise SemanticMapError('zones 第 %d 条格式错误：%r' % (i + 1, e))
    sm = SemanticMap(places, zones)
    validate(sm)
    return sm


def validate(sm):
    for kind, items in (('地点', sm.places), ('区域', sm.zones)):
        names = [it.name.strip() for it in items]
        if any(not n for n in names):
            raise SemanticMapError('%s名字不能为空' % kind)
        dup = sorted({n for n in names if names.count(n) > 1})
        if dup:
            raise SemanticMapError('%s名字重复：%s' % (kind, '、'.join(dup)))
    for z in sm.zones:
        if len(z.polygon) < 3:
            raise SemanticMapError('区域 %s 的多边形至少要 3 个点' % z.name)
        if z.speed_limit is not None and not 0.0 < z.speed_limit <= MAX_SPEED_LIMIT:
            raise SemanticMapError('区域 %s 的 speed_limit 必须在 (0, %g] m/s 之间（车的最高速度）'
                                   % (z.name, MAX_SPEED_LIMIT))
        if z.speed_limit is not None and z.keepout:
            raise SemanticMapError('区域 %s 不能同时设置 speed_limit 和 keepout' % z.name)


def to_dict(sm):
    zones = []
    for z in sm.zones:
        d = {'name': z.name, 'polygon': [[float(x), float(y)] for x, y in z.polygon]}
        if z.speed_limit is not None:
            d['speed_limit'] = float(z.speed_limit)
        if z.keepout:
            d['keepout'] = True
        zones.append(d)
    return {'places': [{'name': p.name, 'x': float(p.x), 'y': float(p.y), 'yaw': float(p.yaw)} for p in sm.places],
            'zones': zones}


def load(path):
    """文件不存在时返回空的语义地图（第一次添加地点时再创建）"""
    if not os.path.exists(path):
        return SemanticMap()
    with open(path, encoding='utf-8') as f:
        try:
            data = yaml.safe_load(f)
        except yaml.YAMLError as e:
            raise SemanticMapError('YAML 解析失败：%s' % e)
    return from_dict(data)


def save(sm, path):
    """先校验；写临时文件再改名，中途出错不会损坏原文件"""
    validate(sm)
    folder = os.path.dirname(os.path.abspath(path))
    os.makedirs(folder, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=folder, suffix='.tmp')
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            yaml.safe_dump(to_dict(sm), f, allow_unicode=True, sort_keys=False)
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.remove(tmp)
        raise


def point_in_polygon(x, y, polygon):
    """射线法；落在边上（含顶点）算在内"""
    n = len(polygon)
    inside = False
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        cross = (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)
        if (abs(cross) < 1e-9 and min(x1, x2) - 1e-9 <= x <= max(x1, x2) + 1e-9
                and min(y1, y2) - 1e-9 <= y <= max(y1, y2) + 1e-9):
            return True
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


def zones_at(sm, x, y):
    return [z for z in sm.zones if point_in_polygon(x, y, z.polygon)]


def speed_limit_at(sm, x, y):
    limits = [z.speed_limit for z in zones_at(sm, x, y) if z.speed_limit is not None]
    return min(limits) if limits else None


def keepout_zone_at(sm, x, y):
    return next((z for z in zones_at(sm, x, y) if z.keepout), None)


def zones_hash(sm):
    """只由 zones 决定：用来判断掩码是否过期（改地点不需要重新生成掩码）"""
    text = yaml.safe_dump(to_dict(sm)['zones'], allow_unicode=True, sort_keys=True)
    return hashlib.sha1(text.encode('utf-8')).hexdigest()


def encode_speed(limit):
    """限速 m/s -> 掩码值 1~100（限速 = 掩码值 x 0.01 m/s；0 留给“不限速”）"""
    return int(min(100, max(1, round(limit * 100))))


def decode_speed(value):
    return value * 0.01


def _polygon_mask(polygon, xs, ys):
    """射线法的向量化版本：xs、ys 是格子中心坐标数组"""
    inside = np.zeros(xs.shape, bool)
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        crosses = (y1 > ys) != (y2 > ys)
        with np.errstate(divide='ignore', invalid='ignore'):
            x_cross = x1 + (ys - y1) * (x2 - x1) / (y2 - y1)
        inside ^= crosses & (xs < x_cross)
    return inside


def rasterize(sm, origin, resolution, width, height):
    """区域 -> (禁行掩码, 限速掩码)，格子中心落在多边形内算在区域里；返回 pgm 行序（第 0 行是 y 最大）"""
    xs = origin[0] + (np.arange(width) + 0.5) * resolution
    ys = origin[1] + (np.arange(height) + 0.5) * resolution
    X, Y = np.meshgrid(xs, ys)                  # 第 0 行 y 最小
    keep = np.zeros((height, width), np.uint8)
    speed = np.zeros((height, width), np.uint8)
    for z in sm.zones:
        m = _polygon_mask(z.polygon, X, Y)
        if z.keepout:
            keep[m] = 100
        if z.speed_limit is not None:
            v = encode_speed(z.speed_limit)
            speed[m] = np.where(speed[m] == 0, v, np.minimum(speed[m], v))
    return np.ascontiguousarray(keep[::-1]), np.ascontiguousarray(speed[::-1])


def read_pgm_size(path):
    """二进制 pgm（P5）的宽、高；头里可以有 # 注释行（map_saver 会写）"""
    tokens = []
    with open(path, 'rb') as f:
        while len(tokens) < 4:
            line = f.readline()
            if not line:
                raise SemanticMapError('pgm 文件头不完整：%s' % path)
            tokens += line.split(b'#')[0].split()
    if tokens[0] != b'P5':
        raise SemanticMapError('只支持二进制 pgm（P5）：%s' % path)
    return int(tokens[1]), int(tokens[2])


def write_pgm(path, array):
    h, w = array.shape
    with open(path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h))
        f.write(np.ascontiguousarray(array, np.uint8).tobytes())


def read_map_yaml(path):
    with open(path) as f:
        m = yaml.safe_load(f)
    image = m['image']
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(path)), image)
    return image, float(m['resolution']), (float(m['origin'][0]), float(m['origin'][1]))


def mask_paths(semantic_file):
    suffix = '_semantic.yaml'
    base = semantic_file[:-len(suffix)] if semantic_file.endswith(suffix) else os.path.splitext(semantic_file)[0]
    return {'keepout_yaml': base + '_keepout_mask.yaml', 'keepout_pgm': base + '_keepout_mask.pgm',
            'speed_yaml': base + '_speed_mask.yaml', 'speed_pgm': base + '_speed_mask.pgm',
            'sha1': base + '_masks.sha1'}


def write_masks(map_yaml, semantic_file):
    """按 map.yaml 的尺寸、原点、分辨率生成两张掩码（raw 模式）和区域哈希"""
    sm = load(semantic_file)
    image, res, origin = read_map_yaml(map_yaml)
    width, height = read_pgm_size(image)
    keep, speed = rasterize(sm, origin, res, width, height)
    paths = mask_paths(semantic_file)
    for key, array in (('keepout', keep), ('speed', speed)):
        write_pgm(paths[key + '_pgm'], array)
        with open(paths[key + '_yaml'], 'w') as f:
            yaml.safe_dump({'image': os.path.basename(paths[key + '_pgm']), 'mode': 'raw', 'resolution': res,
                            'origin': [origin[0], origin[1], 0.0], 'negate': 0,
                            'occupied_thresh': 0.65, 'free_thresh': 0.196}, f, sort_keys=False)
    with open(paths['sha1'], 'w') as f:
        f.write(zones_hash(sm) + '\n')
    return paths

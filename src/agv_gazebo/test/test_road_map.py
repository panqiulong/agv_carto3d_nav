import importlib.util
import math
import pathlib
import subprocess
import xml.etree.ElementTree as ET

import pytest
from lanelet2 import geometry, routing, traffic_rules

PKG = pathlib.Path(__file__).resolve().parent.parent
WORLDS = PKG / 'worlds'
spec = importlib.util.spec_from_file_location('road_map', PKG / 'scripts' / 'road_map.py')
rm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rm)

VALIDATE = '/opt/ros/humble/lib/x86_64-linux-gnu/lanelet2_validation/lanelet2_validate'
CW = {('n', 'eb'), ('e', 'sb'), ('s', 'wb'), ('w', 'nb')}   # 环路内侧半幅：顺时针


@pytest.fixture(scope='module')
def road():
    return rm.build()


def lanelets(road):
    return list(road.lmap.laneletLayer)


def by_name(road, n):
    return next(ll for ll in lanelets(road) if ll.attributes['name'] == n)


def graph(road):
    tr = traffic_rules.create(traffic_rules.Locations.Germany, traffic_rules.Participants.Vehicle)
    return routing.RoutingGraph(road.lmap, tr)


def attr(obj, k):
    return obj.attributes[k] if k in obj.attributes else None


def xy(p):
    return (p.x, p.y)


def circum_radius(a, b, c):
    cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    if abs(cross) < 1e-9:
        return math.inf
    return math.dist(a, b) * math.dist(b, c) * math.dist(a, c) / (2 * abs(cross))


def seg_dist(p, a, b):
    ax, ay = b[0] - a[0], b[1] - a[1]
    t = max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / (ax * ax + ay * ay)))
    return math.dist(p, (a[0] + t * ax, a[1] + t * ay))


def is_cw(name):
    parts = name.split('_')
    return '_cw_' in name if '_corner_' in name else (parts[1], parts[2]) in CW


def test_every_lanelet_named_and_unique(road):
    names = [ll.attributes['name'] for ll in lanelets(road)]
    assert all(names)
    assert len(names) == len(set(names))
    for n in ('gate_out', 'gate_in', 'ring_n_eb_in_0', 'ring_n_eb_out_0', 'x_s2w_left', 'x_e2s_left', 't_n2e_left'):
        assert n in names


def test_centerline_radius_at_least_3m(road):
    bad = []
    for ll in lanelets(road):
        c = [xy(p) for p in ll.centerline]
        for i in range(len(c) - 2):
            r = circum_radius(*c[i:i + 3])
            if r < 3.0:
                bad.append((ll.attributes['name'], i, round(r, 3)))
    assert bad == []


def test_ring_is_strongly_connected(road):
    g = graph(road)
    ring = [ll for ll in lanelets(road) if ll.attributes['name'].startswith('ring_')]
    for group in (True, False):
        lls = [ll for ll in ring if is_cw(ll.attributes['name']) == group]
        assert len(lls) >= 20
        missing = [(a.attributes['name'], b.attributes['name']) for a in lls for b in lls
                   if a is not b and g.shortestPath(a, b) is None]
        assert missing == []


def test_gate_out_reaches_every_ring_lanelet_and_back_to_gate_in(road):
    g = graph(road)
    out, inn = by_name(road, 'gate_out'), by_name(road, 'gate_in')
    for ll in lanelets(road):
        if ll.attributes['name'].startswith('ring_'):
            assert g.shortestPath(out, ll) is not None, ll.attributes['name']
            assert g.shortestPath(ll, inn) is not None, ll.attributes['name']


def test_lane_change_only_across_dashed(road):
    g = graph(road)
    changes = 0
    for ll in lanelets(road):
        for nb, shared in ((g.left(ll), ll.leftBound), (g.right(ll), ll.rightBound)):
            if nb is not None:
                changes += 1
                assert attr(shared, 'subtype') == 'dashed', ll.attributes['name']
            elif attr(shared, 'subtype') == 'dashed':
                pytest.fail(f"{ll.attributes['name']}: dashed bound but no lane change")
    assert changes > 0


def test_dashed_lanelets_at_least_10m(road):
    for ll in lanelets(road):
        if 'dashed' in (attr(ll.leftBound, 'subtype'), attr(ll.rightBound, 'subtype')):
            assert geometry.length2d(ll) >= 10.0 - 1e-6, ll.attributes['name']


def test_speed_limits(road):
    for ll in lanelets(road):
        n = ll.attributes['name']
        slow = '_corner' in n or n.startswith(('x_', 't_'))
        assert ll.attributes['speed_limit'] == ('1.08' if slow else '1.8'), n


def test_four_traffic_lights_with_stop_lines(road):
    g = graph(road)
    tls = [r for r in road.lmap.regulatoryElementLayer if r.attributes['subtype'] == 'traffic_light']
    assert len(tls) == 4
    assert sorted(r.attributes['signal_group'] for r in tls) == ['ew', 'ew', 'ns', 'ns']
    for r in tls:
        assert r.stopLine is not None and r.stopLine.attributes['type'] == 'stop_line'
        assert [ls.attributes['type'] for ls in r.trafficLights] == ['traffic_light']
    approach = [ll for ll in lanelets(road)
                if not ll.attributes['name'].startswith('x_')
                and any(f.attributes['name'].startswith('x_') for f in g.following(ll))]
    assert len(approach) == 6                    # 东西进口各 2 条，南北进口各 1 条
    for ll in lanelets(road):
        refs = ll.trafficLights()
        if ll in approach:
            assert len(refs) == 1, ll.attributes['name']
            sl = [xy(p) for p in refs[0].stopLine]
            assert seg_dist(xy(ll.centerline[-1]), sl[0], sl[-1]) < 1e-6   # 停止线在进口车道末端
        else:
            assert refs == [], ll.attributes['name']


def test_handoff_lanelets_reach_into_warehouse():
    road = rm.build((0.0, 0.0))
    out = [xy(p) for p in by_name(road, 'gate_out').centerline]
    assert out[0] == pytest.approx((1.75, -9.3))
    assert out[1][1] < out[0][1] and abs(out[1][0] - out[0][0]) < 1e-9     # 车头朝南
    inn = [xy(p) for p in by_name(road, 'gate_in').centerline]
    assert inn[-1] == pytest.approx((4.25, -9.3))
    assert inn[-2][1] < inn[-1][1]


def test_offset_shifts_everything(road):
    b = rm.build((2.0, -4.0))
    for p in road.lmap.pointLayer:
        q = b.lmap.pointLayer[p.id]
        assert (q.x, q.y, q.z) == pytest.approx((p.x - 2.0, p.y + 4.0, p.z))
        assert float(q.attributes['local_x']) == pytest.approx(q.x, abs=1e-4)
        assert float(q.attributes['local_y']) == pytest.approx(q.y, abs=1e-4)
    assert [k for k, _ in b.markings] == [k for k, _ in road.markings]
    for (_, pa), (_, pb) in zip(road.markings, b.markings):
        assert pb == pytest.approx([(x - 2.0, y + 4.0) for x, y in pa])
    assert b.surface == pytest.approx([(cx - 2.0, cy + 4.0, yaw, ln, w) for cx, cy, yaw, ln, w in road.surface])
    assert [g for g, *_ in b.lights] == [g for g, *_ in road.lights]
    assert [l[1:] for l in b.lights] == pytest.approx([(x - 2.0, y + 4.0, yaw) for _, x, y, yaw in road.lights])


def test_markings_match_map_lines(road):
    drawn = [ls for ls in road.lmap.lineStringLayer
             if attr(ls, 'type') in ('line_thin', 'stop_line')]
    assert len(road.markings) == len(drawn)
    ends = {(round(ls[0].x, 6), round(ls[0].y, 6), round(ls[-1].x, 6), round(ls[-1].y, 6)) for ls in drawn}
    for kind, pts in road.markings:
        assert kind in ('solid', 'dashed', 'stop_line')
        assert (round(pts[0][0], 6), round(pts[0][1], 6), round(pts[-1][0], 6), round(pts[-1][1], 6)) in ends


def test_osm_validates_and_roundtrips(road, tmp_path):
    a, b = tmp_path / 'a.osm', tmp_path / 'b.osm'
    rm.write_osm(road, str(a))
    rm.write_osm(road, str(b))
    assert a.read_bytes() == b.read_bytes()
    for node in ET.parse(a).getroot().iter('node'):
        tags = {t.get('k') for t in node.iter('tag')}
        assert {'local_x', 'local_y'} <= tags, node.get('id')
    out = subprocess.run([VALIDATE, '--map_file', str(a), '--lat', '35', '--lon', '139'],
                         capture_output=True, text=True).stdout
    assert 'issues found' in out
    assert [ln for ln in out.splitlines() if 'error' in ln.lower()] == []
    assert rm.PROJECTOR_INFO == {'projector_type': 'Local'}

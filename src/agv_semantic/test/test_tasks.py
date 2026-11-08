import os
import sys

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import tasks as T  # noqa: E402

PLACES = {'HOME', 'R1', 'R2', 'S1', 'A1', 'A2', 'M1'}


def cfg_dict(**over):
    d = {'home': 'HOME', 'dwell': 5.0, 'receiving': ['R1', 'R2'], 'shipping': ['S1'],
         'slots': {'A1': False, 'A2': True, 'M1': False}}
    d.update(over)
    return d


def write(tmp_path, data, name='m_tasks.yaml'):
    p = tmp_path / name
    p.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding='utf-8')
    return str(p)


def test_roundtrip_keeps_slot_order(tmp_path):
    path = write(tmp_path, cfg_dict(slots={'M1': True, 'A1': False, 'A2': True}))
    cfg = T.load(path, PLACES)
    assert list(cfg.slots) == ['M1', 'A1', 'A2']
    out = str(tmp_path / 'out' / 'm_tasks.yaml')
    T.save(cfg, out)
    assert T.load(out, PLACES) == cfg
    assert list(T.load(out, PLACES).slots) == ['M1', 'A1', 'A2']


def test_missing_file_is_an_error(tmp_path):
    with pytest.raises(T.TaskError, match='不存在'):
        T.load(str(tmp_path / 'nope_tasks.yaml'), PLACES)


@pytest.mark.parametrize('over, msg', [
    ({'home': 'X'}, '没有这些地点：X'),
    ({'receiving': ['R1', 'Q9']}, '没有这些地点：Q9'),
    ({'shipping': ['S1', 'S1']}, '重复：S1'),
    ({'receiving': ['R1'], 'slots': {'R1': False}}, '重复：R1'),
    ({'dwell': -1}, 'dwell'),
    ({'dwell': float('inf')}, 'dwell'),
    ({'dwell': float('nan')}, 'dwell'),
    ({'slots': {}}, '不能为空'),
    ({'shipping': []}, '不能为空'),
])
def test_validate_errors(tmp_path, over, msg):
    with pytest.raises(T.TaskError, match=msg):
        T.load(write(tmp_path, cfg_dict(**over)), PLACES)


def test_missing_slots_section_is_an_error(tmp_path):
    d = cfg_dict()
    del d['slots']
    with pytest.raises(T.TaskError, match='不能为空'):
        T.load(write(tmp_path, d), PLACES)


@pytest.mark.parametrize('value', ['true', 'full', 1])
def test_slot_state_must_be_bool(tmp_path, value):
    with pytest.raises(T.TaskError, match='A1'):
        T.load(write(tmp_path, cfg_dict(slots={'A1': value})), PLACES)


def test_save_is_atomic_on_error(tmp_path):
    path = write(tmp_path, cfg_dict())
    before = open(path, encoding='utf-8').read()
    bad = T.from_dict(cfg_dict(), PLACES)
    bad.dwell = -1
    with pytest.raises(T.TaskError):
        T.save(bad, path)                    # 校验失败：原文件不动，也不留临时文件
    assert open(path, encoding='utf-8').read() == before
    assert [f for f in os.listdir(tmp_path) if f.endswith('.tmp')] == []


def board(**over):
    return T.TaskBoard(T.from_dict(cfg_dict(**over), PLACES))


def test_submit_auto_selects_in_slot_order_and_skips_reserved():
    b = board(slots={'A1': False, 'A2': True, 'M1': False})
    t1 = b.submit('inbound')
    t2 = b.submit('inbound')
    assert (t1.source, t1.target) == ('R1', 'A1')           # 收货月台默认第一个
    assert t2.target == 'M1'                                # A1 已预留，跳过
    t3 = b.submit('outbound')
    assert (t3.source, t3.target) == ('A2', 'S1')
    assert b.reserved == {'A1', 'M1', 'A2'}
    assert [t.id for t in b.queue] == [1, 2, 3]


def test_submit_rejects_when_nothing_available():
    b = board(slots={'A1': True})
    with pytest.raises(T.TaskError, match='没有空的库位'):
        b.submit('inbound')
    b.submit('outbound')
    with pytest.raises(T.TaskError, match='没有有货的库位'):
        b.submit('outbound')


@pytest.mark.parametrize('args, msg', [
    (('inbound', 'R1', 'A2'), 'A2 有货，不能入库'),
    (('outbound', 'A1', 'S1'), 'A1 是空的，不能出库'),
    (('inbound', 'S1', 'A1'), '收货月台'),
    (('outbound', 'A2', 'R1'), '发货月台'),
    (('inbound', 'R1', 'HOME'), 'HOME 不是库位'),
    (('move', 'R1', 'A1'), 'inbound 或 outbound'),
])
def test_submit_validation(args, msg):
    b = board()
    with pytest.raises(T.TaskError, match=msg):
        b.submit(*args)
    assert b.queue == [] and b.reserved == set()


def test_submit_rejects_reserved_slot():
    b = board()
    b.submit('inbound', 'R1', 'A1')
    with pytest.raises(T.TaskError, match='A1 已被其他任务预留'):
        b.submit('inbound', 'R2', 'A1')


def test_submit_strips_spaces_and_lists_slots_on_unknown():
    b = board()
    t = b.submit('inbound', ' R2 ', ' A1 ')
    assert (t.source, t.target) == ('R2', 'A1')
    with pytest.raises(T.TaskError, match='a1 不是库位。库位有：A1、A2、M1'):
        b.submit('inbound', 'R1', 'a1')


def test_inbound_lifecycle_updates_stock_on_drop():
    b = board()
    b.submit('inbound', 'R1', 'A1')
    t = b.start_next()
    assert t.state == 'to_pickup' and b.queue == []
    b.set_state('loading')
    b.picked()
    assert t.state == 'to_dropoff' and b.cfg.slots['A1'] is False and 'A1' in b.reserved
    b.set_state('unloading')
    b.dropped()
    assert t.state == 'done' and b.cfg.slots['A1'] is True and b.reserved == set()
    assert b.active is None and b.recent[-1] is t and not b.paused


def test_outbound_lifecycle_empties_slot_on_pickup():
    b = board()
    b.submit('outbound', 'A2', 'S1')
    b.start_next()
    b.picked()
    assert b.cfg.slots['A2'] is False and b.reserved == set()
    b.dropped()
    assert b.recent[-1].state == 'done' and b.cfg.slots['A2'] is False


@pytest.mark.parametrize('type_, src, dst, picked, slot, full_after, on_board', [
    ('inbound', 'R1', 'A1', False, 'A1', False, False),
    ('outbound', 'A2', 'S1', False, 'A2', True, False),
    ('inbound', 'R1', 'A1', True, 'A1', False, True),
    ('outbound', 'A2', 'S1', True, 'A2', False, True),
])
@pytest.mark.parametrize('canceled', [False, True])
def test_end_active_releases_and_pauses(type_, src, dst, picked, slot, full_after, on_board, canceled):
    b = board()
    b.submit(type_, src, dst)
    b.start_next()
    if picked:
        b.picked()
    b.end_active('去 X 失败', canceled=canceled)
    t = b.recent[-1]
    assert t.state == ('canceled' if canceled else 'failed')
    assert b.paused and b.active is None and b.reserved == set()
    assert b.cfg.slots[slot] is full_after
    assert ('货在车上' in t.message) is on_board


def test_paused_board_does_not_start_until_resume():
    b = board()
    b.submit('inbound', 'R1', 'A1')
    b.submit('inbound', 'R1', 'M1')
    b.start_next()
    b.end_active('失败')
    assert b.start_next() is None
    assert b.resume() is True
    assert b.start_next().target == 'M1'
    assert b.resume() is False                      # 没有暂停时 resume 返回 False


def test_cancel_queued_releases_without_pausing():
    b = board()
    b.submit('inbound', 'R1', 'A1')
    t2 = b.submit('outbound', 'A2', 'S1')
    b.cancel_queued(t2.id)
    assert [t.id for t in b.queue] == [1] and b.reserved == {'A1'} and not b.paused
    assert b.recent[-1].state == 'canceled'
    with pytest.raises(T.TaskError, match='#9'):
        b.cancel_queued(9)


def test_set_stock():
    b = board()
    b.set_stock('A1', True)
    assert b.cfg.slots['A1'] is True
    b.submit('outbound', 'A1', 'S1')
    with pytest.raises(T.TaskError, match='预留'):
        b.set_stock('A1', False)
    with pytest.raises(T.TaskError, match='不是库位'):
        b.set_stock('R1', True)


def test_recent_keeps_last_50_and_status_shows_10():
    b = board(slots={'A1': False})
    for _ in range(60):
        b.submit('inbound', 'R1', 'A1')
        b.cancel_queued(b.queue[0].id)
    assert len(b.recent) == 50
    s = b.status()
    assert len(s['recent']) == 10 and s['recent'][-1]['id'] == 60
    assert set(s) == {'paused', 'active', 'queue', 'recent', 'slots', 'reserved'}
    assert s['slots'] == {'A1': False}


def test_describe_mentions_active_queue_and_stock():
    b = board()
    b.submit('inbound', 'R1', 'A1')
    b.submit('outbound', 'A2', 'S1')
    b.start_next()
    text = b.describe()
    assert '#1' in text and 'to_pickup' in text and '#2' in text
    assert 'A2 有货' in text and 'A1 空（已预留）' in text


def test_slot_names_duplicated_after_strip_are_rejected(tmp_path):
    with pytest.raises(T.TaskError, match='重复：A1'):
        T.load(write(tmp_path, cfg_dict(slots={'A1': False, 'A1 ': True})), PLACES)

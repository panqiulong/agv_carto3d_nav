"""出入库任务：任务配置与库存、任务队列、库位预留、状态流转（纯 Python，不依赖 ROS，便于单元测试）

文件格式和规则见 docs/superpowers/specs/2026-09-30-warehouse-tasks-design.md 第 4、6、7 节。
"""
import collections
import math
import os
import tempfile
from dataclasses import asdict, dataclass
from typing import Dict, List

import yaml


class TaskError(ValueError):
    """任务配置不合法，或请求不能执行"""


@dataclass
class TaskConfig:
    home: str
    dwell: float
    receiving: List[str]
    shipping: List[str]
    slots: Dict[str, bool]          # 库位 -> 是否有货；顺序 = 自动选择的优先顺序


def from_dict(data, place_names):
    """YAML 内容 -> TaskConfig；place_names 是语义文件里的地点名，用来检查名字是否存在"""
    if not isinstance(data, dict):
        raise TaskError('任务配置顶层必须是字典（home / dwell / receiving / shipping / slots）')
    try:
        slots = {}
        for name, full in (data.get('slots') or {}).items():
            if not isinstance(full, bool):
                raise TaskError('库位 %s 的状态必须是 true（有货）或 false（空），不能是 %r' % (name, full))
            name = str(name).strip()
            if name in slots:
                raise TaskError('月台和库位不能重名或重复：%s' % name)
            slots[name] = full
        cfg = TaskConfig(str(data['home']).strip(), float(data.get('dwell', 5.0)),
                         [str(n).strip() for n in data.get('receiving') or []],
                         [str(n).strip() for n in data.get('shipping') or []], slots)
    except (KeyError, TypeError, ValueError, AttributeError) as e:
        if isinstance(e, TaskError):
            raise
        raise TaskError('任务配置格式错误：%r' % e)
    validate(cfg, place_names)
    return cfg


def validate(cfg, place_names):
    if not cfg.receiving or not cfg.shipping or not cfg.slots:
        raise TaskError('receiving、shipping、slots 都不能为空')
    used = cfg.receiving + cfg.shipping + list(cfg.slots)
    missing = [n for n in dict.fromkeys([cfg.home] + used) if n not in place_names]
    if missing:
        raise TaskError('语义文件里没有这些地点：%s' % '、'.join(missing))
    dup = sorted({n for n in used if used.count(n) > 1})
    if dup:
        raise TaskError('月台和库位不能重名或重复：%s' % '、'.join(dup))
    if not math.isfinite(cfg.dwell) or cfg.dwell < 0:
        raise TaskError('dwell 必须是非负的有限数（秒）')


def to_dict(cfg):
    return {'home': cfg.home, 'dwell': float(cfg.dwell), 'receiving': list(cfg.receiving),
            'shipping': list(cfg.shipping), 'slots': dict(cfg.slots)}


def load(path, place_names):
    if not os.path.exists(path):
        raise TaskError('任务配置文件不存在：%s' % path)
    with open(path, encoding='utf-8') as f:
        try:
            data = yaml.safe_load(f)
        except yaml.YAMLError as e:
            raise TaskError('YAML 解析失败：%s' % e)
    return from_dict(data, place_names)


def save(cfg, path):
    """先检查基本约束；写临时文件再改名，中途出错不会损坏原文件"""
    if not math.isfinite(cfg.dwell) or cfg.dwell < 0:
        raise TaskError('dwell 必须是非负的有限数（秒）')
    folder = os.path.dirname(os.path.abspath(path))
    os.makedirs(folder, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=folder, suffix='.tmp')
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            yaml.safe_dump(to_dict(cfg), f, allow_unicode=True, sort_keys=False)
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.remove(tmp)
        raise


RECENT = 50                 # 内存里保留的已结束任务数
STATUS_RECENT = 10          # /task_status 和 list 里显示的已结束任务数
ACTIVE_STATES = ('to_pickup', 'loading', 'to_dropoff', 'unloading')
TYPE_NAMES = {'inbound': '入库', 'outbound': '出库'}


@dataclass
class Task:
    id: int
    type: str               # inbound / outbound
    source: str
    target: str
    state: str = 'queued'
    message: str = ''
    picked: bool = False    # 装货已完成（货在车上或已送达）


class TaskBoard:
    """任务队列 + 库存 + 预留。不加锁，调用方负责串行化"""

    def __init__(self, cfg):
        self.cfg = cfg
        self.queue = []
        self.active = None
        self.recent = collections.deque(maxlen=RECENT)
        self.reserved = set()
        self.paused = False
        self.next_id = 1

    @staticmethod
    def slot_of(task):
        return task.target if task.type == 'inbound' else task.source

    def _first_slot(self, full):
        return next((s for s, f in self.cfg.slots.items() if f == full and s not in self.reserved), None)

    def _check_slot(self, slot, full):
        if slot not in self.cfg.slots:
            raise TaskError('%s 不是库位。库位有：%s' % (slot, '、'.join(self.cfg.slots)))
        if slot in self.reserved:
            raise TaskError('库位 %s 已被其他任务预留' % slot)
        if self.cfg.slots[slot] != full:
            raise TaskError('库位 %s %s，不能%s' % (slot, '有货' if self.cfg.slots[slot] else '是空的',
                                                   '出库' if full else '入库'))

    def submit(self, type_, source='', target=''):
        source, target = source.strip(), target.strip()
        if type_ == 'inbound':
            source = source or self.cfg.receiving[0]
            if source not in self.cfg.receiving:
                raise TaskError('入库的起点必须是收货月台（%s），不能是 %s' % ('、'.join(self.cfg.receiving), source))
            if not target:
                target = self._first_slot(False)
                if target is None:
                    raise TaskError('没有空的库位可以入库')
            self._check_slot(target, full=False)
        elif type_ == 'outbound':
            if not source:
                source = self._first_slot(True)
                if source is None:
                    raise TaskError('没有有货的库位可以出库')
            self._check_slot(source, full=True)
            target = target or self.cfg.shipping[0]
            if target not in self.cfg.shipping:
                raise TaskError('出库的终点必须是发货月台（%s），不能是 %s' % ('、'.join(self.cfg.shipping), target))
        else:
            raise TaskError('任务类型必须是 inbound 或 outbound，不能是 %s' % type_)
        task = Task(self.next_id, type_, source, target)
        self.next_id += 1
        self.reserved.add(self.slot_of(task))
        self.queue.append(task)
        return task

    def cancel_queued(self, task_id):
        task = next((t for t in self.queue if t.id == task_id), None)
        if task is None:
            raise TaskError('排队的任务里没有 #%d' % task_id)
        self.queue.remove(task)
        self.reserved.discard(self.slot_of(task))
        task.state, task.message = 'canceled', '排队时取消'
        self.recent.append(task)
        return task

    def start_next(self):
        if self.paused or self.active is not None or not self.queue:
            return None
        self.active = self.queue.pop(0)
        self.active.state = 'to_pickup'
        return self.active

    def set_state(self, state):
        if state not in ACTIVE_STATES:
            raise TaskError('执行中的状态只能是 %s' % '、'.join(ACTIVE_STATES))
        self.active.state = state

    def picked(self):
        """装货完成：出库的来源库位变空、解除预留"""
        t = self.active
        t.picked = True
        if t.type == 'outbound':
            self.cfg.slots[t.source] = False
            self.reserved.discard(t.source)
        t.state = 'to_dropoff'

    def dropped(self):
        """卸货完成：入库的目标库位变为有货、解除预留"""
        t = self.active
        if t.type == 'inbound':
            self.cfg.slots[t.target] = True
            self.reserved.discard(t.target)
        t.state = 'done'
        self._finish()

    def end_active(self, message, canceled=False):
        """当前任务失败或被取消：解除预留，库存不回滚，队列暂停"""
        t = self.active
        self.reserved.discard(self.slot_of(t))
        if t.picked:
            message += '；货在车上，请人工处理'
        t.state, t.message = ('canceled' if canceled else 'failed'), message
        self.paused = True
        self._finish()

    def _finish(self):
        self.recent.append(self.active)
        self.active = None

    def resume(self):
        was = self.paused
        self.paused = False
        return was

    def set_stock(self, slot, full):
        if slot not in self.cfg.slots:
            raise TaskError('%s 不是库位。库位有：%s' % (slot, '、'.join(self.cfg.slots)))
        if slot in self.reserved:
            raise TaskError('库位 %s 已被任务预留，不能修改' % slot)
        self.cfg.slots[slot] = bool(full)

    def status(self):
        return {'paused': self.paused,
                'active': asdict(self.active) if self.active else None,
                'queue': [asdict(t) for t in self.queue],
                'recent': [asdict(t) for t in list(self.recent)[-STATUS_RECENT:]],
                'slots': dict(self.cfg.slots),
                'reserved': sorted(self.reserved)}

    def describe(self):
        def line(t):
            return '  #%d %s %s → %s  %s%s' % (t.id, TYPE_NAMES[t.type], t.source, t.target, t.state,
                                              ('  ' + t.message) if t.message else '')
        out = ['队列%s：' % ('（已暂停，resume 继续）' if self.paused else '')]
        out += [line(self.active)] if self.active else []
        out += [line(t) for t in self.queue] or ([] if self.active else ['  （空）'])
        out.append('最近结束：')
        out += [line(t) for t in list(self.recent)[-STATUS_RECENT:]] or ['  （无）']
        out.append('库位：')
        out += ['  %s %s%s' % (s, '有货' if f else '空', '（已预留）' if s in self.reserved else '')
                for s, f in self.cfg.slots.items()]
        return '\n'.join(out)

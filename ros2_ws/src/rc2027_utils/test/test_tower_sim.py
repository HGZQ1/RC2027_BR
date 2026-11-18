import math
import os
import random

import pytest
import yaml

from rc2027_utils.sim_check.block_scene import SIZES, regions_from_geometry
from rc2027_utils.sim_check.tower_sim import (find_spot, projection_margin, SimBlock, supply_trip,
                                              tower_kinds, tower_report, TowerWorld, trip_blocks)

HERE = os.path.dirname(os.path.abspath(__file__))
GEOMETRY = os.path.join(HERE, '..', '..', '..', '..', 'docs', 'rules', 'field_geometry.yaml')
SCENE = os.path.join(HERE, '..', '..', '..', '..', 'sim_ws', 'src', 'rc2027_sim_bridge', 'config',
                     'block_scenes', 'towers_plan2.yaml')


def world():
    with open(GEOMETRY, encoding='utf-8') as f:
        return TowerWorld(regions=regions_from_geometry(yaml.safe_load(f)))


def test_flip_turns_blue_sky_red():
    w = world()
    cx, cy, cz = w.regions['spot_01']
    z = cz + w.z_offset
    w.add(SimBlock('g1', 'ground', cx, cy, z, 0.0))
    w.add(SimBlock('g2', 'ground', cx, cy, z + 0.35, 0.0))
    w.add(SimBlock('sky', 'sky', cx, cy, z + 0.70, 0.0, 'blue'))
    # 放置位：车在塔西边，朝东，夹爪落在塔心
    robot = (cx - w.gripper_offset, cy, 0.0)
    err, block = w.flip(robot)
    assert err == '' and block.face == 'red'
    err, block = w.flip(robot)
    assert block.face == 'blue'


def test_random_field_is_only_l1_shared_blue():
    from rc2027_utils.sim_check.tower_sim import BLUE_KINDS, blue_next, random_field_blocks
    w = world()
    for seed in range(12):
        blocks, lines = random_field_blocks(random.Random(seed), w.regions, w.z_offset, 0)
        assert len(lines) == 2
        assert all(line.split(': ')[1] in BLUE_KINDS for line in lines)
        assert all(line.startswith(('spot_01', 'spot_02')) for line in lines)
        cx, cy, _ = w.regions['spot_07']
        assert not any(math.hypot(b.x - cx, b.y - cy) < 0.4 for b in blocks)
    cx, cy, cz = w.regions['spot_01']
    one = {'g': SimBlock('g', 'ground', cx, cy, cz + w.z_offset, 0.0)}
    second = blue_next(one, w.regions, 'spot_01', 1)
    assert second.kind == 'ground' and second.face == 'blue'
    two = dict(one)
    two[second.name] = second
    sky = blue_next(two, w.regions, 'spot_01', 2)
    assert sky.kind == 'sky' and sky.face == 'blue'
    done = dict(two)
    done[sky.name] = sky
    assert blue_next(done, w.regions, 'spot_01', 3) is None


def test_margin():
    b = SimBlock('a', 'ground', 0.0, 0.0, 0.0, 0.0)
    assert projection_margin(b, (0, 0)) == pytest.approx(0.075)
    off = SimBlock('a', 'ground', 0.1, 0, 0, 0)
    assert projection_margin(off, (0, 0)) == pytest.approx(-0.025)
    # 转 45°：对角线 0.495 / 2 = 0.247 → 余量只剩 0.3 cm
    assert projection_margin(SimBlock('a', 'ground', 0, 0, 0, math.pi / 4), (0, 0)) == \
        pytest.approx(0.25 - 0.175 * math.sqrt(2))
    assert projection_margin(SimBlock('a', 'sky', 0, 0, 0, 0), (0, 0)) == pytest.approx(0.15)


def test_pick_and_place_tower():
    w = world()
    rng = random.Random(0)
    # 交接区一摞 2 地；车停在东边 gripper_offset（0.575）、朝西
    w.add(SimBlock('g1', 'ground', -3.7, -1.05, 0.65, 0.0))
    w.add(SimBlock('g2', 'ground', -3.7, -1.05, 1.0, 0.0))
    robot = (-3.125, -1.05, math.pi)
    assert w.pick(robot, 2) == ''
    assert w.carried == ['g1', 'g2']
    assert w.region_empty('transfer_red')
    # spot_07 (-2.7, -2.7) 北边 0.575 m、朝南放
    near = (-2.7, -2.125, -math.pi / 2)
    assert w.place(near, 2, rng).startswith('要放到第 2 层')   # 空的地方放第 2 层：会砸
    assert w.place(near, 1, rng) == ''
    assert [round(w.blocks[n].z_bottom, 3) for n in ('g1', 'g2')] == [0.65, 1.0]
    assert find_spot(w, w.blocks['g1'].x, w.blocks['g1'].y) == 'spot_07'
    # 封顶
    w.add(SimBlock('s', 'sky', -3.25, -1.55, 0.65, 0.0))
    assert w.pick((-2.675, -1.55, math.pi), 1) == ''
    assert w.place(near, 2, rng).startswith('要放到第 2 层')
    assert w.place(near, 3, rng) == ''
    report = '\n'.join(tower_report(w, ['spot_07']))
    assert '完整的塔 ✓' in report and report.count('余量') == 3


def test_lone_sky_does_not_land_on_one_ground():
    """手里只有一块天，塔上只有一层地：放上去不得分，仿真里直接拒绝。"""
    w = world()
    cx, cy, cz = w.regions['spot_05']
    z = cz + w.z_offset
    w.add(SimBlock('g', 'ground', cx, cy, z, 0.0))
    w.add(SimBlock('s', 'sky', -3.66, -1.16, 0.65, 0.0))
    w.add(SimBlock('under', 'ground', -3.66, -1.16, 0.30, 0.0))
    robot = (-3.66 + w.gripper_offset, -1.16, math.pi)
    assert w.pick(robot, 1) == ''
    assert w.carried == ['s']
    place = (cx, cy - w.gripper_offset, math.pi / 2)
    assert '两层地' in w.place(place, 2, random.Random(0))
    assert w.carried == ['s']


def test_pick_errors():
    w = world()
    assert '只有 0 块' in w.pick((0, 0, 0), 1)
    w.add(SimBlock('g', 'ground', 0.45, 0, 0.95, 0))
    assert w.pick((0, 0, 0), 1) == ''
    assert '已经有' in w.pick((0, 0, 0), 1)


def test_scene_trips_load():
    with open(SCENE, encoding='utf-8') as f:
        scene = yaml.safe_load(f)
    w = world()
    blocks = []
    for trip in scene['trips']:
        blocks += trip_blocks(trip, w.regions, scene['z_offset'], len(blocks))
    kinds = sorted(b.kind for b in blocks)
    assert kinds == ['ground'] * 8 + ['sky'] * 4       # 四趟，四座塔
    assert len({b.name for b in blocks}) == 12


def test_cap_on_two_grounds_keeps_the_ground():
    """两层地上放下地+天：只盖天空，地还在手里。"""
    w = world()
    cx, cy, cz = w.regions['spot_02']
    z = cz + w.z_offset
    g = SIZES['ground']
    w.add(SimBlock('g1', 'ground', cx, cy, z, 0.0))
    w.add(SimBlock('g2', 'ground', cx, cy, z + g, 0.0))
    w.add(SimBlock('hg', 'ground', 0, 0, 0, 0.0))
    w.add(SimBlock('hs', 'sky', 0, 0, 0, 0.0, 'red'))
    w.carried = ['hg', 'hs']
    err = w.place((cx - w.gripper_offset, cy, 0.0), 3, random.Random(0), 0, 0)
    assert err == ''
    assert w.carried == ['hg']
    assert tower_kinds(w, 'spot_02') == ['ground', 'ground', 'sky']


def test_supply_trip_alternates():
    assert [b for stack in supply_trip(0) for b in stack['blocks']] == [
        'ground', 'ground', 'ground']
    assert [b for stack in supply_trip(1) for b in stack['blocks']] == ['ground', 'sky', 'sky']

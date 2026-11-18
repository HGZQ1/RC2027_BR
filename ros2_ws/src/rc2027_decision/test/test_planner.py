"""规则规划器（world/planner.py）。不起仿真。"""

import math
import os

from rc2027_decision.world.field_model import SKY_NONE, SKY_OTHERS, SKY_OURS
from rc2027_decision.flow.steps import Action, Call, Detect, Nav, Relocalize, Wait
from rc2027_decision.world.planner import (CLIMB, FLIP, Choice, Goods, LOOK, PLACE_GROUND,
                                           PLACE_SKY, STOP,
                                           Tower, WAIT, after_changed_look, apply_choice, bind_points,
                                           carry_choice, choose,
                                           close_look, compile_choice, load_approaches,
                                           mark_unseen, on_arrival, pickable_goods,
                                           trusted_transfer_blocks)
from rc2027_decision.flow.pick_pose import Block

HERE = os.path.dirname(os.path.abspath(__file__))
APPROACHES = os.path.join(HERE, '..', 'config', 'spot_approaches.yaml')


def towers(**over):
    """L1 共享阶段的基准场况。测上楼的用例显式打开 L2 塔位。"""
    base = {
        'spot_07': Tower(seen=True),
        'spot_01': Tower(),
        'spot_03': Tower(seen=True),
        'spot_08': Tower(ground=2, sky_owner=SKY_OURS, seen=True),
        'spot_05': Tower(ground=2, sky_owner=SKY_OURS, seen=True),
        'spot_02': Tower(),
    }
    base.update(over)
    return base


def test_empty_south_shared_does_not_jump_our_tower():
    """南共享还是 0 层：不抢，先做自己的塔。"""
    field = towers(spot_01=Tower(seen=True))
    choice = choose(field, Goods(ground=3, sky=2))
    assert choice.action == PLACE_GROUND and choice.spot == 'spot_07'


def test_near_sky_prevents_reaching_across_it_for_a_cap():
    blocks = [Block('ground', -3.68, -1.13, 0.1, 1), Block('sky', -3.68, -1.13, 0.1, 2),
              Block('sky', -3.28, -1.53, 0.26, 1)]
    assert pickable_goods(blocks) == Goods(sky=1)
    field = towers(spot_08=Tower(ground=2, seen=True, blocked=True),
                   spot_03=Tower(ground=1, seen=True))
    assert choose(field, pickable_goods(blocks)).action == WAIT
    field['spot_08'] = Tower(ground=2, seen=True)
    assert choose(field, pickable_goods(blocks)).spot == 'spot_08'
    assert pickable_goods(blocks[:2]) == Goods(sky=1, cap_stack=1)


def test_opening_cap_is_picked_before_looking_at_south_shared():
    """开局只有地天一摞。南共享还没看：先拿上，去放的路上再看，不要空手先跑一趟。"""
    approaches = load_approaches(APPROACHES)
    choice = choose(towers(), Goods(sky=1, cap_stack=1), approaches)
    assert choice.action == PLACE_SKY and choice.spot == 'spot_01' and choice.count == 2
    blocks = [Block('ground', -3.70, -1.15, 0, 1), Block('sky', -3.70, -1.15, 0, 2)]
    steps = compile_choice(choice, approaches, blocks)
    pick = next(i for i, s in enumerate(steps) if isinstance(s, Action) and s.name == 'PICK')
    face = next(i for i, s in enumerate(steps)
                if isinstance(s, Nav) and s.point == 'transfer_see' and s.yaw is not None)
    assert pick < face
    assert any(isinstance(s, Detect) and s.slot == 'check' for s in steps[face:])
    assert all(getattr(s, 'point', None) != 'spot_07_glance' for s in steps)


def test_shared_is_looked_at_before_the_first_touch():
    field = towers(spot_07=Tower(ground=2, sky_owner=SKY_OURS, seen=True))
    choice = choose(field, Goods(ground=1, sky=1))
    assert choice.action == LOOK and choice.spot == 'spot_01'
    assert '共享区' in choice.why


def test_shared_look_turns_at_the_observation_point():
    """看南北共享：开到观察点转向塔心，不再沿西侧开到塔旁的观察位。"""
    approaches = load_approaches(APPROACHES)
    field = towers(spot_07=Tower(ground=2, sky_owner=SKY_OURS, seen=True))
    choice = choose(field, Goods(ground=1, sky=1), approaches)
    steps = compile_choice(choice, approaches, [])
    navs = [s for s in steps if isinstance(s, Nav)]
    assert [s.point for s in navs] == ['transfer_see']
    assert navs[0].yaw is not None and navs[0].yaw < 0
    assert any(isinstance(s, Wait) for s in steps)


def test_opponent_sky_jumps_the_queue():
    """南边自己的塔和北边自己的塔都不吃地了，北共享的对方天空才插队。"""
    field = towers(
        spot_07=Tower(ground=2, sky_owner=SKY_OURS, seen=True),
        spot_03=Tower(ground=2, sky_owner=SKY_OURS, seen=True),
        spot_02=Tower(ground=2, sky_owner=SKY_OTHERS, seen=True))
    choice = choose(field, Goods(ground=3, sky=2))
    assert choice.action == FLIP and choice.spot == 'spot_02'
    assert '观察区' in choice.why


def test_north_own_ground_is_picked_on_the_way_to_the_flip():
    """南边自己的塔已经放完。去北共享翻天之前，先把北边自己的地放上，不要空手去、回来再拿。"""
    field = towers(
        spot_07=Tower(ground=2, sky_owner=SKY_OURS, seen=True),
        spot_02=Tower(ground=2, sky_owner=SKY_OTHERS, seen=True))
    choice = choose(field, Goods(ground=1))
    assert choice.action == PLACE_GROUND and choice.spot == 'spot_03'


def test_south_own_ground_is_not_taken_on_a_detour_through_south_shared():
    """手里这块地南边自己的塔还要。先放下，不要先去南共享晃。"""
    field = towers(spot_01=Tower(ground=1, sky_owner=SKY_OTHERS, seen=True))
    choice = choose(field, Goods(ground=1))
    assert choice.action == PLACE_GROUND and choice.spot == 'spot_07'


def test_held_ground_prefers_north_own_over_north_shared():
    approaches = load_approaches(APPROACHES)
    field = towers(
        spot_07=Tower(ground=2, sky_owner=SKY_OURS, seen=True),
        spot_01=Tower(ground=2, seen=True),
        spot_02=Tower(ground=1, seen=True),
        spot_03=Tower(seen=True))
    choice = carry_choice(field, 'ground', approaches, 1)
    assert choice is not None and choice.action == PLACE_GROUND and choice.spot == 'spot_03'


def test_red_ground_is_placed_before_going_to_north_shared():
    """南边自己的塔还要地：先拿红地放下，再去北共享。不要拿完先跑去看一眼再回来放。"""
    field = towers(spot_02=Tower(ground=2, sky_owner=SKY_OTHERS, seen=True))
    choice = choose(field, Goods(ground=1))
    assert choice.action == PLACE_GROUND and choice.spot == 'spot_07'
    field['spot_07'] = Tower(ground=2, sky_owner=SKY_OURS, seen=True)
    field['spot_01'] = Tower(ground=2, sky_owner=SKY_OURS, seen=True)
    nxt = choose(field, Goods())
    assert nxt.action == FLIP and nxt.spot == 'spot_02'


def test_ground_goes_north_before_the_flip_even_if_south_shared_wants_it():
    """南共享只有一层蓝地、北共享要翻：手里的地先放到北边自己的塔，再翻。不要空手去翻。"""
    field = towers(
        spot_07=Tower(ground=2, seen=True),
        spot_01=Tower(ground=1, seen=True),
        spot_02=Tower(ground=2, sky_owner=SKY_OTHERS, seen=True))
    choice = choose(field, Goods(ground=1))
    assert choice.action == PLACE_GROUND and choice.spot == 'spot_03'
    field['spot_03'] = Tower(ground=1, seen=True)
    nxt = choose(field, Goods())
    assert nxt.action == FLIP and nxt.spot == 'spot_02'


def test_already_on_the_north_side_flips_before_leaving():
    """人已经在北边，北共享能翻：先翻再回去拿。不要先回交接区，放完下一趟的天地再回来翻。"""
    field = towers(
        spot_07=Tower(ground=2, seen=True),
        spot_01=Tower(ground=2, sky_owner=SKY_OURS, seen=True),
        spot_02=Tower(ground=2, sky_owner=SKY_OTHERS, seen=True),
        spot_03=Tower(ground=1, seen=True))
    goods = Goods(sky=1, cap_stack=1)
    away = choose(field, goods)
    assert away.action == PLACE_SKY and away.spot == 'spot_03'
    here = choose(field, goods, at_north=True)
    assert here.action == FLIP and here.spot == 'spot_02'


def test_bring_the_spare_ground_before_flipping_north_shared():
    """北共享能翻、南边用不上这块地：先放到 spot_03，再翻，不要空手去翻完再跑一趟。"""
    field = towers(
        spot_07=Tower(ground=2, seen=True),
        spot_01=Tower(ground=2, sky_owner=SKY_OURS, seen=True),
        spot_02=Tower(ground=2, sky_owner=SKY_OTHERS, seen=True))
    choice = choose(field, Goods(ground=1))
    assert choice.action == PLACE_GROUND and choice.spot == 'spot_03'
    field['spot_03'] = Tower(ground=1, seen=True)
    nxt = choose(field, Goods())
    assert nxt.action == FLIP and nxt.spot == 'spot_02'


def test_north_shared_flip_returns_along_the_west_side():
    approaches = load_approaches(APPROACHES)
    field = towers(spot_02=Tower(ground=2, sky_owner=SKY_OTHERS, seen=True))
    choice = choose(field, Goods(), approaches)
    steps = compile_choice(choice, approaches, [])
    names = [s.point for s in steps if isinstance(s, Nav)]
    flip_at = next(i for i, s in enumerate(steps) if isinstance(s, Action) and s.name == 'FLIP_SKY')
    after = [s for s in steps[flip_at + 1:] if isinstance(s, Nav)]
    assert [s.point for s in after] == ['spot_02_west', 'transfer_see']
    assert math.isclose(after[0].yaw, 2.415)
    assert after[1].yaw is None
    assert names[:4] == ['transfer_see', 'l2_stair_south', 'l2_stair_west', 'spot_02_west']
    face = next(s for s in steps if isinstance(s, Nav) and s.point == 'transfer_see')
    assert face.yaw is not None and face.yaw > 0
    outbound = next(s for s in steps if isinstance(s, Nav) and s.point == 'spot_02_west')
    assert outbound.yaw is None


def test_return_from_the_north_goes_straight_to_the_see_point():
    """人在北共享：回交接区先收到西侧再去观察点，不绕走廊南端。"""
    approaches = load_approaches(APPROACHES)
    field = towers(
        spot_07=Tower(ground=2, sky_owner=SKY_OURS, seen=True),
        spot_01=Tower(ground=1, seen=True))
    choice = choose(field, Goods(ground=1), approaches)
    blocks = [Block('ground', -3.50, -1.30, 0, 1)]
    steps = compile_choice(choice, approaches, blocks, from_north=True)
    navs = [s for s in steps if isinstance(s, Nav)]
    assert navs[0].point == 'spot_02_west'
    assert math.isclose(navs[0].yaw, 2.415)
    assert navs[1].point == 'transfer_see' and navs[1].yaw is None
    assert sum(s.point == 'spot_02_west' for s in navs) == 1
    assert all(s.point != 'l2_stair_south' for s in navs)


def test_opponent_sky_unseen_is_a_look_not_a_flip():
    field = towers(spot_02=Tower(ground=2, sky_owner=SKY_OTHERS, clipped=True, seen=True))
    choice = choose(field)
    assert choice.action == LOOK and choice.spot == 'spot_02'
    assert '视场' in choice.why


def test_clipped_own_tower_is_not_built_on():
    field = towers(spot_07=Tower(ground=1, seen=True, clipped=True))
    choice = choose(field, Goods(ground=1, sky=1))
    assert choice.action == LOOK and choice.spot == 'spot_07'


def test_missing_sky_does_not_block_a_tower_that_needs_ground():
    """南共享只差天空，但第 1 趟只有地块：先往自己的塔上放地，不要干等。"""
    field = towers(spot_01=Tower(ground=2, seen=True))
    choice = choose(field, Goods(ground=3, sky=0))
    assert choice.action == PLACE_GROUND and choice.spot == 'spot_07'


def test_lone_sky_caps_our_south_tower_before_the_cap_goes_north():
    """单块天先盖南边自己的塔。地+天放到北边自己，排在这一步后面。"""
    field = towers(
        spot_07=Tower(ground=2, seen=True),
        spot_01=Tower(ground=2, seen=True),
        spot_03=Tower(ground=1, seen=True))
    choice = choose(field, Goods(sky=2, cap_stack=1))
    assert choice.action == PLACE_SKY and choice.spot == 'spot_07' and choice.count == 1
    field['spot_07'] = Tower(ground=2, sky_owner=SKY_OURS, seen=True)
    nxt = choose(field, Goods(sky=1, cap_stack=1))
    assert nxt.action == PLACE_SKY and nxt.spot == 'spot_03' and nxt.count == 2


def test_a_blocked_shared_tower_is_left_for_later():
    """对方车挡在南共享上：这趟不去看、不去放，先把自己的空塔垫上。"""
    field = towers(spot_01=Tower(blocked=True))
    choice = choose(field, Goods(ground=2, ground_stack=1))
    assert choice.action == PLACE_GROUND and choice.spot == 'spot_07' and choice.count == 2


def test_no_sky_cancels_waiting_on_a_blocked_north_shared():
    """第三趟只有地。北共享两层、这一眼还被挡着：否掉放天，直接把地放到 L2。"""
    field = south_done(
        spot_08=Tower(seen=True), spot_05=Tower(seen=True),
        spot_02=Tower(ground=2, seen=True, blocked=True),
        spot_03=Tower(ground=2, sky_owner=SKY_OURS, seen=True))
    choice = choose(field, Goods(ground=3, ground_stack=1))
    assert choice.action == PLACE_GROUND and choice.spot == 'spot_08' and choice.count == 2
    assert choice.action != LOOK


def test_grounds_go_to_l2_when_the_trip_has_no_sky_for_north_shared():
    """第三趟只有地，北共享只差天空：直接放到 L2，先拿再上台，不要空手先上楼。"""
    field = south_done(
        spot_08=Tower(seen=True), spot_05=Tower(seen=True),
        spot_02=Tower(ground=2, seen=True),
        spot_03=Tower(ground=2, sky_owner=SKY_OURS, seen=True))
    choice = choose(field, Goods(ground=3, ground_stack=1))
    assert choice.action == PLACE_GROUND and choice.spot == 'spot_08' and choice.count == 2
    up = choose(field, Goods(ground=3, ground_stack=1), climbed=True)
    assert up.action == PLACE_GROUND and up.spot == 'spot_08' and up.count == 2


def test_cap_goes_to_north_own_before_north_shared():
    """地+天不先送给北共享。北边自己的塔正好一层，就放那儿。"""
    field = south_done(
        spot_02=Tower(ground=1, seen=True),
        spot_03=Tower(ground=1, seen=True))
    choice = choose(field, Goods(sky=1, cap_stack=1))
    assert choice.action == PLACE_SKY and choice.spot == 'spot_03' and choice.count == 2


def test_holding_on_the_north_side_does_not_cut_across_to_the_south():
    """人在北共享跟前改去放北边自己的塔：先收到西侧，不要直开南边的 spot_01_west。"""
    approaches = load_approaches(APPROACHES)
    field = south_done(spot_03=Tower(ground=1, seen=True))
    choice = choose(field, Goods(sky=1, cap_stack=1), approaches)
    steps = compile_choice(choice, approaches, [], holding=True, from_north=True)
    names = [s.point for s in steps if isinstance(s, Nav)]
    assert names[0] == 'spot_02_west'
    assert 'spot_01_west' not in names


def test_south_shared_is_seen_by_turning_at_the_observation_point():
    """去南共享之前，在观察点转向塔心看层数，不再开到自己南塔的放置位。"""
    approaches = load_approaches(APPROACHES)
    field = towers(spot_01=Tower(ground=1, seen=True),
                   spot_07=Tower(ground=2, sky_owner=SKY_OURS, seen=True))
    choice = choose(field, Goods(ground=1), approaches)
    blocks = [Block('ground', -3.50, -1.30, 0, 1)]
    steps = compile_choice(choice, approaches, blocks)
    navs = [s for s in steps if isinstance(s, Nav)]
    face = next(s for s in navs if s.point == 'transfer_see' and s.yaw is not None)
    assert face.yaw < 0
    names = [s.point for s in navs]
    assert names.index('transfer_see') < names.index('spot_01_west')
    assert 'spot_07_glance' not in names
    assert 'spot_01_look' not in names


def test_goods_choose_the_place_not_the_missing_sky():
    """北共享还差天空，交接区只有一块地：把地放到北边自己的塔，不要先定了去拿天空。"""
    field = south_done(spot_02=Tower(ground=2, seen=True))
    choice = choose(field, Goods(ground=1))
    assert choice.action == PLACE_GROUND and choice.spot == 'spot_03'
    sky = choose(field, Goods(sky=1))
    assert sky.action == PLACE_SKY and sky.spot == 'spot_02'


def test_wait_names_the_tower_that_still_needs_a_sky():
    """spot_07 的天空已经放上。还在等的是北共享，不要写成第 1 趟没有天空。"""
    field = south_done(
        spot_02=Tower(ground=2, seen=True),
        spot_03=Tower(ground=2, sky_owner=SKY_OURS, seen=True))
    choice = choose(field, Goods())
    assert choice.action == WAIT
    assert 'spot_02 还差天空' in choice.why
    assert 'spot_07' not in choice.why


def test_wait_only_when_nothing_else_can_move():
    """L1 都吃不下地、交接区也没有天：把地放到 L2，不空手先上楼。真没货才等。"""
    field = towers(
        spot_08=Tower(seen=True), spot_05=Tower(seen=True),
        spot_07=Tower(ground=2, seen=True),
        spot_01=Tower(ground=2, seen=True),
        spot_03=Tower(ground=2, seen=True))
    choice = choose(field, Goods(ground=1, sky=0))
    assert choice.action == PLACE_GROUND and choice.spot == 'spot_08'
    empty = choose(field, Goods())
    assert empty.action == WAIT


def south_done(**over):
    field = towers(
        spot_07=Tower(ground=2, sky_owner=SKY_OURS, seen=True),
        spot_01=Tower(ground=2, sky_owner=SKY_OURS, seen=True),
        spot_02=Tower(seen=True))
    field.update(over)
    return field


def test_climb_drives_to_the_stair_then_jumps():
    """没货就在观察位等。有地要上 L2 时，先拿，回到观察点朝东，再开到楼梯口换位。"""
    approaches = load_approaches(APPROACHES)
    field = south_done(
        spot_08=Tower(seen=True), spot_05=Tower(seen=True),
        spot_02=Tower(ground=2, sky_owner=SKY_OURS, seen=True),
        spot_03=Tower(ground=2, sky_owner=SKY_OURS, seen=True))
    waiting = choose(field, Goods(), approaches)
    assert waiting.action == WAIT
    wait_steps = compile_choice(waiting, approaches, [])
    assert [s.point for s in wait_steps if isinstance(s, Nav)] == ['transfer_see']
    choice = choose(field, Goods(ground=1), approaches)
    assert choice.action == PLACE_GROUND and choice.spot == 'spot_08'
    blocks = [Block('ground', -3.50, -1.30, 0, 1)]
    steps = compile_choice(choice, approaches, blocks)
    names = [s.point if isinstance(s, Nav) else getattr(s, 'name', None) for s in steps]
    assert 'l2_stair_south' not in names
    assert names.index('transfer_see') < names.index('@pick_pre') < names.index('PICK')
    assert 'transfer_gate' not in names
    assert names[names.index('PICK') + 1] == 'transfer_see'
    back = [s for s in steps if isinstance(s, Nav) and s.point == 'transfer_see'][-1]
    assert math.isclose(back.yaw, 0.0)
    assert names.index('PICK') < names.index('l2_stair_foot')
    assert names.index('l2_stair_foot') < names.index('CLIMB_L2') < names.index('spot_08_place')


def test_leaving_the_platform_goes_down_before_the_transfer():
    """人在台沿上要回交接区拿货：先换回楼梯口，直线开出台面会掉下去。手里已经有块就留在台上放。"""
    approaches = load_approaches(APPROACHES)
    field = south_done(
        spot_08=Tower(seen=True), spot_05=Tower(seen=True),
        spot_02=Tower(ground=2, sky_owner=SKY_OURS, seen=True),
        spot_03=Tower(ground=2, sky_owner=SKY_OURS, seen=True))
    choice = choose(field, Goods(ground=1), approaches, climbed=True)
    assert choice.action == PLACE_GROUND and choice.spot == 'spot_08'
    blocks = [Block('ground', -3.50, -1.30, 0, 1)]
    steps = compile_choice(choice, approaches, blocks, on_platform=True)
    from rc2027_decision.flow import FlowEngine, FlowRegistry
    from rc2027_decision.world.planner import planner_subflows
    registry = FlowRegistry.from_dict({'main': 'parent', 'flows': {'parent': 'WAIT 1'}})
    registry.flows.update(planner_subflows())
    engine = FlowEngine(registry)
    assert steps[0] == Call('descend_l2')
    ref = engine.run_steps('parent', steps)
    executed = []
    while ref is not None:
        executed.append(ref)
        ref = engine.step_done()
    assert 'descend_l2#0' in executed[0].path
    assert 'descend_l2#1' in executed[1].path
    assert 'descend_l2' not in executed[2].path
    names = [getattr(r.step, 'name', None) or getattr(r.step, 'point', None) for r in executed]
    assert names[0] == 'l2_stair_land' and names[1] == 'CLIMB_DOWN' and names[2] == 'transfer_see'
    assert 'l2_stair_south' not in names
    assert names.index('transfer_see') < names.index('PICK') < names.index('l2_stair_foot')
    assert names.index('CLIMB_L2') < names.index('spot_08_place')
    assert names.index('PICK') < names.index('CLIMB_L2')
    held = compile_choice(choice, approaches, blocks, holding=True, on_platform=True)
    assert not any(isinstance(s, Action) and s.name in ('CLIMB_DOWN', 'CLIMB_L2') for s in held)
    below = compile_choice(choice, approaches, blocks, holding=True, on_platform=False)
    below_names = [getattr(s, 'name', None) or getattr(s, 'point', None) for s in below]
    assert below_names[0] == 'l2_stair_foot' and 'CLIMB_L2' in below_names
    assert 'CLIMB_DOWN' not in below_names
    assert 'l2_stair_south' not in below_names


def test_sky_on_our_south_tower_before_flipping_the_shared_one():
    """从北边回来翻南共享：自己的南塔只差天空、交接区有单块天空，先盖上再翻。"""
    field = towers(
        spot_07=Tower(ground=2, seen=True),
        spot_01=Tower(ground=2, sky_owner=SKY_OTHERS, seen=True),
        spot_02=Tower(ground=2, sky_owner=SKY_OURS, seen=True))
    choice = choose(field, Goods(sky=1))
    assert choice.action == PLACE_SKY and choice.spot == 'spot_07'
    field['spot_07'] = Tower(ground=2, sky_owner=SKY_OURS, seen=True)
    nxt = choose(field, Goods())
    assert nxt.action == FLIP and nxt.spot == 'spot_01'
    bare = choose(towers(
        spot_07=Tower(ground=2, seen=True),
        spot_01=Tower(ground=2, sky_owner=SKY_OTHERS, seen=True)), Goods())
    assert bare.action == FLIP and bare.spot == 'spot_01'


def test_cap_goes_on_north_own_while_north_shared_is_still_open():
    """北共享还没封顶。交接区只剩地+天，北边自己的塔正好一层：先放上，不要干等然后报错。"""
    field = south_done(spot_03=Tower(ground=1, seen=True))
    choice = choose(field, Goods(sky=1, cap_stack=1))
    assert choice.action == PLACE_SKY and choice.spot == 'spot_03' and choice.count == 2


def test_previous_cap_is_parked_on_north_own_before_the_climb():
    """上一趟只剩地+天，北边自己的塔正好一层：先放那儿，等下一趟有地再上楼。"""
    field = south_done(
        spot_08=Tower(seen=True), spot_05=Tower(seen=True),
        spot_02=Tower(ground=2, sky_owner=SKY_OURS, seen=True),
        spot_03=Tower(ground=1, seen=True))
    choice = choose(field, Goods(sky=1, cap_stack=1))
    assert choice.action == PLACE_SKY and choice.spot == 'spot_03' and choice.count == 2
    field['spot_03'] = Tower(ground=2, sky_owner=SKY_OURS, seen=True)
    waiting = choose(field, Goods(sky=1, cap_stack=1))
    assert waiting.action == WAIT
    up = choose(field, Goods(sky=1, cap_stack=1), climbed=True)
    assert up.action == WAIT
    done = dict(field)
    done['spot_08'] = Tower(ground=2, sky_owner=SKY_OURS, seen=True)
    done['spot_05'] = Tower(ground=2, sky_owner=SKY_OURS, seen=True)
    assert choose(done, Goods(), climbed=True).action == STOP


def test_shared_flip_and_placement_wait_for_both_l2_tops():
    done = Tower(ground=2, sky_owner=SKY_OURS, seen=True)
    field = south_done(spot_08=done, spot_05=Tower(ground=2, seen=True),
                       spot_02=Tower(ground=2, sky_owner=SKY_OTHERS, seen=True),
                       spot_01=Tower(ground=2, seen=True), spot_03=done)
    for climbed in (False, True):
        assert choose(field, Goods(), climbed=climbed, at_north=True).action == WAIT
        assert choose(field, Goods(sky=1), climbed=climbed).spot == 'spot_05'
    assert carry_choice(field, 'sky').spot == 'spot_05'
    assert carry_choice(field, 'sky', avoid='spot_05') is None
    field['spot_05'] = done
    assert choose(field, Goods()).action == FLIP
    field['spot_02'] = done
    assert choose(field, Goods(sky=1)).spot == 'spot_01'


def test_going_north_to_our_tower_does_not_stop_halfway():
    """从南边去 spot_03 已经朝北。楼梯口和预备位朝向一样，不再在北行中间停。"""
    approaches = load_approaches(APPROACHES)
    field = towers(
        spot_07=Tower(ground=2, seen=True),
        spot_01=Tower(ground=2, sky_owner=SKY_OURS, seen=True),
        spot_02=Tower(ground=2, sky_owner=SKY_OTHERS, seen=True))
    choice = choose(field, Goods(ground=1), approaches)
    assert choice.action == PLACE_GROUND and choice.spot == 'spot_03'
    blocks = [Block('ground', -3.50, -1.30, 0, 1)]
    steps = compile_choice(choice, approaches, blocks)
    south = next(i for i, s in enumerate(steps)
                 if isinstance(s, Nav) and s.point == 'l2_stair_south')
    place = next(i for i, s in enumerate(steps)
                 if isinstance(s, Action) and s.name == 'PLACE')
    between = steps[south + 1:place]
    assert not any(isinstance(s, Nav) and s.point == 'l2_stair_west' for s in between)
    assert not any(isinstance(s, Relocalize) for s in between)
    assert next(s.point for s in between if isinstance(s, Nav)) == 'spot_03_pre'


def test_after_the_pick_goes_back_to_the_see_point_facing_the_place():
    """拿完近处的货直接回观察点，朝南边自己的塔，不再停在入口。"""
    approaches = load_approaches(APPROACHES)
    field = towers(spot_01=Tower(ground=1, seen=True))
    choice = choose(field, Goods(ground=3, ground_stack=1), approaches)
    blocks = [Block('ground', -3.25, -1.55, 0, 1), Block('ground', -3.25, -1.55, 0, 2)]
    steps = compile_choice(choice, approaches, blocks)
    pick = next(i for i, s in enumerate(steps) if isinstance(s, Action) and s.name == 'PICK')
    after = [s for s in steps[pick + 1:] if isinstance(s, Nav)]
    assert after[0].point == 'transfer_see'
    assert math.isclose(after[0].yaw, -1.5708, abs_tol=1e-4)
    assert all(s.point not in ('transfer_gate', '@pick_pre') for s in after)


def test_one_blue_layer_does_not_jump_our_south_tower():
    """南共享只有一层地：先把两块地一摞放到空的 spot_07，不要穿过去垫共享区。"""
    field = towers(spot_01=Tower(ground=1, seen=True))
    choice = choose(field, Goods(ground=3, sky=0, ground_stack=1))
    assert choice.action == PLACE_GROUND and choice.spot == 'spot_07' and choice.count == 2


def test_full_blue_stack_goes_on_our_tower_before_the_flip():
    field = towers(spot_01=Tower(ground=2, sky_owner=SKY_OTHERS, seen=True))
    goods = Goods(ground=3, sky=0, ground_stack=1)
    choice = choose(field, goods)
    assert choice.action == PLACE_GROUND and choice.spot == 'spot_07' and choice.count == 2
    apply_choice(field, goods, choice)
    nxt = choose(field, Goods(ground=1))
    assert nxt.action == FLIP and nxt.spot == 'spot_01'
    field['spot_01'] = Tower(ground=2, sky_owner=SKY_OURS, seen=True)
    after = choose(field, Goods(ground=1))
    assert after.action == PLACE_GROUND and after.spot == 'spot_03'


def test_last_ground_goes_north_only_after_the_south_flip():
    """南共享还是对方天空时先翻。翻完，最后一块地南边用不上，才送到 spot_03。"""
    field = towers(
        spot_07=Tower(ground=2, seen=True),
        spot_01=Tower(ground=2, sky_owner=SKY_OTHERS, seen=True))
    choice = choose(field, Goods(ground=1))
    assert choice.action == FLIP and choice.spot == 'spot_01'
    field['spot_01'] = Tower(ground=2, sky_owner=SKY_OURS, seen=True)
    choice = choose(field, Goods(ground=1))
    assert choice.action == PLACE_GROUND and choice.spot == 'spot_03'
    assert '下一趟' in choice.why
    field['spot_01'] = Tower(ground=1, seen=True)
    choice = choose(field, Goods(ground=1))
    assert choice.spot == 'spot_01'


def test_south_shared_only_sky_still_jumps_ahead():
    goods = Goods(ground=2, sky=1)
    only_sky = towers(spot_01=Tower(ground=2, seen=True))
    choice = choose(only_sky, goods)
    assert choice.action == PLACE_SKY and choice.spot == 'spot_01'
    assert '只差天空' in choice.why
    apply_choice(only_sky, goods, choice)
    nxt = choose(only_sky, goods)
    assert nxt.action == PLACE_GROUND and nxt.spot == 'spot_07'


def test_north_shared_stays_last():
    """北共享哪怕只差天空，也不抢在自己的塔、南共享、五色石前面。"""
    field = towers(spot_02=Tower(ground=2, seen=True))
    choice = choose(field, Goods(ground=2, sky=1))
    assert choice.spot == 'spot_07'
    done = south_done(spot_02=Tower(ground=2, seen=True))
    choice = choose(done, Goods(ground=2, sky=1))
    assert choice.action == PLACE_SKY and choice.spot == 'spot_02'


def test_arrival_switches_when_someone_else_placed():
    field = towers()
    goods = Goods(ground=1, sky=1)
    planned = choose(field, goods)
    assert planned.action == PLACE_GROUND and planned.assumed_ground == 0
    field['spot_07'] = Tower(ground=2, sky_owner=SKY_OTHERS, seen=True)
    new, note = on_arrival(planned, field, goods)
    assert new.action == FLIP and new.spot == 'spot_07'
    assert '开到 spot_07 跟前' in note
    same, note = on_arrival(planned, towers(), goods)
    assert same is planned and note == ''


def test_place_does_not_insert_a_look():
    field = towers()
    goods = Goods(ground=2, sky=0)
    first = choose(field, goods)
    assert first.action == PLACE_GROUND
    goods = apply_choice(field, goods, first)
    second = choose(field, goods)
    assert second.action == PLACE_GROUND and second.spot == 'spot_07'
    assert field['spot_07'].ground == 1


def test_resume_makes_every_spot_need_a_look():
    field = towers(spot_07=Tower(ground=1, seen=True))
    mark_unseen(field)
    choice = choose(field, Goods(ground=1, sky=1))
    assert choice.action == LOOK and choice.spot == 'spot_07'


def test_flip_does_not_stop_to_relocalize():
    """翻转去南共享：回头看一眼就走，路上的校准实测都在死区里。"""
    approaches = load_approaches(APPROACHES)
    field = towers(spot_01=Tower(ground=2, sky_owner=SKY_OTHERS, seen=True))
    choice = choose(field, Goods(), approaches)
    steps = compile_choice(choice, approaches, [])
    flip_at = next(i for i, s in enumerate(steps) if isinstance(s, Action) and s.name == 'FLIP_SKY')
    assert any(isinstance(s, Nav) and s.point == 'transfer_see' and s.yaw is not None
               for s in steps[:flip_at])
    assert any(isinstance(s, Detect) for s in steps[:flip_at])
    assert not any(getattr(s, 'point', None) == 'spot_07_glance' for s in steps)
    assert not any(isinstance(s, Relocalize) for s in steps[:flip_at])


def test_leaving_our_north_tower_does_not_jog_east():
    """人已经在 spot_03 西侧。回交接区直接南下，不再往东收到北共享。"""
    approaches = load_approaches(APPROACHES)
    field = south_done(spot_03=Tower(ground=1, seen=True))
    choice = choose(field, Goods(sky=1, cap_stack=1), approaches)
    blocks = [Block('ground', -3.50, -1.30, 0, 1), Block('sky', -3.50, -1.30, 0, 2)]
    steps = compile_choice(choice, approaches, blocks, on_west=True)
    navs = [s for s in steps if isinstance(s, Nav)]
    assert navs[0].point == 'transfer_see'
    assert all(s.point != 'spot_02_west' for s in navs)


def test_after_the_pick_the_north_corridor_skips_the_see_point():
    """拿完去北边，走廊南端就在观察点旁边。直接朝北开，不再先拐到观察点。"""
    approaches = load_approaches(APPROACHES)
    field = towers(
        spot_07=Tower(ground=2, seen=True),
        spot_01=Tower(ground=2, sky_owner=SKY_OURS, seen=True),
        spot_02=Tower(ground=2, sky_owner=SKY_OTHERS, seen=True))
    choice = choose(field, Goods(ground=1), approaches)
    assert choice.action == PLACE_GROUND and choice.spot == 'spot_03'
    blocks = [Block('ground', -3.50, -1.30, 0, 1)]
    steps = compile_choice(choice, approaches, blocks)
    pick = next(i for i, s in enumerate(steps) if isinstance(s, Action) and s.name == 'PICK')
    after = [s for s in steps[pick + 1:] if isinstance(s, Nav)]
    assert after[0].point == 'l2_stair_south'
    assert after[1].point == 'spot_03_pre'
    assert all(s.point not in ('transfer_see', 'l2_stair_west') for s in after)


def test_our_own_tower_is_placed_without_another_look():
    """自己刚放上的层不用再停车看。共享区对方会加块，放之前仍看一眼。"""
    approaches = load_approaches(APPROACHES)
    own = towers(spot_07=Tower(ground=1, seen=True))
    own_choice = choose(own, Goods(ground=1), approaches)
    assert own_choice.spot == 'spot_07'
    own_steps = compile_choice(own_choice, approaches, [Block('ground', -3.50, -1.30, 0, 1)])
    assert not any(isinstance(s, Detect) and s.slot == 'check' for s in own_steps)
    shared = towers(spot_07=Tower(ground=2, sky_owner=SKY_OURS, seen=True),
                    spot_01=Tower(ground=1, seen=True))
    shared_choice = choose(shared, Goods(ground=1), approaches)
    assert shared_choice.spot == 'spot_01'
    shared_steps = compile_choice(
        shared_choice, approaches, [Block('ground', -3.50, -1.30, 0, 1)])
    assert any(isinstance(s, Detect) and s.slot == 'check' for s in shared_steps)


def test_flip_drives_to_the_place_pose():
    approaches = load_approaches(APPROACHES)
    field = towers(spot_01=Tower(ground=2, sky_owner=SKY_OTHERS, seen=True))
    choice = choose(field, Goods(), approaches)
    assert choice.action == FLIP and choice.spot == 'spot_01'
    steps = compile_choice(choice, approaches, [])
    assert any(isinstance(s, Action) and s.name == 'FLIP_SKY' for s in steps)
    assert any(isinstance(s, Nav) and s.point == 'spot_01_west' for s in steps)
    assert [s.point for s in steps if isinstance(s, Nav)] == [
        'transfer_see', 'spot_01_west', 'spot_01_place']


def test_inner_block_is_taken_before_the_outer_stack():
    """靠近内场的单块先拿。外面那摞贴着北栏杆，等里面的拿走再去。"""
    from rc2027_decision.world.planner import pickable_goods
    approaches = load_approaches(APPROACHES)
    blocks = [Block('ground', -3.70, -1.15, 0, 1), Block('ground', -3.70, -1.15, 0, 2),
              Block('ground', -3.25, -1.55, 0, 1)]
    goods = pickable_goods(blocks)
    assert goods.ground == 1 and goods.ground_stack == 0
    choice = choose(towers(), goods, approaches)
    assert choice.action == PLACE_GROUND and choice.count == 1
    steps = compile_choice(choice, approaches, blocks)
    detect = next(s for s in steps if isinstance(s, Detect) and s.slot == 'pick')
    assert detect.what == 'ground'


def test_two_ground_stack_is_one_pick_and_one_place():
    """空塔碰到两块地叠在一起：一次拿走，放到第 1 层，两层一起落下。"""
    from rc2027_decision.world.planner import pickable_goods
    approaches = load_approaches(APPROACHES)
    blocks = [Block('ground', -3.70, -1.15, 0, 1), Block('ground', -3.70, -1.15, 0, 2)]
    goods = pickable_goods(blocks)
    assert goods.ground_stack == 1 and goods.ground == 2
    choice = choose(towers(), goods, approaches)
    assert choice.action == PLACE_GROUND and choice.spot == 'spot_07' and choice.count == 2
    steps = compile_choice(choice, approaches, blocks)
    detect = next(s for s in steps if isinstance(s, Detect) and s.slot == 'pick')
    assert detect.what == 'stack'
    pick = next(s for s in steps if isinstance(s, Action) and s.name == 'PICK')
    assert pick.param == 2
    place = next(s for s in steps if isinstance(s, Action) and s.name == 'PLACE')
    assert place.param == 1
    field = towers()
    goods = apply_choice(field, goods, choice)
    assert field['spot_07'].ground == 2
    assert goods.ground == 0 and goods.ground_stack == 0


def test_ground_and_sky_stack_finishes_a_one_layer_tower():
    """1 层地碰到「地+天空」：一次拿走，第 2 层和天空一起放下。"""
    approaches = load_approaches(APPROACHES)
    field = towers(spot_01=Tower(ground=1, seen=True),
                   spot_07=Tower(ground=2, sky_owner=SKY_OURS, seen=True))
    goods = Goods(sky=1, cap_stack=1)
    choice = choose(field, goods, approaches)
    assert choice.action == PLACE_SKY and choice.spot == 'spot_01' and choice.count == 2
    blocks = [Block('ground', -3.68, -1.13, 0, 1), Block('sky', -3.68, -1.13, 0, 2),
              Block('sky', -3.28, -1.53, 0, 1)]
    steps = compile_choice(choice, approaches, blocks)
    detect = next(s for s in steps if isinstance(s, Detect) and s.slot == 'pick')
    assert detect.what == 'groundsky'
    pick = next(s for s in steps if isinstance(s, Action) and s.name == 'PICK')
    assert pick.param == 2
    place = next(s for s in steps if isinstance(s, Action) and s.name == 'PLACE')
    assert place.param == 2
    apply_choice(field, goods, choice)
    assert field['spot_01'].ground == 2 and field['spot_01'].sky_owner == SKY_OURS


def test_one_sky_picks_the_loose_sky_not_the_stack():
    """只盖一块天空时拿单块，不拆地天摞。"""
    from rc2027_decision.world.planner import pickable_goods
    approaches = load_approaches(APPROACHES)
    field = towers(spot_07=Tower(ground=2, seen=True))
    blocks = [Block('ground', -3.68, -1.13, 0, 1), Block('sky', -3.68, -1.13, 0, 2),
              Block('sky', -3.28, -1.53, 0, 1)]
    goods = pickable_goods(blocks)
    choice = choose(field, goods, approaches)
    assert choice.action == PLACE_SKY and choice.spot == 'spot_07' and choice.count == 1
    steps = compile_choice(choice, approaches, blocks)
    detect = next(s for s in steps if isinstance(s, Detect) and s.slot == 'pick')
    assert detect.what == 'sky'


def test_compile_place_picks_the_single_ground_and_places_on_layer_1():
    approaches = load_approaches(APPROACHES)
    blocks = [Block('ground', -3.70, -1.15, 0, 1), Block('ground', -3.70, -1.15, 0, 2),
              Block('ground', -3.25, -1.55, 0, 1)]
    choice = choose(towers(), Goods(ground=3), approaches)
    steps = compile_choice(choice, approaches, blocks)
    detect = next(s for s in steps if isinstance(s, Detect) and s.slot == 'pick')
    assert detect.what == 'ground' and detect.nominal == 'transfer_single'
    place = next(s for s in steps if isinstance(s, Action) and s.name == 'PLACE')
    assert place.param == 1
    assert not any(isinstance(s, Detect) and s.slot == 'check' for s in steps)


def test_buried_ground_is_taken_with_its_sky():
    """地块压在天空下面时，单块天空先盖自己已经两层的南塔，不插去垫南共享。"""
    from rc2027_decision.world.planner import pickable_goods
    approaches = load_approaches(APPROACHES)
    field = towers(
        spot_07=Tower(ground=2, seen=True),
        spot_01=Tower(ground=1, seen=True))
    blocks = [Block('ground', -3.68, -1.13, 0, 1), Block('sky', -3.68, -1.13, 0, 2),
              Block('sky', -3.28, -1.53, 0, 1)]
    goods = pickable_goods(blocks)
    # 单块天更靠近内场。南边自己的塔已经两层，先盖这块天。地天摞还在货里，这座塔吃不下。
    assert goods == Goods(sky=1)
    choice = choose(field, goods, approaches)
    assert choice.action == PLACE_SKY and choice.spot == 'spot_07' and choice.count == 1


def test_cap_covers_the_south_shared_that_grew_instead_of_going_north():
    """手里是地+天。南共享半路被垫成两层，北共享记账里还是一层：先盖南共享，整摞不要送去北边。"""
    approaches = load_approaches(APPROACHES)
    field = towers(
        spot_01=Tower(ground=2, seen=True),
        spot_02=Tower(ground=1, seen=True))
    choice = carry_choice(field, 'cap', approaches)
    assert choice.action == PLACE_SKY and choice.spot == 'spot_01' and choice.count == 1


def test_cap_on_a_tower_that_grew_a_layer_places_the_sky_only():
    """出发时北共享一层。半路被垫成两层，手里的地天摞放不上去：先盖天，地留着。"""
    approaches = load_approaches(APPROACHES)
    field = towers(
        spot_07=Tower(ground=2, sky_owner=SKY_OURS, seen=True),
        spot_01=Tower(ground=2, sky_owner=SKY_OURS, seen=True),
        spot_02=Tower(ground=2, seen=True),
        spot_03=Tower(ground=2, sky_owner=SKY_OURS, seen=True))
    choice = carry_choice(field, 'cap', approaches)
    assert choice.action == PLACE_SKY and choice.spot == 'spot_02' and choice.count == 1
    field['spot_02'] = Tower(ground=2, sky_owner=SKY_OURS, seen=True)
    assert carry_choice(field, 'ground', approaches, 1) is None


def test_opponent_caps_the_tower_while_we_carry_the_sky():
    """南边自己的塔和南共享都差天空：单块天先盖自己的。南共享被对方盖上之后，这块天还是留在自己的塔。"""
    approaches = load_approaches(APPROACHES)
    field = towers(
        spot_07=Tower(ground=2, seen=True),
        spot_01=Tower(ground=2, seen=True))
    planned = choose(field, Goods(sky=1), approaches)
    assert planned.action == PLACE_SKY and planned.spot == 'spot_07'
    field['spot_01'] = Tower(ground=2, sky_owner=SKY_OTHERS, seen=True)
    new, note = on_arrival(planned, field, Goods(sky=1), approaches)
    assert new.action == PLACE_SKY and new.spot == 'spot_07' and not note
    held = carry_choice(field, 'sky', approaches, avoid='spot_01')
    assert held is not None and held.action == PLACE_SKY and held.spot == 'spot_07'


def test_clipped_glance_still_uses_a_taller_tower():
    """出发时南共享 1 层。半路对方又盖了一层地，跟前把顶看丢了，也不能再往第 2 层放地。"""
    approaches = load_approaches(APPROACHES)
    field = towers(
        spot_07=Tower(ground=2, seen=True),
        spot_01=Tower(ground=1, seen=True))
    planned = choose(field, Goods(ground=1), approaches)
    assert planned.action == PLACE_GROUND and planned.spot == 'spot_01' and planned.count == 1
    field['spot_01'] = Tower(ground=2, seen=True, clipped=True)
    new, note = on_arrival(planned, field, Goods(sky=1), approaches)
    assert note and new.action == PLACE_SKY and new.spot == 'spot_07'
    held = carry_choice(field, 'ground', approaches, 1)
    assert held is not None and held.action == PLACE_GROUND and held.spot == 'spot_03'


def test_unclear_close_look_keeps_the_plan():
    """预备位出了视场，层数和天空都没变，不能改去抓别的。"""
    approaches = load_approaches(APPROACHES)
    field = towers(spot_07=Tower(ground=2, seen=True))
    planned = choose(field, Goods(sky=1), approaches)
    assert planned.action == PLACE_SKY and planned.spot == 'spot_07'
    field['spot_07'] = Tower(ground=2, seen=True, clipped=True)
    new, note = on_arrival(planned, field, Goods(sky=1), approaches)
    assert new is planned and note == ''


def test_close_look_does_not_drop_our_layers():
    """贴着塔看少了一层，不能把已经放过的两层地改成一层。"""
    remembered = Tower(ground=2, seen=True)
    seen = close_look(remembered, ground=1, has_sky=False, clipped=False)
    assert seen.ground == 2 and seen.sky_owner == SKY_NONE
    kept = close_look(remembered, ground=0, has_sky=False, clipped=True)
    assert kept == remembered


def test_close_look_accepts_an_opponents_sky():
    remembered = Tower(ground=2, seen=True)
    seen = close_look(remembered, ground=2, has_sky=True, clipped=False)
    assert seen.sky_owner == SKY_OTHERS
    ours = close_look(Tower(ground=2, sky_owner=SKY_OURS, seen=True), 2, True, False)
    assert ours.sky_owner == SKY_OURS


def test_carry_choice_places_the_held_sky_on_the_other_south_tower():
    approaches = load_approaches(APPROACHES)
    field = towers(
        spot_07=Tower(ground=2, sky_owner=SKY_OTHERS, seen=True),
        spot_01=Tower(ground=2, seen=True))
    choice = carry_choice(field, 'sky', approaches)
    assert choice.action == PLACE_SKY and choice.spot == 'spot_01'


def test_compile_holding_goes_straight_to_place():
    approaches = load_approaches(APPROACHES)
    choice = choose(towers(spot_07=Tower(ground=2, seen=True)), Goods(sky=1), approaches)
    steps = compile_choice(choice, approaches, [], holding=True)
    assert any(isinstance(s, Action) and s.name == 'PLACE' for s in steps)
    assert not any(isinstance(s, Action) and s.name == 'PICK' for s in steps)
    assert not any(isinstance(s, Detect) and s.slot == 'check' for s in steps)


def test_compile_leftover_ground_uses_the_stack_nominal():
    """单块拿走后，剩下的地块还在一摞的位置，示教点跟着它，不然限幅会拒。"""
    approaches = load_approaches(APPROACHES)
    blocks = [Block('ground', -3.72, -1.15, 0, 1)]
    choice = choose(towers(
        spot_07=Tower(ground=2, sky_owner=SKY_OURS, seen=True),
        spot_01=Tower(seen=True)), Goods(ground=1), approaches)
    steps = compile_choice(choice, approaches, blocks)
    detect = next(s for s in steps if isinstance(s, Detect) and s.slot == 'pick')
    assert detect.what == 'ground' and detect.nominal == 'transfer_stack'


def test_pose_comes_from_the_approaches_file():
    approaches = load_approaches(APPROACHES)
    field = towers()
    choice = choose(field, Goods(ground=1), approaches)
    assert choice.pose[0] == approaches['spots']['spot_07']['place']['x']
    looked = choose(towers(spot_07=Tower(ground=2, sky_owner=SKY_OURS, seen=True)),
                    Goods(sky=1), approaches)
    assert looked.action == LOOK
    assert looked.pose[1] == approaches['spots']['spot_01']['look']['y']


def test_l2_place_faces_a_cardinal_axis():
    """L2 斜 45° 放，方块对角线擦出 500 mm 建造点，这一层不得分。朝向取正轴，夹爪落在塔心。"""
    import math
    approaches = load_approaches(APPROACHES)
    grip = approaches['gripper_offset']
    spots = {'spot_08': (-1.25, -1.25), 'spot_05': (-1.25, 1.25)}
    axes = (0.0, math.pi / 2, math.pi, -math.pi / 2)
    for name, (sx, sy) in spots.items():
        place = approaches['spots'][name]['place']
        yaw = place['theta']
        err = min(abs(math.atan2(math.sin(yaw - axis), math.cos(yaw - axis))) for axis in axes)
        assert err < math.radians(1), name
        gx = place['x'] + grip * math.cos(yaw)
        gy = place['y'] + grip * math.sin(yaw)
        assert abs(gx - sx) < 0.02 and abs(gy - sy) < 0.02, name


def test_flip_glance_with_empty_hands_does_not_place():
    """翻之前在观察点改了计划。这一步还没拿块，不能当成手里有地去放。"""
    field = towers(
        spot_07=Tower(ground=2, sky_owner=SKY_OURS, seen=True),
        spot_01=Tower(ground=2, sky_owner=SKY_OURS, seen=True),
        spot_03=Tower(ground=1, seen=True),
        spot_02=Tower(ground=2, seen=True))
    planned = Choice(FLIP, 'spot_02', '北共享有对方天空', None, 2, SKY_OTHERS)
    new = Choice(PLACE_SKY, 'spot_03', '上一趟的地和天空', None, 1, SKY_NONE, count=2)
    kind, nxt = after_changed_look(planned, new, field, {}, held_n=0)
    assert kind == 'empty' and nxt is new
    bogus = carry_choice(field, 'ground', {}, 1)
    assert bogus.action == PLACE_GROUND and bogus.spot == 'spot_03'


def test_held_ground_still_moves_to_another_tower():
    """已经拿着一块地，跟前改了计划，还是按手里这块找塔。"""
    field = towers(spot_07=Tower(ground=2, seen=True))
    planned = Choice(PLACE_GROUND, 'spot_07', '放地', None, 0, SKY_NONE)
    new = Choice(PLACE_SKY, 'spot_07', '改盖天空', None, 2, SKY_NONE)
    kind, nxt = after_changed_look(planned, new, field, {}, held_n=1)
    assert kind == 'carry'
    assert nxt.action == PLACE_GROUND and nxt.spot == 'spot_03'


def test_same_load_keeps_the_new_place():
    """手里一块地，新计划还是放一块地：换塔，不再按种类重选。"""
    planned = Choice(PLACE_GROUND, 'spot_01', '放地', None, 0, SKY_NONE)
    new = Choice(PLACE_GROUND, 'spot_07', '换到自己的塔', None, 0, SKY_NONE)
    kind, nxt = after_changed_look(planned, new, towers(), {}, held_n=1)
    assert kind == 'same' and nxt is new


def test_held_block_prevents_flipping():
    """手里有货而且没有放置位时，停车。"""
    done = Tower(ground=2, sky_owner=SKY_OURS, seen=True)
    field = towers(
        spot_07=done, spot_01=done, spot_03=done,
        spot_02=Tower(ground=2, sky_owner=SKY_OTHERS, seen=True))
    field['spot_08'] = done
    field['spot_05'] = done
    planned = Choice(PLACE_GROUND, 'spot_07', '放地', None, 2, SKY_NONE)
    new = Choice(FLIP, 'spot_02', '先翻', None, 2, SKY_OTHERS)
    kind, nxt = after_changed_look(planned, new, field, {}, held_n=1)
    assert kind == 'stuck' and nxt is None


def test_clipped_lone_ground_is_not_taken_off_a_cap():
    """顶出视场时单独一块地不算能拿的地。那是没看全的地天摞，拿起来揭走的是天。"""
    fake_ground = Block('ground', -3.66, -1.16, 0.0, 1)
    lone_sky = Block('sky', -3.24, -1.50, 0.0, 1)
    goods = pickable_goods(trusted_transfer_blocks([fake_ground, lone_sky], True))
    assert goods.ground == 0 and goods.sky == 1 and goods.cap_stack == 0
    seen = pickable_goods(trusted_transfer_blocks([fake_ground], False))
    assert seen.ground == 1
    cap = [Block('ground', -3.66, -1.16, 0.0, 1), Block('sky', -3.66, -1.16, 0.0, 2)]
    full = pickable_goods(trusted_transfer_blocks(cap, True))
    assert full.cap_stack == 1 and full.ground == 0


def test_held_block_climbs_from_the_west_after_north_shared():
    """人在北共享东边，手里的块改去 L2。先收到西侧再去楼梯口。"""
    approaches = load_approaches(APPROACHES)
    choice = Choice(PLACE_SKY, 'spot_08', '手里是天空', None, 2, SKY_NONE)
    steps = compile_choice(choice, approaches, [], holding=True, from_north=True)
    names = [s.point if isinstance(s, Nav) else getattr(s, 'name', None) for s in steps]
    assert names[0] == 'spot_02_west'
    assert names.index('spot_02_west') < names.index('l2_stair_west') < names.index('l2_stair_foot')
    straight = compile_choice(choice, approaches, [], holding=True)
    straight_names = [s.point if isinstance(s, Nav) else getattr(s, 'name', None)
                      for s in straight]
    assert straight_names[0] == 'l2_stair_foot'


def test_leaving_either_shared_tower_does_not_cross_l2():
    """北共享往南、南共享往北，人还在东边时直线不能穿 L2。台面 |x|、|y| ≤ 1.5，车身再让 0.16 m。"""
    approaches = load_approaches(APPROACHES)
    points = bind_points(approaches)
    half = 1.5 + 0.16
    blocks = [Block('ground', -3.25, -1.55, 0.0, 1), Block('sky', -3.66, -1.16, 0.0, 1)]
    spots = ('spot_07', 'spot_01', 'spot_03', 'spot_02', 'spot_08', 'spot_05')
    actions = (PLACE_GROUND, PLACE_SKY, FLIP, LOOK, WAIT, CLIMB)

    def hits(a, b):
        for i in range(81):
            t = i / 80
            x = a[0] + (b[0] - a[0]) * t
            y = a[1] + (b[1] - a[1]) * t
            if abs(x) <= half and abs(y) <= half:
                return True
        return False

    starts = (
        ('北', points['spot_02_place'], dict(from_north=True)),
        ('北', points['spot_02_pre'], dict(from_north=True)),
        ('南', points['spot_01_place'], dict(from_south=True)),
        ('南', points['spot_01_pre'], dict(from_south=True)),
    )
    for side, start, flags in starts:
        for action in actions:
            targets = (None,) if action in (WAIT, CLIMB) else spots
            for spot in targets:
                holding = action in (PLACE_GROUND, PLACE_SKY)
                choice = Choice(action, spot, 'check', None, 1, SKY_NONE)
                steps = compile_choice(choice, approaches, blocks, holding=holding, **flags)
                chain = [start[:2]]
                for step in steps:
                    if isinstance(step, Action) and step.name == 'CLIMB_L2':
                        break
                    if isinstance(step, Nav) and step.point in points:
                        chain.append(points[step.point][:2])
                for a, b in zip(chain, chain[1:]):
                    assert not hits(a, b), (side, action, spot, a, b)


def test_wait_checks_supply_without_unconditional_relocalization():
    steps = compile_choice(Choice(WAIT, None, '货物暂时不匹配'),
                           load_approaches(APPROACHES), [])
    assert any(isinstance(s, Detect) and s.slot == 'supply' for s in steps)
    assert not any(isinstance(s, Relocalize) for s in steps)
    assert isinstance(steps[-1], Wait)


def test_north_shared_cap_rerouted_to_l2_leaves_l1_waypoints_below():
    choice = Choice(PLACE_SKY, 'spot_08', '北共享已封顶，改放手里的天', assumed_ground=2)
    steps = compile_choice(choice, load_approaches(APPROACHES), [], holding=True, from_north=True)
    climb = steps.index(Action('CLIMB_L2'))
    assert [s.point for s in steps[:climb] if isinstance(s, Nav)] == [
        'spot_02_west', 'l2_stair_west', 'l2_stair_foot']
    navs = [s.point for s in steps[climb + 1:] if isinstance(s, Nav)]
    assert navs[0] == 'spot_08_lane'
    assert all(name.startswith('spot_08_') for name in navs)
    assert steps[-1] == Action('PLACE', 3)


def test_pick_after_south_flip_leaves_transfer_facing_the_stairs():
    approaches = load_approaches(APPROACHES)
    choice = Choice(PLACE_SKY, 'spot_05', '放地天', assumed_ground=1, count=2)
    blocks = [Block('ground', -3.66, -1.16, 0, 1), Block('sky', -3.66, -1.16, 0, 2)]
    steps = compile_choice(choice, approaches, blocks, from_south=True)
    depart = steps[steps.index(Action('PICK', 2)) + 1:steps.index(Action('CLIMB_L2'))]
    yaw = bind_points(approaches)['l2_stair_foot'][2]
    assert depart == [Nav('transfer_see', yaw=yaw), Nav('l2_stair_foot')]


def test_near_double_ground_can_top_up_a_one_ground_shared_tower():
    field = towers(spot_07=Tower(ground=2, sky_owner=SKY_OURS, seen=True),
                   spot_01=Tower(ground=1, seen=True), spot_02=Tower(ground=2, seen=True))
    goods = Goods(ground=2, ground_stack=1)
    choice = choose(field, goods)
    assert (choice.spot, choice.pick_count, choice.count) == ('spot_01', 2, 1)
    blocks = [Block('ground', -3.25, -1.55, 0, layer) for layer in (1, 2)]
    steps = compile_choice(choice, load_approaches(APPROACHES), blocks)
    assert Action('PICK', 2) in steps and steps[-1] == Action('PLACE_ONE', 2)
    field['spot_03'] = Tower(ground=2, seen=True)
    assert choose(field, goods).pick_count == 1

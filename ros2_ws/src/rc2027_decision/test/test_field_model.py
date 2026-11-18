"""世界模型（world/field_model.py）的合并规则（不需要ROS）。"""

import math

import pytest

from rc2027_decision.flow.pick_pose import Block
from rc2027_decision.world import (FieldModel, ModelParams, Observation, PRIOR, SEEN, SKY_NONE,
                                   SKY_OTHERS, SKY_OURS)

SPOTS = {'spot_03': (-2.7, 2.7), 'spot_07': (-2.7, -2.7)}
FAR = (0.0, 0.0)   # 车离两个建造点都远（朝向没给：只能用来"变多"）
LOOK = (-2.7, 0.4)     # far_look：正对着 spot_03，2.3 m
LOOK_YAW = math.pi / 2


def tower(x, y, ground, sky=False, yaw=0.0, reliable=True):
    """一摞方块（block_detector 的顺序：从下往上）。"""
    out = [Block('ground', x, y, yaw, k + 1, reliable) for k in range(ground)]
    if sky:
        out.append(Block('sky', x, y, yaw, ground + 1, reliable))
    return tuple(out)


def obs(t, ground, sky=False, region='spot_03', blocks=None, clipped=False, coverage=0.8,
        visible=True):
    x, y = SPOTS[region]
    if blocks is None:
        blocks = tower(x, y, ground, sky)
    return Observation(region, t, visible, coverage, ground, sky, clipped, blocks)


def state(m, region='spot_03'):
    s = m.spots[region]
    return s.ground, s.sky


def test_starts_empty_from_rules():
    m = FieldModel(SPOTS, start_time=5.0)
    s = m.spots['spot_03']
    assert state(m) == (0, False) and s.source == PRIOR and s.since == 5.0


def test_empty_observation_confirms_prior():
    m = FieldModel(SPOTS)
    assert m.observe(obs(1.0, 0), FAR) is None
    s = m.spots['spot_03']
    assert s.source == SEEN and s.confirmations == 1 and s.since == 1.0


def test_going_up_needs_two_consecutive():
    m = FieldModel(SPOTS)
    assert m.observe(obs(1.0, 1), FAR) is None
    assert state(m) == (0, False)
    msg = m.observe(obs(2.0, 1), FAR)
    assert state(m) == (1, False) and '0 层 → 1 层' in msg


def test_flicker_resets_candidate():
    m = FieldModel(SPOTS)
    m.observe(obs(1.0, 1), FAR)
    m.observe(obs(2.0, 0), FAR)      # 又看到空的：候选作废
    m.observe(obs(3.0, 1), FAR)
    assert state(m) == (0, False)


def test_going_down_needs_more():
    m = FieldModel(SPOTS)
    for t in (1, 2):
        m.observe(obs(t, 2), FAR)
    assert state(m) == (2, False)
    for t in (3, 4, 5):
        m.observe(obs(t, 1), LOOK, LOOK_YAW)
    assert state(m) == (2, False)
    msg = m.observe(obs(6, 1), LOOK, LOOK_YAW)
    assert state(m) == (1, False) and '变少了' in msg


def test_flip_marks_the_sky_as_ours():
    m = FieldModel(SPOTS)
    for t in (1.0, 2.0):
        m.observe(obs(t, 2, True, region='spot_07'), FAR)
    assert m.spots['spot_07'].sky_owner == SKY_OTHERS
    _name, message = m.on_flip(SPOTS['spot_07'], 3.0)
    assert '我们的' in message
    assert m.spots['spot_07'].sky_owner == SKY_OURS


def test_partial_look_keeps_the_sky_we_just_placed():
    """先看到两层地、后看到天空时，不能把刚放的天空划成对方的。"""
    m = FieldModel(SPOTS)
    m.on_pick(1)
    m.on_place(1, SPOTS['spot_07'], 1.0)
    m.observe(obs(2.0, 1, region='spot_07'), FAR)
    m.observe(obs(3.0, 1, region='spot_07'), FAR)
    m.on_pick(2)
    m.on_place(2, SPOTS['spot_07'], 4.0)
    m.observe(obs(5.0, 2, region='spot_07'), FAR)
    m.observe(obs(6.0, 2, region='spot_07'), FAR)
    m.observe(obs(7.0, 2, True, region='spot_07'), FAR)
    s = m.spots['spot_07']
    assert (s.ground, s.sky) == (2, True)
    assert s.sky_owner == SKY_OURS


def test_going_down_ignored_when_view_not_clear():
    """2026-10-06 仿真：车在交接区拿货，spot_07 在车后方 1.7 m，4 次没看到天空块 → 不能报"变少了"。"""
    m = FieldModel(SPOTS)
    for t in (1, 2):
        m.observe(obs(t, 2, sky=True, region='spot_07'), LOOK_07, LOOK_07_YAW)
    assert state(m, 'spot_07') == (2, True)
    behind = (-3.241, -1.070)       # @pick，车头朝西
    for t in range(3, 20):
        assert m.observe(obs(t, 2, region='spot_07'), behind, math.radians(-174.2)) is None
    assert state(m, 'spot_07') == (2, True)
    # 位姿没给 / 太近 / 在车侧面，都不算看得清
    for xy, yaw in ((None, None), ((-2.7, -2.0), -math.pi / 2), ((-2.7, 0.4), 0.0)):
        for t in range(20, 30):
            m.observe(obs(t, 2, region='spot_07'), xy, yaw)
    assert state(m, 'spot_07') == (2, True)
    # 变多不受限制
    for t in (30, 31):
        m.observe(obs(t, 2, sky=True, region='spot_07'), behind, math.radians(-174.2))
    assert state(m, 'spot_07') == (2, True)


LOOK_07 = (-2.7, -0.4)    # near_look：正对着 spot_07，2.3 m
LOOK_07_YAW = -math.pi / 2


def test_unknown_in_region_is_ignored_and_flagged():
    m = FieldModel(SPOTS)
    x, y = SPOTS['spot_03']
    robot = (Block('unknown', x, y, 0.0, 1, False),)
    for t in range(1, 6):
        assert m.observe(obs(t, 0, blocks=robot), FAR) is None
    s = m.spots['spot_03']
    assert state(m) == (0, False) and s.source == PRIOR and s.obstructed_at == 5
    assert '最近被挡' in m.note(s, now=6.0)
    assert '最近被挡' not in m.note(s, now=20.0)


def test_clipped_unknown_is_not_something_blocking_the_tower():
    """跟前把顶看丢，高度对不上，不能记成被挡。被挡会让已经确认的对方天空改去退远看。"""
    m = FieldModel(SPOTS)
    x, y = SPOTS['spot_03']
    robot = (Block('unknown', x, y, 0.0, 1, False),)
    assert m.observe(obs(1, 1, blocks=robot, clipped=True), FAR) is None
    assert m.spots['spot_03'].obstructed_at is None


def test_clipped_can_raise_but_not_lower():
    m = FieldModel(SPOTS)
    for t in (1, 2):
        m.observe(obs(t, 2, sky=True), FAR)
    for t in range(3, 10):
        m.observe(obs(t, 2, clipped=True), FAR)   # 天空块出了视场
    assert state(m) == (2, True)
    m2 = FieldModel(SPOTS)
    for t in (1, 2):
        m2.observe(obs(t, 1, clipped=True), FAR)
    assert state(m2) == (1, False)


def test_weak_empty_is_not_evidence():
    m = FieldModel(SPOTS)
    for t in (1, 2):
        m.observe(obs(t, 1), FAR)
    for t in range(3, 10):
        m.observe(obs(t, 0, coverage=0.1), FAR)
    assert state(m) == (1, False)


def test_invisible_is_ignored():
    m = FieldModel(SPOTS)
    m.observe(obs(1.0, 0, visible=False), FAR)
    assert m.spots['spot_03'].source == PRIOR


def test_spot_next_to_robot_is_ignored():
    m = FieldModel(SPOTS)
    near = (-2.7, 2.25)      # far_place：拿 / 放时车就停在这
    for t in range(1, 6):
        m.observe(obs(t, 1), near)
    assert state(m) == (0, False)
    m.observe(obs(6, 1), FAR)
    m.observe(obs(7, 1), FAR)
    assert state(m) == (1, False)


def test_own_place_confirmed_by_first_matching_look():
    m = FieldModel(SPOTS)
    m.on_pick(2)
    spot, msg = m.on_place(1, (-2.7, -2.7 + 0.03), 1.0)
    assert spot == 'spot_07' and m.held == 0
    s = m.spots['spot_07']
    assert s.expected_total == 2 and s.ours == {1, 2}
    assert state(m, 'spot_07') == (0, False)    # 还没看到不改
    msg = m.observe(obs(2.0, 2, region='spot_07'), FAR)
    assert state(m, 'spot_07') == (2, False) and '刚放完' in msg and s.expected_total == 0


def test_own_place_mismatch_uses_normal_rule():
    m = FieldModel(SPOTS)
    m.on_pick(2)
    m.on_place(1, SPOTS['spot_07'], 1.0)
    m.observe(obs(2.0, 1, region='spot_07'), FAR)    # 只看到 1 块（上面那块掉了？）
    assert state(m, 'spot_07') == (0, False)
    m.observe(obs(3.0, 1, region='spot_07'), FAR)
    s = m.spots['spot_07']
    assert state(m, 'spot_07') == (1, False) and s.expected_total == 2
    assert '等雷达确认一共 2 块' in m.note(s, 3.0)


def test_place_off_spot_not_recorded():
    m = FieldModel(SPOTS)
    m.on_pick(1)
    spot, msg = m.on_place(1, (-3.5, -1.3), 1.0)      # 交接区
    assert spot is None and '不在任何建造点上' in msg and m.held == 0


def test_sky_owner():
    m = FieldModel(SPOTS)
    m.on_pick(2)
    m.on_place(1, SPOTS['spot_07'], 1.0)
    m.observe(obs(2.0, 2, region='spot_07'), FAR)
    m.on_pick(1)
    m.on_place(3, SPOTS['spot_07'], 3.0)
    m.observe(obs(4.0, 2, sky=True, region='spot_07'), FAR)
    assert m.spots['spot_07'].sky_owner == SKY_OURS
    # spot_03：对方搭的
    for t in (1, 2):
        m.observe(obs(t, 2, sky=True), FAR)
    assert m.spots['spot_03'].sky_owner == SKY_OTHERS
    assert FieldModel(SPOTS).spots['spot_03'].sky_owner == SKY_NONE


def test_ours_trimmed_when_tower_shrinks():
    m = FieldModel(SPOTS, params=ModelParams(confirm_down=1))
    m.on_pick(2)
    m.on_place(1, SPOTS['spot_03'], 1.0)
    m.observe(obs(2.0, 2), FAR)
    m.observe(obs(3.0, 1), LOOK, LOOK_YAW)
    assert m.spots['spot_03'].ours == {1}


def test_base_position_tracked():
    m = FieldModel(SPOTS)
    x, y = SPOTS['spot_03']
    for t in (1, 2):
        m.observe(obs(t, 1, blocks=tower(x + 0.04, y, 1, yaw=math.radians(3))), FAR)
    s = m.spots['spot_03']
    assert s.base[0] == pytest.approx(x + 0.04) and s.base[2] == pytest.approx(math.radians(3))
    # 小抖动平均掉
    m.observe(obs(3, 1, blocks=tower(x + 0.05, y, 1, yaw=math.radians(3))), FAR)
    assert x + 0.04 < s.base[0] < x + 0.05
    # 朝向跨 ±45° 不能平均成 0
    m.observe(obs(4, 1, blocks=tower(x + 0.04, y, 1, yaw=math.radians(-44))), FAR)
    m2 = FieldModel(SPOTS)
    for t, deg in ((1, 44), (2, 44), (3, -44)):
        m2.observe(obs(t, 1, blocks=tower(x, y, 1, yaw=math.radians(deg))), FAR)
    assert abs(math.degrees(m2.spots['spot_03'].base[2])) > 40
    # 被撞开了（跳 > 5 cm）：直接换
    m.observe(obs(5, 1, blocks=tower(x - 0.10, y, 1)), FAR)
    assert s.base[0] == pytest.approx(x - 0.10)


def test_unreliable_base_not_used():
    m = FieldModel(SPOTS)
    x, y = SPOTS['spot_03']
    for t in (1, 2):
        m.observe(obs(t, 1, blocks=tower(x, y, 1, reliable=False)), FAR)
    assert state(m) == (1, False) and m.spots['spot_03'].base is None


def test_retry_drops_held():
    m = FieldModel(SPOTS)
    m.on_pick(2)
    m.on_retry()
    assert m.held == 0
    m.on_place(1, SPOTS['spot_03'], 1.0)
    assert m.spots['spot_03'].expected_total == 1   # 不知道拿着几块时按 1 块记


def test_other_regions_ignored():
    m = FieldModel(SPOTS)
    assert m.observe(Observation('transfer_red', 1.0, True, 0.9, 2, False), FAR) is None

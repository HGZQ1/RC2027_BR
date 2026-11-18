"""上 L2 的落点。不起仿真。"""

import math

from rc2027_utils.sim_check.l2_hop import (
    CLEARANCE, L2_SPOTS, footprint_z, hop_target, just_on_platform)


def test_red_lands_just_on_the_platform_clear_of_the_pillar():
    x, y, yaw = hop_target('red')
    assert just_on_platform(x, y)
    assert abs(x) < 1.5          # 台沿里面，不是楼梯
    assert math.hypot(x, y) > 0.8  # 中央柱在原点，半径约 0.14 m
    assert math.isclose(yaw, 0.0)
    for sx, sy in L2_SPOTS:
        assert math.hypot(x - sx, y - sy) > 0.6


def test_blue_is_the_mirror():
    rx, ry, ryaw = hop_target('red')
    bx, by, byaw = hop_target('blue')
    assert math.isclose(bx, -rx) and math.isclose(by, ry)
    assert math.isclose(math.atan2(math.sin(byaw), math.cos(byaw)), math.pi)


def test_footprint_sits_just_above_the_surface():
    assert math.isclose(footprint_z(0.95), 0.95 + CLEARANCE)

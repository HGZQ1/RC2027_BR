import math

import numpy as np

from rc2027_utils.sim_check.climb_replay import (
    attach_terrain, build_xy_path, mirror_route, ROUTE_RED_STAIRS_TO_L1_RETRY, velocities)


def _stairs(x, y):
    """跟 Gazebo 场地实测一致的西侧楼梯：y < −2.725 地面 0.05，每 0.3 m 高 0.15，−1.825 以后 0.65。"""
    if y < -2.725:
        return 0.05
    if y >= -1.825:
        return 0.65
    return 0.05 + 0.15 * (int((y + 2.725) / 0.3) + 1)


def _climb():
    path = build_xy_path((-3.45, -3.6, math.pi / 2),
                         [('move', -3.45, -1.0, 0.15), ('hold', 1.0)])
    return attach_terrain(path, _stairs)


def test_xy_path_reaches_waypoints_and_holds():
    p = build_xy_path((0.0, 0.0, 1.0), [('move', 1.0, 0.0, 0.3), ('hold', 1.0)])
    assert np.allclose(p[-1, 1:4], [1.0, 0.0, 1.0])
    assert np.allclose(np.diff(p[:, 0]), 0.02)
    # 速度不超过 0.3（数值差分）
    v = np.hypot(np.diff(p[:, 1]), np.diff(p[:, 2])) / 0.02
    assert v.max() <= 0.3 + 1e-6


def test_stairs_bottom_top_heights_and_pitch():
    t = _climb()
    assert math.isclose(t[0, 3], 0.05 + 0.01, abs_tol=1e-6)     # 楼梯下：地面 + 1 cm
    assert math.isclose(t[-1, 3], 0.65 + 0.01, abs_tol=1e-6)    # 楼梯顶平台
    # 爬升中车头朝上（pitch 为负），坡度在台阶斜率附近（0.15/0.3 = 26.6°，最陡处）
    worst = math.degrees(t[:, 5].min())
    assert -40 < worst < -15
    assert abs(t[0, 5]) < 1e-6 and abs(t[-1, 5]) < 1e-6           # 平地上是平的


def test_body_never_cuts_into_steps():
    t = _climb()
    for _, x, y, z, roll, pitch, yaw in t[::5]:
        # 车体底面四个角和中线上的点都要在台阶表面上方
        for u in np.linspace(-0.15, 0.15, 7):
            wy = y + u   # 车头朝 +y：前后方向 u 就是 y
            zb = z - math.tan(pitch) * u
            assert zb >= _stairs(x, wy) - 1e-6


def test_velocities_integrate_back_to_positions():
    t = _climb()
    v = velocities(t)
    dt = t[1, 0] - t[0, 0]
    pos = t[0, 1:4] + np.cumsum(v[:-1, :3] * dt, axis=0)
    assert np.abs(pos[-1] - t[-1, 1:4]).max() < 0.02
    assert np.allclose(v[-1], 0.0, atol=1e-9)                     # 最后 hold 是静止的


def test_mirror_route_for_blue():
    blue = mirror_route(ROUTE_RED_STAIRS_TO_L1_RETRY)
    moves = [s for s in blue if s[0] == 'move']
    assert all(s[1] > 0 for s in moves)
    assert moves[-1][1:3] == (1.8, 2.6)
    turns = [s for s in blue if s[0] == 'turn']
    assert math.isclose(turns[-1][1], -math.pi / 2)            # 朝南镜像后还是朝南


def test_turn_in_place_takes_short_way_to_south():
    p = build_xy_path((0.0, 0.0, math.pi / 2), [('turn', -math.pi / 2, 0.8)])
    assert math.isclose(p[-1, 3] % (2 * math.pi), (-math.pi / 2) % (2 * math.pi), abs_tol=1e-9)
    assert np.allclose(p[:, 1:3], 0.0)                         # 原地
    w = np.abs(np.diff(p[:, 3])) / 0.02
    assert w.max() <= 0.8 + 1e-6


def test_smooth_upper_never_below_and_no_jumps():
    from rc2027_utils.sim_check.climb_replay import smooth_upper
    a = np.array([0.0] * 20 + [0.15] * 20 + [0.3] * 20)
    s = smooth_upper(a, 9)
    assert (s >= a - 1e-12).all()
    assert np.abs(np.diff(s)).max() <= 0.15 / 9 + 1e-9    # 台阶跳变被摊成斜坡


def test_vertical_speed_reasonable_on_stairs():
    t = _climb()
    v = velocities(t)
    assert np.abs(v[:, 2]).max() < 0.5   # 0.15 m/s 爬 33° 的楼梯，竖直速度应在 0.1 量级


def test_com_velocity_adds_lever_arm():
    from rc2027_utils.sim_check.climb_replay import com_velocity
    # 车头朝 +x，绕 y 轴（俯仰）以 1 rad/s 转，质心在原点上方 0.06 m：质心多出 x 方向 +0.06 m/s
    v = com_velocity([0.0, 0.0, 0.0], [0.0, 1.0, 0.0], 0.0, 0.0, 0.0, [0.0, 0.0, 0.06])
    assert np.allclose(v, [0.06, 0.0, 0.0])
    # 不转就不变
    assert np.allclose(com_velocity([0.1, 0.2, 0.3], [0, 0, 0], 0.3, -0.5, 1.0, [0, 0, 0.06]),
                       [0.1, 0.2, 0.3])

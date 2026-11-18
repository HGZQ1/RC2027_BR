import math

from rc2027_utils.sim_check.pose_error import pose_error, wrap_angle


def test_wrap_angle():
    assert math.isclose(wrap_angle(3 * math.pi / 2), -math.pi / 2)
    assert math.isclose(wrap_angle(-3 * math.pi / 2), math.pi / 2)
    assert math.isclose(wrap_angle(0.3), 0.3)


def test_pose_error_translation():
    dx, dy, dist, dyaw = pose_error((1.03, 2.0, 0.0), (1.0, 2.04, 0.0))
    assert math.isclose(dx, 0.03, abs_tol=1e-12)
    assert math.isclose(dy, -0.04, abs_tol=1e-12)
    assert math.isclose(dist, 0.05, abs_tol=1e-12)
    assert dyaw == 0.0


def test_pose_error_yaw_across_pi():
    # 179° 和 -179° 只差 2°，不是 358°
    _, _, _, dyaw = pose_error((0, 0, math.radians(179)), (0, 0, math.radians(-179)))
    assert math.isclose(dyaw, math.radians(-2), abs_tol=1e-9)


def test_interpolate_pose_midpoint_and_yaw_across_pi():
    from rc2027_utils.sim_check.pose_error import interpolate_pose
    hist = [(1.0, 0.0, 0.0, math.radians(170)), (2.0, 1.0, 2.0, math.radians(-170))]
    x, y, yaw = interpolate_pose(hist, 1.5)
    assert math.isclose(x, 0.5) and math.isclose(y, 1.0)
    # 170° -> -170° 走近路，中点是 180°，不是 0°
    assert math.isclose(abs(yaw), math.pi, abs_tol=1e-9)


def test_interpolate_pose_out_of_range_is_none():
    from rc2027_utils.sim_check.pose_error import interpolate_pose
    hist = [(1.0, 0.0, 0.0, 0.0), (2.0, 1.0, 0.0, 0.0)]
    assert interpolate_pose(hist, 0.5) is None
    assert interpolate_pose(hist, 2.5) is None
    assert interpolate_pose([], 1.0) is None
    assert interpolate_pose(hist, 2.0) == (1.0, 0.0, 0.0)


def test_speed_over():
    from rc2027_utils.sim_check.pose_error import speed_over
    hist = [(t * 0.1, t * 0.05, 0.0, 0.0) for t in range(11)]  # 0.5 m/s，1 s
    assert math.isclose(speed_over(hist, 0.5), 0.5)
    assert speed_over(hist[:1], 0.5) is None


def test_yaw_rate_over_spinning_in_place():
    from rc2027_utils.sim_check.pose_error import yaw_rate_over
    # 原地以 1 rad/s 转，跨过 ±180° 也要算对
    hist = [(t * 0.1, 0.0, 0.0, math.atan2(math.sin(3.0 + t * 0.1), math.cos(3.0 + t * 0.1)))
            for t in range(11)]
    assert math.isclose(yaw_rate_over(hist, 0.5), 1.0, rel_tol=1e-6)

import math
import os

import pytest

from rc2027_utils.teach_point.pose_stats import (
    circular_mean_pose, load_points, save_point, yaw_from_quaternion)


def test_circular_mean_pose_simple_average():
    samples = [(1.0, 2.0, 0.0), (3.0, 4.0, 0.0)]
    x, y, theta = circular_mean_pose(samples)
    assert math.isclose(x, 2.0)
    assert math.isclose(y, 3.0)
    assert math.isclose(theta, 0.0)


def test_circular_mean_pose_handles_angle_wraparound():
    # 179° 和 -179°（弧度）的算术平均是 0°，但物理上应该接近 ±180°
    near_pi = math.pi - math.radians(1)
    near_minus_pi = -math.pi + math.radians(1)
    samples = [(0.0, 0.0, near_pi), (0.0, 0.0, near_minus_pi)]
    _, _, theta = circular_mean_pose(samples)
    assert abs(theta) > math.radians(170), f"应该接近 ±180°，算出来是 {math.degrees(theta)}°"


def test_circular_mean_pose_empty_raises():
    with pytest.raises(ValueError):
        circular_mean_pose([])


def test_circular_mean_pose_single_sample_is_itself():
    x, y, theta = circular_mean_pose([(1.5, -2.5, 0.3)])
    assert (x, y, theta) == (1.5, -2.5, 0.3)


def test_save_and_load_point_roundtrip(tmp_path):
    path = str(tmp_path / "taught_points.yaml")
    save_point(path, "point_a", (1.0, 2.0, 0.5))
    points = load_points(path)
    assert points["point_a"] == {"x": 1.0, "y": 2.0, "theta": 0.5}


def test_save_point_creates_missing_parent_directory(tmp_path):
    # field_data/<site>_<side>.yaml 这种按场地分文件的路径，目录不一定
    # 已经存在（第一次在新场地录点时），save_point 要能自己建出来
    path = str(tmp_path / "field_data" / "training_red.yaml")
    save_point(path, "point_a", (1.0, 2.0, 0.0))
    points = load_points(path)
    assert points["point_a"] == {"x": 1.0, "y": 2.0, "theta": 0.0}


def test_save_point_keeps_other_existing_points(tmp_path):
    path = str(tmp_path / "taught_points.yaml")
    save_point(path, "point_a", (1.0, 2.0, 0.0))
    save_point(path, "point_b", (3.0, 4.0, 0.0))
    points = load_points(path)
    assert set(points.keys()) == {"point_a", "point_b"}


def test_save_point_overwrites_same_name(tmp_path):
    path = str(tmp_path / "taught_points.yaml")
    save_point(path, "point_a", (1.0, 2.0, 0.0))
    save_point(path, "point_a", (9.0, 9.0, 0.0))
    points = load_points(path)
    assert points["point_a"] == {"x": 9.0, "y": 9.0, "theta": 0.0}


def test_load_points_missing_file_returns_empty_dict():
    assert load_points("/tmp/definitely_does_not_exist_rc2027.yaml") == {}


def _quat_from_rpy(roll, pitch, yaw):
    # 跟 tf_transformations.quaternion_from_euler(r, p, y)（'sxyz'）同一个公式
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


@pytest.mark.parametrize('yaw', [0.0, 0.5, math.pi / 2, -2.0, math.pi - 1e-6])
def test_yaw_from_quaternion_pure_yaw(yaw):
    assert math.isclose(yaw_from_quaternion(*_quat_from_rpy(0.0, 0.0, yaw)), yaw, abs_tol=1e-9)


def test_yaw_from_quaternion_with_small_tilt():
    # 车在场地上会有一点点 roll/pitch（地面不平、雷达抖），yaw 要照样取对
    q = _quat_from_rpy(0.03, -0.02, 1.2)
    assert math.isclose(yaw_from_quaternion(*q), 1.2, abs_tol=1e-9)

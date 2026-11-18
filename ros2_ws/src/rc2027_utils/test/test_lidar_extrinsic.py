import numpy as np
import pytest

from rc2027_utils.calibration.lidar_extrinsic import (
    observability_warning,
    quaternion_to_matrix,
    solve_extrinsic,
)


def _random_rotation(rng):
    """生成一个随机旋转矩阵（QR 分解法），覆盖各种姿态，不只是绕 z 轴转。"""
    a = rng.normal(size=(3, 3))
    q, r = np.linalg.qr(a)
    q = q @ np.diag(np.sign(np.diag(r)))
    if np.linalg.det(q) < 0:
        q[:, -1] *= -1
    return q


def _yaw_rotation(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ])


def test_recovers_known_offset_with_diverse_rotation():
    """旋转足够多样（不只是绕竖直轴）时，b 和 P 应该被精确解出来。"""
    rng = np.random.default_rng(0)
    b_true = np.array([0.12, -0.05, 0.30])
    p_true = np.array([1.0, 2.0, 0.0])

    n = 30
    positions = np.empty((n, 3))
    rotations = np.empty((n, 3, 3))
    for i in range(n):
        r = _random_rotation(rng)
        rotations[i] = r
        positions[i] = p_true - r @ b_true

    b_est, p_est, singular_values = solve_extrinsic(positions, rotations)

    assert np.allclose(b_est, b_true, atol=1e-8)
    assert np.allclose(p_est, p_true, atol=1e-8)
    assert observability_warning(singular_values) is None


def test_yaw_only_spin_recovers_horizontal_offset_but_warns_on_vertical():
    """只绕竖直轴转一圈（最常见的标定动作）：水平分量能解出来，
    竖直分量解不出来——这正是模块文档里说明的已知退化情况，
    必须能被 observability_warning 检测出来，否则会被人当成可信结果用。"""
    rng = np.random.default_rng(1)
    b_true = np.array([0.12, -0.05, 0.30])
    p_true = np.array([1.0, 2.0, 0.0])

    n = 40
    thetas = np.linspace(0, 2 * np.pi, n, endpoint=False)
    positions = np.empty((n, 3))
    rotations = np.empty((n, 3, 3))
    for i, theta in enumerate(thetas):
        r = _yaw_rotation(theta)
        rotations[i] = r
        positions[i] = p_true - r @ b_true

    b_est, p_est, singular_values = solve_extrinsic(positions, rotations)

    # 水平分量（x, y）应该仍然是准的
    assert np.allclose(b_est[:2], b_true[:2], atol=1e-8)
    # 竖直分量 b_z 和 P_z 纠缠在一起（约束是 P_z = p_i_z + b_z，而 p_i_z
    # 在纯偏航旋转下对所有帧都相同），单独的 b_z 不保证等于真值，
    # 但二者之差（约束本身）必须仍然满足
    assert np.isclose(b_est[2] - p_est[2], b_true[2] - p_true[2], atol=1e-8)
    # 质量检测必须能抓到这个退化情况
    assert observability_warning(singular_values) is not None


def test_rejects_too_few_frames():
    positions = np.zeros((2, 3))
    rotations = np.stack([np.eye(3)] * 2)
    with pytest.raises(ValueError):
        solve_extrinsic(positions, rotations)


def test_observability_warning_none_when_ratio_is_healthy():
    singular_values = np.array([10.0, 9.0, 8.0, 7.0, 6.0, 5.0])
    assert observability_warning(singular_values) is None


def test_observability_warning_fires_when_ratio_is_tiny():
    singular_values = np.array([10.0, 9.0, 8.0, 7.0, 6.0, 1e-6])
    assert observability_warning(singular_values) is not None


def test_quaternion_to_matrix_identity():
    r = quaternion_to_matrix(0.0, 0.0, 0.0, 1.0)
    assert np.allclose(r, np.eye(3), atol=1e-12)


def test_quaternion_to_matrix_90deg_yaw_matches_known_rotation():
    # 绕 z 轴转 90 度的四元数: (x,y,z,w) = (0, 0, sin(45°), cos(45°))
    half = np.pi / 4
    r = quaternion_to_matrix(0.0, 0.0, np.sin(half), np.cos(half))
    expected = _yaw_rotation(np.pi / 2)
    assert np.allclose(r, expected, atol=1e-12)


def test_quaternion_to_matrix_rejects_zero_quaternion():
    with pytest.raises(ValueError):
        quaternion_to_matrix(0.0, 0.0, 0.0, 0.0)

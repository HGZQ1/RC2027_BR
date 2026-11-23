import math

import numpy as np

from rc2027_utils.sim_check.mesh_ray import field_triangles_in_world, ray_mesh_distance


def _floor(z):
    # 2 个三角形拼成 z=z 的大地板
    return np.array([
        [[-10, -10, z], [10, -10, z], [10, 10, z]],
        [[-10, -10, z], [10, 10, z], [-10, 10, z]],
    ], dtype=float)


def test_ray_hits_floor_at_expected_distance():
    # 从 1 m 高往斜下 45° 打，到 z=0 地面的距离是 sqrt(2)
    d = np.array([1.0, 0.0, -1.0]) / math.sqrt(2)
    t = ray_mesh_distance(np.array([0.0, 0.0, 1.0]), d, _floor(0.0))
    assert math.isclose(t, math.sqrt(2), rel_tol=1e-9)


def test_ray_pointing_away_misses():
    t = ray_mesh_distance(np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, 1.0]), _floor(0.0))
    assert math.isnan(t)


def test_field_pose_rotates_y_up_to_z_up_and_lifts():
    # CAD 里 Y 朝上、毫米：(0, 100, 0) mm -> world (0, 0, 0.1 + 0.025)
    tri = np.array([[[0, 100, 0], [1000, 100, 0], [0, 100, 1000]]], dtype=float)
    w = field_triangles_in_world(tri)
    assert np.allclose(w[0, 0], [0.0, 0.0, 0.125])

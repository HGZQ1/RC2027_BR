"""五色石球体拟合。合成点云，不起仿真。"""

import numpy as np

from rc2027_utils.mustika import MUSTIKA_RADIUS, find_mustika


def _rng():
    return np.random.default_rng(7)


def _hemisphere(center, view, radius=MUSTIKA_RADIUS, n=180, noise=0.004):
    """朝向雷达的那半个球面上的点。"""
    rng = _rng()
    c = np.asarray(center, float)
    direction = c - np.asarray(view, float)
    direction /= np.linalg.norm(direction)
    pts = []
    while len(pts) < n:
        p = rng.normal(size=3)
        p /= np.linalg.norm(p)
        if np.dot(p, -direction) <= 0.05:
            continue
        pts.append(c + radius * p)
    out = np.array(pts)
    out += rng.normal(scale=noise, size=out.shape)
    return out


def _face(origin, normal, half, n=180, noise=0.004):
    """一块正方形的面（方块朝向雷达的那一面）。"""
    rng = _rng()
    normal = np.asarray(normal, float)
    normal /= np.linalg.norm(normal)
    tmp = np.array([1.0, 0.0, 0.0]) if abs(normal[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(normal, tmp)
    u /= np.linalg.norm(u)
    v = np.cross(normal, u)
    s = rng.uniform(-half, half, size=(n, 2))
    pts = np.asarray(origin, float) + s[:, 0:1] * u + s[:, 1:2] * v
    pts += rng.normal(scale=noise, size=pts.shape)
    return pts


def test_partial_sphere_is_a_mustika():
    view = (-2.45, -1.70, 0.95)
    center = (-3.50, -1.30, 0.80)
    found = find_mustika(_hemisphere(center, view), view)
    assert found is not None
    assert np.allclose(found, center, atol=0.02)


def test_ground_block_face_is_not_a_sphere():
    """350 mm 地块的一个面。"""
    view = (-2.45, -1.70, 0.95)
    face = _face((-3.50, -1.30, 0.95), (1.0, 0.2, 0.0), half=0.175)
    assert find_mustika(face, view) is None


def test_sky_block_face_is_not_a_sphere():
    """200 mm 天空块和球的直径差不多，靠「是不是球面」分开，不靠尺寸。"""
    view = (-2.45, -1.70, 0.95)
    face = _face((-3.50, -1.30, 0.80), (1.0, 0.2, 0.0), half=0.10)
    assert find_mustika(face, view) is None


def test_too_few_points_is_not_a_sphere():
    view = (0.0, -1.0, 1.0)
    pts = _hemisphere((0.0, 0.0, 0.8), view)[:10]
    assert find_mustika(pts, view) is None

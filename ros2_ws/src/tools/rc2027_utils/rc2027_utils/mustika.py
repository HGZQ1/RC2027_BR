"""五色石识别：交接区里的球，不是方块。

规则场上的五色石碰撞体是半径 0.10 m 的球（field_with_objects.xml 的 mustika_col）。
雷达只看得到朝向自己的那一侧。方块检测按「几层方块」的高度来认，球对不上，会标成不认识的东西。

这里用已知半径把球心从可见表面往里推。天空块也是大约 0.20 m 见方，所以不能只看尺寸：
球面上的点到球心距离一致，方块的面对不上这个半径。
"""

import numpy as np

# field_with_objects.xml: <geom name="mustika_col" type="sphere" size="0.1" />
MUSTIKA_RADIUS = 0.10


def find_mustika(points, viewpoint, radius=MUSTIKA_RADIUS, tol=0.015, min_points=40):
    """点云里有没有一颗半径已知的球。有就返回球心 (x, y, z)，没有返回 None。

    points：这个区域里、台面以上的点，形状 (N, 3)。viewpoint：雷达在同一坐标系里的位置。
    """
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3 or len(pts) < min_points:
        return None
    view = np.asarray(viewpoint, dtype=float)
    center = _fit_center(pts, view, radius)
    if center is None:
        return None
    dist = np.linalg.norm(pts - center, axis=1)
    err = np.abs(dist - radius)
    if np.median(err) > tol or np.mean(err < tol) < 0.8:
        return None
    # 球心必须在可见表面的里面（比点更远离雷达），不然是把一块平板硬套成了球
    outward = pts.mean(axis=0) - view
    if np.dot(center - pts.mean(axis=0), outward) <= 0:
        return None
    return tuple(float(v) for v in center)


def _fit_center(pts, view, radius, steps=12):
    """可见的那一侧贴在球面上。初值把质心沿远离雷达的方向推进去，再迭代到每个点都在半径上。"""
    mean = pts.mean(axis=0)
    away = mean - view
    nrm = float(np.linalg.norm(away))
    if nrm < 1e-4:
        return None
    center = mean + away / nrm * (radius * 0.5)
    for _ in range(steps):
        diff = pts - center
        dist = np.linalg.norm(diff, axis=1)
        dist = np.maximum(dist, 1e-6)
        center = np.mean(pts - radius * diff / dist[:, None], axis=0)
    return center

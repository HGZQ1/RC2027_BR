"""仿真场地网格 + 射线求交——纯 numpy，不依赖 ROS，给 sim_lidar_range_check 用。

来源：助教 2026-10-04 的诊断脚本（notes/工具/check_sim_lidar_range.py），收进仓库时拆成
纯函数 + ROS 薄壳，算法没改（Möller–Trumbore）。
"""

import struct

import numpy as np

# Gazebo 场地 model.sdf：<pose>0 0 0.025 1.5708 0 0</pose>、<scale>0.001 ...>（CAD 毫米、Y 朝上）
FIELD_STL = ('/workspace/sim_ws/src/robocon_2027_gazebo/robocon_2027_field/'
             'meshes/robocon_2027_field.stl')


def load_binary_stl(path):
    """读 binary STL，返回 (N, 3, 3) 的三角形顶点（原始单位）。"""
    with open(path, 'rb') as f:
        b = f.read()
    n = struct.unpack('<I', b[80:84])[0]
    if len(b) < 84 + 50 * n:
        raise ValueError(f'{path} 不是完整的 binary STL')
    a = np.frombuffer(b[84:84 + n * 50],
                      dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return a['v'].reshape(-1, 3, 3).astype(float)


def field_triangles_in_world(raw):
    """按 Gazebo 场地 model.sdf 的 pose/scale 把 CAD 三角形摆到 world 系。"""
    v = raw * 0.001
    v = np.stack([v[..., 0], -v[..., 2], v[..., 1]], -1)   # 绕 x 转 +90°（Y 朝上 -> Z 朝上）
    v[..., 2] += 0.025
    return v


def ray_mesh_distance(origin, direction, tris):
    """从 origin 沿单位向量 direction 打射线，返回到最近三角形的距离，没打到返回 nan。"""
    e1 = tris[:, 1] - tris[:, 0]
    e2 = tris[:, 2] - tris[:, 0]
    pv = np.cross(direction, e2)
    det = (e1 * pv).sum(1)
    ok = np.abs(det) > 1e-12
    inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
    s = origin - tris[:, 0]
    u = (s * pv).sum(1) * inv
    qv = np.cross(s, e1)
    w = (direction * qv).sum(1) * inv
    t = (e2 * qv).sum(1) * inv
    hit = ok & (u >= 0) & (w >= 0) & (u + w <= 1) & (t > 1e-6)
    return t[hit].min() if hit.any() else np.nan


class HeightField:
    """场地表面高度查询：从上往下打射线，返回 (x, y) 处最高表面的 z。

    三角形按 xy 分桶（只保留法向不水平的），每次只和所在格子里的三角形求交，
    比全量快几十倍。爬楼梯回放生成轨迹时要查几万次。
    """

    def __init__(self, tris, cell=0.5):
        n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
        tris = tris[np.abs(n[:, 2]) > 1e-9]
        self.cell = cell
        self.buckets = {}
        lo = np.floor(tris[:, :, :2].min(axis=1) / cell).astype(int)
        hi = np.floor(tris[:, :, :2].max(axis=1) / cell).astype(int)
        for i in range(len(tris)):
            for cx in range(lo[i, 0], hi[i, 0] + 1):
                for cy in range(lo[i, 1], hi[i, 1] + 1):
                    self.buckets.setdefault((cx, cy), []).append(i)
        self.buckets = {k: tris[v] for k, v in self.buckets.items()}

    def cached(self, resolution=0.01):
        """返回一个按 resolution 取整做缓存的 height 函数（轨迹上相邻采样点大量重复）。"""
        cache = {}

        def fn(x, y):
            key = (int(round(x / resolution)), int(round(y / resolution)))
            if key not in cache:
                cache[key] = self.height(key[0] * resolution, key[1] * resolution)
            return cache[key]
        return fn

    def height(self, x, y, top=10.0):
        """(x, y) 处的表面高度；下面什么都没有返回 nan。"""
        cand = self.buckets.get((int(np.floor(x / self.cell)), int(np.floor(y / self.cell))))
        if cand is None:
            return np.nan
        t = ray_mesh_distance(np.array([x, y, top]), np.array([0.0, 0.0, -1.0]), cand)
        return top - t if np.isfinite(t) else np.nan

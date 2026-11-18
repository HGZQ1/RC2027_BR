"""从 MuJoCo 场地网格"模拟扫描"生成一份先验点云地图（map 系 = 场地中心原点）。

来源：助教 2026-10-03 写的（~/Documents/ros/notes/工具/make_prior_map.py），10-04 收进仓库，
内容没改。同目录的 prior_map_sim.pcd 就是它生成的（35744 点）。只用于仿真——真车的先验
地图要在训练场/赛场实地用 Point-LIO 建图再对齐到场地坐标，见 0003 决定 9。
需要 open3d（镜像里 pip 装了，host 上没有）。重新生成：
    python3 make_prior_map.py /workspace/sim_ws/src/rc2027_sim_bridge/maps/prior_map_sim.pcd
注意：生成时虚拟雷达高 0.35 m，跟仿真车（雷达离地约 0.23 m）不完全一样，地图覆盖的是
"从各处看得到的表面"，影响不大。

2026-10-04 改：输出整体 z + GAZEBO_Z_OFFSET（0.025）。Gazebo 场地 model.sdf 的 link pose 把
整个场地抬了 2.5 cm（地面上表面 z=0.050），MuJoCo 的 field.xml 没有这个偏移（地面 0.025）。
仿真车看的是 Gazebo，地图要跟它一致（助教查到的；之前仓库里那份 pcd 已经直接平移过了）。
x/y 两边一致，不用改。

做法：在场地上按网格摆放虚拟雷达（高度 = 脚下表面 + 0.35 m），
按 Mid-360 的视场（水平 360°，竖直 -7°～52°）发射射线，收集命中点，合并后降采样。
只用静态结构；会移动的道具（天空块、灵石）不放进地图。
"""
import glob, os, sys
import numpy as np
import open3d as o3d

MESH_DIR = '/workspace/sim_ws/src/rc2027_field_assets/meshes/field'
OUT = sys.argv[1] if len(sys.argv) > 1 else '/tmp/rc2027_prior_map.pcd'
SKIP = ('sky_blocks', 'mustika_in_place')          # 可移动道具
SENSOR_H = 0.35                                    # 雷达离脚下表面的高度
STEP = 1.0                                         # 虚拟雷达摆放间距 (m)
VOXEL = 0.05                                       # 输出降采样 (m)
GAZEBO_Z_OFFSET = 0.025                            # Gazebo 场地 model.sdf 的 z 抬高，见文件头

import xml.etree.ElementTree as ET


# 按 field.xml 里每个 geom 的 pos / quat 摆放网格（有 6 个过渡带网格带了平移和 180° 旋转）
XML = os.path.join(os.path.dirname(os.path.dirname(MESH_DIR)), 'field.xml')
root = ET.parse(XML).getroot()
mesh_file = {m.get('name'): m.get('file') for m in root.iter('mesh')}
scene = o3d.t.geometry.RaycastingScene()
used = []
for g in root.iter('geom'):
    mname = g.get('mesh')
    if not mname or mname not in mesh_file:
        continue
    f = os.path.join(os.path.dirname(MESH_DIR), mesh_file[mname])
    name = os.path.basename(f)[:-4]
    if name.startswith(SKIP):
        continue
    m = o3d.io.read_triangle_mesh(f)
    if len(m.triangles) == 0:
        continue
    pos = np.array([float(v) for v in (g.get('pos') or '0 0 0').split()])
    w, x, y, z = [float(v) for v in (g.get('quat') or '1 0 0 0').split()]   # MuJoCo 的四元数是 w x y z
    T = np.eye(4); T[:3, :3] = o3d.geometry.get_rotation_matrix_from_quaternion([w, x, y, z]); T[:3, 3] = pos
    m.transform(T)
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(m))
    used.append(name)
print(f'使用 {len(used)} 个网格')

# Mid-360 视场的射线方向（1° 分辨率）
az = np.deg2rad(np.arange(0, 360, 1.0))
el = np.deg2rad(np.arange(-7, 53, 1.0))
A, E = np.meshgrid(az, el)
dirs = np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)], -1).reshape(-1, 3)

# 虚拟雷达位置：网格点，往下打一条射线找到脚下表面
xs = np.arange(-5.0, 5.01, STEP)
grid = np.array([[x, y, 5.0] for x in xs for y in xs], dtype=np.float32)
down = np.tile([0, 0, -1], (len(grid), 1)).astype(np.float32)
hit = scene.cast_rays(o3d.core.Tensor(np.hstack([grid, down])))['t_hit'].numpy()
poses = grid.copy()
poses[:, 2] = 5.0 - hit + SENSOR_H
poses = poses[np.isfinite(hit)]
print(f'虚拟雷达位置 {len(poses)} 个，高度范围 {poses[:,2].min():.2f}~{poses[:,2].max():.2f} m')

pts = []
for p in poses:
    rays = np.hstack([np.tile(p, (len(dirs), 1)), dirs]).astype(np.float32)
    t = scene.cast_rays(o3d.core.Tensor(rays))['t_hit'].numpy()
    ok = np.isfinite(t) & (t < 40.0) & (t > 0.1)    # Mid-360 量程约 0.1～40 m
    pts.append(p + dirs[ok] * t[ok, None])
pts = np.vstack(pts)

pts[:, 2] += GAZEBO_Z_OFFSET
pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
pc = pc.voxel_down_sample(VOXEL)
o3d.io.write_point_cloud(OUT, pc)
b = np.asarray(pc.points)
print(f'原始命中 {len(pts)} 点 → 降采样后 {len(b)} 点，写入 {OUT}')
print(f'范围 x[{b[:,0].min():.2f},{b[:,0].max():.2f}] y[{b[:,1].min():.2f},{b[:,1].max():.2f}] z[{b[:,2].min():.2f},{b[:,2].max():.2f}]')

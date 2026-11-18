"""检查仿真 Mid-360 每条射线量的距离对不对（仿真验收用）。

取一帧 /livox/lidar_PointCloud2（雷达系），用 Gazebo 真值 /ground_truth/odom + TF 外参算出雷达
在 world 里的位置；对抽出来的点沿同一方向往 Gazebo 场地碰撞网格打射线得到"真实距离"，和实测
距离相减。正常应该是 0 ± 1 cm（xacro 噪声 stddev 1 cm）。

为什么要有这个（2026-10-04）：插件把每个点都画近了 10 cm（minRange 没加回去），ICP 因此把正确
位姿"修"偏 8 cm。之前的两个检查都测不出来——"先验地图 vs 场地网格"比的不是实时点云，host 单测的
扫描是从地图里截的。这个工具直接比实时点云和真实几何。助教写的诊断脚本，收进仓库。

用法（sim_mapping 在跑，车停着，车附近别有别的机器人/道具）：
    ros2 run rc2027_utils sim_lidar_range_check
"""

import time

import numpy as np
import rclpy
import tf2_ros
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from rc2027_utils.sim_check.mesh_ray import (
    FIELD_STL, field_triangles_in_world, load_binary_stl, ray_mesh_distance)

N_RAYS = 600
PASS_MEDIAN_CM = 1.0


def quat_to_rot(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Grab(Node):

    def __init__(self):
        super().__init__('sim_lidar_range_check')
        self.cloud = None
        self.truth = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(
            PointCloud2, '/livox/lidar_PointCloud2', lambda m: setattr(self, 'cloud', m), 5)
        self.create_subscription(
            Odometry, '/ground_truth/odom', lambda m: setattr(self, 'truth', m), 50)


def main(args=None):
    rclpy.init(args=args)
    node = Grab()
    t0 = time.time()
    while node.cloud is None or node.truth is None or time.time() - t0 < 3:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - t0 > 20:
            print('20 s 内没收到 /livox/lidar_PointCloud2 或 /ground_truth/odom，仿真在跑吗？')
            return
    c = node.cloud
    tf = node.tf_buffer.lookup_transform('base_footprint', c.header.frame_id, rclpy.time.Time())
    tr, q = tf.transform.translation, tf.transform.rotation
    r_bs, t_bs = quat_to_rot(q.x, q.y, q.z, q.w), np.array([tr.x, tr.y, tr.z])
    p, o = node.truth.pose.pose.position, node.truth.pose.pose.orientation
    r_wb, t_wb = quat_to_rot(o.x, o.y, o.z, o.w), np.array([p.x, p.y, p.z])

    off = {f.name: f.offset for f in c.fields}
    raw = np.frombuffer(c.data, np.uint8).reshape(-1, c.point_step)
    pts = np.stack(
        [raw[:, off[k]:off[k] + 4].copy().view(np.float32)[:, 0] for k in 'xyz'], 1)
    pts = pts[np.isfinite(pts).all(1) & (np.linalg.norm(pts, axis=1) > 0.3)].astype(float)
    world = (pts @ r_bs.T + t_bs) @ r_wb.T + t_wb
    origin = r_wb @ t_bs + t_wb
    print(f'雷达在 world: {origin.round(3)}  点数 {len(world)}')

    tris = field_triangles_in_world(load_binary_stl(FIELD_STL))
    sel = world[np.random.default_rng(0).choice(len(world), min(N_RAYS, len(world)),
                                                replace=False)]
    r = np.linalg.norm(sel - origin, axis=1)
    d = (sel - origin) / r[:, None]
    t = np.array([ray_mesh_distance(origin, x, tris) for x in d])
    ok = np.isfinite(t)
    err = r[ok] - t[ok]
    med = np.median(err) * 100
    print(f'实测距离 − 真实距离：中位 {med:+.1f} cm，25%/75% '
          f'{np.percentile(err, 25) * 100:+.1f}/{np.percentile(err, 75) * 100:+.1f} cm'
          f'（n={ok.sum()}）')
    for lo, hi in ((0, 2), (2, 4), (4, 8), (8, 20)):
        m = (t[ok] >= lo) & (t[ok] < hi)
        if m.any():
            print(f'  真实距离 {lo}~{hi} m：中位 {np.median(err[m]) * 100:+.1f} cm（{m.sum()} 条）')
    if abs(med) <= PASS_MEDIAN_CM:
        print(f'通过（|中位| <= {PASS_MEDIAN_CM} cm）')
    else:
        print(f'不通过：系统偏差 {med:+.1f} cm。固定 -10 cm = 插件没把 <range><min> 加回去'
              '（livox_laser_simulation_ros2 NOTES.md）')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

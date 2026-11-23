"""仿真里检查定位链路算得对不对：map -> base_footprint 对比 Gazebo 真值。

为什么有这个工具（0003 决定 8）：定位链路把 Point-LIO 的 IMU 位姿经过雷达外参
换算成车体位姿，再用 Start Zone 配置位姿锁定 map -> lidar_odom。外参换算写错了
（比如乘反了），雷达在车体正中间时看不出来，偏心或转了角才会露馅。Gazebo 知道
车的真实位置（rc2027_sim_bridge 的仿真 URDF 里加了 p3d 插件，发
/ground_truth/odom），直接比。

用法（先起 sim_mapping.launch.py）：
    ros2 run rc2027_utils ground_truth_check
    ros2 launch rc2027_sim_bridge sim_mapping.launch.py xacro_overrides:="lidar_x:=0.2 lidar_yaw:=0.3"

每秒打一行：定位结果、**同一时刻**的真值、误差、延迟、车速。Gazebo 世界系 =
field.xml 坐标系 = map，两者可以直接比。

**按同一时刻比（2026-10-04 改）**：第一版拿"最新的 TF"和"最新的真值"比，但 TF 的
时间戳来自 Point-LIO 的雷达帧（10 Hz + 处理耗时），天然比真值晚 0.1～0.2 s，车速
0.5 m/s 时就差出 5～10 cm，而且误差方向总是"落后于运动方向"——那是延迟，不是定位
不准（助教在 0002 指出）。现在先拿 TF 自己的时间戳 t_est，再从真值历史里插值出
t_est 时刻的真值来比；延迟（最新真值时间 - t_est）单独打印。

怎么看：
  - **停稳时**的误差才是放方块时关心的精度，应该是一两厘米
  - 运动中误差随速度变大、方向跟运动方向有关 → 还有时间没对齐的地方
  - 误差随车的朝向变化（转个身就差出雷达偏心那么多）→ 外参换算有问题
  - z：真值 base_footprint 的高度和定位的 map -> base_footprint z 之差（2026-10-04 加，上楼梯、
    L1 平台上看高度对不对）。仿真 Gazebo 场地整体比"规则坐标"高 5 cm，开局 ICP 修完以后 z 才对得上
"""

import math
from collections import deque

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
import tf2_ros
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter

from rc2027_utils.sim_check.pose_error import (
    interpolate_pose, pose_error, speed_over, yaw_rate_over)
from rc2027_utils.teach_point.pose_stats import yaw_from_quaternion


class GroundTruthCheck(Node):

    def __init__(self):
        super().__init__('ground_truth_check')
        # 这个工具只在仿真里用，直接打开仿真时间（rclpy 默认已经声明了这个参数），
        # 免得每次都要记得加 --ros-args -p use_sim_time:=true
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # 最近约 3 s 的真值（p3d 50 Hz），用来按 TF 的时间戳插值
        self.truth_history = deque(maxlen=150)
        self.truth_z = deque(maxlen=150)   # (t, z)：上楼梯 / L1 平台上要看高度对不对
        self.create_subscription(Odometry, 'ground_truth/odom', self._on_truth, 50)
        self.create_timer(1.0, self._report)

    def _on_truth(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if self.truth_history and t <= self.truth_history[-1][0]:
            return  # 仿真重置 / 乱序，丢掉
        self.truth_history.append((t, p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)))
        self.truth_z.append((t, p.z))

    def _report(self):
        if not self.truth_history:
            self.get_logger().warn('还没收到 /ground_truth/odom（仿真 URDF 里的 p3d 插件）')
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(f'查不到 {self.map_frame} -> {self.base_frame}: {exc}')
            return

        t_est = rclpy.time.Time.from_msg(tf.header.stamp).nanoseconds * 1e-9
        history = list(self.truth_history)
        truth = interpolate_pose(history, t_est)
        latency = history[-1][0] - t_est
        if truth is None:
            self.get_logger().warn(
                f'定位的时间戳 {t_est:.3f} 不在真值历史 [{history[0][0]:.3f}, '
                f'{history[-1][0]:.3f}] 里（延迟 {latency * 1000:.0f} ms），这一秒不比')
            return

        q = tf.transform.rotation
        est = (tf.transform.translation.x, tf.transform.translation.y,
               yaw_from_quaternion(q.x, q.y, q.z, q.w))
        dx, dy, dist, dyaw = pose_error(est, truth)
        zs = list(self.truth_z)
        truth_z = float(np.interp(t_est, [a for a, _ in zs], [b for _, b in zs]))
        dz = tf.transform.translation.z - truth_z
        speed = speed_over(history, 0.5)
        yaw_rate = yaw_rate_over(history, 0.5)
        # 原地转也算运动（之前只看线速度，原地转圈显示"静止"，助教指出）
        moving = (speed or 0.0) > 0.02 or (yaw_rate or 0.0) > 0.05
        self.get_logger().info(
            f'{"运动" if moving else "静止"} {speed or 0.0:.2f} m/s '
            f'{math.degrees(yaw_rate or 0.0):.0f}°/s | '
            f'定位 ({est[0]:+.3f}, {est[1]:+.3f}, {math.degrees(est[2]):+6.1f}°)  '
            f'同时刻真值 ({truth[0]:+.3f}, {truth[1]:+.3f}, {math.degrees(truth[2]):+6.1f}°) | '
            f'误差 {dist * 100:.1f} cm (dx {dx * 100:+.1f}, dy {dy * 100:+.1f}), '
            f'{math.degrees(dyaw):+.2f}° | z {truth_z:.3f} 误差 {dz * 100:+.1f} cm | '
            f'延迟 {latency * 1000:.0f} ms')


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthCheck()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

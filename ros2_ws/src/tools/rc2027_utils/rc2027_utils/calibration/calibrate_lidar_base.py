#!/usr/bin/env python3
"""雷达到底盘旋转中心的外参标定——ROS2 节点，采数据 + 调用 lidar_extrinsic 求解。

用法：卡住机器人身上某个固定点（比如一个脚轮），原地转动车身至少一整圈，
尽量转得慢一点、匀速一点，让 Point-LIO 输出足够多不同角度的位姿。
采集过程中按 Ctrl+C 结束，脚本会自动计算并打印结果。

    ros2 run rc2027_utils calibrate_lidar_base --ros-args -p odom_topic:=/aft_mapped_to_init

默认监听 Point-LIO 的 /aft_mapped_to_init（nav_msgs/Odometry，世界系 frame_id
是 lidar_odom，child_frame_id 是 body——即 IMU 旋转 patch 生效后、安装角度
已经补偿过的坐标系，细节见 external_ws/src/point_lio/NOTES.md 的验证记录）。
"""

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from rc2027_utils.calibration.lidar_extrinsic import (
    observability_warning,
    quaternion_to_matrix,
    solve_extrinsic,
)


class LidarBaseCalibrator(Node):

    def __init__(self):
        super().__init__('lidar_base_calibrator')

        self.declare_parameter('odom_topic', '/aft_mapped_to_init')
        self.declare_parameter('min_frames', 50)
        topic_name = self.get_parameter('odom_topic').value
        self.min_frames = self.get_parameter('min_frames').value

        self.positions = []
        self.rotations = []

        self.subscription = self.create_subscription(
            Odometry, topic_name, self._odom_callback, 10)

        self.get_logger().info('开始记录雷达轨迹，标定旋转中心偏移...')
        self.get_logger().info(f'正在监听话题: {topic_name}')
        self.get_logger().info('卡住车身上某个固定点，原地转动车身至少一整圈（越慢越匀速越好）')
        self.get_logger().info('采集完成后按 Ctrl+C 结束，脚本会自动计算')

    def _odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.positions.append([p.x, p.y, p.z])
        self.rotations.append(quaternion_to_matrix(q.x, q.y, q.z, q.w))

        n = len(self.positions)
        if n % 50 == 0:
            self.get_logger().info(f'已采集 {n} 帧...')

    def report(self):
        n = len(self.positions)
        print('-' * 60)
        if n < self.min_frames:
            print(f'采集到 {n} 帧，少于建议的 {self.min_frames} 帧，结果可能不可靠')
        if n < 3:
            print('帧数太少，无法求解')
            return

        positions = np.array(self.positions)
        rotations = np.array(self.rotations)
        b, p_world, singular_values = solve_extrinsic(positions, rotations)

        print('【标定结果】')
        print(f'采集帧数: {n}')
        print(f'旋转中心在雷达(body)坐标系下的坐标 b = {b}')
        print(f'旋转中心在世界坐标系下的坐标 P = {p_world}')

        warning = observability_warning(singular_values)
        if warning:
            print(f'[警告] {warning}')

        print()
        print('换算成 URDF / robot_params.yaml 里 base_link -> lidar_link 的平移：')
        print(f'  -b = {-b}')
        print('前提：body 坐标系已经和 base_link 轴向对齐（即 IMU 旋转 patch 的安装角度')
        print('配置是正确的，没有残留倾斜）——用 point_lio NOTES.md 里记录的方法先确认')
        print('body 系在启动时是否水平，再采信这里的平移值；高度分量 z 建议用卷尺实测核对，')
        print('见上面关于竖直旋转轴退化情况的警告。')
        print('-' * 60)


def main(args=None):
    rclpy.init(args=args)
    node = LidarBaseCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

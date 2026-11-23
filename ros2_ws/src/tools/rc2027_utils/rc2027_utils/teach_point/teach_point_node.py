#!/usr/bin/env python3
"""示教录点：把车放到目标位置，查当前 TF，记下来。

照搬华工 calc_mean_position.py 的交互方式（终端输入 j/k 控制采样），
数学部分（角度圆均值）原样保留，见 pose_stats.py。跟原版的区别：

- **默认查 `map -> base_link`，不是 `lidar_odom -> lidar_link`**：这个
  项目的架构（docs/decisions/0003-field-localization.md）要求"目标点
  描述的是车体中心"，不是雷达。华工自己也在 notes 里写了同样的结论
  （概念/场地原点怎么选.md）
- 多了一个 's <名字>' 命令，把算好的平均位姿存进一个 yaml 文件，不是只
  打印在终端看一眼就没了

⚠️ **这个默认配置现在还查不到值**：`map -> base_link` 这条链路还没完整
连上——`map -> lidar_odom`（field_localizer）接 `lidar_odom -> body`
（Point-LIO）接 `body -> base_link` 这最后一环还没做（2026-10-03 讨论，
见 0003"待定/下一步"里"lidar_odom 原点到底是车体还是雷达"那条，用户已经
确认延后到真正有车、要给 field_localizer 补雷达外参合成的时候一起做）。
等那个修完，这个工具不用改就能直接用。现在要先验证这个工具本身能不能跑，
可以手动传 `target_frame:=lidar_odom source_frame:=body` 改成查
Point-LIO 已经在发的那条链路（结果是雷达位姿不是车体位姿，仅用于冒烟测试）。

用法：
    ros2 run rc2027_utils teach_point
    ros2 run rc2027_utils teach_point --ros-args -p target_frame:=lidar_odom -p source_frame:=body

命令（在运行这个节点的终端里输入，回车生效）：
    j         开始采样（会清掉上一次没保存的采样）
    k         停止采样，计算并打印平均位姿
    s <名字>  把上一次 k 算出的平均位姿存进点文件（见 points_path 参数）
"""

import math
import sys
from threading import Thread

import rclpy
import tf2_ros
from rclpy.node import Node

from rc2027_utils.teach_point.pose_stats import (
    circular_mean_pose, save_point, yaw_from_quaternion)


class TeachPointNode(Node):

    def __init__(self):
        super().__init__('teach_point')

        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('source_frame', 'base_link')
        self.declare_parameter('sample_rate_hz', 10.0)
        # 2026-10-03 组长提醒：docs/rules/ 放的是"规则书翻译出来的数据"，
        # 示教点是现场实测、换场地要重录的数据，性质不一样，不该混在一起；
        # 默认按 <场地>_<红/蓝> 分文件，方便赛场上快速切换。想完全自己定
        # 路径就直接传 points_path，留空（默认）才按 site/side 自动算
        self.declare_parameter('site', 'training')
        self.declare_parameter('side', 'red')
        self.declare_parameter('points_path', '')

        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value

        points_path = self.get_parameter('points_path').value
        if not points_path:
            site = self.get_parameter('site').value
            side = self.get_parameter('side').value
            points_path = f'/workspace/docs/field_data/{site}_{side}.yaml'
        self.points_path = points_path

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.is_sampling = False
        self.samples = []
        self.last_mean_pose = None
        self._last_warn_ns = 0

        rate_hz = float(self.get_parameter('sample_rate_hz').value)
        self.create_timer(1.0 / max(rate_hz, 1e-6), self._on_timer)

        self._listener_thread = Thread(target=self._read_commands, daemon=True)
        self._listener_thread.start()

        self.get_logger().info(f'查询 TF: {self.target_frame} <- {self.source_frame}')
        self.get_logger().info(f'点文件: {self.points_path}')
        self.get_logger().info("命令: j=开始采样  k=停止并计算  s <名字>=保存上次结果")

    def _warn_throttled(self, text):
        now = self.get_clock().now().nanoseconds
        if now - self._last_warn_ns >= int(2e9):
            self._last_warn_ns = now
            self.get_logger().warn(text)

    def _on_timer(self):
        if not self.is_sampling:
            return
        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame, self.source_frame, rclpy.time.Time())
        except tf2_ros.TransformException as exc:
            self._warn_throttled(
                f'查不到 {self.target_frame} <- {self.source_frame}: {exc}')
            return

        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        theta = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.samples.append((x, y, theta))

    def _read_commands(self):
        while rclpy.ok():
            line = sys.stdin.readline()
            if not line:
                break
            self._handle_command(line.strip())

    def _handle_command(self, command):
        if command == 'j':
            self.samples = []
            self.is_sampling = True
            self.get_logger().info('开始采样...')
        elif command == 'k':
            self._stop_and_compute()
        elif command.startswith('s'):
            name = command[1:].strip()
            self._save(name)
        elif command:
            self.get_logger().warn(f'不认识的命令: {command!r}（只支持 j / k / s <名字>）')

    def _stop_and_compute(self):
        if not self.is_sampling:
            self.get_logger().warn("还没开始采样，先输入 j")
            return
        self.is_sampling = False
        n = len(self.samples)
        if n == 0:
            self.get_logger().warn('一帧数据都没采到，没法算平均')
            return

        self.last_mean_pose = circular_mean_pose(self.samples)
        x, y, theta = self.last_mean_pose
        self.get_logger().info(
            f'基于 {n} 帧: x={x:.4f} y={y:.4f} theta={theta:.4f} rad '
            f'({math.degrees(theta):.2f} deg)')
        self.get_logger().info("输入 's <名字>' 保存这个点")

    def _save(self, name):
        if not name:
            self.get_logger().warn("保存要带名字，比如 's l1_corner_red'")
            return
        if self.last_mean_pose is None:
            self.get_logger().warn('还没有算出来的位姿，先 j 采样、k 计算')
            return
        save_point(self.points_path, name, self.last_mean_pose)
        self.get_logger().info(f'已保存: {name} -> {self.points_path}')


def main(args=None):
    rclpy.init(args=args)
    node = TeachPointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

"""仿真"爬楼梯回放"：按贴地形的 3D 轨迹直接设置车的位姿和速度，测高度变化后定位还准不准。

测什么（2026-10-04 用户要求）：BR 上楼梯以后 Point-LIO 会不会发散、上了 L1 平台 ICP 还能不能
对上、**上楼以后的 Retry（L1 Retry Zone 在 0.6 m 高的平台上）能不能用**。为什么用回放、
局限是什么，见 climb_replay.py 开头。

前提：sim_mapping.launch.py 用 replay:=true 启动（车关掉重力、去掉 planar_move，不然插件会
每步把竖直速度清零、重力会把车拉下来）；world 里有 gazebo_ros_state 插件（本包 world 已加）。

用法（Point-LIO 初始化、field_localizer 锁定 + 开局 ICP 做完以后再跑）：
    ros2 run rc2027_utils sim_climb_replay                      # 红方：楼梯 -> L1 Retry Zone
    ros2 run rc2027_utils sim_climb_replay --ros-args -p side:=blue
另开终端：ros2 run rc2027_utils ground_truth_check（看 xy、yaw 和 z 的误差）
跑完车停在 L1 Retry Zone 上不动（车头朝南，L1 Retry 的摆法），可以：
    ros2 service call /field_localizer/relocalize std_srvs/srv/Trigger
    ros2 service call /field_localizer/retry rc2027_navigation_msgs/srv/Retry "{zone: l1}"
"""

import math

import numpy as np
import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter

from rc2027_utils.sim_check.climb_replay import (
    attach_terrain, build_xy_path, com_velocity, mirror_route, ROUTE_RED_STAIRS_TO_L1_RETRY,
    rpy_to_quaternion, velocities)
from rc2027_utils.sim_check.mesh_ray import (
    FIELD_STL, field_triangles_in_world, HeightField, load_binary_stl)
from rc2027_utils.teach_point.pose_stats import yaw_from_quaternion


class ClimbReplay(Node):

    def __init__(self):
        super().__init__('sim_climb_replay')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.side = self.declare_parameter('side', 'red').value
        self.entity = self.declare_parameter('entity', 'rc2027_br').value
        self.rate = self.declare_parameter('rate', 50.0).value
        # 车的质心离 base_footprint 原点的高度：按 URDF 质量算（底盘 5 kg、4 个轮子各 0.5 kg 在
        # 轮轴高度 0.05 m，雷达和相机在 0.2 m 上下）≈ 0.06 m。底盘尺寸/质量改了要跟着改
        self.com_height = self.declare_parameter('com_height', 0.06).value
        self.start = None
        self.truth = None
        self.failures = 0
        self.not_following = 0
        self.traj = None
        self.vel = None
        self.t0 = None
        self.last_report = -1
        self.create_subscription(Odometry, 'ground_truth/odom', self._on_truth, 10)
        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')

    def _on_truth(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.truth = (p.x, p.y, p.z)
        if self.start is None:
            self.start = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def _on_response(self, future):
        # 2026-10-04 用户第一次跑：Gazebo 里车没动，脚本却照常打进度——原来不看返回值。
        # set_entity_state 失败（实体名不对等）只会打在 gzserver 的终端里
        try:
            res = future.result()
        except Exception as exc:  # noqa: B902
            self.failures += 1
            reason = str(exc)
        else:
            if res.success:
                return
            self.failures += 1
            reason = f'success=False（实体名 "{self.entity}" 不对？看 gzserver 终端）'
        if self.failures in (1, 50) or self.failures % 500 == 0:
            self.get_logger().error(f'set_entity_state 失败 {self.failures} 次：{reason}')

    def prepare(self):
        route = ROUTE_RED_STAIRS_TO_L1_RETRY
        if self.side == 'blue':
            route = mirror_route(route)
        self.get_logger().info(
            f'起点（真值）({self.start[0]:.2f}, {self.start[1]:.2f}, '
            f'{math.degrees(self.start[2]):.0f}°)，按场地网格算轨迹……')
        hf = HeightField(field_triangles_in_world(load_binary_stl(FIELD_STL)))
        path = build_xy_path(self.start, route)
        self.traj = attach_terrain(path, hf.cached())
        self.vel = velocities(self.traj)
        self.get_logger().info(
            f'轨迹 {self.traj[-1, 0]:.1f} s（仿真时间）：z {self.traj[0, 3]:.2f} -> '
            f'{self.traj[-1, 3]:.2f} m，最大俯仰 {math.degrees(-self.traj[:, 5].min()):.0f}°（车头朝上）')

    def start_timer(self):
        # 仿真时钟还没收到 /clock 时 now() 是 0，t0 记成 0 的话一开始就直接跳到轨迹末尾
        while rclpy.ok() and self.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.create_timer(1.0 / self.rate, self._tick)

    def _tick(self):
        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        dt = self.traj[1, 0] - self.traj[0, 0]
        i = min(len(self.traj) - 1, max(0, int(round(t / dt))))
        _, x, y, z, roll, pitch, yaw = self.traj[i]
        v = self.vel[i] if i < len(self.traj) - 1 else np.zeros(6)

        req = SetEntityState.Request()
        state = EntityState()
        state.name = self.entity
        state.reference_frame = 'world'
        state.pose.position.x, state.pose.position.y, state.pose.position.z = x, y, z
        qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)
        state.pose.orientation.x, state.pose.orientation.y = qx, qy
        state.pose.orientation.z, state.pose.orientation.w = qz, qw
        # Gazebo 设的是质心速度，轨迹给的是 base_footprint 原点速度，要加上 ω × r
        v_com = com_velocity(v[:3], v[3:], roll, pitch, yaw, (0.0, 0.0, self.com_height))
        state.twist.linear.x, state.twist.linear.y, state.twist.linear.z = v_com
        state.twist.angular.x, state.twist.angular.y, state.twist.angular.z = v[3:]
        req.state = state
        self.client.call_async(req).add_done_callback(self._on_response)

        sec = int(t)
        if sec != self.last_report and sec % 2 == 0:
            self.last_report = sec
            done = i >= len(self.traj) - 1
            # 真值跟不上指令 = Gazebo 里车没在动（上一个周期设的位姿和真值比）
            if self.truth is not None and t > 1.0:
                gap = math.dist(self.truth, (x, y, z))
                self.not_following = self.not_following + 1 if gap > 0.2 else 0
                if self.not_following >= 2:
                    self.get_logger().error(
                        f'Gazebo 里的车没跟上回放（真值离指令 {gap:.2f} m）。检查：sim_mapping 是不是用 '
                        'replay:=true 启动的（否则 planar_move / 重力会跟回放抢）；sim_ws 改完有没有重新 '
                        'colcon build（world 里要有 gazebo_ros_state）；实体名对不对')
            self.get_logger().info(
                f't={t:5.1f}s ({x:+.2f}, {y:+.2f}) z={z:.3f} 俯仰 {math.degrees(pitch):+.0f}°'
                + ('  —— 到了，停在这里（Ctrl-C 结束；可以 relocalize / retry zone l1）'
                   if done else ''))


def main(args=None):
    rclpy.init(args=args)
    node = ClimbReplay()
    try:
        while rclpy.ok() and node.start is None:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not node.client.wait_for_service(timeout_sec=10.0):
            node.get_logger().error(
                '/gazebo/set_entity_state 不存在：world 里没有 gazebo_ros_state 插件。sim_ws 改完要 '
                'colcon build rc2027_sim_bridge，sim_mapping 默认用的 rc2027_sim.world 里才有它')
            return
        node.prepare()
        node.start_timer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

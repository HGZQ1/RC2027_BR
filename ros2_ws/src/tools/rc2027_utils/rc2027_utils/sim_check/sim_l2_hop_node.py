"""仿真里上 L2：直接把车挪到刚跨上的台沿。机构图纸来了再改成沿楼梯走。

和 L1 的 sim_climb_replay 用的是同一个办法（/gazebo/set_entity_state），只是不沿楼梯放轨迹，
一下放到落点。仿真底盘是 planar_move，自己上不了楼梯。

前提和爬楼梯回放一样：sim_mapping 用 replay:=true 启动（车关重力、去掉 planar_move，
不然重力会把车拉下来、插件会把竖直速度清零）。world 里要有 gazebo_ros_state。

用法（定位锁定以后）：
    ros2 run rc2027_utils sim_l2_hop
    ros2 run rc2027_utils sim_l2_hop --ros-args -p side:=blue
另开终端看误差：ros2 run rc2027_utils ground_truth_check
挪完是瞬移，Point-LIO 可能要再 relocalize 一次才重新咬住。
"""

import math
import time

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool

from rc2027_navigation_msgs.srv import Relatch
from rclpy.node import Node
from rclpy.parameter import Parameter

from rc2027_utils.sim_check.climb_replay import rpy_to_quaternion
from rc2027_utils.sim_check.l2_hop import footprint_z, hop_target
from rc2027_utils.sim_check.mesh_ray import (
    FIELD_STL, field_triangles_in_world, HeightField, load_binary_stl)
from rc2027_utils.teach_point.pose_stats import yaw_from_quaternion


class L2Hop(Node):

    def __init__(self):
        super().__init__('sim_l2_hop')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.side = self.declare_parameter('side', 'red').value
        self.entity = self.declare_parameter('entity', 'rc2027_br').value
        self.truth = None
        self.goal = None
        self.sent = 0
        self.create_subscription(Odometry, 'ground_truth/odom', self._on_truth, 10)
        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.floor_change = self.create_client(SetBool, '/field_localizer/floor_change')
        self.relatch = self.create_client(Relatch, '/field_localizer/relatch')
        self.surface = None

    def _on_truth(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.truth = (p.x, p.y, p.z, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def prepare(self):
        x, y, yaw = hop_target(self.side)
        hf = HeightField(field_triangles_in_world(load_binary_stl(FIELD_STL)))
        surface = hf.height(x, y)
        if not math.isfinite(surface):
            raise RuntimeError(f'落点 ({x:.2f}, {y:.2f}) 下面没有台面')
        z = footprint_z(surface)
        self.surface = surface
        self.goal = (x, y, z, yaw)
        self.get_logger().info(
            f'上 L2（直接换位，刚跨上台沿）：({x:+.2f}, {y:+.2f}) z={z:.3f} '
            f'朝向 {math.degrees(yaw):.0f}°，脚下 {surface:.3f} m')

    def widen_height_gate(self):
        """上楼：定位的高度门限从 0.10 m 放到能过一层。下楼时再调这个服务把 data 设成 false。"""
        deadline = time.monotonic() + 2.0
        while not self.floor_change.service_is_ready() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)
        if not self.floor_change.service_is_ready():
            self.get_logger().warn(
                '没有 /field_localizer/floor_change，高度门限仍是 0.10 m，重定位可能被拒')
            return
        fut = self.floor_change.call_async(SetBool.Request(data=True))
        deadline = time.monotonic() + 2.0
        while not fut.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)
        if not fut.done() or fut.result() is None or not fut.result().success:
            self.get_logger().warn('放宽高度门限没有成功')
            return
        self.get_logger().info('上楼：定位高度门限已放宽（下楼再收回）')

    def lock_at_landing(self):
        """瞬移后 Point-LIO 还在旧位置。重置里程计，按已知落点锁定，不再让 ICP 一次挪 1 m。"""
        deadline = time.monotonic() + 2.0
        while not self.relatch.service_is_ready() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)
        if not self.relatch.service_is_ready():
            self.get_logger().error('没有 /field_localizer/relatch，定位还停在换位之前')
            return
        x, y, _, yaw = self.goal
        # 网格比规则书高 5 cm。锁定用规则书台面高度，和 L1 Retry 一样。
        req = Relatch.Request()
        req.x, req.y, req.yaw = x, y, yaw
        req.z = self.surface - 0.05
        fut = self.relatch.call_async(req)
        deadline = time.monotonic() + 12.0
        while not fut.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)
        if not fut.done() or fut.result() is None or not fut.result().success:
            why = fut.result().message if fut.done() and fut.result() is not None else '超时'
            self.get_logger().error('按落点重新锁定失败：' + why)
            return
        self.get_logger().info(
            f'已按落点锁定 ({x:+.2f}, {y:+.2f}) 台面 z={req.z:.2f}。'
            'Point-LIO 断一两秒是瞬移造成的，重置后会自己恢复')

    def send_once(self):
        x, y, z, yaw = self.goal
        req = SetEntityState.Request()
        state = EntityState()
        state.name = self.entity
        state.reference_frame = 'world'
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        qx, qy, qz, qw = rpy_to_quaternion(0.0, 0.0, yaw)
        state.pose.orientation.x = qx
        state.pose.orientation.y = qy
        state.pose.orientation.z = qz
        state.pose.orientation.w = qw
        req.state = state
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or not future.result().success:
            self.get_logger().error(
                f'set_entity_state 失败（实体名 "{self.entity}" ？replay:=true 了吗？）')
            return False
        self.sent += 1
        return True

    def followed(self):
        if self.truth is None or self.goal is None:
            return False
        x, y, z, yaw = self.goal
        tx, ty, tz, tyaw = self.truth
        gap = math.dist((tx, ty, tz), (x, y, z))
        dyaw = abs(math.atan2(math.sin(tyaw - yaw), math.cos(tyaw - yaw)))
        return gap < 0.05 and dyaw < math.radians(5)


def main(args=None):
    rclpy.init(args=args)
    node = L2Hop()
    try:
        while rclpy.ok() and node.truth is None:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not node.client.wait_for_service(timeout_sec=10.0):
            node.get_logger().error(
                '/gazebo/set_entity_state 不存在。和 sim_climb_replay 一样：'
                'sim_mapping 要 replay:=true，world 里要有 gazebo_ros_state')
            return
        node.widen_height_gate()
        node.prepare()
        # 用墙钟等。use_sim_time 时 spin_once 一有旧里程计就返回，13 ms 里对的还是挪之前的位置。
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if not node.send_once():
                return
            rclpy.spin_once(node, timeout_sec=0.0)
            if node.followed():
                x, y, z, _ = node.goal
                node.get_logger().info(
                    f'车已经在 L2 台沿 ({x:+.2f}, {y:+.2f}, z={z:.3f})')
                node.lock_at_landing()
                return
            time.sleep(0.05)
        tx, ty, tz, tyaw = node.truth
        x, y, z, yaw = node.goal
        node.get_logger().error(
            f'设了位姿，3 s 后车还在 ({tx:+.2f}, {ty:+.2f}, z={tz:.3f} '
            f'{math.degrees(tyaw):.0f}°)，目标 ({x:+.2f}, {y:+.2f}, z={z:.3f} '
            f'{math.degrees(yaw):.0f}°)。replay:=true 时重力和 planar_move 应该已经关掉')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

"""重复上下 L2，再回定位观察点。独占导航/动作入口，不启动比赛规划。"""

import csv
from datetime import datetime
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rc2027_interface.msg import ActionCommand, ActionState
from rc2027_navigation_msgs.action import NavigateToPoint
from rc2027_navigation_msgs.srv import Relatch
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener
import yaml


class CycleTest(Node):
    def __init__(self):
        super().__init__('l2_cycle_test')
        defaults = dict(cycles=10, mode='sim',
                        approaches_path='/workspace/ros2_ws/src/decision/rc2027_decision/config/spot_approaches.yaml',
                        action_codes_path='/workspace/ros2_ws/src/communication/rc2027_interface/config/action_codes.yaml',
                        output_dir='/tmp/rc2027_l2_cycles', surface_offset=0.05,
                        l1_height=0.6, l2_height=0.9, dwell_seconds=5.0,
                        height_tolerance=0.08, xy_tolerance=0.05, yaw_tolerance_deg=3.0,
                        navigation_timeout=45.0, action_timeout=45.0, service_timeout=12.0,
                        startup_timeout=120.0)
        self.cfg = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        if self.cfg['mode'] not in ('sim', 'hardware') or self.cfg['cycles'] < 1:
            raise ValueError('mode 必须 sim/hardware，cycles 必须 >= 1')
        for key in ('dwell_seconds', 'height_tolerance', 'xy_tolerance',
                    'yaw_tolerance_deg', 'navigation_timeout', 'action_timeout', 'service_timeout',
                    'startup_timeout'):
            if self.cfg[key] <= 0:
                raise ValueError(key + ' 必须大于 0')
        with open(self.cfg['approaches_path'], encoding='utf-8') as f:
            self.points = yaml.safe_load(f)
        with open(self.cfg['action_codes_path'], encoding='utf-8') as f:
            self.codes = yaml.safe_load(f)['actions']
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.nav = ActionClient(self, NavigateToPoint, 'navigate_to_point')
        self.cmd = self.create_publisher(ActionCommand, 'action_cmd', 10)
        self.stop_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(ActionState, 'action_state', self.on_action, 10)
        self.health = self.create_client(Trigger, '/field_localizer/check_health')
        self.relocalize = self.create_client(Trigger, '/field_localizer/relocalize')
        self.floor = self.create_client(SetBool, '/field_localizer/floor_change')
        self.relatch = self.create_client(Relatch, '/field_localizer/relatch')
        self.seq = time.time_ns() % 256
        self.action_result = None
        self.goal_handle = None
        self.cycle = 0
        self.phase = 'prepare'
        self.expected_z = None
        self.rows = []
        self.started = time.monotonic()
        directory = Path(self.cfg['output_dir']) / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        directory.mkdir(parents=True, exist_ok=False)
        self.directory = directory
        self.trace_file = (directory / 'trace.csv').open('w', newline='', encoding='utf-8')
        self.trace = csv.writer(self.trace_file)
        self.trace.writerow(['cycle', 'phase', 'wall_elapsed', 'ros_time', 'map_x', 'map_y',
                             'map_z', 'yaw', 'expected_z', 'z_error', 'odom_z'])
        self.last_trace = 0.0
        self.max_z_error = 0.0
        self.samples = 0
        self.latest_pose = None

    def on_action(self, msg):
        if msg.cmd_seq == self.seq and msg.state in (msg.STATE_DONE, msg.STATE_FAILED):
            self.action_result = msg

    def spin(self):
        if not rclpy.ok():
            raise RuntimeError('ROS 已停止')
        rclpy.spin_once(self, timeout_sec=0.05)
        now = time.monotonic()
        if now - self.last_trace < 0.1:
            return
        self.last_trace = now
        try:
            tf = self.buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            age = (self.get_clock().now() - rclpy.time.Time.from_msg(tf.header.stamp)).nanoseconds / 1e9
            if age < -0.1 or age > 1.0:
                raise RuntimeError(f'map→base TF 不新鲜：{age:.2f}s')
            t, q = tf.transform.translation, tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
            self.latest_pose = (t.x, t.y, t.z, yaw)
            odom_z = self.buffer.lookup_transform(
                'lidar_odom', 'base_footprint', rclpy.time.Time()).transform.translation.z
        except Exception:
            self.latest_pose = None
            return
        error = '' if self.expected_z is None else t.z - self.expected_z
        self.trace.writerow([self.cycle, self.phase, now-self.started,
                             self.get_clock().now().nanoseconds / 1e9,
                             t.x, t.y, t.z, yaw, self.expected_z, error, odom_z])
        self.trace_file.flush()
        if self.expected_z is not None:
            if not math.isfinite(t.z):
                raise RuntimeError('定位高度不是有限数')
            self.samples += 1
            self.max_z_error = max(self.max_z_error, abs(error))
            if abs(error) > self.cfg['height_tolerance']:
                raise RuntimeError(f'{self.phase} 高度偏差 {error:+.3f}m')

    def wait(self, predicate, seconds, why):
        end = time.monotonic() + seconds
        while not predicate():
            if time.monotonic() >= end:
                raise RuntimeError(why + ' 超时')
            self.spin()

    def call(self, client, request):
        self.wait(client.service_is_ready, self.cfg['service_timeout'], '服务等待')
        future = client.call_async(request)
        self.wait(future.done, self.cfg['service_timeout'], '服务调用')
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError('服务失败：' + (response.message if response else '没有回应'))
        return response

    def set_phase(self, phase, floor=None):
        self.phase = phase
        self.expected_z = None if floor is None else floor + self.cfg['surface_offset']
        self.get_logger().info(f'第 {self.cycle}/{self.cfg["cycles"]} 轮：{phase}')

    def navigate(self, point, name, floor):
        self.set_phase(name, floor)
        self.wait(lambda: self.nav.server_is_ready(), self.cfg['service_timeout'], '导航服务')
        goal = NavigateToPoint.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(point['x']), float(point['y'])
        goal.pose.pose.orientation.z = math.sin(float(point['theta']) / 2)
        goal.pose.pose.orientation.w = math.cos(float(point['theta']) / 2)
        goal.xy_tolerance = self.cfg['xy_tolerance']
        goal.yaw_tolerance = math.radians(self.cfg['yaw_tolerance_deg'])
        future = self.nav.send_goal_async(goal)
        self.wait(future.done, self.cfg['service_timeout'], '导航应答')
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            raise RuntimeError('导航拒绝目标')
        result = self.goal_handle.get_result_async()
        self.wait(result.done, self.cfg['navigation_timeout'], '导航')
        outcome = result.result().result.outcome
        self.goal_handle = None
        if outcome != 'ARRIVED':
            raise RuntimeError('导航 ' + outcome)

    def action(self, name):
        self.action_result = None
        self.seq = (self.seq + 1) % 256
        self.wait(lambda: self.cmd.get_subscription_count() > 0,
                  self.cfg['service_timeout'], '动作执行器')
        self.cmd.publish(ActionCommand(action_id=int(self.codes[name]['id']), cmd_seq=self.seq))
        self.wait(lambda: self.action_result is not None, self.cfg['action_timeout'], name)
        if self.action_result.state != ActionState.STATE_DONE:
            raise RuntimeError(f'{name} 失败 err_code={self.action_result.err_code}')

    def change_floor(self, up):
        self.set_phase('climb_up' if up else 'climb_down')
        # 上下楼都先解除旧台面约束，真实模式由 ICP 确认新楼层。
        self.call(self.floor, SetBool.Request(data=True))
        self.action('CLIMB_L2' if up else 'CLIMB_DOWN')
        if self.cfg['mode'] == 'sim':
            point = self.points['l2_stair']['land' if up else 'foot']
            yaw = point['theta'] if up else self.points['transfer']['see']['theta']
            self.call(self.relatch, Relatch.Request(
                x=float(point['x']), y=float(point['y']),
                z=float(self.cfg['l2_height'] if up else self.cfg['l1_height']), yaw=float(yaw)))
        else:
            # 不用已知 x/y 覆盖真车定位；必须由实际机构动作和 ICP 验收。
            self.call(self.relocalize, Trigger.Request())
        self.call(self.floor, SetBool.Request(data=False))
        # 接收锁定后新发布的 TF；这段仍记 trace，但不拿旧楼层缓存判新楼层。
        end = time.monotonic() + 0.3
        while time.monotonic() < end:
            self.spin()

    def observe(self, name, floor, point=None):
        self.set_phase(name, floor)
        begin_samples = self.samples
        end = time.monotonic() + self.cfg['dwell_seconds']
        while time.monotonic() < end:
            self.spin()
            if point is not None and self.latest_pose is not None:
                x, y, _, yaw = self.latest_pose
                xy = math.hypot(x - point['x'], y - point['y'])
                dyaw = abs(math.atan2(math.sin(yaw-point['theta']), math.cos(yaw-point['theta'])))
                if xy > self.cfg['xy_tolerance'] * 2 or dyaw > math.radians(self.cfg['yaw_tolerance_deg']) * 2:
                    raise RuntimeError(f'{name} 停稳后位置或朝向继续漂移')
        if self.samples - begin_samples < max(3, int(self.cfg['dwell_seconds'] * 3)):
            raise RuntimeError(name + ' 有效 TF 样本不足')
        self.call(self.health, Trigger.Request())

    def run(self):
        end = time.monotonic() + self.cfg['startup_timeout']
        next_note = 0.0
        while not self.buffer.can_transform('map', 'base_footprint', rclpy.time.Time()):
            now = time.monotonic()
            nodes = sorted({name for name, _ in self.get_node_names_and_namespaces()})
            if now >= end:
                raise RuntimeError(
                    '初始定位超时：没有 map→base_footprint；可见节点=' + ','.join(nodes)
                    + '。本命令只执行测试，不启动仿真/RViz；请用 run_l2_cycle_sim.sh，'
                    '并查看同批 sim.log。还需确认 ROS_DOMAIN_ID 一致。')
            if now >= next_note:
                self.get_logger().info(
                    '等待初始定位（最多 ' + str(self.cfg['startup_timeout'])
                    + 's），可见节点：' + ','.join(nodes))
                next_note = now + 5.0
            self.spin()
        if any(name == 'flow_runner' for name, _ in self.get_node_names_and_namespaces()):
            raise RuntimeError('请先关闭 flow_runner，避免两个控制器同时发导航/动作')
        stair, see = self.points['l2_stair'], self.points['transfer']['see']
        l1, l2 = self.cfg['l1_height'], self.cfg['l2_height']
        # 开始时必须已在 L1 西侧安全区域；不自动跨场绕障。
        self.navigate(see, 'initial_see', l1)
        for self.cycle in range(1, self.cfg['cycles'] + 1):
            started = time.monotonic()
            self.max_z_error = 0.0
            self.navigate(dict(see, theta=stair['foot']['theta']), 'face_stairs', l1)
            self.navigate(stair['foot'], 'stair_foot', l1)
            self.change_floor(True)
            self.observe('l2_after_lock', l2)
            self.navigate(stair['land'], 'return_to_edge', l2)
            self.change_floor(False)
            self.observe('l1_after_lock', l1)
            self.navigate(see, 'return_to_see', l1)
            self.observe('see_before_icp', l1, see)
            self.set_phase('see_icp', l1)
            self.call(self.relocalize, Trigger.Request())
            self.observe('see_after_icp', l1, see)
            self.rows.append(dict(cycle=self.cycle, passed=True,
                                  seconds=time.monotonic()-started, max_z_error=self.max_z_error))
            self.get_logger().info(f'第 {self.cycle} 轮通过，最大高度偏差 {self.max_z_error*100:.1f}cm')

    def save(self, error=None):
        if error:
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self.stop_pub.publish(Twist())
            # 不自动 Retry/再次上下楼，保留故障现场。
            self.rows.append(dict(cycle=self.cycle, passed=False, phase=self.phase, error=error,
                                  max_z_error=self.max_z_error))
        with (self.directory / 'summary.json').open('w', encoding='utf-8') as f:
            json.dump(dict(parameters=self.cfg, results=self.rows, error=error),
                      f, ensure_ascii=False, indent=2)
        self.trace_file.close()
        self.get_logger().info('测试记录：' + str(self.directory))


def main(args=None):
    rclpy.init(args=args)
    node = None
    error = None
    try:
        node = CycleTest()
        node.run()
    except (Exception, KeyboardInterrupt) as exc:
        error = str(exc) or '用户中断'
        if node:
            node.get_logger().error(error)
        else:
            print(error)
    finally:
        if node:
            node.save(error)
            node.destroy_node()
        rclpy.try_shutdown()
    if error:
        raise SystemExit(1)

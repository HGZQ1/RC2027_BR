"""field_state：世界模型的 ROS 外壳，发 /field_state。

合并逻辑在 world/field_model.py（有单测），这里只负责喂数据、发结果：
    /block_detector/regions   block_detector 定时扫（每秒一次）和 DETECT / VERIFY 服务的结果都从这里来
    /action_cmd + /action_state
                              PICK n 做完：车上多 n 块；PLACE L 做完：按那一刻车的位姿 + gripper_offset
                              算夹爪作用点落在哪个建造点上，记"第 L 层起是我们放的、应该一共几块"
    /field_localizer/retry_status
                              RESUMED：手里的清零（场上的塔不变）
    TF map → base_footprint   车跟前的建造点不看（被挡 / 夹爪举着方块），PLACE 时算夹爪在哪

发布：/field_state（rc2027_perception_msgs/FieldState，transient local），publish_rate Hz；
结论变了打一行日志。
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from rc2027_interface.msg import ActionCommand, ActionState
from rc2027_perception_msgs.msg import FieldState, RegionStateArray, SpotState
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from .flow.pick_pose import Block, PickParams
from .world import FieldModel, ModelParams, Observation, PRIOR, SKY_OTHERS, SKY_OURS

KIND_NAMES = {0: 'ground', 1: 'sky', 2: 'unknown'}
SKY_CODES = {SKY_OURS: SpotState.SKY_OURS, SKY_OTHERS: SpotState.SKY_OTHERS}


def stamp_seconds(stamp):
    """builtin_interfaces/Time → 秒。"""
    return stamp.sec + stamp.nanosec * 1e-9


def to_observation(state):
    """RegionState → Observation。"""
    blocks = tuple(Block(KIND_NAMES.get(b.type, 'unknown'), b.x, b.y, b.yaw, b.layer,
                         b.center_reliable) for b in state.blocks)
    return Observation(state.region, stamp_seconds(state.stamp), state.visible, state.coverage,
                       state.ground_layers, state.has_sky, state.top_maybe_clipped, blocks,
                       state.note)


class FieldStateNode(Node):
    """喂 FieldModel、发 /field_state。"""

    def __init__(self):
        """读场地配置和动作码表，建好接口。"""
        super().__init__('field_state')
        geometry_path = self.declare_parameter(
            'field_geometry_path', '/workspace/docs/rules/field_geometry.yaml').value
        codes_path = self.declare_parameter('action_codes_path', '').value
        if not codes_path:
            codes_path = os.path.join(get_package_share_directory('rc2027_interface'),
                                      'config', 'action_codes.yaml')
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        # 跟 flow_runner / PickParams 是同一个数（15 cm 间隙，2026-10-07；以后从 URDF 的 gripper_link 算）
        self.gripper_offset = self.declare_parameter(
            'gripper_offset', PickParams().gripper_offset).value
        d = ModelParams()
        params = ModelParams(
            confirm_up=self.declare_parameter('confirm_up', d.confirm_up).value,
            confirm_down=self.declare_parameter('confirm_down', d.confirm_down).value,
            min_empty_coverage=self.declare_parameter(
                'min_empty_coverage', d.min_empty_coverage).value,
            self_ignore_radius=self.declare_parameter(
                'self_ignore_radius', d.self_ignore_radius).value,
            place_match_radius=self.declare_parameter(
                'place_match_radius', d.place_match_radius).value,
            clear_min_dist=self.declare_parameter('clear_min_dist', d.clear_min_dist).value,
            clear_max_dist=self.declare_parameter('clear_max_dist', d.clear_max_dist).value)
        rate = self.declare_parameter('publish_rate', 2.0).value

        with open(geometry_path, 'r', encoding='utf-8') as f:
            spots = {str(k): (float(v[0]), float(v[1]))
                     for k, v in yaml.safe_load(f)['building_spots'].items()}
        with open(codes_path, 'r', encoding='utf-8') as f:
            self.action_names = {int(v['id']): str(k).upper()
                                 for k, v in yaml.safe_load(f)['actions'].items()}
        self.start_time = self.now()
        self.model = FieldModel(spots, self.start_time, params)
        self.pending = {}   # cmd_seq → (动作名, 参数)，等 DONE
        self.transfer_msgs = {}   # 交接区最近一次看到的 RegionState，原样转发

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(RegionStateArray, 'block_detector/regions', self.on_regions, 10)
        self.create_subscription(ActionCommand, 'action_cmd', self.on_action_cmd, 10)
        self.create_subscription(ActionState, 'action_state', self.on_action_state, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'field_localizer/retry_status', self.on_retry_status,
                                 latched)
        self.pub = self.create_publisher(FieldState, 'field_state', latched)
        self.create_timer(1.0 / rate, self.publish)
        self.get_logger().info(f'{len(spots)} 个建造点（{geometry_path}），开局都当空的')

    def now(self):
        """ROS 时间（秒）。"""
        return self.get_clock().now().nanoseconds * 1e-9

    def robot_pose(self):
        """车在 map 里的 (x, y, yaw)，查不到返回 None。"""
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    # ── 输入 ──
    def on_regions(self, msg):
        """block_detector 看到的一批区域。"""
        pose = self.robot_pose()
        robot_xy = None if pose is None else pose[:2]
        robot_yaw = None if pose is None else pose[2]
        for state in msg.regions:
            if state.region not in self.model.spots:
                if state.visible:
                    self.transfer_msgs[state.region] = state
                continue
            change = self.model.observe(to_observation(state), robot_xy, robot_yaw)
            if change:
                self.get_logger().info(change)
                self.publish()

    def on_action_cmd(self, msg):
        """记下发出去的动作，等它 DONE。"""
        name = self.action_names.get(msg.action_id)
        if name in ('PICK', 'PLACE', 'FLIP_SKY'):
            self.pending[msg.cmd_seq] = (name, msg.param)
        else:
            self.pending.pop(msg.cmd_seq, None)

    def on_action_state(self, msg):
        """PICK / PLACE 做完了才算（FAILED 不记）。同一个 cmd_seq 只处理一次。"""
        if msg.state not in (ActionState.STATE_DONE, ActionState.STATE_FAILED):
            return
        item = self.pending.pop(msg.cmd_seq, None)
        if item is None or msg.state != ActionState.STATE_DONE:
            return
        name, param = item
        if name == 'PICK':
            self.model.on_pick(param)
            self.get_logger().info(f'PICK {param} 做完，车上 {self.model.held} 块')
            return
        pose = self.robot_pose()
        if name == 'FLIP_SKY':
            if pose is None:
                self.get_logger().warn('FLIP 做完，但查不到车的位姿，不记')
                return
            x, y, yaw = pose
            gripper = (x + self.gripper_offset * math.cos(yaw),
                       y + self.gripper_offset * math.sin(yaw))
            _, message = self.model.on_flip(gripper, self.now())
            self.get_logger().info(message)
            self.publish()
            return
        if pose is None:
            self.get_logger().warn(f'PLACE {param} 做完，但查不到车的位姿，不知道放在哪，不记')
            self.model.on_place(param, (math.inf, math.inf), self.now())
            return
        x, y, yaw = pose
        gripper = (x + self.gripper_offset * math.cos(yaw),
                   y + self.gripper_offset * math.sin(yaw))
        _, message = self.model.on_place(param, gripper, self.now())
        self.get_logger().info(message)
        self.publish()

    def on_retry_status(self, msg):
        """Retry 成功：手里的不算了。"""
        if msg.data.split()[:1] == ['RESUMED']:
            self.model.on_retry()

    # ── 输出 ──
    def publish(self):
        """/field_state。"""
        now = self.now()
        out = FieldState()
        out.stamp = self.get_clock().now().to_msg()
        out.held = min(255, self.model.held)
        for s in self.model.spot_list():
            m = SpotState()
            m.region = s.name
            m.x, m.y = s.x, s.y
            m.ground_layers = s.ground
            m.has_sky = s.sky
            m.source = SpotState.SOURCE_PRIOR if s.source == PRIOR else SpotState.SOURCE_SEEN
            m.confirmations = min(65535, s.confirmations)
            m.age = max(0.0, now - s.since)
            m.sky_owner = SKY_CODES.get(s.sky_owner, SpotState.SKY_NONE)
            m.ours_mask = sum(1 << (layer - 1) for layer in s.ours if 1 <= layer <= 8)
            m.expected_total = s.expected_total
            m.obstructed = s.obstructed_at is not None and now - s.obstructed_at < 5.0
            m.base_known = s.base is not None
            if s.base is not None:
                m.base_x, m.base_y, m.base_yaw = s.base
            m.note = self.model.note(s, now)
            out.spots.append(m)
        out.transfer = [self.transfer_msgs[name] for name in sorted(self.transfer_msgs)]
        self.pub.publish(out)


def main(args=None):
    """入口。"""
    rclpy.init(args=args)
    try:
        node = FieldStateNode()
    except (OSError, KeyError, TypeError) as exc:
        rclpy.logging.get_logger('field_state').fatal(str(exc))
        rclpy.try_shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

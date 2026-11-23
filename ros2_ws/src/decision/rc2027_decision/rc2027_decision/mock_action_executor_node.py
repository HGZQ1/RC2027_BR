"""mock_action_executor：假装电控执行动作组，给 flow_runner 回 ActionState。

串口业务消息还没定（0004 草稿等电控确认），先在 ROS 话题这一层假装：
    收到 /action_cmd（ActionCommand）→ 立刻回 RUNNING，执行中 10 Hz 重复（跟 0004 约定的一样）
    → 按动作码表里的 duration × time_scale 等够 → 回 DONE（或者 fail_actions 里的动作回 FAILED）
同一个 cmd_seq 重复收到只当一次（0004 "电控必做清单"第 2 条：按 cmd_seq 去重）。
新命令到了旧的还没做完：旧的直接丢掉（真电控会怎样要跟电控约定）。

参数：
    action_codes_path  动作码表（默认 rc2027_interface/config/action_codes.yaml）
    time_scale         duration 乘的系数，0 = 立刻完成（只测流程逻辑时用）
    fail_actions       这些动作名回 FAILED（测失败分支）
"""

import os

from ament_index_python.packages import get_package_share_directory
from rc2027_interface.msg import ActionCommand, ActionState
import rclpy
from rclpy.node import Node
import yaml


class MockActionExecutor(Node):
    """假电控。"""

    def __init__(self):
        """读动作码表，建话题。"""
        super().__init__('mock_action_executor')
        codes_path = self.declare_parameter('action_codes_path', '').value
        if not codes_path:
            codes_path = os.path.join(get_package_share_directory('rc2027_interface'),
                                      'config', 'action_codes.yaml')
        self.time_scale = float(self.declare_parameter('time_scale', 1.0).value)
        fail = self.declare_parameter('fail_actions', ['']).value
        self.fail_actions = {name.upper() for name in fail if name}
        with open(codes_path, 'r', encoding='utf-8') as f:
            table = yaml.safe_load(f)['actions']
        self.by_id = {int(v['id']): (str(name).upper(), float(v.get('duration', 1.0)))
                      for name, v in table.items()}
        self.pub = self.create_publisher(ActionState, 'action_state', 10)
        self.create_subscription(ActionCommand, 'action_cmd', self.on_cmd, 10)
        self.create_timer(0.1, self.on_timer)
        self.current = None        # (cmd_seq, 名字, 结束时间)
        self.last_seq = None
        self.get_logger().info(
            f'假电控：{len(self.by_id)} 个动作，time_scale={self.time_scale:g}，'
            f'会失败的: {sorted(self.fail_actions) or "无"}')

    def now(self):
        """ROS 时间（秒）。"""
        return self.get_clock().now().nanoseconds * 1e-9

    def send(self, seq, state, err=0):
        """发一条 ActionState。"""
        self.pub.publish(ActionState(cmd_seq=seq, state=state, err_code=err))

    def on_cmd(self, msg):
        """新动作。"""
        if msg.cmd_seq == self.last_seq:
            return
        self.last_seq = msg.cmd_seq
        if msg.action_id not in self.by_id:
            self.get_logger().error(f'不认识的 action_id {msg.action_id}')
            self.send(msg.cmd_seq, ActionState.STATE_FAILED, err=255)
            return
        name, duration = self.by_id[msg.action_id]
        if self.current is not None:
            self.get_logger().warn(f'动作 {self.current[1]} 没做完就来了新命令，丢掉')
        self.get_logger().info(f'执行 {name} param={msg.param}（seq {msg.cmd_seq}，'
                               f'{duration * self.time_scale:g} s）')
        self.current = (msg.cmd_seq, name, self.now() + duration * self.time_scale)
        self.send(msg.cmd_seq, ActionState.STATE_RUNNING)

    def on_timer(self):
        """执行中 10 Hz 报 RUNNING，到时间报 DONE / FAILED。"""
        if self.current is None:
            return
        seq, name, end = self.current
        if self.now() < end:
            self.send(seq, ActionState.STATE_RUNNING)
            return
        if name in self.fail_actions:
            self.get_logger().warn(f'{name} 假装失败')
            self.send(seq, ActionState.STATE_FAILED, err=1)
        else:
            self.send(seq, ActionState.STATE_DONE)
        self.current = None


def main(args=None):
    """入口。"""
    rclpy.init(args=args)
    node = MockActionExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

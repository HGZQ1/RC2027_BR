"""sim_tower：搭塔仿真用的"假电控 + 假 TR"，PICK / PLACE 时真的挪 Gazebo 里的方块。

替代 rc2027_decision 的 mock_action_executor（那个只回 DONE、不动方块）。flow.launch.py 用 mock_actions:=false 关掉它。

- 开局：删掉上次的 blk_xxx，按场景文件（trips）摆第 1 趟货到交接区
- 收到 /action_cmd：立刻回 RUNNING，按动作码表的时长（× time_scale）以后：
    PICK n   夹爪前面那一摞最上面的 n 块"拿走"（从 Gazebo 里删掉；车上货仓的样子用 sim_cargo 另外模拟）
    PLACE L  放到夹爪前面的第 L 层（1 = 台面上），位置按**车的真值位姿**算、加 1 cm / 1° 的随机误差——
             定位偏多少方块就真的放偏多少；放的高度跟那里已有的层数对不上 → 回 FAILED（真车会砸）。
             放下 = 在新位置重新 spawn：用 set_entity_state 挪 static 模型时碰撞体（雷达）跟着动了，但 Gazebo 画面不刷新
             （2026-10-05 用户：仿真画面里方块不动、只有 RViz 变了）
    FLIP_SKY 翻夹爪前面最上面的天空块，朝上的一面红蓝对调（删掉再按新颜色摆）
    CLIMB_L2 不爬楼梯，把车换到 L2 台沿（规划器里这一步接在 L1 动作后面）
    CLIMB_DOWN 不开下台阶，把车换回 L1 楼梯口
    其它动作  直接 DONE
- 交接区被拿空、还有下一趟 → 等 BR 离开交接区（车体中心离交接区中心 > tr_clear_dist）再等 tr_delay 秒摆下一趟
  （模拟 TR 送货；BR 还在交接区里就摆的话方块会摆进车身里）。观察位在交接区东边 1.1 m 处
  （2026-10-07），所以这个距离要小于观察位、大于"方块会摆进车身"
- 每次 PLACE 打一张表：那个建造点每层的类型、中心偏差、投影离 500x500 边的余量（规则图 7：每层投影都要在里面）；
  货全放完时打总表和总用时（第一个动作到最后一次 PLACE）

用法见 rc2027_decision/config/flows/two_towers_sim.yaml 开头。
"""

import math
import os
import random

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import DeleteEntity, SetEntityState, SpawnEntity
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rc2027_interface.msg import ActionCommand, ActionState
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import yaml

from rc2027_utils.sim_check.block_scene import block_sdf, regions_from_geometry
from rc2027_utils.sim_check.climb_replay import rpy_to_quaternion
from rc2027_utils.sim_check.l2_hop import footprint_z, hop_target
from rc2027_utils.sim_check.mesh_ray import (
    FIELD_STL, HeightField, field_triangles_in_world, load_binary_stl)
from rc2027_utils.sim_check.tower_sim import (
    L1_SHARED, blue_next, find_spot, random_field_blocks, supply_trip, tower_kinds,
    tower_report, TowerWorld, trip_blocks)
from rc2027_utils.teach_point.pose_stats import yaw_from_quaternion

DEFAULT_SCENE = '/workspace/sim_ws/src/rc2027_sim_bridge/config/block_scenes/towers_plan2.yaml'


class SimTower(Node):

    def __init__(self):
        super().__init__('sim_tower')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        scene_path = self.declare_parameter('scene', DEFAULT_SCENE).value
        geometry_path = self.declare_parameter(
            'field_geometry_path', '/workspace/docs/rules/field_geometry.yaml').value
        codes_path = self.declare_parameter('action_codes_path', '').value or os.path.join(
            get_package_share_directory('rc2027_interface'), 'config', 'action_codes.yaml')
        self.time_scale = float(self.declare_parameter('time_scale', 1.0).value)
        self.tr_delay = float(self.declare_parameter('tr_delay', 2.0).value)
        # 1.0 m：新观察位 transfer_look 离交接区中心只有 1.12 m（旧的 1.5 m 时，拿空后车去近点、
        # 再回观察位，全程都 < 1.5 m，第二趟永远不送）。1.0 m 时车身还碰不到交接区里的方块
        # 0.7 m：观察位离交接区中心 1.12 m。旧的 1.0 m 时车停在观察位往往刚过线，
        # 第二趟来不及刷出来，流程就因找不到天空退出了。
        self.tr_clear_dist = float(self.declare_parameter('tr_clear_dist', 0.7).value)
        self.tr_pending = False
        self.sigma_xy = float(self.declare_parameter('place_sigma', 0.01).value)
        self.sigma_yaw = float(self.declare_parameter('place_yaw_sigma_deg', 1.0).value)
        self.transfer = self.declare_parameter('transfer_region', 'transfer_red').value
        seed = int(self.declare_parameter('seed', 0).value)
        if seed == 0:
            seed = random.SystemRandom().randrange(1, 10 ** 9)
        self.field_seed = seed
        self.random_field = bool(self.declare_parameter('random_field', False).value)
        self.rng = random.Random(seed)
        self.get_logger().info(f'随机种子 {seed}')

        with open(scene_path, encoding='utf-8') as f:
            self.scene = yaml.safe_load(f)
        with open(geometry_path, encoding='utf-8') as f:
            regions = regions_from_geometry(yaml.safe_load(f))
        with open(codes_path, encoding='utf-8') as f:
            table = yaml.safe_load(f)['actions']
        self.actions = {int(v['id']): (str(k).upper(), float(v.get('duration', 1.0)))
                        for k, v in table.items()}
        self.world = TowerWorld(
            regions=regions, z_offset=float(self.scene.get('z_offset', 0.05)),
            gripper_offset=float(self.declare_parameter('gripper_offset', 0.575).value))
        self.get_logger().info(
            f'夹爪偏移 {self.world.gripper_offset:.3f} m（要和 flow_runner 的 gripper_offset 一样，'
            '不然方块会放偏一个这个差值）')
        self.trips = list(self.scene['trips'])
        self.next_trip = 0
        self.extra_trips = 0
        self.supply_lock = False
        self.n_blocks = 0
        self.tr_at = None            # 下一趟货什么时候摆（sim 时间）
        self.robot = None            # 真值 (x, y, yaw)
        self.current = None          # (seq, 名字, param, 结束时间)
        self.last_seq = None
        self.t_first = None
        self.t_last_place = None
        self.placed_spots = []
        self.gz_q = []               # ('del', 名字, 第几次) / ('spawn', 方块, 第几次)，一个做完再做下一个
        self.gz_future = None
        self.gz_what = None
        self.tr_wait_logged = False

        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.pose_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.climb_goal = None
        self.climb_since = None
        self._height = None
        self.pub = self.create_publisher(ActionState, 'action_state', 10)
        self.create_subscription(ActionCommand, 'action_cmd', self.on_cmd, 10)
        self.create_subscription(Odometry, 'ground_truth/odom', self.on_truth, 10)
        self.create_subscription(String, 'flow_runner/status', self.on_flow, 10)
        self.blue_plans = set()
        self.create_timer(0.1, self.on_timer)

    # ── 开局 ──
    def call_sync(self, client, req, timeout):
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()

    def clear_old(self, n=40):
        """删上次的 blk_xxx（删不存在的有时不返回：3 s 超时当没有，连续 3 次就不删了）。"""
        if not self.delete_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('/delete_entity 不在（Gazebo 没起？）')
        misses = 0
        for i in range(n):
            req = DeleteEntity.Request(name=f'blk_{i:03d}')
            res = self.call_sync(self.delete_client, req, 3.0)
            misses = misses + 1 if res is None else 0
            if misses >= 3:
                break

    def spawn_field(self):
        """开局只在 L1 共享区摆蓝方的塔：一层蓝 / 两层蓝 / 全蓝。自己的塔是空的。"""
        blocks, lines = random_field_blocks(
            self.rng, self.world.regions, self.world.z_offset, self.n_blocks)
        self.n_blocks += len(blocks)
        for b in blocks:
            self.world.add(b)
            self.spawn_block(b)
        self.get_logger().info(
            f'随机场（种子 {self.field_seed}）：\n  ' + '\n  '.join(lines))

    def spawn_trip(self):
        """摆下一趟货。"""
        trip = self.trips[self.next_trip]
        self.next_trip += 1
        blocks = trip_blocks(trip, self.world.regions, self.world.z_offset, self.n_blocks)
        self.n_blocks += len(blocks)
        for b in blocks:
            self.world.add(b)
            self.spawn_block(b)
        self.get_logger().info(
            f'TR 第 {self.next_trip} 趟送到 {self.transfer}：' +
            '、'.join(f'{b.kind}({b.x:.2f}, {b.y:.2f})' for b in blocks))

    def spawn_block(self, b):
        """排队摆。一次塞好几个 spawn，Gazebo 会丢掉其中几个，画面上就少方块。"""
        self.gz_q.append(('spawn', b, 0))

    def pump_gazebo(self):
        """删 / 摆一个一个来，失败再试两次。"""
        if self.gz_future is not None:
            if not self.gz_future.done():
                return
            kind, payload = self.gz_what
            res = self.gz_future.result()
            self.gz_future = None
            if kind == 'spawn':
                b, attempt = payload
                if res is None or not res.success:
                    why = '没回应' if res is None else res.status_message
                    if attempt < 2:
                        self.get_logger().warn(f'{b.name} 没摆上（{why}），再试')
                        self.gz_q.append(('spawn', b, attempt + 1))
                    else:
                        self.get_logger().error(f'{b.name} 摆了 3 次还是失败：{why}')
            elif kind == 'respawn':
                block, attempt = payload
                ok = res is not None and res.success
                if not ok:
                    if attempt < 2:
                        self.get_logger().warn(f'{block.name} 翻面先删旧的没成功，再试')
                        self.gz_q.append(('respawn', block, attempt + 1))
                    else:
                        self.get_logger().error(f'{block.name} 翻面删了 3 次还在，新的没摆')
                else:
                    # 旧的确认删掉再摆。删的重试不能排在新方块后面，否则红的闪一下就没了。
                    self.gz_q.insert(0, ('spawn', block, 0))
            elif res is None:
                name, attempt = payload
                if attempt < 2:
                    self.gz_q.append(('del', name, attempt + 1))
        if not self.gz_q or self.gz_future is not None:
            return
        kind, payload, attempt = self.gz_q.pop(0)
        self.gz_what = (kind, (payload, attempt))
        if kind == 'del':
            self.gz_future = self.delete_client.call_async(DeleteEntity.Request(name=payload))
            return
        if kind == 'respawn':
            self.gz_future = self.delete_client.call_async(DeleteEntity.Request(name=payload.name))
            return
        b = payload
        req = SpawnEntity.Request()
        req.name = b.name
        req.xml = block_sdf({'name': b.name, 'type': b.kind, 'size': b.size, 'face': b.face})
        req.initial_pose = self.pose_msg(b.x, b.y, b.z_bottom + b.size / 2, b.yaw)
        self.gz_future = self.spawn_client.call_async(req)

    @staticmethod
    def pose_msg(x, y, z, yaw):
        p = Pose()
        p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
        p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return p

    # ── 回调 ──
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def targets_need_blocks(self):
        """全场目标仍缺方块时继续供货；已封顶待翻的塔不用补货。"""
        return any(tower_kinds(self.world, spot) != ['ground', 'ground', 'sky']
                   for spot in ('spot_07', 'spot_08', 'spot_05', *L1_SHARED))

    def offer_supply(self):
        """交接区空了、脚本里的趟也送完了，全场目标仍缺块就再补一趟同样的货。"""
        if self.supply_lock or self.tr_pending or self.tr_at is not None:
            return
        if self.next_trip < len(self.trips) or not self.world.region_empty(self.transfer):
            return
        if not self.targets_need_blocks() or self.extra_trips >= 8:
            return
        self.trips.append(supply_trip(self.extra_trips))
        self.extra_trips += 1
        self.supply_lock = True
        self.get_logger().info(f'全场目标还缺块，补第 {len(self.trips)} 趟')
        self.spawn_trip()

    def on_flow(self, msg):
        """我们正要去碰一座还没封顶的共享塔时，蓝方先盖下一块，用来测开到跟前改计划。"""
        if not msg.data.startswith('RUNNING'):
            return
        text = msg.data.split('#', 1)[0]
        if '等货' in text:
            self.offer_supply()
        if not self.random_field:
            return
        if text in self.blue_plans or ('放' not in text and '翻' not in text):
            return
        for spot in L1_SHARED:
            if spot not in text:
                continue
            self.blue_plans.add(text)
            block = blue_next(self.world.blocks, self.world.regions, spot, self.n_blocks)
            if block is None:
                return
            self.n_blocks += 1
            self.world.add(block)
            self.spawn_block(block)
            what = '蓝天空' if block.kind == 'sky' else '一块蓝地'
            self.get_logger().info(
                f'蓝方自动放 {spot}：{what} {block.name} 在 ({block.x:.2f}, {block.y:.2f})')
            return

    def on_truth(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.robot = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        self.robot_z = p.z

    def send(self, seq, state, err=0):
        self.pub.publish(ActionState(cmd_seq=seq, state=state, err_code=err))

    def on_cmd(self, msg):
        if msg.cmd_seq == self.last_seq:
            return
        self.last_seq = msg.cmd_seq
        name, duration = self.actions.get(msg.action_id, (f'#{msg.action_id}', 1.0))
        if self.t_first is None:
            self.t_first = self.now()
        self.current = (msg.cmd_seq, name, msg.param, self.now() + duration * self.time_scale)
        self.send(msg.cmd_seq, ActionState.STATE_RUNNING)

    def on_timer(self):
        self.pump_gazebo()
        if self.tr_pending and self.robot is not None:
            cx, cy, _ = self.world.regions[self.transfer]
            dist = math.hypot(self.robot[0] - cx, self.robot[1] - cy)
            if dist > self.tr_clear_dist:
                self.tr_pending = False
                self.tr_wait_logged = False
                self.tr_at = self.now() + self.tr_delay
                self.get_logger().info(
                    f'车离交接区中心 {dist:.2f} m，{self.tr_delay:.0f} s 后送下一趟')
            elif not self.tr_wait_logged:
                self.tr_wait_logged = True
                self.get_logger().info(
                    f'下一趟先不送：车离交接区中心 {dist:.2f} m，要超过 {self.tr_clear_dist:.2f} m')
        if self.tr_at is not None and self.now() >= self.tr_at:
            self.tr_at = None
            self.spawn_trip()
        if self.climb_goal is not None:
            self._finish_climb()
            return
        if self.current is None:
            return
        seq, name, param, end = self.current
        if self.now() < end:
            self.send(seq, ActionState.STATE_RUNNING)
            return
        self.current = None
        if self.robot is None:
            self.get_logger().error('没收到 /ground_truth/odom，不知道车在哪')
            self.send(seq, ActionState.STATE_FAILED, err=1)
            return
        err = ''
        if name == 'PICK':
            err = self.do_pick(param)
        elif name in ('PLACE', 'PLACE_ONE'):
            err = self.do_place(param, one=name == 'PLACE_ONE')
        elif name == 'FLIP_SKY':
            err = self.do_flip()
        elif name == 'CLIMB_L2':
            err = self.do_climb(seq, up=True)
            if not err:
                return
        elif name == 'CLIMB_DOWN':
            err = self.do_climb(seq, up=False)
            if not err:
                return
        if err:
            self.get_logger().error(f'{name} {param} 失败：{err}')
            self.send(seq, ActionState.STATE_FAILED, err=2)
        else:
            self.send(seq, ActionState.STATE_DONE)

    def do_climb(self, seq, up=True):
        """不爬。换到台沿，或换回楼梯口。真值跟上再回 DONE。"""
        if up:
            x, y, yaw = hop_target('red')
            where = 'L2 台沿'
        else:
            # 朝向跟 transfer.see 一样（138°）。定位按这个角锁定，下一步才能平移到观察点。
            x, y, yaw = -2.15, 0.0, 2.415
            where = '楼梯口'
        if self._height is None:
            self._height = HeightField(field_triangles_in_world(load_binary_stl(FIELD_STL)))
        surface = self._height.height(x, y)
        if not math.isfinite(surface):
            return f'落点 ({x:.2f}, {y:.2f}) 下面没有台面'
        z = footprint_z(surface)
        self.climb_goal = (x, y, z, yaw, seq)
        self.climb_where = where
        self.climb_since = self.now()
        self._send_climb_pose()
        self.get_logger().info(f'换到{where} ({x:+.2f}, {y:+.2f}) z={z:.3f}')
        return ''

    def _send_climb_pose(self):
        x, y, z, yaw, _ = self.climb_goal
        state = EntityState()
        state.name = 'rc2027_br'
        state.reference_frame = 'world'
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        qx, qy, qz, qw = rpy_to_quaternion(0.0, 0.0, yaw)
        state.pose.orientation.x = qx
        state.pose.orientation.y = qy
        state.pose.orientation.z = qz
        state.pose.orientation.w = qw
        req = SetEntityState.Request()
        req.state = state
        self.pose_client.call_async(req)

    def _finish_climb(self):
        self._send_climb_pose()
        x, y, z, yaw, seq = self.climb_goal
        if self.robot is not None:
            gap = math.dist((self.robot[0], self.robot[1], self.robot_z), (x, y, z))
            dyaw = abs(math.atan2(math.sin(self.robot[2] - yaw), math.cos(self.robot[2] - yaw)))
            if gap < 0.08 and dyaw < math.radians(8):
                self.climb_goal = None
                self.get_logger().info(
                    f'车已在{self.climb_where} ({x:+.2f}, {y:+.2f}, z={z:.3f})')
                self.send(seq, ActionState.STATE_DONE)
                return
        if self.now() - self.climb_since > 3.0:
            self.climb_goal = None
            self.get_logger().error(f'换位 3 s 后车还没到{self.climb_where}')
            self.send(seq, ActionState.STATE_FAILED, err=2)

    def do_pick(self, n):
        err = self.world.pick(self.robot, n)
        if err:
            return err
        names = self.world.carried
        for name in names:   # 从 Gazebo 里删掉（放下时再按新位置 spawn），排队一个一个删
            self.gz_q.append(('del', name, 0))
        self.get_logger().info(f'PICK {n}：拿起 {"、".join(names)}')
        if self.world.region_empty(self.transfer):
            self.supply_lock = False
            if self.next_trip >= len(self.trips) and self.targets_need_blocks() and self.extra_trips < 8:
                self.trips.append(supply_trip(self.extra_trips))
                self.extra_trips += 1
                self.get_logger().info(f'全场目标还缺块，补第 {len(self.trips)} 趟')
            if self.next_trip < len(self.trips):
                self.tr_pending = True   # 等 BR 离开交接区再送
        return ''

    def do_place(self, level, *, one=False):
        names = list(self.world.carried)
        err = self.world.place(self.robot, level, self.rng, self.sigma_xy, self.sigma_yaw, one=one)
        if err:
            return err
        names = [name for name in names if name not in self.world.carried]
        for name in names:
            self.spawn_block(self.world.blocks[name])
        self.t_last_place = self.now()
        b0 = self.world.blocks[names[0]]
        spot = find_spot(self.world, b0.x, b0.y)
        if spot and spot not in self.placed_spots:
            self.placed_spots.append(spot)
        self.get_logger().info('\n'.join(
            [f'PLACE {level}：放下 {"、".join(names)}（{spot or "不在任何建造点上！"}）'] +
            tower_report(self.world, [spot] if spot else [])))
        if self.next_trip >= len(self.trips) and not self.world.carried and \
                self.world.region_empty(self.transfer):
            self.get_logger().info('\n'.join(
                ['', '════ 货全放完了 ════'] + tower_report(self.world, self.placed_spots) +
                [f'用时 {self.t_last_place - self.t_first:.1f} s（第一个动作开始到最后一次放下，sim 时间）']))
        return ''

    def do_flip(self):
        err, block = self.world.flip(self.robot)
        if err:
            return err
        self.gz_q.append(('respawn', block, 0))
        self.get_logger().info(f'FLIP：{block.name} 朝上改成{"红" if block.face == "red" else "蓝"}')
        return ''


def main(args=None):
    rclpy.init(args=args)
    node = SimTower()
    try:
        node.clear_old()
        if not node.spawn_client.wait_for_service(timeout_sec=10.0):
            node.get_logger().error('/spawn_entity 不在（仿真没起？）')
            return
        if node.random_field:
            node.spawn_field()
        node.spawn_trip()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError) as exc:
        if isinstance(exc, RuntimeError):
            node.get_logger().error(str(exc))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

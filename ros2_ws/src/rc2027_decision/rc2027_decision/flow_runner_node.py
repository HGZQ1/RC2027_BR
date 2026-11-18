"""flow_runner：Flow 引擎的 ROS 外壳，一步一步执行流程。

引擎（rc2027_decision/flow/）只回答"现在该做哪一步"，这里负责真的去做：
    NAV         → point_navigator 的 navigate_to_point action（带这一步的到位门限），ARRIVED 才算完
    RELOCALIZE  → /field_localizer/relocalize。"车还没停稳 / 点云不够"就隔 0.3 s 再试，最多
                  relocalize_timeout 秒；被 ICP 验收拒绝或者一直做不成只报警、接着走（拒绝说明
                  原来的定位比这次配准更可信，停下来也没用）
    WAIT        → 等够秒数（ROS 时钟，仿真里跟 /clock 走）
    ACTION      → /action_cmd 发 ActionCommand，等 /action_state 回同一个 cmd_seq 的 DONE；
                  FAILED 或超过 action_timeout 算失败。串口桥没接好以前用 mock_action_executor
    DETECT      → /block_detector/detect 看一个区域，按检测到的方块算拿取位姿（flow/pick_pose.py），存成
                  @<名字> / @<名字>_pre 给后面的 NAV 用。没找到 / 超限幅就隔 0.5 s 再看，detect_timeout 秒内
                  还不行算失败（TR 可能还在交接区里挡着）
任何一步失败：停住、在 ~/status 上报是哪一步、为什么（v1 不自动重试，等人处理 / Retry）。

启动 / 停止：
    ros2 service call /flow_runner/start std_srvs/srv/Trigger   跑主流程（流程文件里的 main）
    ros2 service call /flow_runner/stop std_srvs/srv/Trigger
    ros2 topic pub --once /flow_runner/run std_msgs/msg/String "{data: 'NAV p1; NAV p2'}"
        临时跑一个流程名或者一段流程文本（华工的动态流程），会顶掉正在跑的
状态：/flow_runner/status（String，transient local），状态变了就发，另外 1 Hz 重发。

Retry：订阅 /field_localizer/retry_status（"<状态> <zone>"），交给引擎。PAUSED 时这里也停掉
当前步骤（point_navigator 自己也会停车，这里只是不再等它的结果）。电控正在做的动作组停不下来，
只是不再等它的 ack。

启动时先做赛前检查：流程文件语法、CALL 的流程在不在、NAV 的点录没录、ACTION 在不在动作码表
——有问题直接报错退出，不要比赛开始了才发现点名打错了。
"""

import math
import os
from dataclasses import replace

from ament_index_python.packages import get_package_share_directory
from rc2027_interface.msg import ActionCommand, ActionState
from rc2027_navigation_msgs.action import NavigateToPoint
from rc2027_navigation_msgs.srv import Retry
from rc2027_perception_msgs.msg import FieldState
from rc2027_perception_msgs.srv import DetectBlocks
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml

from .flow import (Action, DONE, FAILED, FlowEngine, FlowRegistry, FlowSyntaxError, IDLE,
                   Nav, parse_flow, RUNNING)
from .flow.executor import KIND_NAMES, TaskExecutor
from .flow.recovery import FlowRecovery
from .flow.pick_pose import Block as PickBlock, PickParams
from .world.field_model import SKY_NONE, SKY_OTHERS, SKY_OURS
from rc2027_navigation_msgs.srv import Relatch
from std_srvs.srv import SetBool

from .world.planner import (MUSTIKA, PLACE_GROUND, PLACE_SKY, STOP,
                            Tower,
                            after_changed_look, bind_points, carry_choice,
                            choose as plan, close_look,
                            compile_choice, planner_subflows, load_approaches, on_arrival, pickable_goods,
                            trusted_transfer_blocks)


def load_yaml(path):
    """读 yaml，空文件当成 {}。"""
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def load_points(path):
    """teach_point 的点文件：{名字: {x, y, theta}} → {名字: (x, y, theta)}。"""
    out = {}
    for name, p in load_yaml(path).items():
        out[str(name)] = (float(p['x']), float(p['y']), float(p['theta']))
    return out


def load_action_codes(path):
    """动作码表：{动作名: action_id}。"""
    return {str(name).upper(): int(v['id']) for name, v in load_yaml(path)['actions'].items()}


class FlowRunner(TaskExecutor):
    """执行 Flow 引擎给出的步骤。"""

    def __init__(self):
        """读流程 / 点 / 动作码表，做赛前检查，建好接口。"""
        super().__init__('flow_runner')
        flows_path = self.declare_parameter('flows_path', '').value
        points_path = self.declare_parameter('points_path', '').value
        codes_path = self.declare_parameter('action_codes_path', '').value
        if not codes_path:
            codes_path = os.path.join(get_package_share_directory('rc2027_interface'),
                                      'config', 'action_codes.yaml')
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.action_timeout = self.declare_parameter('action_timeout', 30.0).value
        self.relocalize_timeout = self.declare_parameter('relocalize_timeout', 3.0).value
        self.detect_timeout = self.declare_parameter('detect_timeout', 3.0).value
        # 拿取位姿（flow/pick_pose.py）。默认跟 PickParams 一样（15 cm 间隙，2026-10-07）；
        # 以后 URDF 加了 gripper_link（动作组作用点）就从 TF 算。sim_tower / field_state 用的是同一个数
        self.pick_params = PickParams(
            gripper_offset=self.declare_parameter(
                'gripper_offset', PickParams().gripper_offset).value,
            approach_distance=self.declare_parameter('approach_distance', 0.30).value,
            max_shift=self.declare_parameter('max_pick_shift', 0.35).value,
            max_yaw=math.radians(self.declare_parameter('max_pick_yaw_deg', 20.0).value))
        autostart = self.declare_parameter('autostart', False).value

        if not flows_path or not points_path:
            raise RuntimeError('flows_path 和 points_path 两个参数都要给')
        self.registry = FlowRegistry.from_dict(load_yaml(flows_path))
        # 流程文件自带的点（仿真演示）打底，点文件（teach_point 录的）覆盖同名的
        self.points = dict(self.registry.points)
        self.points.update(load_points(points_path))
        self.slots = {}             # DETECT 算出来的 @点，Retry / 重新开始时清掉
        self.planner = bool(self.declare_parameter('planner', False).value)
        strategy = self.declare_parameter('strategy', 'full').value
        if strategy not in ('full', 'one_l2'):
            raise RuntimeError(f'未知赛程 {strategy}')
        self.one_l2 = strategy == 'one_l2'
        self.approaches = None
        if self.planner:
            path = self.declare_parameter('approaches_path', '').value
            self.approaches = load_approaches(path or None)
            self.points.update(bind_points(self.approaches))
            for name, steps in planner_subflows().items():
                if name in self.registry.flows:
                    raise RuntimeError(f'规划器子流程重名：{name}')
                self.registry.flows[name] = steps
        self.action_codes = load_action_codes(codes_path)
        errors = self.registry.validate(set(self.points), set(self.action_codes))
        if errors:
            for e in errors:
                self.get_logger().error('流程检查: ' + e)
            raise RuntimeError(f'流程文件 {flows_path} 有 {len(errors)} 个问题，见上面')
        self.engine = FlowEngine(self.registry)
        self.get_logger().info(
            f'流程 {flows_path}（主流程 {self.registry.main}，'
            f'Retry 后 {self.registry.retry_flows}），点 {points_path}，'
            f'会用到的点: {self.registry.used_points()}')

        # 当前步骤的执行状态。token 每换一步 +1：取消 / 暂停以后，旧步骤迟到的回调对不上 token
        # 就丢掉，不会把新步骤错当成做完了
        self.token = 0
        self.step = None
        self.deadline = None        # WAIT / ACTION / RELOCALIZE 的截止时间（ROS 时间，秒）
        self.nav_goal_handle = None
        self.locking_token = None
        self.cmd_seq = 0
        self.localizer_paused = False

        self.nav_client = ActionClient(self, NavigateToPoint, 'navigate_to_point')
        self.relocalize_client = self.create_client(Trigger, 'field_localizer/relocalize')
        self.detect_client = self.create_client(DetectBlocks, 'block_detector/detect')
        self.action_pub = self.create_publisher(ActionCommand, 'action_cmd', 10)
        self.create_subscription(ActionState, 'action_state', self.on_action_state, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'field_localizer/retry_status', self.on_retry_status,
                                 latched)
        self.status_pub = self.create_publisher(String, '~/status', latched)
        self.create_service(Trigger, '~/start', self.on_start)
        self.create_service(Trigger, '~/stop', self.on_stop)
        # 手动 Retry：车已经搬回 Retry Zone 以后调。定位起飞 / 导航 STUCK 也是这条服务，
        # 但不在场中自动调——车不在 zone 里就锁定，地图会锁飞（0002 2026-10-07）
        self.retry_zone = self.declare_parameter('retry_zone', 'l1').value
        self.retry_client = self.create_client(Retry, 'field_localizer/retry')
        self.create_service(Trigger, '~/manual_retry', self.on_manual_retry)
        # 规划器只记自己扫过的。自己的塔按规则开局是空的；共享区没扫过就是不知道
        self.memory = {
            'spot_07': Tower(seen=True),
            'spot_03': Tower(seen=True),
            'spot_08': Tower(seen=True),
            'spot_05': Tower(seen=True),
            'spot_01': Tower(),
            'spot_02': Tower(),
        }
        self.latest_field = None
        self.planned = None
        self.climbed = False
        self.on_platform = False
        self.at_transfer = False
        self.capped_en_route = False
        self.held_kind = None
        self.held_n = 0
        self.decisions = []
        self.relatch_client = self.create_client(Relatch, '/field_localizer/relatch')
        self.floor_client = self.create_client(SetBool, '/field_localizer/floor_change')
        self.pick_misses = 0
        self.recovery = FlowRecovery(self)
        self.health_client = self.create_client(Trigger, '/field_localizer/check_health')
        if self.planner:
            self.create_subscription(FieldState, 'field_state', self.on_field, latched)
            self.get_logger().info('规划器接入栈：做完一件再选下一件，不跑写死的主流程')
        self.create_subscription(String, '~/run', self.on_run, 10)
        self.create_timer(0.1, self.on_timer)
        self.create_timer(1.0, self.publish_status)
        self.last_status = None
        self.publish_status()

        if autostart:
            self.create_timer(1.0, self._autostart_once)
            self._autostarted = False

    # ── 启动 / 停止 ──
    def _autostart_once(self):
        if not self._autostarted:
            self._autostarted = True
            self.begin(None)

    def begin(self, flow_name):
        """从 flow_name（None = 主流程）开始跑。"""
        if self.localizer_paused:
            return False, 'Retry 进行中，等 RESUMED'
        self.cancel_step()
        self.slots.clear()
        if self.planner:
            self.plan_and_run()
            return self.engine.state != FAILED, self.engine.describe_state()
        ref = self.engine.start(flow_name)
        self.execute(ref)
        return self.engine.state != FAILED, self.engine.describe_state()

    def on_field(self, msg):
        """最近一次 /field_state。"""
        self.latest_field = msg
        if (self.held_n and self.planned and self.planned.action in (PLACE_GROUND, PLACE_SKY)
                and self.planned.spot in ('spot_01', 'spot_02')):
            spot = self._spot_msg(self.planned.spot)
            if spot and spot.source == spot.SOURCE_SEEN and spot.has_sky:
                self.capped_en_route = True

    def world_view(self):
        """四座塔用雷达已经扫到的结果。开局扫到共享区算数。自己放过的层数和天空用记账补上。"""
        spots = {}
        blocks = []
        if self.latest_field is not None:
            spots = {s.region: s for s in self.latest_field.spots}
            for region in self.latest_field.transfer:
                if region.region == 'transfer_red' and region.visible:
                    blocks = [PickBlock(KIND_NAMES.get(b.type, 'unknown'), b.x, b.y, b.yaw,
                                        b.layer, b.center_reliable) for b in region.blocks]
                    blocks = trusted_transfer_blocks(blocks, region.top_maybe_clipped)
        towers = {name: self.tower_of(name, spots.get(name))
                  for name in ('spot_07', 'spot_01', 'spot_03', 'spot_02',
                               'spot_08', 'spot_05')}
        return towers, pickable_goods(blocks), blocks

    def tower_of(self, name, spot):
        """雷达扫到的算数。自己放过的天空不要被看不出颜色的雷达盖掉。"""
        if spot is None or spot.source != spot.SOURCE_SEEN:
            reported = Tower(seen=name in ('spot_07', 'spot_03', 'spot_08', 'spot_05'))
        else:
            sky = {1: SKY_OURS, 2: SKY_OTHERS}.get(spot.sky_owner, SKY_NONE)
            reported = Tower(int(spot.ground_layers), sky, True,
                             '视场' in (spot.note or ''), bool(spot.obstructed))
        mem = self.memory.get(name)
        if mem is None:
            return reported
        if mem.sky_owner == SKY_OURS:
            sky = SKY_OURS
        elif reported.sky_owner == SKY_OTHERS:
            sky = SKY_OTHERS
        else:
            sky = reported.sky_owner if reported.seen else mem.sky_owner
        # 新的一次看清了，就用这次的出视场 / 被挡。旧的出视场不能一直把后来的对方天空挡掉。
        clipped = reported.clipped if reported.seen else mem.clipped
        blocked = reported.blocked if reported.seen else mem.blocked
        return Tower(max(reported.ground, mem.ground), sky, reported.seen or mem.seen,
                     clipped, blocked)

    def _spot_msg(self, name):
        if self.latest_field is None:
            return None
        return next((s for s in self.latest_field.spots if s.region == name), None)

    def snapshot_scan(self, name):
        """观察位停稳之后，把这一眼写进记账。"""
        self.memory[name] = self.tower_of(
            name,
            next((s for s in self.latest_field.spots if s.region == name), None)
            if self.latest_field is not None else None)
        tower = self.memory[name]
        self.get_logger().info(
            f'扫到 {name}：{tower.ground} 层地，天空={tower.sky_owner}')

    def plan_and_run(self, holding=False, choice=None, blocks=None, waypoint=None):
        """选下一件，编成步骤，清栈跑。五色石就停。

        holding：这块已经在手里。blocks：用这一眼交接区的块，不用 /field_state 里可能过期的。
        """
        towers, goods, seen = self.world_view()
        if blocks is not None:
            goods = pickable_goods(blocks)
            seen = blocks
        left_north = (self.planned is not None
                      and self.planned.spot in ('spot_02', 'spot_03')
                      and not self.on_platform and not self.at_transfer)
        at_north = not holding and left_north
        if choice is None:
            choice = plan(towers, goods, self.approaches, climbed=self.climbed,
                          at_north=at_north, one_l2=self.one_l2)
        self.get_logger().info('规划: ' + choice.line())
        self.decisions.append(choice.line())
        if choice.action in (MUSTIKA, STOP):
            self.engine._stack = []
            self.engine.state = DONE
            self.engine.reason = choice.why
            self.execute(None)
            return
        # 北共享在东边，回去要先收到西侧。spot_03 已经在西侧，再往东是多拐一步。
        from_north = left_north and self.planned.spot == 'spot_02'
        on_west = left_north and self.planned.spot == 'spot_03'
        # 南共享在东边。上楼若直开楼梯口，直线切进 L2 南沿。
        from_south = (self.planned is not None and self.planned.spot == 'spot_01'
                      and not self.on_platform and not self.at_transfer)
        if waypoint:
            # 途中改派用实际到达的点。处在共享塔一侧时仍要从原路退到西廊。
            from_north = waypoint.startswith('spot_02_')
            from_south = waypoint.startswith('spot_01_')
            on_west = waypoint.startswith('spot_03_')
        steps = compile_choice(choice, self.approaches, seen, holding=holding,
                               on_platform=self.on_platform, from_north=from_north,
                               on_west=on_west, from_south=from_south)
        if (waypoint and steps and isinstance(steps[0], Nav) and steps[0].point == waypoint
                and steps[0].yaw is None and steps[0].xy_tolerance is None):
            steps.pop(0)
        self.capped_en_route = False
        if not steps:
            self.engine._fail('编不出流程: ' + choice.line())
            self.execute(None)
            return
        self.planned = choice
        self.slots.clear()
        self.execute(self.engine.run_steps(choice.line(), steps))

    def replan_after_observation(self, blocks):
        """恢复后在交接区停稳且看到了货，按这一眼重新选任务。"""
        self.plan_and_run(blocks=blocks, waypoint='transfer_see')

    def redirect_held_at(self, point, token):
        if not (self.capped_en_route and self.held_n and point and not point.startswith('@')):
            return False
        towers, _, _ = self.world_view()
        target = self.planned.spot
        alt = carry_choice(towers, self.held_kind, self.approaches, self.held_n,
                           avoid=target, one_l2=self.one_l2)
        if alt is None:
            self.fail(token, f'{target} 途中已封顶，手里的块没有能放的塔')
        else:
            self.get_logger().info(f'{target} 途中已封顶，在 {point} 改放 {alt.spot}')
            self.plan_and_run(holding=True, choice=alt, waypoint=point)
        return True

    def _held_load(self):
        """手里这块是什么。"""
        if self.planned is None:
            return 'ground', 1
        if self.planned.action == PLACE_SKY and self.planned.count == 2:
            return 'cap', 2
        if self.planned.action == PLACE_SKY:
            return 'sky', 1
        return 'ground', self.planned.pick_count or self.planned.count

    def on_spot_check(self, res, step, token):
        """放到跟前再看一眼。和出发时不一样就丢掉后面的放置，重选。"""
        if not res.success:
            if self.deadline is not None and self.now() >= self.deadline:
                if self.recovery.stage == 'second':
                    self.recovery.retry_required(token, '重定位后仍看不清目标：' + res.message)
                else:
                    self.recovery.probe_detection_health(step, token, res, res.message)
                return
            self.get_logger().warn(f'跟前 {step.region}: {res.message}，0.5 s 后再看')
            timer = None

            def retry():
                self.destroy_timer(timer)
                self.call_detect(step, token)
            timer = self.create_timer(0.5, retry)
            return
        if self.planned is None:
            self.done(token)
            return
        towers, goods, _ = self.world_view()
        state = res.state
        remembered = towers[step.region]
        fresh = self._spot_msg(step.region)
        # 半路上雷达已经确认有别人的天空。贴太近会把顶看丢，不能再照原计划往上放。
        opponent = (self.planned.action == PLACE_SKY and fresh is not None and fresh.has_sky
                    and fresh.sky_owner != fresh.SKY_OURS)
        if opponent:
            towers[step.region] = Tower(
                max(int(fresh.ground_layers), remembered.ground, 2), SKY_OTHERS, True)
            new, note = on_arrival(
                self.planned, towers, goods, self.approaches, climbed=self.climbed, one_l2=self.one_l2)
            self.get_logger().warn(
                f'跟前 {step.region} 已经有对方天空，不往上放，改去翻')
        # 出视场只说明这一眼不能用来改计划。记账和出发时一样，就照原计划放。
        elif state.top_maybe_clipped:
            new, note = on_arrival(
                self.planned, towers, goods, self.approaches, climbed=self.climbed, one_l2=self.one_l2)
            if not note:
                self.get_logger().warn(
                    f'跟前 {step.region} 顶部出视场，这一眼不改计划，照原计划放')
                self.done(token)
                return
        else:
            towers[step.region] = close_look(
                remembered, state.ground_layers, state.has_sky, False)
            new, note = on_arrival(
                self.planned, towers, goods, self.approaches, climbed=self.climbed, one_l2=self.one_l2)
            if not note:
                self.done(token)
                return
        self.get_logger().warn(note)
        self.memory[step.region] = towers[step.region]
        kind, nxt = after_changed_look(
            self.planned, new, towers, self.approaches, self.held_n,
            avoid=step.region if opponent else None, one_l2=self.one_l2)
        if kind == 'empty':
            self.plan_and_run(holding=False, choice=nxt)
            return
        if kind == 'same':
            self.plan_and_run(holding=True, choice=nxt)
            return
        if kind == 'stuck':
            self.engine._fail('手里的块没有能放的塔：' + note)
            self.execute(None)
            return
        self.get_logger().warn('手里已经拿着，改放到 ' + nxt.line())
        self.plan_and_run(holding=True, choice=nxt)

    def remember_done_action(self):
        """PLACE / 翻转做完：层数先记账，不等雷达确认。"""
        if not self.planner or self.planned is None or self.planned.spot is None or \
                self.step is None or not isinstance(self.step.step, Action):
            return
        spot = self.planned.spot
        current = self.memory.get(spot, Tower(seen=True))
        if self.step.step.name in ('PLACE', 'PLACE_ONE') and self.planned.action == PLACE_GROUND:
            self.memory[spot] = replace(
                current, ground=max(current.ground, self.planned.assumed_ground or 0) + self.planned.count, seen=True)
        elif self.step.step.name in ('PLACE', 'PLACE_ONE') and self.planned.action == PLACE_SKY:
            ground = current.ground + 1 if self.planned.count == 2 else max(current.ground, 2)
            self.memory[spot] = replace(current, ground=ground, sky_owner=SKY_OURS, seen=True)
        elif self.step.step.name == 'FLIP_SKY':
            self.memory[spot] = replace(current, sky_owner=SKY_OURS, seen=True)
            self.get_logger().info('翻转记下了：这座塔的天空算我们的')

    def on_start(self, _request, response):
        """~/start：跑主流程。"""
        response.success, response.message = self.begin(None)
        return response

    def on_manual_retry(self, _request, response):
        """~/manual_retry：先停当前步骤，再调 /field_localizer/retry。等它的 RESUMED。"""
        self.cancel_step()
        if not self.retry_client.wait_for_service(timeout_sec=1.0):
            response.success, response.message = False, 'field_localizer/retry 不在'
            return response
        req = Retry.Request()
        req.zone = self.retry_zone
        self.retry_client.call_async(req)
        response.success = True
        response.message = f'已请求 Retry zone={self.retry_zone}，等 RESUMED 再继续'
        return response

    def on_stop(self, _request, response):
        """~/stop：停掉当前步骤，回到 IDLE。"""
        self.cancel_step()
        self.engine.stop()
        self.publish_status()
        response.success, response.message = True, '已停'
        return response

    def on_run(self, msg):
        """~/run：流程名，或者一段流程文本（临时注册成 _adhoc 再跑）。"""
        text = msg.data.strip()
        if text not in self.registry.flows:
            try:
                steps = parse_flow(text, name='_adhoc')
            except FlowSyntaxError as exc:
                self.get_logger().error(str(exc))
                return
            previous = self.registry.flows.get('_adhoc')
            self.registry.flows['_adhoc'] = steps
            errors = self.registry.validate(set(self.points), set(self.action_codes))
            if errors:
                # 放回原来的，别让写错的临时流程留在注册表里
                if previous is None:
                    del self.registry.flows['_adhoc']
                else:
                    self.registry.flows['_adhoc'] = previous
                self.get_logger().error('临时流程有问题: ' + '；'.join(errors))
                return
            text = '_adhoc'
        ok, message = self.begin(text)
        (self.get_logger().info if ok else self.get_logger().warn)(message)

    # ── Retry ──
    def on_retry_status(self, msg):
        """field_localizer/retry_status："<状态> [zone]"。"""
        words = msg.data.split()
        if not words:
            return
        status, zone = words[0], (words[1] if len(words) > 1 else '')
        if status not in ('PAUSED', 'RESUMED', 'RETRY_FAILED'):
            return  # IDLE：field_localizer 刚启动（重启），跟正在跑的流程无关
        self.localizer_paused = status != 'RESUMED'
        if self.localizer_paused:
            self.cancel_step()
        if self.planner and status == 'RESUMED' and self.engine.state != IDLE:
            for name in list(self.memory):
                self.memory[name] = replace(self.memory[name], seen=False)
            self.slots.clear()
            self.planned = None
            self.at_transfer = False
            self.on_platform = False  # start 和 l1 都不在 L2 台上
            # 人工搬回 Retry Zone 按 field_model.on_retry 的约定清持货账。
            self.held_kind, self.held_n = None, 0
            self.get_logger().warn(f'Retry 成功（zone={zone}），按重试区和现场塔况重选')
            self.plan_and_run()
            return
        before = self.engine.state
        ref = self.engine.on_retry_status(status, zone)
        if self.engine.state == IDLE:
            return  # 还没开始跑，引擎不理 Retry
        if ref is None and self.engine.state == before and before != RUNNING:
            self.publish_status()  # 已经失败 / 走完时又来 PAUSED：状态没变，不重复报
            return
        if ref is not None:
            self.slots.clear()   # 车被搬走了，之前看到的拿取位姿作废
            self.get_logger().warn(f'Retry 成功（zone={zone}），清栈，从 {ref.path} 重新开始')
        self.execute(ref)

    # ── 状态 ──
    def publish_status(self):
        """~/status。"""
        text = self.engine.describe_state()
        stage = self.recovery.stage
        if stage in ('relocalize', 'settle', 'second'):
            text += ' | RECOVERING ' + stage
        elif stage == 'health':
            text += ' | CHECK_LOCALIZATION'
        self.status_pub.publish(String(data=text))
        self.last_status = text


def main(args=None):
    """入口。"""
    rclpy.init(args=args)
    try:
        node = FlowRunner()
    except (RuntimeError, FlowSyntaxError, OSError, KeyError) as exc:
        rclpy.logging.get_logger('flow_runner').fatal(str(exc))
        rclpy.try_shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

"""在一个 FlowEngine 栈上执行导航、检测、换层及机构动作。"""
import math

from rc2027_interface.msg import ActionCommand, ActionState
from rc2027_navigation_msgs.action import NavigateToPoint
from rc2027_navigation_msgs.srv import Relatch
from rc2027_perception_msgs.srv import DetectBlocks
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

from .engine import DONE, FAILED, RUNNING
from .pick_pose import Block as PickBlock, choose
from .steps import Action, Detect, Nav, Realign, Relocalize, Verify, Wait, WaitAfterCorrection
from ..world.planner import (PLACE_SKY, nearest_transfer_blocks, _pick_how, carry_choice, pickable_goods,
                             trusted_transfer_blocks)


KIND_NAMES = {0: 'ground', 1: 'sky', 2: 'unknown'}


class TaskExecutor(Node):
    """执行单步并验证返回结果；需要换任务时交回规划器。"""

    # ── 执行步骤 ──
    def now(self):
        """ROS 时间（秒）。"""
        return self.get_clock().now().nanoseconds * 1e-9

    def execute(self, ref):
        """开始执行一个步骤；ref 是 None 说明流程停了（走完 / 失败 / 暂停）。"""
        self.token += 1
        self.step = ref
        self.recovery.reset()
        self.deadline = None
        self.publish_status()
        if ref is None:
            if self.engine.state != RUNNING:
                log = self.get_logger().error if self.engine.state == FAILED \
                    else self.get_logger().info
                log(f'流程: {self.engine.describe_state()}')
                if self.engine.state == FAILED and self.decisions:
                    lines = [f'  {i}. {text}' for i, text in enumerate(self.decisions, 1)]
                    self.get_logger().error('决策流程：\n' + '\n'.join(lines))
            return
        self.get_logger().info(f'执行 {ref}')
        step = ref.step
        if isinstance(step, Nav):
            self.start_nav(step, self.token)
        elif isinstance(step, Relocalize):
            self.deadline = self.now() + self.relocalize_timeout
            self.call_relocalize(self.token)
        elif isinstance(step, Wait):
            self.deadline = self.now() + step.seconds
        elif isinstance(step, (Detect, Verify)):
            self.deadline = self.now() + self.detect_timeout
            self.call_detect(step, self.token)
        elif isinstance(step, Action):
            self.cmd_seq = (self.cmd_seq + 1) % 256
            cmd = ActionCommand(action_id=self.action_codes[step.name], cmd_seq=self.cmd_seq,
                                param=step.param)
            self.action_pub.publish(cmd)
            self.deadline = self.now() + self.action_timeout
        else:
            self.fail(self.token, f'不认识的步骤 {step!r}')

    def done(self, token, *, pose_unchanged=False):
        """这个 token 对应的步骤做完了。"""
        if token != self.token:
            return
        ref = self.engine.step_done()
        if pose_unchanged and ref and isinstance(ref.step, (Realign, WaitAfterCorrection)):
            self.get_logger().info(f'ICP 未改定位，跳过 {ref}')
            ref = self.engine.step_done()
        if ref is None and self.planner and self.engine.state == DONE:
            if self.planned is not None and self.planned.action == '看' and self.planned.spot:
                self.snapshot_scan(self.planned.spot)
            self.plan_and_run()
            return
        self.execute(ref)

    def fail(self, token, why):
        """这个 token 对应的步骤失败了。"""
        self.recovery.stop_failed_step(token, why)

    def cancel_step(self):
        """不再等当前步骤（导航目标顺手取消）。"""
        self.token += 1
        self.step = None
        self.deadline = None
        self.recovery.reset()
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None

    def on_timer(self):
        """WAIT 到时、ACTION 超时、RELOCALIZE 重试截止。"""
        if self.recovery.expired():
            self.recovery.retry_required(self.token, '定位检查或恢复超时')
            return
        if self.step is None or self.deadline is None or self.now() < self.deadline:
            return
        # RELOCALIZE 的截止时间由 on_relocalize 自己看（服务调用可能还没回来）
        step, token = self.step.step, self.token
        if isinstance(step, Wait):
            self.done(token)
        elif isinstance(step, Action):
            self.fail(token, f'等了 {self.action_timeout:g} s 电控没回 DONE')

    # NAV
    def start_nav(self, step, token):
        """发 navigate_to_point。"""
        if not self.nav_client.wait_for_server(timeout_sec=0.5):
            self.fail(token, 'point_navigator 的 navigate_to_point 不在（导航没启动？）')
            return
        if step.point.startswith('@'):
            if step.point not in self.slots:
                self.fail(token, f'{step.point} 还没算出来（前面的 DETECT 没跑？）')
                return
            x, y, theta = self.slots[step.point]
        else:
            x, y, theta = self.points[step.point]
        if step.yaw is not None:
            theta = step.yaw
        goal = NavigateToPoint.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal.pose.pose.orientation.w = math.cos(theta / 2.0)
        goal.xy_tolerance = float(step.xy_tolerance or 0.0)
        goal.yaw_tolerance = float(step.yaw_tolerance or 0.0)
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self.on_nav_accepted(f, token))

    def on_nav_accepted(self, future, token):
        """point_navigator 收没收这个目标。"""
        handle = future.result()
        if token != self.token:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.fail(token, 'point_navigator 拒绝了目标（Retry 进行中？）')
            return
        self.nav_goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self.on_nav_result(f, token))

    def on_nav_result(self, future, token):
        """导航结束：ARRIVED 算完成。"""
        if token != self.token:
            return
        self.nav_goal_handle = None
        result = future.result().result
        if result.outcome == 'ARRIVED':
            point = getattr(getattr(self.step, 'step', None), 'point', None)
            if point == 'transfer_see':
                self.at_transfer = True
            elif point and not str(point).startswith('@'):
                self.at_transfer = False
            if self.redirect_held_at(point, token):
                return
            self.done(token)
        else:
            self.fail(token, f'导航 {result.outcome or "失败"}（还差 '
                             f'{result.distance_remaining * 100:.1f} cm）')
            if result.outcome == 'STUCK':
                self.get_logger().error(
                    '导航 STUCK：先停车。把车搬回 Retry Zone 后调 /flow_runner/manual_retry'
                    f'（zone={self.retry_zone}，走 /field_localizer/retry）。'
                    '车还在场中不要调，锁定会把地图锁飞')

    # RELOCALIZE
    def call_relocalize(self, token):
        """调一次 relocalize；没准备好就过一会儿再调，直到 deadline。"""
        if token != self.token:
            return
        if not self.relocalize_client.service_is_ready():
            self.get_logger().warn('relocalize 服务不在（没给先验地图？），跳过')
            self.done(token)
            return
        future = self.relocalize_client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: self.on_relocalize(f, token))

    def on_relocalize(self, future, token):
        """处理 relocalize 的结果。field_localizer 的约定：被验收拒绝的 message 以"拒绝"开头。"""
        if token != self.token:
            return
        try:
            response = future.result()
        except Exception as exc:
            self.fail(token, f'重定位服务调用失败：{exc}')
            return
        if response.success:
            self.get_logger().info('relocalize ' + response.message)
            unchanged = response.message.startswith('要应用的修正') and '在死区内，不改' in response.message
            self.done(token, pose_unchanged=unchanged)
        elif response.message.startswith('拒绝'):
            self.get_logger().warn('relocalize 被拒绝，保持原定位接着走: ' + response.message)
            self.done(token, pose_unchanged=True)
        elif self.deadline is not None and self.now() >= self.deadline:
            self.get_logger().warn(
                f'relocalize {self.relocalize_timeout:g} s 内没做成（{response.message}），接着走')
            self.done(token)
        else:
            timer = None

            def retry():
                self.destroy_timer(timer)
                self.call_relocalize(token)
            timer = self.create_timer(0.15, retry)

    # DETECT
    def call_detect(self, step, token):
        """调一次 block_detector；没找到合适的就 0.5 s 后再看，直到 deadline。"""
        if token != self.token:
            return
        if not self.detect_client.service_is_ready():
            self.fail(token, 'block_detector 的 detect 服务不在（方块检测没启动？）')
            return
        future = self.detect_client.call_async(DetectBlocks.Request(region=step.region))
        future.add_done_callback(lambda f: self.on_detect(f, step, token))

    def on_detect(self, future, step, token):
        """检测结果 → 挑一摞 → 拿取位姿。"""
        if token != self.token:
            return
        try:
            res = future.result()
        except Exception as exc:
            self.fail(token, f'检测服务调用失败：{exc}')
            return
        if isinstance(step, Verify):
            self.on_verify(res, step, token)
            return
        if step.slot == 'supply':
            blocks = [] if not res.success else [
                PickBlock(KIND_NAMES.get(b.type, 'unknown'), b.x, b.y, b.yaw, b.layer,
                          b.center_reliable) for b in res.state.blocks]
            if res.success:
                blocks = trusted_transfer_blocks(blocks, res.state.top_maybe_clipped)
            goods = pickable_goods(blocks)
            if goods.ground or goods.sky:
                if self.planner and self.recovery.stage == 'second':
                    self.replan_after_observation(blocks)
                else:
                    self.done(token)
            elif self.recovery.stage == 'second':
                self.recovery.retry_required(token, '重定位后交接区仍看不到可用货物')
            else:
                self.recovery.probe_detection_health(step, token, res, '交接区没有可用货物')
            return
        if step.slot == 'check':
            self.on_spot_check(res, step, token)
            return
        if res.success:
            blocks = [PickBlock(KIND_NAMES.get(b.type, 'unknown'), b.x, b.y, b.yaw, b.layer,
                                b.center_reliable) for b in res.state.blocks]
            what, nominal = step.what, self.points[step.nominal]
            # 顶出了视场还报一块地：上面可能是没看见的天。按这块地去拿会把天揭下来。
            clipped_ground = (step.slot == 'pick' and step.what == 'ground'
                              and res.state.top_maybe_clipped)
            eligible = None
            if self.planner and step.slot == 'pick' and self.planned is not None:
                fresh = trusted_transfer_blocks(blocks, res.state.top_maybe_clipped)
                eligible = nearest_transfer_blocks(fresh)
                how = _pick_how(fresh, 'sky' if self.planned.action == PLACE_SKY else 'ground',
                                self.planned.pick_count or self.planned.count)
                if how is None and eligible:
                    self.get_logger().info('近侧货物已变化，在交接区重新选塔')
                    self.replan_after_observation(fresh)
                    return
                if how:
                    what, nominal = how[0], self.points[how[1]]
            if clipped_ground:
                result, why = None, '顶部出视场，这块地上面可能还有天空，先不拿'
            else:
                result, why = choose(blocks, what, nominal, self.pick_params, eligible=eligible)
        else:
            result, why = None, res.message
        if result is not None:
            if self.planner and step.slot == 'pick' and self.recovery.stage == 'second':
                self.replan_after_observation(trusted_transfer_blocks(blocks, res.state.top_maybe_clipped))
                return
            self.slots['@' + step.slot] = result.pick
            self.slots['@' + step.slot + '_pre'] = result.pre
            self.get_logger().info(
                f'DETECT {step.region}: {why} → @{step.slot} ({result.pick[0]:.3f}, '
                f'{result.pick[1]:.3f}, {math.degrees(result.pick[2]):.1f}°)')
            self.pick_misses = 0
            self.done(token)
        elif self.deadline is not None and self.now() >= self.deadline:
            if self.recovery.stage == 'second':
                self.recovery.retry_required(token, f'重定位后再次观察仍失败：{why}')
            else:
                self.recovery.probe_detection_health(step, token, res, why)
        else:
            self.get_logger().warn(f'DETECT {step.region}: {why}，0.5 s 后再看')
            timer = None

            def retry():
                self.destroy_timer(timer)
                self.call_detect(step, token)
            timer = self.create_timer(0.5, retry)

    def finish_detect_miss(self, step, token, res, why):
        """定位正常时维持原策略，不因为空交接区就启用重定位备用。"""
        if step.slot in ('supply', 'check'):
            self.done(token)
        elif self.planner and step.slot == 'pick' and res.success:
            self.pick_misses += 1
            if self.pick_misses > 2:
                self.fail(token, f'{self.detect_timeout:g} s 内没找到能拿的：{why}')
                return
            blocks = [PickBlock(KIND_NAMES.get(b.type, 'unknown'), b.x, b.y, b.yaw, b.layer,
                                b.center_reliable) for b in res.state.blocks]
            blocks = trusted_transfer_blocks(blocks, res.state.top_maybe_clipped)
            self.get_logger().warn(f'定位正常，交接区没有要的 {step.what}，按眼前的货重选：{why}')
            self.plan_and_run(blocks=blocks)
        else:
            self.fail(token, f'{self.detect_timeout:g} s 内没找到能拿的：{why}')

    def on_verify(self, res, step, token):
        """VERIFY：层数（和天空块）对上就完成；对不上 0.5 s 后再看，到 deadline 还不对算失败。"""
        want = f'{step.ground_layers} 层{"+天空块" if step.sky else ""}'
        if res.success:
            s = res.state
            got = f'{s.ground_layers} 层{"+天空块" if s.has_sky else ""}'
            if s.ground_layers == step.ground_layers and s.has_sky == step.sky:
                self.get_logger().info(f'VERIFY {step.region}: {got} ✓')
                self.done(token)
                return
            why = f'看到 {got}（要 {want}）{"，" + s.note if s.note else ""}'
        else:
            why = res.message
        if self.deadline is not None and self.now() >= self.deadline:
            self.fail(token, f'{step.region} 层数不对：{why}')
            return
        self.get_logger().warn(f'VERIFY {step.region}: {why}，0.5 s 后再看')
        timer = None

        def retry():
            self.destroy_timer(timer)
            self.call_detect(step, token)
        timer = self.create_timer(0.5, retry)

    def _lock_after_climb(self):
        """车已经被换到台沿。放宽高度门限，按已知落点重新锁定，再选下一件。"""
        land = (self.approaches or {}).get('l2_stair', {}).get('land')
        if not land:
            self.fail(self.token, 'spot_approaches 里没有 l2_stair.land')
            return
        widen = SetBool.Request()
        widen.data = True
        self.floor_client.call_async(widen)
        req = Relatch.Request()
        req.x = float(land['x'])
        req.y = float(land['y'])
        req.z = float(land['z'])
        req.yaw = float(land['theta'])
        fut = self.relatch_client.call_async(req)

        token = self.token
        self.locking_token = token

        def locked(future):
            if token != self.token:
                return
            try:
                res = future.result()
            except Exception as exc:
                self.fail(token, f'换层锁定服务失败：{exc}')
                return
            if res is None or not res.success:
                why = res.message if res is not None else '没有回应'
                self.fail(self.token, '上楼后重新锁定失败：' + why)
                return
            self.climbed = True
            self.on_platform = True
            self.get_logger().info(
                f'已在 L2 台沿锁定 ({req.x:+.2f}, {req.y:+.2f}) z={req.z:.2f}')
            self.done(self.token)

        fut.add_done_callback(locked)

    def _lock_after_descend(self):
        """下楼：换回楼梯口，高度门限收回到一层。朝向用观察点，锁定后直接平移过去。"""
        foot = (self.approaches or {}).get('l2_stair', {}).get('foot')
        if not foot:
            self.fail(self.token, 'spot_approaches 里没有 l2_stair.foot')
            return
        narrow = SetBool.Request()
        narrow.data = False
        self.floor_client.call_async(narrow)
        req = Relatch.Request()
        see = (self.approaches or {}).get('transfer', {}).get('see') or {}
        req.x = float(foot['x'])
        req.y = float(foot['y'])
        req.z = 0.6
        # 朝向跟观察点一样。下一步平移过去，楼梯口不再转一次。
        req.yaw = float(see['theta']) if see else float(foot['theta'])
        fut = self.relatch_client.call_async(req)

        token = self.token
        self.locking_token = token

        def locked(future):
            if token != self.token:
                return
            try:
                res = future.result()
            except Exception as exc:
                self.fail(token, f'换层锁定服务失败：{exc}')
                return
            if res is None or not res.success:
                why = res.message if res is not None else '没有回应'
                self.fail(self.token, '下楼后重新锁定失败：' + why)
                return
            self.on_platform = False
            self.get_logger().info(
                f'已在楼梯口锁定 ({req.x:+.2f}, {req.y:+.2f}) z={req.z:.2f}')
            self.done(self.token)

        fut.add_done_callback(locked)

    # ACTION
    def on_action_state(self, msg):
        """电控回报：同一个 cmd_seq 的 DONE / FAILED 结束这一步。"""
        if self.step is None or not isinstance(self.step.step, Action) or \
                msg.cmd_seq != self.cmd_seq or self.locking_token == self.token:
            return
        if msg.state == ActionState.STATE_DONE:
            self.remember_done_action()
            name = self.step.step.name
            if name == 'PICK' and self.planned is not None:
                self.held_kind, self.held_n = self._held_load()
            if name in ('PLACE', 'PLACE_ONE') and self.planned is not None and self.held_n > self.planned.count:
                self.held_n -= self.planned.count
                self.held_kind = 'ground'
                towers, _, _ = self.world_view()
                alt = carry_choice(towers, 'ground', self.approaches, self.held_n, one_l2=self.one_l2)
                if alt is None:
                    self.fail(self.token, '手里还剩一块地，没有塔能放')
                    return
                self.get_logger().info('本塔已放好，手里还剩一块地：' + alt.line())
                self.plan_and_run(holding=True, choice=alt)
                return
            if name in ('PLACE', 'PLACE_ONE'):
                self.held_kind, self.held_n = None, 0
            if name == 'CLIMB_L2':
                self._lock_after_climb()
                return
            if name == 'CLIMB_DOWN':
                self._lock_after_descend()
                return
            self.done(self.token)
        elif msg.state == ActionState.STATE_FAILED:
            self.fail(self.token, f'电控报动作失败，err_code={msg.err_code}')

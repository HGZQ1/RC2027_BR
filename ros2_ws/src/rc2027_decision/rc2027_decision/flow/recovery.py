"""处理执行故障，管理停稳后的观测恢复；终止故障留给人工处理。"""
import time

from std_srvs.srv import Trigger


class FlowRecovery:
    def __init__(self, runner):
        self.runner = runner
        self.reset()

    def reset(self):
        self.stage = None
        self.until = None

    def expired(self):
        return self.until is not None and time.monotonic() >= self.until

    def stop_failed_step(self, token, why):
        """已经失败的导航或未确认的机构动作不能自动重放。"""
        if token != self.runner.token:
            return
        self.runner.engine.step_failed(why)
        self.runner.execute(None)

    def retry_required(self, token, why):
        """只停车上报；不能在场中自动按 Retry Zone 重新锁定。"""
        self.until = None
        self.runner.fail(token, why + '；需要人工检查，搬回 Retry Zone 后调用 manual_retry')

    def probe_detection_health(self, step, token, res, why, after=False):
        """看不见只触发健康检查，确认定位漂移才进入备用恢复。"""
        if token != self.runner.token:
            return
        if not after:
            if self.stage is not None:
                return
            self.runner.token += 1
            token = self.runner.token
        self.stage = 'health'
        self.until = time.monotonic() + self.runner.relocalize_timeout + 2.0
        if not self.runner.health_client.service_is_ready():
            self.retry_required(token, '定位健康服务不可用，不能确认是否漂移')
            return
        future = self.runner.health_client.call_async(Trigger.Request())

        def checked(f):
            if token != self.runner.token:
                return
            try:
                health = f.result()
            except Exception as exc:
                self.retry_required(token, f'定位健康检查失败：{exc}')
                return
            if health.success:
                self.until = None
                if after:
                    self.stage = 'second'
                    self.until = time.monotonic() + self.runner.detect_timeout + 1.0
                    self.runner.deadline = self.runner.now() + self.runner.detect_timeout
                    self.runner.call_detect(step, token)
                else:
                    self.stage = None
                    self.runner.finish_detect_miss(step, token, res, why)
            elif after or not health.message.startswith('高度漂移'):
                self.retry_required(token, '定位仍异常：' + health.message)
            else:
                self.runner.get_logger().warn('确认高度漂移，启用备用流程：重定位后再观察一次')
                self.stage = 'relocalize'
                self.until = time.monotonic() + self.runner.relocalize_timeout
                self.recover_detection(step, token, res, why)
        future.add_done_callback(checked)

    def recover_detection(self, step, token, res, why):
        if token != self.runner.token:
            return
        if not self.runner.relocalize_client.service_is_ready():
            self.retry_required(token, '备用重定位服务不可用')
            return
        future = self.runner.relocalize_client.call_async(Trigger.Request())

        def localized(f):
            if token != self.runner.token:
                return
            try:
                result = f.result()
            except Exception as exc:
                self.retry_required(token, f'备用重定位失败：{exc}')
                return
            if not result.success:
                if (not result.message.startswith('拒绝')
                        and time.monotonic() < self.until):
                    self.after_detection_delay(
                        0.3, token, lambda: self.recover_detection(step, token, res, why))
                else:
                    self.retry_required(token, '备用重定位未通过：' + result.message)
                return
            # 等旧高度下积累的检测帧淘汰，同时观察高度是否继续漂。
            self.stage = 'settle'
            self.until = time.monotonic() + 4.0
            self.after_detection_delay(
                2.5, token, lambda: self.probe_detection_health(step, token, res, why, after=True))
        future.add_done_callback(localized)

    def after_detection_delay(self, seconds, token, callback):
        timer = None

        def ready():
            self.runner.destroy_timer(timer)
            if token == self.runner.token:
                callback()
        timer = self.runner.create_timer(seconds, ready)

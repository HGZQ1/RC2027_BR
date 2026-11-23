"""异步备用恢复：只有定位异常才重定位，正常缺货不触发。"""
from concurrent.futures import Future
from types import SimpleNamespace as NS

import pytest

pytest.importorskip('rclpy')
from rc2027_decision.flow_runner_node import FlowRunner
from rc2027_decision.flow.recovery import FlowRecovery
from rc2027_decision.flow import Detect


class Client:
    def __init__(self, ready=True):
        self.ready = ready
        self.calls = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        future = Future()
        self.calls.append(future)
        return future

    def reply(self, success, message=''):
        self.calls[-1].set_result(NS(success=success, message=message))


class Harness:
    def __init__(self):
        self.token = 1
        self.health_client = Client()
        self.relocalize_client = Client()
        self.relocalize_timeout = 3
        self.detect_timeout = 2
        self.recovery = FlowRecovery(self)
        self.events = []
        self.timers = []

    def now(self):
        return 100

    def get_logger(self):
        return NS(warn=lambda text: None)

    def create_timer(self, seconds, callback):
        timer = NS(callback=callback)
        self.timers.append(timer)
        return timer

    def destroy_timer(self, timer):
        self.timers.remove(timer)

    def fail(self, token, why):
        self.events.append(('failed', why))
        self.token += 1

    def finish_detect_miss(self, *args):
        self.events.append(('normal_miss',))

    def call_detect(self, *args):
        self.events.append(('second_look',))


def probe(h):
    h.recovery.probe_detection_health(Detect('transfer_red', 'ground', 'transfer_stack'), 1,
                             NS(success=False), '看不见')


def test_missing_blocks_with_healthy_localization_never_relocalizes():
    h = Harness()
    probe(h)
    h.health_client.reply(True, '高度正常')
    assert h.events == [('normal_miss',)]
    assert not h.relocalize_client.calls
    assert h.recovery.until is None


def test_drift_relocalizes_then_checks_stability_before_second_look():
    h = Harness()
    probe(h)
    h.health_client.reply(False, '高度漂移，先重定位再观察')
    assert len(h.relocalize_client.calls) == 1
    h.relocalize_client.reply(True)
    assert not h.events
    h.timers[0].callback()
    h.health_client.reply(True)
    assert h.events == [('second_look',)]
    assert h.recovery.stage == 'second'
    assert h.deadline == 102
    assert h.token == 2


def test_drift_after_relocalize_reports_retry_without_second_look():
    h = Harness()
    probe(h)
    h.health_client.reply(False, '高度漂移')
    h.relocalize_client.reply(True)
    h.timers[0].callback()
    h.health_client.reply(False, '高度漂移')
    assert [e[0] for e in h.events] == ['failed']
    assert 'manual_retry' in h.events[0][1]


def test_rejected_icp_is_not_bypassed():
    h = Harness()
    probe(h)
    h.health_client.reply(False, '高度漂移')
    h.relocalize_client.reply(False, '拒绝：内点不够')
    assert h.events[0][0] == 'failed'
    assert not h.timers


def test_reply_after_stop_does_not_start_recovery():
    h = Harness()
    probe(h)
    h.token += 1
    h.health_client.reply(False, '高度漂移')
    assert not h.events and not h.relocalize_client.calls


def test_stale_tf_is_reported_without_guessing_a_new_pose():
    h = Harness()
    probe(h)
    h.health_client.reply(False, '定位 TF 已过期')
    assert h.events[0][0] == 'failed'
    assert not h.relocalize_client.calls


def test_second_look_still_missing_stops_instead_of_waiting_again():
    h = Harness()
    h.recovery.stage = 'second'
    future = Future()
    future.set_result(NS(success=False))
    FlowRunner.on_detect(h, future, Detect('transfer_red', 'any', 'transfer_stack', 'supply'), 1)
    assert h.events[0][0] == 'failed'
    assert not h.health_client.calls and not h.relocalize_client.calls


def test_detection_recovery_watchdog_does_not_wait_forever():
    h = Harness()
    h.recovery.until = 0
    FlowRunner.on_timer(h)
    assert h.events[0][0] == 'failed'


def test_no_health_service_reports_unavailable_instead_of_relocalizing_blindly():
    h = Harness()
    h.health_client.ready = False
    probe(h)
    assert h.events[0][0] == 'failed'
    assert not h.relocalize_client.calls


@pytest.mark.parametrize('unchanged', [True, False])
def test_alignment_only_skips_the_conditional_second_nav(unchanged):
    from rc2027_decision.flow import FlowEngine, FlowRegistry, Nav, Relocalize
    from rc2027_decision.flow.steps import Realign

    engine = FlowEngine(FlowRegistry.from_dict({'main': 'idle', 'flows': {'idle': 'WAIT 1'}}))
    fine = Realign('pre', 0.01, 0.017)
    final = Nav('place', 0.01, 0.017)
    engine.run_steps('放块', [Relocalize(), fine, final])
    dispatched = []
    runner = NS(token=1, engine=engine, planner=True, execute=dispatched.append,
                get_logger=lambda: NS(info=lambda message: None))
    FlowRunner.done(runner, 1, pose_unchanged=unchanged)
    assert dispatched[0].step == (final if unchanged else fine)


def test_health_probe_takes_over_the_step_only_once():
    h = Harness()
    probe(h)
    probe(h)
    assert len(h.health_client.calls) == 1 and h.token == 2

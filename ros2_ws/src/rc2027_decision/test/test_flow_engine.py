"""Flow 引擎栈逻辑的单测（不需要 ROS）：压栈 / 出栈 / 跳转 / Retry / 失败。"""

from rc2027_decision.flow import (DONE, FAILED, FlowEngine, FlowRegistry, IDLE, Nav, PAUSED,
                                  RUNNING, Wait)


def make(flows, main='main', retry_flows=None):
    return FlowEngine(FlowRegistry.from_dict(
        {'flows': flows, 'main': main, 'retry_flows': retry_flows}))


def run_all(engine, first):
    """一路 step_done 到底，返回执行过的 (路径, 步骤)。"""
    out, ref = [], first
    while ref is not None:
        out.append((ref.path, ref.step))
        ref = engine.step_done()
    return out


def test_run_steps_replaces_the_stack():
    engine = make({'main': 'NAV a'})
    engine.start()
    ref = engine.run_steps('放地 spot_07', [Nav('p'), Wait(1)])
    assert ref.step == Nav('p')
    assert engine.step_done().step == Wait(1)
    assert engine.step_done() is None and engine.state == DONE


def test_linear_route():
    engine = make({'main': 'NAV p1; NAV p2; NAV p3'})
    trace = run_all(engine, engine.start())
    assert [s for _, s in trace] == [Nav('p1'), Nav('p2'), Nav('p3')]
    assert engine.state == DONE


def test_call_pushes_and_pops():
    engine = make({'main': 'NAV a; CALL sub; NAV d', 'sub': 'NAV b; NAV c'})
    trace = run_all(engine, engine.start())
    assert trace == [
        ('main#0', Nav('a')),
        ('main#1 > sub#0', Nav('b')),
        ('main#1 > sub#1', Nav('c')),
        ('main#2', Nav('d')),
    ]
    assert engine.state == DONE


def test_nested_calls_and_call_as_last_step():
    engine = make({'main': 'CALL a', 'a': 'NAV x; CALL b', 'b': 'NAV y'})
    trace = run_all(engine, engine.start())
    assert [p for p, _ in trace] == ['main#0 > a#0', 'main#0 > a#1 > b#0']
    assert engine.state == DONE


def test_call_then_jumps_in_parent():
    # 华工的 on_child_done_next：子流程走完跳过父流程的 NAV skipped
    engine = make({'main': 'CALL sub THEN after; NAV skipped; LABEL after; NAV d',
                   'sub': 'NAV b'})
    trace = run_all(engine, engine.start())
    assert [s for _, s in trace] == [Nav('b'), Nav('d')]


def test_empty_flows_are_skipped():
    engine = make({'main': 'CALL empty; NAV a', 'empty': ''})
    assert engine.start().step == Nav('a')
    engine2 = make({'main': ''})
    assert engine2.start() is None and engine2.state == DONE


def test_same_subflow_called_twice():
    engine = make({'main': 'CALL place; NAV t; CALL place', 'place': 'NAV n; WAIT 1'})
    trace = run_all(engine, engine.start())
    assert [s for _, s in trace] == [Nav('n'), Wait(1.0), Nav('t'), Nav('n'), Wait(1.0)]


def test_depth_limit_catches_recursion():
    engine = FlowEngine(FlowRegistry.from_dict({'flows': {'main': 'CALL main'}, 'main': 'main'}),
                        max_depth=5)
    assert engine.start() is None
    assert engine.state == FAILED and '5 层' in engine.reason


def test_step_failed_reports_path():
    engine = make({'main': 'CALL sub', 'sub': 'NAV a; NAV b'})
    engine.start()
    engine.step_done()
    engine.step_failed('STUCK')
    assert engine.state == FAILED
    assert engine.reason == 'main#0 > sub#1 NAV b: STUCK'
    assert engine.current() is None and engine.step_done() is None


def test_start_unknown_flow_fails():
    engine = make({'main': 'NAV a'})
    engine.start('nope')
    assert engine.state == FAILED


def test_retry_pause_then_resume_switches_flow_and_clears_stack():
    engine = make({'main': 'CALL sub; NAV never', 'sub': 'NAV a; NAV b',
                   'after_l1': 'NAV r1; NAV r2'},
                  retry_flows={'l1': 'after_l1'})
    engine.start()
    engine.on_retry_status('PAUSED', 'l1')
    assert engine.state == PAUSED and engine.current() is None
    assert engine.step_done() is None          # 暂停时节点报"做完了"也不推进
    ref = engine.on_retry_status('RESUMED', 'l1')
    assert ref.path == 'after_l1#0' and ref.step == Nav('r1')
    trace = run_all(engine, ref)
    assert [s for _, s in trace] == [Nav('r1'), Nav('r2')]
    assert engine.state == DONE                 # 原来的 main 不会接着跑（栈清了）


def test_retry_failed_stops():
    engine = make({'main': 'NAV a', 'r': 'NAV b'}, retry_flows={'l1': 'r'})
    engine.start()
    engine.on_retry_status('PAUSED', 'l1')
    engine.on_retry_status('RETRY_FAILED', 'l1')
    assert engine.state == FAILED and 'Retry 失败' in engine.reason
    # 操作员重摆车再 retry 成功 → 照样能接着跑
    ref = engine.on_retry_status('RESUMED', 'l1')
    assert ref.step == Nav('b') and engine.state == RUNNING


def test_resume_after_step_failure_recovers():
    # 某一步失败停住了，操作员把车搬回 Retry Zone、retry 成功 → 从 Retry 后流程接着跑
    engine = make({'main': 'NAV a', 'r': 'NAV b'}, retry_flows={'start': 'r'})
    engine.start()
    engine.step_failed('STUCK')
    assert engine.on_retry_status('RESUMED', 'start').step == Nav('b')


def test_retry_ignored_before_start():
    engine = make({'main': 'NAV a', 'r': 'NAV b'}, retry_flows={'start': 'r'})
    assert engine.on_retry_status('PAUSED', 'start') is None
    assert engine.on_retry_status('RESUMED', 'start') is None
    assert engine.state == IDLE


def test_resume_without_retry_flow_fails():
    engine = make({'main': 'NAV a'})
    engine.start()
    assert engine.on_retry_status('RESUMED', 'l1') is None
    assert engine.state == FAILED and 'zone=l1' in engine.reason


def test_stop_returns_to_idle():
    engine = make({'main': 'NAV a; NAV b'})
    engine.start()
    engine.stop()
    assert engine.state == IDLE and engine.current() is None


def test_describe_state():
    engine = make({'main': 'CALL sub', 'sub': 'NAV a fine'})
    engine.start()
    assert engine.describe_state() == 'RUNNING main#0 > sub#0 NAV a 0.010m 1.0°'
    engine.on_retry_status('PAUSED', 'l1')
    assert engine.describe_state() == 'PAUSED（在 main#0 > sub#0）'

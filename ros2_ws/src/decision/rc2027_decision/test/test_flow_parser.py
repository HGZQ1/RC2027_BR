"""流程文本解析和赛前检查的单测（不需要 ROS）。"""

import math

import pytest

from rc2027_decision.flow import (Action, Call, FlowRegistry, FlowSyntaxError, Nav,
                                  parse_flow, Relocalize, Wait)


def test_basic_steps_and_comments():
    steps = parse_flow("""
        NAV p1            # 普通门限
        nav p2 fine
        NAV p3 0.02 2     # 2 cm / 2°
        RELOCALIZE; WAIT 1.5
        ACTION pick 2
        CALL sub
    """)
    assert steps == [
        Nav('p1'),
        Nav('p2', 0.01, math.radians(1.0)),
        Nav('p3', 0.02, math.radians(2.0)),
        Relocalize(),
        Wait(1.5),
        Action('PICK', 2),
        Call('sub'),
    ]


def test_list_form_same_as_text():
    assert parse_flow(['NAV a', 'WAIT 1']) == parse_flow('NAV a; WAIT 1')


def test_point_names_keep_case():
    assert parse_flow('NAV Tower_N') == [Nav('Tower_N')]


def test_align_expands_to_three_steps():
    fine = (0.012, math.radians(1.5))
    assert parse_flow('ALIGN N', {'fine': fine}) == [
        Nav('N'), Relocalize(), Nav('N', *fine)]


def test_labels_survive_align_expansion():
    # ALIGN 展开成 3 步，标签算的是展开后的序号——这就是不让流程里直接写数字序号的原因
    steps = parse_flow("""
        CALL sub THEN after
        ALIGN N
        LABEL after
        WAIT 1
    """)
    assert steps[0] == Call('sub', 4)
    assert steps[4] == Wait(1.0)


def test_label_at_end_means_finish():
    steps = parse_flow('CALL sub THEN end; WAIT 1; LABEL end')
    assert steps[0] == Call('sub', 2) and len(steps) == 2


@pytest.mark.parametrize('text,fragment', [
    ('FLY p1', '不认识'),
    ('NAV', 'NAV <点名>'),
    ('NAV p1 abc', '应该是数字'),
    ('NAV p1 0.8', '门限不合理'),
    ('WAIT -1', '负的'),
    ('ACTION PICK x', '整数'),
    ('CALL a THEN nowhere', '没有标签'),
    ('LABEL x; LABEL x', '重复'),
    ('RELOCALIZE now', '不带参数'),
])
def test_syntax_errors_say_where(text, fragment):
    with pytest.raises(FlowSyntaxError) as exc:
        parse_flow(text, name='demo')
    assert fragment in str(exc.value)
    assert '流程 demo 第 1 行' in str(exc.value)


def registry(flows, main='main', retry_flows=None, **extra):
    return FlowRegistry.from_dict(dict(flows=flows, main=main, retry_flows=retry_flows, **extra))


def test_registry_custom_tolerance_profile():
    reg = registry({'main': 'NAV a slow'}, tolerances={'slow': {'xy': 0.05, 'yaw_deg': 5}})
    assert reg.get('main') == [Nav('a', 0.05, math.radians(5))]


def test_validate_ok():
    reg = registry({'main': 'NAV a; CALL sub; ACTION PICK', 'sub': 'NAV b'},
                   retry_flows={'l1': 'sub'})
    assert reg.validate(points={'a', 'b'}, actions={'PICK'}) == []


def test_validate_reports_everything():
    reg = registry({'main': 'NAV a; CALL nope; ACTION JUMP'}, retry_flows={'l1': 'gone'})
    errors = reg.validate(points={'b'}, actions={'PICK'})
    text = '\n'.join(errors)
    assert '点 a 没录' in text
    assert 'nope 不存在' in text
    assert '动作 JUMP 不在' in text
    assert 'retry_flows.l1' in text


def test_validate_skips_points_when_not_given():
    reg = registry({'main': 'NAV a'})
    assert reg.validate() == []


def test_validate_finds_call_cycle():
    reg = registry({'main': 'CALL a', 'a': 'CALL b', 'b': 'NAV x; CALL a'})
    errors = reg.validate()
    assert any('成环' in e and 'a -> b -> a' in e for e in errors)


def test_used_points_in_route_order():
    # 路线顺序来自流程，不是点文件（teach_point 存点按键名排序）
    reg = registry({'main': 'NAV z; CALL sub; NAV a', 'sub': 'ALIGN m'},
                   retry_flows={'l1': 'other'})
    reg.flows['other'] = [Nav('q')]
    assert reg.used_points() == ['z', 'm', 'a', 'q']


def test_missing_flows_section():
    with pytest.raises(FlowSyntaxError):
        FlowRegistry.from_dict({'main': 'x'})


def test_detect_step():
    from rc2027_decision.flow import Detect
    assert parse_flow('DETECT transfer_red Stack transfer_stack') == [
        Detect('transfer_red', 'stack', 'transfer_stack', 'pick')]
    assert parse_flow('DETECT spot_03 any spot_03_pick AS put') == [
        Detect('spot_03', 'any', 'spot_03_pick', 'put')]
    with pytest.raises(FlowSyntaxError, match='找什么'):
        parse_flow('DETECT transfer_red rock p')
    with pytest.raises(FlowSyntaxError, match='DETECT <区域>'):
        parse_flow('DETECT transfer_red stack')
    with pytest.raises(FlowSyntaxError, match='@'):
        parse_flow('DETECT transfer_red stack p AS @x')


def test_dynamic_points_validated():
    ok = registry({'main': 'DETECT transfer_red stack nominal; NAV @pick_pre; ALIGN @pick'})
    assert ok.validate(points={'nominal'}) == []
    assert ok.used_points() == []           # @ 开头的不算录的点
    bad = registry({'main': 'NAV @pick; DETECT t stack missing AS other'})
    errors = '\n'.join(bad.validate(points=set()))
    assert '@pick 没有 DETECT 写它' in errors and '示教拿取点 missing 没录' in errors


def test_inline_points():
    reg = registry({'main': 'NAV a'}, points={'a': {'x': 1, 'y': 2, 'theta': 0.5}})
    assert reg.points == {'a': (1.0, 2.0, 0.5)}
    with pytest.raises(FlowSyntaxError, match='points.b'):
        registry({'main': 'NAV b'}, points={'b': {'x': 1}})


def test_verify_step():
    from rc2027_decision.flow import Verify
    assert parse_flow('VERIFY spot_07 2; VERIFY spot_03 2 SKY') == [
        Verify('spot_07', 2, False), Verify('spot_03', 2, True)]
    with pytest.raises(FlowSyntaxError, match='VERIFY'):
        parse_flow('VERIFY spot_07')
    with pytest.raises(FlowSyntaxError, match='0～3'):
        parse_flow('VERIFY spot_07 5')

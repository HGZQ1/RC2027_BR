"""仓库里带的流程文件都能解析、通过赛前检查（点文件有的话连点一起查）。"""

import os

import pytest
import yaml

from rc2027_decision.flow import FlowEngine, FlowRegistry

HERE = os.path.dirname(os.path.abspath(__file__))
FLOWS_DIR = os.path.join(HERE, '..', 'config', 'flows')
ACTION_CODES = os.path.join(HERE, '..', '..', 'rc2027_interface', 'config', 'action_codes.yaml')
SIM_POINTS = os.path.join(HERE, '..', '..', '..', '..', 'docs', 'field_data', 'sim_red.yaml')


def load(path):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def action_names():
    return {name.upper() for name in load(ACTION_CODES)['actions']}


@pytest.mark.parametrize('name', sorted(os.listdir(FLOWS_DIR)))
def test_flow_file_validates(name):
    reg = FlowRegistry.from_dict(load(os.path.join(FLOWS_DIR, name)))
    assert reg.validate(actions=action_names()) == []


def test_action_ids_unique():
    ids = [v['id'] for v in load(ACTION_CODES)['actions'].values()]
    assert len(ids) == len(set(ids))


def test_route_demo_matches_sim_points():
    if not os.path.exists(SIM_POINTS):
        pytest.skip('没有 docs/field_data/sim_red.yaml')
    reg = FlowRegistry.from_dict(load(os.path.join(FLOWS_DIR, 'route_demo.yaml')))
    assert reg.validate(points=set(load(SIM_POINTS)), actions=action_names()) == []


def test_plan2_skeleton_order():
    # 方案 2 的顺序：N 放 2 层 → F 塔基 → N 封顶 → F 中段 + 塔顶
    reg = FlowRegistry.from_dict(load(os.path.join(FLOWS_DIR, 'two_towers_plan2.yaml')))
    engine = FlowEngine(reg)
    ref, places, last_nav = engine.start(), [], None
    while ref is not None:
        step = ref.step
        if type(step).__name__ == 'Nav':
            last_nav = step.point
        if type(step).__name__ == 'Action' and step.name == 'PLACE':
            places.append((last_nav, step.param))
        ref = engine.step_done()
    assert places == [('tower_near', 1), ('tower_far', 1), ('tower_near', 3), ('tower_far', 2)]
    assert reg.used_points()[:2] == ['stairs', 'transfer_wait']

import math
import os

import pytest
import yaml

from rc2027_utils.sim_check.block_scene import (
    block_center, block_sdf, center_stack, compare, regions_from_geometry, scene_blocks,
    yaw_diff_mod90)

HERE = os.path.dirname(os.path.abspath(__file__))
GEOMETRY = os.path.join(HERE, '..', '..', '..', '..', 'docs', 'rules', 'field_geometry.yaml')
SCENES = os.path.join(HERE, '..', '..', '..', '..', 'sim_ws', 'src', 'rc2027_sim_bridge',
                      'config', 'block_scenes')


def regions():
    with open(GEOMETRY, encoding='utf-8') as f:
        return regions_from_geometry(yaml.safe_load(f))


def test_regions_levels():
    r = regions()
    assert r['spot_03'] == (-2.7, 2.7, 0.6)      # L1
    assert r['spot_05'] == (-1.25, 1.25, 0.9)    # L2
    assert r['transfer_red'] == (-3.5, -1.3, 0.6)
    assert r['transfer_blue'][0] == 3.5


def test_stack_heights_and_offsets():
    scene = {'z_offset': 0.05, 'stacks': [{
        'region': 'spot_03', 'dx': 0.02, 'blocks': ['ground', 'ground', 'sky'],
        'yaw_deg': [0, 5, 10], 'offsets': [[0, 0], [0.01, 0], [0, -0.02]]}]}
    b = scene_blocks(scene, regions())
    assert [x['layer'] for x in b] == [1, 2, 3]
    assert [round(x['z_bottom'], 3) for x in b] == [0.65, 1.0, 1.35]
    assert b[1]['x'] == pytest.approx(-2.67)
    assert b[2]['y'] == pytest.approx(2.68)
    assert b[2]['yaw'] == pytest.approx(math.radians(10))
    assert block_center(b[2])[2] == pytest.approx(1.45)
    assert len({x['name'] for x in b}) == 3


@pytest.mark.parametrize('stack,fragment', [
    ({'region': 'nope', 'blocks': ['ground']}, '没有区域'),
    ({'region': 'spot_03', 'blocks': ['sky', 'ground']}, '最上面'),
    ({'region': 'spot_03', 'blocks': ['rock']}, '类型'),
    ({'region': 'spot_03', 'blocks': ['ground'], 'yaw_deg': [1, 2]}, '个数'),
])
def test_scene_errors(stack, fragment):
    with pytest.raises(ValueError, match=fragment):
        scene_blocks({'stacks': [stack]}, regions())


def test_sdf_is_static_box():
    sdf = block_sdf({'name': 'blk_000', 'type': 'sky', 'size': 0.2})
    assert '<static>true</static>' in sdf and '<size>0.2 0.2 0.2</size>' in sdf


def test_sdf_blue_face_colors_ground_too():
    sky = block_sdf({'name': 'blk_000', 'type': 'sky', 'size': 0.2, 'face': 'blue'})
    ground = block_sdf({'name': 'blk_001', 'type': 'ground', 'size': 0.35, 'face': 'blue'})
    assert '0.15 0.35 0.9' in sky and '0.15 0.35 0.9' in ground


def test_yaw_mod90():
    assert yaw_diff_mod90(math.radians(89), math.radians(0)) == pytest.approx(math.radians(-1))
    assert yaw_diff_mod90(math.radians(46), math.radians(0)) == pytest.approx(math.radians(-44))


def test_compare_matches_by_layer_and_distance():
    truth = [{'x': 0, 'y': 0, 'yaw': 0, 'layer': 1}, {'x': 0, 'y': 0, 'yaw': 0, 'layer': 2},
             {'x': 1, 'y': 0, 'yaw': 0, 'layer': 1}]
    det = [{'x': 0.01, 'y': 0, 'yaw': math.radians(1), 'layer': 2},
           {'x': 0.0, 'y': 0.02, 'yaw': 0, 'layer': 1},
           {'x': 3, 'y': 3, 'yaw': 0, 'layer': 1}]
    rows, missed, extra = compare(truth, det)
    assert rows[0][2] == pytest.approx(2.0)
    assert rows[1][2] == pytest.approx(1.0) and rows[1][3] == pytest.approx(1.0)
    assert rows[2][1] is None and missed == 1 and extra == 1


def test_center_stack():
    b = scene_blocks({'stacks': [
        {'region': 'transfer_red', 'dx': -0.2, 'blocks': ['ground', 'ground']},
        {'region': 'transfer_red', 'dx': 0.3, 'blocks': ['sky']}]}, regions())
    assert center_stack(b, (-3.5 - 0.2, -1.3)) == (2, False)
    assert center_stack([], (0, 0)) == (0, False)


def test_shipped_scenes_load():
    names = sorted(os.listdir(SCENES))
    assert names
    for name in names:
        with open(os.path.join(SCENES, name), encoding='utf-8') as f:
            scene = yaml.safe_load(f)
        # 搭塔场景按"趟"写（trips），每趟跟 stacks 一样的写法
        for stacks in scene.get('trips') or [scene.get('stacks')]:
            one = {'z_offset': scene.get('z_offset', 0), 'stacks': stacks}
            assert scene_blocks(one, regions())

"""拿取位姿计算 + 交接区演示的几何检查（不需要 ROS）。"""

import math
import os

import pytest
import yaml

from rc2027_decision.flow import FlowEngine, FlowRegistry
from rc2027_decision.flow.pick_pose import (Block, choose, group_stacks, pick_pose_for,
                                            PickParams, wrap)

HERE = os.path.dirname(os.path.abspath(__file__))
DEMO = os.path.join(HERE, '..', 'config', 'flows', 'transfer_pick_demo.yaml')
SCENE = os.path.join(HERE, '..', '..', '..', '..', 'sim_ws', 'src', 'rc2027_sim_bridge', 'config',
                     'block_scenes', 'transfer_trip1.yaml')
D = math.radians


def test_group_stacks():
    b = [Block('ground', 0, 0, 0, 1), Block('ground', 0, 0, 0, 2), Block('sky', 1, 0, 0, 1),
         Block('ground', 2, 0, 0, 1), Block('sky', 2, 0, 0, 2)]
    assert [len(s) for s in group_stacks(b)] == [2, 1, 2]


def test_north_stack_uses_a_heading_that_clears_the_rail():
    """一摞检测在北栏杆边上：按检测点停车车身顶栏杆。往台面里收，车身北缘离栏杆至少 8 cm。"""
    from rc2027_decision.flow.pick_pose import pose_hits, robot_points
    stack = [Block('ground', -3.707, -0.984, math.radians(4.9), 1),
             Block('ground', -3.707, -0.984, math.radians(4.9), 2)]
    single = Block('ground', -3.26, -1.52, math.radians(8), 1)
    result, why = choose(stack + [single], 'stack', (-3.125, -1.15, math.pi))
    assert result is not None, why
    assert not pose_hits(*result.pick, [(single.x, single.y, single.yaw)])
    assert not pose_hits(*result.pre, [(single.x, single.y, single.yaw)])
    north = max(py for _, py in robot_points(*result.pick))
    assert north <= -0.88, north


def test_pick_pose_picks_face_closest_to_nominal_heading():
    blk = Block('ground', -3.7, -1.0, D(7), 1)
    pick, pre = pick_pose_for(blk, math.pi, PickParams())
    assert pick[2] == pytest.approx(wrap(math.pi + D(7)))
    assert math.hypot(pick[0] - blk.x, pick[1] - blk.y) == pytest.approx(PickParams().gripper_offset)
    assert math.hypot(pre[0] - pick[0], pre[1] - pick[1]) == pytest.approx(0.30)
    # 车头朝向方块：方块在拿取位姿正前方
    assert math.atan2(blk.y - pick[1], blk.x - pick[0]) == pytest.approx(pick[2])
    # 换成从北面拿（示教朝南）
    pick2, _ = pick_pose_for(blk, -math.pi / 2, PickParams())
    assert pick2[2] == pytest.approx(-math.pi / 2 + D(7))


def test_choose_by_kind_and_nearest():
    blocks = [Block('ground', -3.70, -1.05, D(-7), 1), Block('ground', -3.70, -1.05, D(-7), 2),
              Block('ground', -3.25, -1.55, D(-12), 1)]
    r, why = choose(blocks, 'stack', (-3.25, -1.05, math.pi))
    assert r is not None and len(r.stack) == 2, why
    r, why = choose(blocks, 'ground', (-2.80, -1.55, math.pi))
    assert r is not None and r.stack[0].x == -3.25, why
    r, why = choose(blocks, 'sky', (-2.8, -1.55, math.pi))
    assert r is None and '没找到 sky' in why and 'ground+ground' in why


def test_choose_limits():
    blocks = [Block('ground', -3.25, -1.55, 0, 1)]
    r, why = choose(blocks, 'ground', (-2.80 + 0.5, -1.55, math.pi))   # 示教点差 50 cm
    assert r is None and '超过限幅' in why
    r, why = choose(blocks, 'ground', (-2.80, -1.55, math.pi + D(30)))  # 示教朝向差 30°：
    assert r is None and '超过限幅' in why                              # 方块面只能差 ≤ 45°


def test_unreliable_and_unknown_are_skipped():
    r, why = choose([Block('ground', 0, 0, 0, 1, reliable=False)], 'ground', (0.45, 0, math.pi))
    assert r is None and '不可靠' in why
    r, why = choose([Block('unknown', 0, 0, 0, 1)], 'any', (0.45, 0, math.pi))
    assert r is None


# ── 演示的几何：车（0.3 x 0.3 + 车头雷达）沿 point_navigator 的走法（先平移、车头不转，到了再原地转）
#    不能碰到交接区两头的围栏和别的方块 ──
FENCES = [(-3.0, -2.95, -0.8, 3.0), (-3.0, -2.95, -3.0, -1.8)]   # x0 x1 y0 y1（10 cm 高，Gazebo 网格查的）


def robot_outline(x, y, yaw, n=8):
    """车体 0.3 x 0.3 的边 + 车头雷达（0.2 m 处、半径 0.05）上的采样点。"""
    c, s = math.cos(yaw), math.sin(yaw)
    pts = []
    for i in range(n + 1):
        t = -0.15 + 0.3 * i / n
        for u, v in ((t, -0.15), (t, 0.15), (-0.15, t), (0.15, t)):
            pts.append((x + c * u - s * v, y + s * u + c * v))
    for k in range(12):
        a = 2 * math.pi * k / 12
        u, v = 0.2 + 0.05 * math.cos(a), 0.05 * math.sin(a)
        pts.append((x + c * u - s * v, y + s * u + c * v))
    return pts


def clearance(points, blocks):
    """采样点到障碍（围栏矩形、方块正方形）的最小距离，负数 = 穿进去了。"""
    best = float('inf')
    for px, py in points:
        for x0, x1, y0, y1 in FENCES:
            dx = max(x0 - px, 0, px - x1)
            dy = max(y0 - py, 0, py - y1)
            inside = dx == 0 and dy == 0
            best = min(best, -0.01 if inside else math.hypot(dx, dy))
        for bx, by, byaw in blocks:
            c, s = math.cos(-byaw), math.sin(-byaw)
            u, v = c * (px - bx) - s * (py - by), s * (px - bx) + c * (py - by)
            du, dv = abs(u) - 0.175, abs(v) - 0.175
            best = min(best, math.hypot(max(du, 0), max(dv, 0)) if max(du, dv) > 0 else -0.01)
    return best


L2_BOXES = [(-1.5, 1.5, -1.5, 1.5), (-1.8, -1.5, -0.5, 0.5)]   # L2 台子、L2 楼梯（比 L1 高，车上不去）


def on_l1_or_platform(px, py):
    """车体点要在 L1（6 x 6）或者交接区平台上（Gazebo 网格查过）。"""
    return (abs(px) <= 3.0 and abs(py) <= 3.0) or (-4.0 <= px <= -3.0 and -1.8 <= py <= -0.8)


def sweep_flow(flow_path, trips, fixed_obstacles, start, spot_blocks=None, towers=None):
    """按 point_navigator 的走法（先平移、车头不转，到了原地转）把车沿整个流程扫一遍，返回 (最近距离, 在哪一步)。

    trips：每趟交接区的货 [(detected 方块列表, 障碍物列表)]，跟 sim_tower 一样——DETECT 用当前在场的方块代替检测结果，
    ACTION PICK 把挑中的那摞拿走，交接区空了摆下一趟。fixed_obstacles 一直在（比如塔）。离开 L1 / 交接区平台算 -1。
    spot_blocks：DETECT 建造点（spot_xx）时当成检测结果的方块 {区域: [Block]}。
    towers：{(建造点 x, y): 障碍物}，ACTION PLACE 放到那个建造点以后才出现（第一次放之前那里是空的）。
    """
    trips = list(trips)
    detected, transfer_obs = trips.pop(0)
    detected, transfer_obs = list(detected), list(transfer_obs)
    chosen = None
    towers = dict(towers or {})
    active_towers = []
    with open(flow_path, encoding='utf-8') as f:
        reg = FlowRegistry.from_dict(yaml.safe_load(f))
    engine = FlowEngine(reg)
    pose, slots, worst = start, {}, (float('inf'), '')
    ref = engine.start()
    while ref is not None:
        step = ref.step
        kind = type(step).__name__
        if kind == 'Detect':
            if step.region.startswith('spot_'):
                r, why = choose(spot_blocks[step.region], step.what, reg.points[step.nominal])
                assert r is not None, f'{ref}: {why}'
                slots['@' + step.slot], slots['@' + step.slot + '_pre'] = r.pick, r.pre
                ref = engine.step_done()
                continue
            if not detected and trips:   # TR 趁 BR 不在交接区时送下一趟（跟 sim_tower 一样）
                detected, transfer_obs = (list(t) for t in trips.pop(0))
            r, why = choose(detected, step.what, reg.points[step.nominal])
            assert r is not None, f'{ref}: {why}'
            slots['@' + step.slot], slots['@' + step.slot + '_pre'] = r.pick, r.pre
            chosen = r.stack
        elif kind == 'Action' and step.name == 'PLACE':
            gap = PickParams().gripper_offset
            gx, gy = pose[0] + gap * math.cos(pose[2]), pose[1] + gap * math.sin(pose[2])
            for (sx, sy), obs in list(towers.items()):
                if math.hypot(gx - sx, gy - sy) < 0.3:
                    active_towers.append(obs)
                    del towers[(sx, sy)]
        elif kind == 'Action' and step.name == 'PICK':
            base = chosen[0]
            detected = [b for b in detected if (b.x, b.y) != (base.x, base.y)]
            transfer_obs = [o for o in transfer_obs if (o[0], o[1]) != (base.x, base.y)]
        elif kind == 'Nav':
            goal = slots[step.point] if step.point.startswith('@') else reg.points[step.point]
            dx, dy = goal[0] - pose[0], goal[1] - pose[1]
            poses = [(pose[0] + dx * i / 40, pose[1] + dy * i / 40, pose[2], '平移')
                     for i in range(41)]
            poses += [(goal[0], goal[1], pose[2] + wrap(goal[2] - pose[2]) * i / 40, '转身')
                      for i in range(41)]
            for x, y, yaw, what in poses:
                pts = robot_outline(x, y, yaw)
                c = clearance(pts, transfer_obs + fixed_obstacles + active_towers)
                for bx0, bx1, by0, by1 in L2_BOXES:
                    for px, py in pts:
                        dx = max(bx0 - px, 0, px - bx1)
                        dy = max(by0 - py, 0, py - by1)
                        c = min(c, -0.01 if dx == 0 and dy == 0 else math.hypot(dx, dy))
                if not all(on_l1_or_platform(px, py) for px, py in pts):
                    c = -1.0
                worst = min(worst, (c, f'{ref} {what}'))
            pose = goal
        ref = engine.step_done()
    return worst


def scene_truth(stacks):
    """一趟货（stacks）→ (检测到的方块, 障碍物正方形)。"""
    center = (-3.5, -1.3)
    detected, obstacles = [], []
    for st in stacks:
        x, y, yaw = center[0] + st['dx'], center[1] + st['dy'], D(st['yaw_deg'])
        detected += [Block(kind, x, y, yaw, k + 1) for k, kind in enumerate(st['blocks'])]
        obstacles.append((x, y, yaw))
    return detected, obstacles


def test_demo_paths_clear_fences_and_blocks():
    with open(SCENE, encoding='utf-8') as f:
        trip = scene_truth(yaml.safe_load(f)['stacks'])
    # 演示的假电控不会真的拿走方块：第二次看的时候那一摞还在
    worst = sweep_flow(DEMO, [trip, trip], [], (-1.8, 2.6, -math.pi / 2))
    print(f'最近 {worst[0] * 100:.1f} cm（{worst[1]}）')
    assert worst[0] > 0.02, f'离围栏 / 方块最近 {worst[0] * 100:.1f} cm（{worst[1]}）'


def test_two_towers_paths_clear():
    """两座塔的流程：交接区两趟货的位置、两座塔（整场都当它们在，保守）、围栏、L2 都不碰，车不出 L1 / 交接区平台。"""
    flow = os.path.join(HERE, '..', 'config', 'flows', 'two_towers_sim.yaml')
    scene_path = os.path.join(os.path.dirname(SCENE), 'towers_plan2.yaml')
    with open(scene_path, encoding='utf-8') as f:
        scene = yaml.safe_load(f)
    # spot_07、spot_03 上的塔：第一次放下以后才出现。故意横向偏 4 cm、转 2°（放的时候车停偏了的样子；塔是在夹爪
    # 正前方放下的，不会朝车这边偏），检查第二次"对着检测到的塔基去放"时间隙还在
    towers = {(-2.7, -2.7): (-2.66, -2.7, D(2)), (-2.7, 2.7): (-2.74, 2.7, D(-2))}
    spots = {'spot_07': [Block('ground', -2.66, -2.7, D(2), 1),
                         Block('ground', -2.66, -2.7, D(2), 2)],
             'spot_03': [Block('ground', -2.74, 2.7, D(-2), 1)]}
    worst = sweep_flow(flow, [scene_truth(t) for t in scene['trips']], [],
                       (-1.8, 2.6, -math.pi / 2), spots, towers)
    print(f'最近 {worst[0] * 100:.1f} cm（{worst[1]}）')
    assert worst[0] > 0.02, f'离障碍最近 {worst[0] * 100:.1f} cm（{worst[1]}）'

"""按雷达检测到的方块算拿取位姿——纯 Python，不依赖 ROS，host 上单测。

交接区"TR 粗放、BR 雷达找块再捡"（0002 10-04 助教那条）：
    示教点当锚点：先在观察位停稳看（DETECT），挑出离"示教的拿取位姿"最近、类型对得上的那一摞，
    算出拿取位姿 = 方块位姿往外退 gripper_offset（车体 base_link 到方块中心的距离），**限幅**以后才去。

方块四面一样，朝向只测得到模 90°，所以车可以从 4 个面中的任意一面去拿：挑跟示教拿取位姿朝向最接近的那一面
（示教时就是从那一面拿的，路线也是按那一面规划的）。

检测结果是 block_detector 的 blocks（离区域中心近的那摞在前，每摞从下往上，layer 从 1 开始）。
"""

from dataclasses import dataclass
import math
from typing import List, Optional, Tuple

GROUND, SKY, UNKNOWN = 'ground', 'sky', 'unknown'

# DETECT 的"找什么" → 一摞从下往上应该是什么
SELECTORS = {
    'ground': (GROUND,),           # 单块地块
    'stack': (GROUND, GROUND),     # 2 块地块一摞（TR 第 1 趟"2 叠"）
    'sky': (SKY,),                 # 单块天空块
    'groundsky': (GROUND, SKY),    # 地 + 天一摞（TR 第 2 趟）
}


@dataclass(frozen=True)
class Block:
    """一块方块（map 系），跟 rc2027_perception_msgs/Block 对应。"""

    kind: str          # ground / sky / unknown
    x: float
    y: float
    yaw: float         # 模 90°
    layer: int
    reliable: bool = True


@dataclass(frozen=True)
class PickParams:
    # 0.575：车头雷达伸到 0.25 m、方块半边 0.175 m，离方块面留 15 cm（2026-10-07 用户定）。
    # 原来 0.45 只留 2.5 cm，比最后一步的定位滞后（5～13 cm）还小，车会顶上方块
    gripper_offset: float = 0.575       # m，拿的时候 base_link 到方块中心（沿车头方向）
    approach_distance: float = 0.30     # m，预备位姿再往后退多少（最后一段沿车头方向直着进）
    max_shift: float = 0.35             # m，拿取位姿离示教点最多差多少（交接区一半宽）
    max_yaw: float = math.radians(20)   # rad


@dataclass(frozen=True)
class PickResult:
    pick: Tuple[float, float, float]     # (x, y, yaw)
    pre: Tuple[float, float, float]
    stack: Tuple[Block, ...]
    shift: float                         # m，离示教点多远
    dyaw: float                          # rad


def wrap(a):
    """角度折回 [-pi, pi)。"""
    return (a + math.pi) % (2 * math.pi) - math.pi


def group_stacks(blocks: List[Block]) -> List[Tuple[Block, ...]]:
    """按 layer 切成一摞一摞（layer == 1 就是新的一摞）。"""
    stacks, current = [], []
    for b in blocks:
        if b.layer == 1 and current:
            stacks.append(tuple(current))
            current = []
        current.append(b)
    if current:
        stacks.append(tuple(current))
    return stacks


def composition(stack):
    """一摞从下往上的类型。"""
    return tuple(b.kind for b in stack)


# 交接区北栏杆是 x=-3.0 的一条，从 y=-0.8 往北（Gazebo 网格）。车体 0.30×0.30，车头雷达在前方 0.20 m。
# 导航只有直线，近距离校准要先开到预备位才开始，所以拿之前先把整段路扫一遍。
FENCE_BOXES = ((-3.0, -2.95, -0.8, 3.0), (-3.0, -2.95, -3.0, -1.8))
BLOCK_HALF = 0.175
BODY = 0.15
# 旁边的块留 2 cm。栏杆要留 8 cm：定位会往东偏 4 cm，余量只有 2 cm 时车身算着离北栏杆 5 cm，真车已经顶上。
CLEAR_MARGIN = 0.02
FENCE_MARGIN = 0.08
TRANSFER_GATE = (-2.30, -1.20, math.pi)
PAD_CENTER = (-3.5, -1.3)
# 检测中心贴着北栏杆时会偏北大约半个方块。往台面里最多收这么多，再多就是真顶到栏杆了。
MAX_NUDGE = 0.16


def _face(block: Block, heading: float, params: PickParams):
    c, s = math.cos(heading), math.sin(heading)
    pick = (block.x - params.gripper_offset * c, block.y - params.gripper_offset * s, heading)
    back = params.gripper_offset + params.approach_distance
    pre = (block.x - back * c, block.y - back * s, heading)
    return pick, pre


def pick_pose_for(block: Block, nominal_yaw: float, params: PickParams):
    """从方块 4 个面里挑朝向最接近 nominal_yaw 的那一面，返回 (拿取位姿, 预备位姿)。"""
    heading = min((wrap(block.yaw + k * math.pi / 2) for k in range(4)),
                  key=lambda h: abs(wrap(h - nominal_yaw)))
    return _face(block, heading, params)


def robot_points(x, y, yaw):
    """车体四角 + 车头雷达圆上的点。"""
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(x + c * u - s * v, y + s * u + c * v)
           for u in (-BODY, BODY) for v in (-BODY, BODY)]
    for k in range(8):
        a = 2 * math.pi * k / 8
        u, v = 0.20 + 0.05 * math.cos(a), 0.05 * math.sin(a)
        pts.append((x + c * u - s * v, y + s * u + c * v))
    return pts


def _on_surface(px, py):
    """L1 的 6×6，或者交接区平台。开出这两块，不是掉下去就是顶栏杆。"""
    return (abs(px) <= 3.0 and abs(py) <= 3.0) or (-4.0 <= px <= -3.0 and -1.8 <= py <= -0.8)


def _hit(px, py, obstacles):
    if not _on_surface(px, py):
        return True
    for x0, x1, y0, y1 in FENCE_BOXES:
        if max(x0 - px, 0, px - x1) == 0 and max(y0 - py, 0, py - y1) == 0:
            return True
        if math.hypot(max(x0 - px, 0, px - x1), max(y0 - py, 0, py - y1)) < FENCE_MARGIN:
            return True
    for bx, by, byaw in obstacles:
        c, s = math.cos(-byaw), math.sin(-byaw)
        u, v = c * (px - bx) - s * (py - by), s * (px - bx) + c * (py - by)
        du, dv = abs(u) - BLOCK_HALF, abs(v) - BLOCK_HALF
        gap = -0.01 if max(du, dv) <= 0 else math.hypot(max(du, 0), max(dv, 0))
        if gap < CLEAR_MARGIN:
            return True
    return False


def pose_hits(x, y, yaw, obstacles):
    return any(_hit(px, py, obstacles) for px, py in robot_points(x, y, yaw))


def _segment_hits(a, b, yaw, obstacles, n=10):
    for i in range(n + 1):
        t = i / n
        if pose_hits(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, yaw, obstacles):
            return True
    return False


def approach_is_clear(pick, pre, obstacles, gate=TRANSFER_GATE):
    """门口直线到预备位（车头先不转），原地转到拿取朝向，再走进去。任一处擦到就不算能拿。"""
    if _segment_hits(gate, pre, gate[2], obstacles):
        return False
    steps = 8
    for i in range(steps + 1):
        yaw = gate[2] + wrap(pick[2] - gate[2]) * i / steps
        if pose_hits(pre[0], pre[1], yaw, obstacles):
            return False
    return not _segment_hits(pre, pick, pick[2], obstacles)


def _body_on_surface(x, y, yaw):
    return all(_on_surface(px, py) for px, py in robot_points(x, y, yaw))


def _backup(pick, pre):
    """预备位在拿取位正后方。退后会开出台面时，收到车身还在台面上的最远位置。"""
    if _body_on_surface(*pre):
        return pre
    best = None
    for i in range(1, 21):
        t = i / 20.0
        x = pick[0] + (pre[0] - pick[0]) * t
        y = pick[1] + (pre[1] - pick[1]) * t
        if not _body_on_surface(x, y, pick[2]):
            break
        best = (x, y, pick[2])
    return best


def _clear_face(block, heading, obstacles, params):
    """这一面，必要时把块心往台面中心收，直到整段路离栏杆和旁边的块都够远。"""
    toward = (PAD_CENTER[0] - block.x, PAD_CENTER[1] - block.y)
    norm = math.hypot(*toward) or 1.0
    step = (toward[0] / norm, toward[1] / norm)
    for k in range(int(MAX_NUDGE / 0.02) + 1):
        dist = k * 0.02
        moved = Block(block.kind, block.x + step[0] * dist, block.y + step[1] * dist,
                      block.yaw, block.layer, block.reliable)
        pick, pre = _face(moved, heading, params)
        pre = _backup(pick, pre)
        if pre is not None and approach_is_clear(pick, pre, obstacles):
            return pick, pre
    return None


def safe_pick(block: Block, nominal_yaw: float, obstacles, params: PickParams):
    """四面里挑示教朝向最近、而且整段不擦栏杆和旁边块的一面。

    方块转了几度时，贴着栏杆的那一面车身一角会扫进去。再试四个正方向：
    车头仍对准块心，只是不跟着那几度一起转。检测中心贴着北栏杆时会偏北，
    按那个点停车车身会顶上栏杆；往台面里收，收到擦不到为止。
    没有擦不到的面就返回 None，这摞先不拿。
    """
    headings = [wrap(block.yaw + k * math.pi / 2) for k in range(4)]
    headings += [0.0, math.pi / 2, math.pi, -math.pi / 2]
    ok = []
    seen = set()
    for heading in headings:
        key = round(heading, 5)
        if key in seen:
            continue
        seen.add(key)
        face = _clear_face(block, heading, obstacles, params)
        if face is not None:
            ok.append((*face, heading))
    if not ok:
        return None
    return min(ok, key=lambda f: abs(wrap(f[2] - nominal_yaw)))


def blocks_we_can_reach(blocks: List[Block], nominal_yaw: float = math.pi,
                        params: PickParams = PickParams()) -> List[Block]:
    """擦得到栏杆或旁边块的那几摞先拿掉，不当成现在能拿的货。不在交接区的块不动。"""
    stacks = group_stacks(list(blocks))
    kept = []
    for stack in stacks:
        if stack[0].x >= -2.5:
            kept.extend(stack)
            continue
        others = [(b.x, b.y, b.yaw) for other in stacks if other is not stack for b in other]
        if safe_pick(stack[0], nominal_yaw, others, params) is not None:
            kept.extend(stack)
    return kept


def choose(blocks: List[Block], what: str, nominal: Tuple[float, float, float],
           params: PickParams = PickParams()) -> Tuple[Optional[PickResult], str]:
    """挑一摞、算拿取位姿。返回 (结果, 说明)；结果是 None 时说明里写为什么。

    Args:
        blocks: 检测到的方块（block_detector 的顺序）。
        what: SELECTORS 里的一个，或者 'any'（任何认得出的一摞）。
        nominal: 示教的拿取位姿 (x, y, yaw)。
    """
    if what != 'any' and what not in SELECTORS:
        return None, f'不认识要找的 {what}（只有 {"/".join(SELECTORS)}/any）'
    stacks = group_stacks(blocks)
    wanted = [s for s in stacks
              if (composition(s) == SELECTORS[what] if what != 'any'
                  else UNKNOWN not in composition(s))]
    if not wanted:
        seen = '、'.join('+'.join(composition(s)) for s in stacks) or '什么都没有'
        return None, f'没找到 {what}（看到的：{seen}）'
    # 抓整摞：按最下面那块定位（上面的块可能放歪，但夹爪 / 导向对的是底下那块）
    usable = [s for s in wanted if s[0].reliable]
    if not usable:
        return None, f'找到 {len(wanted)} 摞 {what}，但都只看到一部分面，中心不可靠（换个角度再看）'
    best = None
    forced = False
    for s in usable:
        others = [(b.x, b.y, b.yaw) for other in stacks if other is not s for b in other]
        face = safe_pick(s[0], nominal[2], others, params)
        if face is None:
            continue
        pick, pre, heading = face
        shift = math.hypot(pick[0] - nominal[0], pick[1] - nominal[1])
        dyaw = wrap(heading - nominal[2])
        straight = pick_pose_for(s[0], nominal[2], params)
        used_other_face = abs(wrap(heading - straight[0][2])) > 1e-6
        if best is None or shift < best.shift:
            best = PickResult(pick, pre, s, shift, dyaw)
            forced = used_other_face
    if best is None:
        return None, f'{what} 的拿取会擦到栏杆或旁边的块，先拿碰不到的'
    # 换面是为了躲开栏杆和旁边的块，朝向可以离示教远一点。
    if not forced and (best.shift > params.max_shift or abs(best.dyaw) > params.max_yaw):
        return None, (f'最近的 {what} 离示教拿取点 {best.shift * 100:.0f} cm / '
                      f'{math.degrees(best.dyaw):+.0f}°，超过限幅 '
                      f'{params.max_shift * 100:.0f} cm / {math.degrees(params.max_yaw):.0f}°'
                      '（TR 放太偏了？）')
    return best, (f'{what} 在 ({best.stack[0].x:.3f}, {best.stack[0].y:.3f}) '
                  f'朝向 {math.degrees(best.stack[0].yaw):+.1f}°，拿取位姿离示教点 '
                  f'{best.shift * 100:.1f} cm / {math.degrees(best.dyaw):+.1f}°')

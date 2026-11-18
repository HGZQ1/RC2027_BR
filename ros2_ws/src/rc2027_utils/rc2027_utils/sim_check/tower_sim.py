"""搭塔仿真里"真的搬方块"的逻辑——纯 Python，不依赖 ROS，host 上单测。

sim_tower 节点（假电控）收到 PICK / PLACE 时用这里算：拿哪几块、放到哪、放完每层投影离建造点边多少（规则图 7：
每一层的竖直投影都要完全在 500x500 建造点里才得分）。方块的位置全按 **Gazebo 真值**（车的真实位姿）算——定位有误差，
方块就真的放偏，这正是要测的。

坐标全是 Gazebo world（= 仿真里的 map）：台面比规则高 z_offset（0.05）。
"""

from dataclasses import dataclass, field, replace
import math
import random
from typing import Dict, List, Optional

from rc2027_utils.sim_check.block_scene import SIZES

SPOT_HALF = 0.25   # 建造点 500x500


@dataclass
class SimBlock:
    name: str
    kind: str            # ground / sky
    x: float
    y: float
    z_bottom: float
    yaw: float
    face: str = ''       # 天空块朝上的颜色：red 我们的 / blue 对方的。地块空着

    @property
    def size(self):
        return SIZES[self.kind]


@dataclass
class TowerWorld:
    """所有方块的真实位置 + 车上拿着的。"""

    blocks: Dict[str, SimBlock] = field(default_factory=dict)
    carried: List[str] = field(default_factory=list)     # 从下往上
    regions: Dict[str, tuple] = field(default_factory=dict)   # 区域名 → (cx, cy, 规则台面高)
    z_offset: float = 0.05
    gripper_offset: float = 0.575   # 跟 flow_runner 的默认一样（离方块面 15 cm，2026-10-07）；不一致的话方块会放短
    grab_radius: float = 0.15

    # ── 查询 ──
    def gripper_point(self, robot):
        """车（x, y, yaw）的夹爪作用点。"""
        x, y, yaw = robot
        return x + self.gripper_offset * math.cos(yaw), y + self.gripper_offset * math.sin(yaw)

    def column_at(self, x, y):
        """(x, y) 附近 grab_radius 内落在场上（不在车上）的方块，从下往上。"""
        out = [b for n, b in self.blocks.items() if n not in self.carried and
               math.hypot(b.x - x, b.y - y) < self.grab_radius]
        return sorted(out, key=lambda b: b.z_bottom)

    def surface_z(self, x, y):
        """(x, y) 处台面高度（仿真）：按 levels 判断 L2 / L1 / 地面，交接区也是 L1 高度。"""
        for name, (cx, cy, z) in self.regions.items():
            if name.startswith('transfer_') and abs(x - cx) <= 0.5 and abs(y - cy) <= 0.5:
                return z + self.z_offset
        if abs(x) <= 1.5 and abs(y) <= 1.5:
            return 0.9 + self.z_offset
        if abs(x) <= 3.0 and abs(y) <= 3.0:
            return 0.6 + self.z_offset
        return self.z_offset

    def in_region(self, name, b):
        cx, cy, _ = self.regions[name]
        half = 0.5 if name.startswith('transfer_') else SPOT_HALF + 0.2
        return abs(b.x - cx) <= half and abs(b.y - cy) <= half

    def region_empty(self, name):
        return not any(self.in_region(name, b) for n, b in self.blocks.items()
                       if n not in self.carried)

    # ── 动作 ──
    def pick(self, robot, n) -> str:
        """拿夹爪前面那一摞最上面的 n 块（n = 整摞就是整摞抓）。成功返回空字符串，失败返回原因。"""
        if self.carried:
            return f'手里已经有 {len(self.carried)} 块'
        gx, gy = self.gripper_point(robot)
        col = self.column_at(gx, gy)
        if len(col) < n:
            return f'夹爪前面只有 {len(col)} 块，要拿 {n} 块'
        self.carried = [b.name for b in col[-n:]]
        return ''

    def place(self, robot, level, rng: random.Random, sigma_xy=0.01, sigma_yaw_deg=1.0) -> str:
        """放到夹爪前面那个位置的第 level 层（1 = 台面上）。放的高度跟那里已有的层数对不上就失败（真车会砸）。"""
        if not self.carried:
            return '手里没有方块'
        gx, gy = self.gripper_point(robot)
        col = self.column_at(gx, gy)
        base = self.surface_z(gx, gy)
        height = sum(b.size for b in col)
        if abs((level - 1) * SIZES['ground'] - height) > 0.02 or \
                any(b.kind == 'sky' for b in col):
            return (f'要放到第 {level} 层，但那里已经是 {len(col)} 块'
                    f'（{"+".join(b.kind for b in col) or "空的"}）')
        x = gx + rng.gauss(0, sigma_xy)
        y = gy + rng.gauss(0, sigma_xy)
        yaw = robot[2] + math.radians(rng.gauss(0, sigma_yaw_deg))
        z = base + height
        # 塔已经两层，手里却是「地+天」。地放上去会变成三层，只把顶上的天空盖上，地留在手里。
        names = list(self.carried)
        tops = [self.blocks[n] for n in names]
        grounds = sum(b.kind == 'ground' for b in col)
        if len(tops) == 1 and tops[0].kind == 'sky' and grounds < 2:
            return f'天空要放在两层地上，这里只有 {grounds} 层地'
        if (len(col) >= 2 and len(tops) >= 2 and tops[-1].kind == 'sky'
                and any(b.kind == 'ground' for b in tops[:-1])):
            names = names[-1:]
            self.carried = self.carried[:-1]
        else:
            self.carried = []
        for name in names:
            b = self.blocks[name]
            # 整摞放：上面那块相对下面那块再抖几毫米
            self.blocks[name] = replace(b, x=x + rng.gauss(0, 0.003), y=y + rng.gauss(0, 0.003),
                                        z_bottom=z, yaw=yaw + math.radians(rng.gauss(0, 0.5)))
            z += b.size
        return ''

    def flip(self, robot):
        """翻夹爪前面最上面的天空块：朝上的一面红蓝对调。返回 (原因或空, 翻过的那块或 None)。"""
        if self.carried:
            return f'手里还有 {len(self.carried)} 块，先放下再翻', None
        gx, gy = self.gripper_point(robot)
        col = [b for b in self.column_at(gx, gy) if b.kind == 'sky']
        if not col:
            return '夹爪前面没有天空块', None
        top = col[-1]
        face = 'blue' if top.face == 'red' else 'red'
        self.blocks[top.name] = replace(top, face=face)
        return '', self.blocks[top.name]

    def add(self, b: SimBlock):
        self.blocks[b.name] = b


def projection_margin(b: SimBlock, spot_xy):
    """方块竖直投影离建造点 500x500 边的最小距离（m），负数 = 伸出去了（这一层不得分）。"""
    sx, sy = spot_xy
    c, s = math.cos(b.yaw), math.sin(b.yaw)
    h = b.size / 2
    worst = float('inf')
    for u, v in ((h, h), (h, -h), (-h, h), (-h, -h)):
        px, py = b.x + c * u - s * v, b.y + s * u + c * v
        worst = min(worst, SPOT_HALF - abs(px - sx), SPOT_HALF - abs(py - sy))
    return worst


def tower_report(world: TowerWorld, spot_names) -> List[str]:
    """每个建造点：每层（从下往上）的类型、中心偏差、投影余量；是不是完整的塔（2 地 + 天、每层都在里面）。"""
    lines = []
    for name in spot_names:
        cx, cy, _ = world.regions[name]
        col = [b for n, b in world.blocks.items()
               if n not in world.carried and world.in_region(name, b)]
        col.sort(key=lambda b: b.z_bottom)
        if not col:
            continue
        parts, ok = [], True
        for k, b in enumerate(col, start=1):
            m = projection_margin(b, (cx, cy))
            ok &= m >= 0
            who = {'red': ' 红', 'blue': ' 蓝'}.get(b.face, '')
            parts.append(f'第{k}层 {b.kind:6s}{who} 偏 {math.hypot(b.x - cx, b.y - cy) * 100:4.1f} cm '
                         f'余量 {m * 100:+5.1f} cm{"" if m >= 0 else " ✗出界"}')
        complete = [b.kind for b in col] == ['ground', 'ground', 'sky'] and ok
        lines.append(f'{name}: {len(col)} 块{"，完整的塔 ✓" if complete else ""}')
        lines += ['    ' + p for p in parts]
    return lines


def supply_trip(which: int):
    """脚本里的趟数用完以后补的一趟。偶数趟远处一块地、近处两地，奇数趟地天一摞加一块天。"""
    nudge = 0.01 * ((which % 3) - 1)
    if which % 2 == 0:
        return [
            {'region': 'transfer_red', 'dx': -0.20 + nudge, 'dy': 0.15,
             'blocks': ['ground'], 'yaw_deg': -4},
            {'region': 'transfer_red', 'dx': 0.24, 'dy': -0.24,
             'blocks': ['ground', 'ground'], 'yaw_deg': 6},
        ]
    return [
        {'region': 'transfer_red', 'dx': -0.18 + nudge, 'dy': 0.16,
         'blocks': ['ground', 'sky'], 'yaw_deg': 4},
        {'region': 'transfer_red', 'dx': 0.22, 'dy': -0.22,
         'blocks': ['sky'], 'yaw_deg': -8},
    ]


def tower_kinds(world: TowerWorld, spot: str):
    """这座塔上从下往上的种类。不算手里的。"""
    if spot not in world.regions:
        return []
    cx, cy, _ = world.regions[spot]
    col = [b for n, b in world.blocks.items()
           if n not in world.carried and math.hypot(b.x - cx, b.y - cy) < 0.3]
    col.sort(key=lambda b: b.z_bottom)
    return [b.kind for b in col]


def trip_blocks(trip, regions, z_offset, start_index) -> List[SimBlock]:
    """场景里一趟 TR 送的货（跟 block_scene 的 stacks 一样的写法）→ 方块，名字从 start_index 接着编。"""
    from rc2027_utils.sim_check.block_scene import scene_blocks
    out = []
    for b in scene_blocks({'z_offset': z_offset, 'stacks': trip}, regions):
        face = 'red' if b['type'] == 'sky' else ''
        out.append(SimBlock(f'blk_{start_index + len(out):03d}', b['type'], b['x'], b['y'],
                            b['z_bottom'], b['yaw'], face))
    return out


# 只随机 L1 共享区。自己的塔开局是空的，不给它们盖蓝天空
L1_SHARED = ('spot_01', 'spot_02')
BLUE_KINDS = ('一层蓝', '两层蓝', '全蓝')


def _stack_at(world_or_blocks, cx, cy):
    blocks = world_or_blocks.values() if isinstance(world_or_blocks, dict) else world_or_blocks
    col = [b for b in blocks if math.hypot(b.x - cx, b.y - cy) < 0.4]
    return sorted(col, key=lambda b: b.z_bottom)


def random_field_blocks(rng, regions, z_offset, start_index, spots=L1_SHARED):
    """L1 两座共享区各随机一种：一层地、两层地、或两层地加蓝天空。自己的塔不摆。"""
    blocks = []
    lines = []
    i = start_index
    for name in spots:
        if name not in regions:
            continue
        kind = rng.choice(BLUE_KINDS)
        ground = 1 if kind == '一层蓝' else 2
        cx, cy, cz = regions[name]
        z = cz + z_offset
        dx, dy = rng.uniform(-0.02, 0.02), rng.uniform(-0.02, 0.02)
        yaw = math.radians(rng.uniform(-8, 8))
        for _layer in range(ground):
            blocks.append(SimBlock(
                f'blk_{i:03d}', 'ground', cx + dx, cy + dy, z, yaw, 'blue'))
            z += SIZES['ground']
            i += 1
        if kind == '全蓝':
            blocks.append(SimBlock(f'blk_{i:03d}', 'sky', cx + dx, cy + dy, z, yaw, 'blue'))
            i += 1
        lines.append(f'{name}: {kind}')
    return blocks, lines


def blue_next(blocks, regions, spot, index):
    """蓝方往这座共享塔上再盖一块。一层蓝→两层蓝，两层蓝→全蓝。已经全蓝返回 None。"""
    if spot not in regions:
        return None
    cx, cy, cz = regions[spot]
    col = _stack_at(blocks, cx, cy)
    if not col or any(b.kind == 'sky' for b in col):
        return None
    top = col[-1]
    z = top.z_bottom + top.size
    if len(col) == 1:
        return SimBlock(f'blk_{index:03d}', 'ground', top.x, top.y, z, top.yaw, 'blue')
    return SimBlock(f'blk_{index:03d}', 'sky', top.x, top.y, z, top.yaw, 'blue')


def find_spot(world: TowerWorld, x, y) -> Optional[str]:
    """(x, y) 落在哪个建造点（中心 0.3 m 内）。"""
    for name, (cx, cy, _) in world.regions.items():
        if name.startswith('spot_') and math.hypot(x - cx, y - cy) < 0.3:
            return name
    return None

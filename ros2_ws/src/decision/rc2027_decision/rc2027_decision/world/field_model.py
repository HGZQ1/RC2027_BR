"""世界模型：每个建造点"现在是什么样"——纯 Python，不依赖 ROS，host 上单测。

把雷达多次看到的（block_detector 的 RegionState）和自己做过的（PICK / PLACE）合并起来，给规划器用
（0002 10-04 助教那条"世界模型 + 规划器"）。要防的坑都是那条里列的：

    1. 看到一次不算数：对方车 / 人站在建造点上、被举着的方块，block_detector 会报 UNKNOWN——这次观测
       **不用**，只记一笔"被挡"。层数变了要连续几次看到同样的结果才改（往上 confirm_up 次；往下更严，
       confirm_down 次，因为正常比赛里塔只会变高）
    2. 顶部出了雷达视场（top_maybe_clipped）时层数是"至少"：只能用来往上改，不能往下改
       同理，**看得不清的时候也只能往上改**：层数变少只认"正对着看、距离合适"的观测（车头朝向建造点、
       离 clear_min_dist ～ clear_max_dist），因为天空块比地块往里缩 7.5 cm、又最高，背对着 / 太近时
       最先看不见（2026-10-06 仿真：车在交接区拿货，spot_07 在车后方 1.7 m，连续 4 次没看到天空块，
       误报"变少了"）
    3. 覆盖率低时"什么都没看到"不当成"空的"
    4. 自己车跟前的建造点不看：车顶到塔跟前时前倾雷达被挡掉一大半、夹爪举着的方块会被当成悬空的东西
       （0002 10-05）。离车 self_ignore_radius 以内的建造点这次观测跳过
    5. 自己刚放完的：记下"应该一共几块"（expected_total），下一次看到正好这么多块就直接确认，
       不用等 confirm_up 次；哪几层是我们放的记在 ours 里（天空块是谁放的 ≈ 塔归谁，雷达看不出颜色）

开局按规则所有建造点都是空的（source = prior），不用先看一眼才知道。
时间都是秒（ROS 时钟），由调用方给。
"""

from dataclasses import dataclass, field
import math
from typing import Dict, List, Mapping, Optional, Set, Tuple

from ..flow.pick_pose import Block, group_stacks, GROUND, UNKNOWN

PRIOR, SEEN = 'prior', 'seen'
SKY_NONE, SKY_OURS, SKY_OTHERS = 'none', 'ours', 'others'


@dataclass(frozen=True)
class Observation:
    """block_detector 看了一次某个区域（rc2027_perception_msgs/RegionState 的子集）。"""

    region: str
    stamp: float
    visible: bool
    coverage: float
    ground_layers: int
    has_sky: bool
    clipped: bool = False
    blocks: Tuple[Block, ...] = ()
    note: str = ''


@dataclass(frozen=True)
class ModelParams:
    confirm_up: int = 2                 # 层数变多：连续看到几次才改
    confirm_down: int = 4               # 层数变少（被撞倒 / 看错）：更严
    min_empty_coverage: float = 0.4     # "空的"要看到建造点这么多面积才算数
    self_ignore_radius: float = 0.8     # m，离车（base）这么近的建造点不看
    clear_min_dist: float = 1.8         # m，"看得清"的距离范围：太近雷达往上看不到天空块
    clear_max_dist: float = 5.0         # m，太远点太稀
    clear_half_angle: float = math.radians(40)   # 建造点要在车头前方这个半角以内
    place_match_radius: float = 0.35    # m，PLACE 时夹爪作用点离建造点中心多远以内算放在它上面
    base_match_radius: float = 0.35     # m，塔基中心离建造点中心多远以内才记
    base_jump: float = 0.05             # m，塔基位置一下子差这么多就直接换（被撞过），不平均
    base_alpha: float = 0.3             # 塔基位置的滑动平均系数


@dataclass
class SpotBelief:
    """一个建造点现在的结论 + 推导它用的记录。"""

    name: str
    x: float
    y: float
    ground: int = 0
    sky: bool = False
    source: str = PRIOR
    confirmations: int = 0
    since: float = 0.0                          # 最后一次跟结论一致的观测（prior 时 = 开局）
    candidate: Optional[Tuple[int, bool]] = None  # 跟结论不一样、还在数次数的观测
    candidate_count: int = 0
    obstructed_at: Optional[float] = None       # 最近一次"被挡、没用"的观测
    obstructed_note: str = ''
    ours: Set[int] = field(default_factory=set)  # 我们放的是第几层（1 起）
    expected_total: int = 0                     # 刚放完、等雷达确认的总块数；0 = 没在等
    base: Optional[Tuple[float, float, float]] = None   # 塔基 (x, y, yaw 模 90°)

    @property
    def total(self):
        return self.ground + (1 if self.sky else 0)

    @property
    def sky_owner(self):
        if not self.sky:
            return SKY_NONE
        return SKY_OURS if self.total in self.ours else SKY_OTHERS


def describe(ground, sky):
    """(2, True) → "2 层 + 天空块"。"""
    return f'{ground} 层{" + 天空块" if sky else ""}'


def dominates(a, b):
    """a 比 b 高（每一项都不少、而且不一样）：正常往上搭的方向。"""
    return a != b and a[0] >= b[0] and a[1] >= b[1]


def _wrap90(a):
    """角度折回 [-45°, 45°)。"""
    q = math.pi / 2
    return (a + q / 2) % q - q / 2


def find_spot(spots: Mapping[str, Tuple[float, float]], x, y, radius):
    """(x, y) 落在哪个建造点上（离中心 radius 以内最近的），都不是返回 None。"""
    best, best_d = None, radius
    for name, (sx, sy) in spots.items():
        d = math.hypot(x - sx, y - sy)
        if d <= best_d:
            best, best_d = name, d
    return best


class FieldModel:
    """所有建造点的结论 + 车上拿着几块。"""

    def __init__(self, spots: Mapping[str, Tuple[float, float]], start_time=0.0,
                 params: ModelParams = ModelParams()):
        self.params = params
        self.spots: Dict[str, SpotBelief] = {
            name: SpotBelief(name, float(x), float(y), since=start_time)
            for name, (x, y) in spots.items()}
        self.held = 0

    # ── 雷达观测 ──
    def clear_view(self, s: 'SpotBelief', robot_xy, robot_yaw) -> bool:
        """车是不是正对着这个建造点、距离合适——这时"层数变少"才可信。查不到车的位姿算看不清。"""
        if robot_xy is None or robot_yaw is None:
            return False
        dx, dy = s.x - robot_xy[0], s.y - robot_xy[1]
        dist = math.hypot(dx, dy)
        if not self.params.clear_min_dist <= dist <= self.params.clear_max_dist:
            return False
        bearing = math.atan2(dy, dx) - robot_yaw
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))
        return abs(bearing) <= self.params.clear_half_angle

    def observe(self, obs: Observation, robot_xy=None, robot_yaw=None) -> Optional[str]:
        """吃一次观测。结论变了返回一句说明（打日志用），否则 None。

        robot_xy / robot_yaw 是车在 map 里的位置和朝向：离得太近的建造点不看；
        层数变少只在 clear_view 时才认。
        """
        if obs.region not in self.spots:
            return None
        s = self.spots[obs.region]
        p = self.params
        if not obs.visible:
            return None
        if robot_xy is not None and \
                math.hypot(robot_xy[0] - s.x, robot_xy[1] - s.y) < p.self_ignore_radius:
            return None
        if any(b.kind == UNKNOWN for b in obs.blocks):
            # 顶出了视场时，对不上的高度是没看全的塔，不是挡在前面的车。
            # 标成被挡的话，已经确认的对方天空就不会去翻，只会退远再看。
            if not obs.clipped:
                s.obstructed_at = obs.stamp
                s.obstructed_note = obs.note
            return None

        value = (int(obs.ground_layers), bool(obs.has_sky))
        if value == (0, False) and obs.coverage < p.min_empty_coverage:
            return None
        belief = (s.ground, s.sky)
        if value == belief:
            s.source = SEEN
            s.confirmations += 1
            s.since = obs.stamp
            s.candidate, s.candidate_count = None, 0
            if s.expected_total == s.total:
                s.expected_total = 0
            self._update_base(s, obs.blocks)
            return None
        up = dominates(value, belief)
        if obs.clipped and not up:
            return None  # 只看到了下半截，跟"更高"不矛盾
        if not up and not self.clear_view(s, robot_xy, robot_yaw):
            return None  # 背对着 / 太近时看不见上面的天空块，不能当成"变少了"

        if s.candidate == value:
            s.candidate_count += 1
        else:
            s.candidate, s.candidate_count = value, 1
        total = value[0] + (1 if value[1] else 0)
        if s.expected_total and total == s.expected_total:
            needed = 1
        else:
            needed = p.confirm_up if up else p.confirm_down
        if s.candidate_count < needed:
            return None

        why = ('正好是刚放完应该有的' if needed == 1 and s.expected_total
               else f'连续 {s.candidate_count} 次')
        # 层数变多而且多出来的是天空，又不是我们刚放的：蓝方自动盖的。
        blue = (up and value[1] and not belief[1]
                and not (s.expected_total and total == s.expected_total))
        message = (f'{s.name}: {describe(*belief)} → {describe(*value)}（{why}'
                   f'{"" if up else "，变少了：被撞了还是之前看错了？"}'
                   f'{"，蓝方自动放的天空" if blue else ""}）')
        s.ground, s.sky = value
        s.source = SEEN
        s.confirmations = s.candidate_count
        s.since = obs.stamp
        s.candidate, s.candidate_count = None, 0
        if s.expected_total and total == s.expected_total:
            s.expected_total = 0
        # 刚放完、雷达先看到下面几层时，不要把还没入镜的那几层从「我们放的」里划掉。
        # 划掉之后天空块会被当成对方的，车就会反复去翻自己的塔。
        if not (s.expected_total and total < s.expected_total):
            s.ours = {layer for layer in s.ours if layer <= total}
        if s.ground == 0:
            s.base = None
        self._update_base(s, obs.blocks)
        return message

    def _update_base(self, s: SpotBelief, blocks):
        """塔基（最下面那块地块）的位置：放第 2 层以上要对着它放。"""
        if s.ground == 0:
            return
        stacks = group_stacks(list(blocks))
        if not stacks:
            return
        b = stacks[0][0]
        if b.kind != GROUND or b.layer != 1 or not b.reliable or \
                math.hypot(b.x - s.x, b.y - s.y) > self.params.base_match_radius:
            return
        yaw = _wrap90(b.yaw)
        if s.base is None or math.hypot(b.x - s.base[0], b.y - s.base[1]) > self.params.base_jump:
            s.base = (b.x, b.y, yaw)
            return
        a = self.params.base_alpha
        bx, by, byaw = s.base
        s.base = (bx + a * (b.x - bx), by + a * (b.y - by),
                  _wrap90(byaw + a * _wrap90(yaw - byaw)))

    # ── 自己做的 ──
    def on_pick(self, count):
        """PICK 做完了：车上多了 count 块。"""
        self.held += max(1, int(count))

    def on_place(self, layer, gripper_xy, stamp, *, one=False) -> Tuple[Optional[str], str]:
        """PLACE 做完了：layer 是最下面那块放到第几层。返回 (建造点名或 None, 说明)。"""
        count = 1 if one else max(1, self.held)
        self.held = max(0, self.held - count)
        name = find_spot({n: (s.x, s.y) for n, s in self.spots.items()},
                         gripper_xy[0], gripper_xy[1], self.params.place_match_radius)
        if name is None:
            return None, (f'PLACE {layer}：夹爪在 ({gripper_xy[0]:.2f}, {gripper_xy[1]:.2f})，'
                          '不在任何建造点上（交接区 / 放偏了？），不记')
        s = self.spots[name]
        layer = max(1, int(layer))
        s.ours |= set(range(layer, layer + count))
        s.expected_total = layer - 1 + count
        return name, (f'{name}: 放了 {count} 块（第 {layer} 层起），应该一共 {s.expected_total} 块，'
                      '等雷达确认')

    def on_flip(self, gripper_xy, stamp) -> Tuple[Optional[str], str]:
        """FLIP 做完：夹爪前面这座塔的天空改成我们的（雷达看不出颜色，只能记账）。"""
        name = find_spot({n: (s.x, s.y) for n, s in self.spots.items()},
                         gripper_xy[0], gripper_xy[1], self.params.place_match_radius)
        if name is None:
            return None, (f'FLIP：夹爪在 ({gripper_xy[0]:.2f}, {gripper_xy[1]:.2f})，'
                          '不在任何建造点上，不记')
        s = self.spots[name]
        if s.ground < 2:
            s.ground = 2
        s.sky = True
        top = s.ground + 1
        s.ours.add(top)
        s.expected_total = top
        s.since = stamp
        return name, f'{name}: 翻转，天空朝上改成我们的'

    def on_retry(self):
        """Retry：车被搬回去重来，手里的不算了（场上的塔不变）。"""
        self.held = 0

    # ── 给规划器 / 话题用 ──
    def note(self, s: SpotBelief, now) -> str:
        """一句中文说明（话题里的 note）。"""
        parts = []
        if s.expected_total:
            parts.append(f'刚放完，等雷达确认一共 {s.expected_total} 块')
        if s.candidate is not None:
            needed = self.params.confirm_up if dominates(s.candidate, (s.ground, s.sky)) \
                else self.params.confirm_down
            parts.append(f'看到过 {describe(*s.candidate)}（{s.candidate_count}/{needed} 次）')
        if s.obstructed_at is not None and now - s.obstructed_at < 5.0:
            parts.append('最近被挡：' + (s.obstructed_note or '有不认识的东西'))
        return '；'.join(parts)

    def spot_list(self) -> List[SpotBelief]:
        """按名字排好的建造点。"""
        return [self.spots[n] for n in sorted(self.spots)]

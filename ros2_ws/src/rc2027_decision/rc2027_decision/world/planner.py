"""规则规划器——纯 Python，不依赖 ROS，host 上单测。

读每个建造点现在什么样（世界模型）+ spot_approaches.yaml 里的接近位姿，选出下一件。
规则以 0002 2026-10-07「主线按这个开工」为准：

    L2 自己两座封顶前只建自己的塔。spot_07、spot_08、spot_05、spot_03 按这批货能否放置选择。
    L2 都封顶以后才开放 L1 翻塔和共享放置。开到跟前发现层数变化时按手持货和同一优先级改派。
    一次最多拿 2 块。空塔碰到两块地叠在一起，一次拿走、一次放下。1 层地碰到「地+天空」
    一摞，一次拿走把这座塔封顶。没有整摞就还是一块一块拿。

停机重试三种触发（定位起飞、导航 STUCK、手动）都先停车。真正调 /field_localizer/retry
要等车已经在 Retry Zone 里——车还在场中就锁定，会把地图锁飞（2026-10-07 那场）。
"""

import math
import os
from dataclasses import dataclass, replace
from typing import Dict, Mapping, Optional, Tuple

import yaml

from .field_model import SKY_NONE, SKY_OTHERS, SKY_OURS

ORDER = ('spot_07', 'spot_01', 'spot_03', 'spot_02')
SHARED = frozenset({'spot_01', 'spot_02'})
# 南边两座。北共享要另外封顶才上楼，不在这里面，否则会插到南边前面。
MANDATE = ('spot_07', 'spot_01')
# 全场结束前还要完成的 L1 目标。
L1_TARGETS = ('spot_07', 'spot_01', 'spot_02')
# L2 靠红方的半边。先南后北。
L2_OURS = ('spot_08', 'spot_05')
OWN_BUILD_ORDER = ('spot_07', *L2_OURS, 'spot_03')
SOUTH_SHARED = 'spot_01'

LOOK, FLIP, PLACE_GROUND, PLACE_SKY, WAIT, CLIMB, MUSTIKA, STOP = (
    '看', '翻转', '放地', '放天空', '等货', '上楼', '五色石', '停下')


@dataclass(frozen=True)
class Tower:
    """一个建造点给规划器用的结论。seen：这次看清了（没出视场、没被挡）。"""

    ground: int = 0
    sky_owner: str = SKY_NONE
    seen: bool = False
    clipped: bool = False
    blocked: bool = False


@dataclass(frozen=True)
class Goods:
    """交接区现在能拿的。ground_stack / cap_stack 是能整摞抓走的摞数（一次最多 2 块）。"""

    ground: int = 0
    sky: int = 0
    ground_stack: int = 0    # 两块地叠在一起
    cap_stack: int = 0       # 地块上叠着天空块


@dataclass(frozen=True)
class Choice:
    """下一件。pose 是接近位姿 (x, y, theta)，看用 look，放用 place，没有就是 None。"""

    action: str
    spot: Optional[str]
    why: str
    pose: Optional[Tuple[float, float, float]] = None
    assumed_ground: Optional[int] = None    # 出发时这个点有几层地。开到跟前对不上就重选
    assumed_sky: Optional[str] = None
    count: int = 1                          # 这次拿几块。2 = 整摞（两地，或一地加天空）

    def line(self):
        """一行字，给脚本打印。"""
        where = self.spot or ''
        pose = ''
        if self.pose is not None:
            pose = f'  ({self.pose[0]:.3f}, {self.pose[1]:.3f}, {self.pose[2]:.3f})'
        return f'{self.action} {where}{pose}  — {self.why}'.replace('  —', ' —')


def load_approaches(path=None) -> dict:
    """spot_approaches.yaml。没给路径就找源码树里那份，找不到再找安装目录。"""
    if path is None:
        path = os.path.join(os.path.dirname(__file__), '..', '..', 'config',
                            'spot_approaches.yaml')
        if not os.path.isfile(path):
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory('rc2027_decision'),
                                'config', 'spot_approaches.yaml')
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def _pose(approaches, spot, key):
    block = approaches.get('spots', {}).get(spot, {})
    p = block.get(key)
    if not p:
        return None
    return (float(p['x']), float(p['y']), float(p['theta']))


def _done(tower: Tower) -> bool:
    return tower.sky_owner == SKY_OURS


def _mandate_done(towers: Mapping[str, Tower]) -> bool:
    return all(_done(towers[name]) for name in MANDATE)


def _unclear(tower: Tower) -> Optional[str]:
    """看不清的原因。清楚返回 None。"""
    if tower.clipped:
        return '顶部出了视场，层数不能用来放或者翻，退远再看'
    if tower.blocked:
        return '被挡着，这次不能用'
    if not tower.seen:
        return '还没看清'
    return None


def _almost_done(tower: Tower) -> bool:
    """看清了，没有天空，只差天空（2 层地）。一层地不插到自己的南塔前面。"""
    return (_unclear(tower) is None and tower.sky_owner == SKY_NONE and tower.ground == 2)


def _south_can_take_ground(tower: Tower) -> bool:
    """这座南边的塔还要地块（没看清也算，先别把最后一块带走）。"""
    if _unclear(tower) is not None:
        return True
    if _done(tower) or tower.sky_owner != SKY_NONE:
        return False
    return tower.ground < 2


def _leftover_ground_goes_north(towers: Mapping[str, Tower], goods: Goods) -> bool:
    """最后一块是地，南边用不上。送到北自己的塔，交接区空了、车也走远，下一趟才刷新。"""
    if goods.ground != 1 or goods.sky or goods.ground_stack or goods.cap_stack:
        return False
    if any(_south_can_take_ground(towers[name]) for name in MANDATE):
        return False
    # 南边已经封顶、北共享还要地：留着给北共享，不要先垫到 spot_03。
    north_shared = towers.get('spot_02', Tower(seen=True))
    if (_mandate_done(towers) and not _done(north_shared)
            and _south_can_take_ground(north_shared)):
        return False
    north = towers['spot_03']
    if _unclear(north) is not None or _done(north) or north.sky_owner != SKY_NONE:
        return False
    return north.ground < 2


def _stack_before_south_flip(towers: Mapping[str, Tower], goods: Goods) -> bool:
    """南共享已经是对方的天空，但空的 spot_07 还能一次吃下两块地。先放再翻。"""
    ours = towers['spot_07']
    south = towers[SOUTH_SHARED]
    if south.sky_owner != SKY_OTHERS or _unclear(south) is not None:
        return False
    if _unclear(ours) is not None or ours.ground != 0 or ours.sky_owner != SKY_NONE:
        return False
    return goods.ground_stack >= 1


def _l1_ready(towers: Mapping[str, Tower]) -> bool:
    return all(_done(towers[name]) for name in L1_TARGETS)


def _still_open(towers: Mapping[str, Tower]) -> str:
    """上楼前还没封顶的塔，写成一句，免得等货时看不出在等谁。"""
    bits = []
    for name in L1_TARGETS:
        tower = towers[name]
        if _done(tower):
            continue
        if _unclear(tower):
            bits.append(f'{name} 还没看清')
        elif tower.sky_owner == SKY_OTHERS:
            bits.append(f'{name} 还是对方天空')
        elif tower.ground >= 2 and tower.sky_owner == SKY_NONE:
            bits.append(f'{name} 还差天空')
        else:
            bits.append(f'{name} 还没封顶')
    return '、'.join(bits) or '还有塔没封顶'


def _l2_done(towers: Mapping[str, Tower]) -> bool:
    return all(_done(towers.get(name, Tower(seen=True))) for name in L2_OURS)


def _south_own_before_north(towers: Mapping[str, Tower], goods: Goods) -> bool:
    """南边自己的塔还能吃红地。先拿、先放，再去北共享看或者翻。"""
    ours = towers['spot_07']
    if _unclear(ours) is not None or _done(ours) or ours.sky_owner != SKY_NONE:
        return False
    if ours.ground >= 2 or (goods.ground < 1 and goods.ground_stack < 1):
        return False
    return True


def _accepts(tower: Tower, kind: str) -> bool:
    if _done(tower) or tower.sky_owner != SKY_NONE:
        return False
    if kind == 'stack':
        return tower.ground == 0
    if kind == 'cap':
        return tower.ground == 1
    if kind == 'sky':
        return tower.ground >= 2
    return tower.ground < 2


def _place_from_goods(towers, goods, at, look):
    """按交接区里实际有的货找塔。没有这批货就不当成下一件，也不为了缺的那种货干等。"""
    lone_sky = goods.sky - goods.cap_stack
    singles = goods.ground - 2 * goods.ground_stack
    have = set()
    if goods.ground_stack >= 1:
        have.add('stack')
    if goods.cap_stack >= 1:
        have.add('cap')
    if lone_sky >= 1:
        have.add('sky')
    if singles >= 1:
        have.add('ground')
    if not have:
        return None

    def place(kind, name, tower):
        if kind == 'stack':
            return Choice(PLACE_GROUND, name, '交接区有两块地一摞，一次拿两块地',
                          at(name, 'place'), tower.ground, tower.sky_owner, count=2)
        if kind == 'cap':
            why = ('上一趟的地和天空先放到北边自己的塔，等下一趟再继续：1 层地，一次拿地和天空'
                   if name == 'spot_03' else '交接区是地和天空一摞，一次拿地和天空')
            return Choice(PLACE_SKY, name, why, at(name, 'place'), tower.ground, tower.sky_owner,
                          count=2)
        if kind == 'sky':
            gap = '南共享只差天空，先做完再动自己的塔：' if name == SOUTH_SHARED else ''
            return Choice(PLACE_SKY, name, gap + f'{tower.ground} 层地，盖天空',
                          at(name, 'place'), tower.ground, tower.sky_owner)
        why = ('最后一块是地，先放北边自己的塔，下一趟才刷得出来：'
               if name == 'spot_03' else '交接区有红地，')
        return Choice(PLACE_GROUND, name, why + f'{tower.ground} 层地，再放一块地',
                      at(name, 'place'), tower.ground, tower.sky_owner)

    def kind_for(tower):
        """这座看得清的塔，现在这批货里它吃得下的那种。整摞优先。"""
        if 'stack' in have and _accepts(tower, 'stack'):
            return 'stack'
        if 'cap' in have and _accepts(tower, 'cap'):
            return 'cap'
        if 'sky' in have and _accepts(tower, 'sky'):
            return 'sky'
        if 'ground' in have and _accepts(tower, 'ground'):
            return 'ground'
        return None

    # 单块天空先盖南边自己的塔。地+天留给北边自己，不要抢在这一步前面。
    own = towers['spot_07']
    if ('sky' in have and _unclear(own) is None and not _done(own)
            and own.sky_owner == SKY_NONE and own.ground >= 2):
        return place('sky', 'spot_07', own)
    # 南共享只差天空、南边自己的塔还不用天：先盖南共享，再拿地去垫自己的塔。
    south = towers[SOUTH_SHARED]
    if 'sky' in have and not _done(south) and _almost_done(south):
        return place('sky', SOUTH_SHARED, south)

    names = ('spot_07', SOUTH_SHARED, 'spot_02')
    pending = None
    for name in names:
        tower = towers.get(name, Tower(seen=True))
        if name in SHARED and tower.blocked:
            continue
        if _done(tower) or tower.sky_owner == SKY_OTHERS:
            continue
        why = _unclear(tower)
        if why:
            # 南边还没封顶时，不去看北共享。这批货对不上就等，或者送到北边自己的塔。
            if name == 'spot_02' and not _mandate_done(towers):
                continue
            # 没有天空。这座塔已经两层，看一眼也不会变成能放天。这批货是否掉放天，去别处放地。
            if ('sky' not in have and 'cap' not in have and tower.ground >= 2):
                continue
            # 开局只有地天一摞，自己的塔是空的吃不下。南共享还没看：先拿上，
            # 去放的路上回头看一眼。空手先跑去看，看完还得回交接区再拿。
            if (name == SOUTH_SHARED and not tower.seen and not tower.clipped
                    and not tower.blocked and 'cap' in have):
                return place('cap', name, replace(tower, ground=1, seen=True))
            if pending is None:
                if name in SHARED and not tower.seen and not tower.clipped and not tower.blocked:
                    pending = look(name, '共享区动手前先看一眼')
                else:
                    pending = look(name, why)
            continue
        kind = kind_for(tower)
        # 地+天给北边自己，不先送给北共享。南边自己的塔要是正好一层，上面已经优先拿走了。
        if kind == 'cap' and name == 'spot_02':
            north = towers.get('spot_03', Tower(seen=True))
            if _unclear(north) is None and _accepts(north, 'cap'):
                continue
        if kind is not None:
            return place(kind, name, tower)
    if pending is not None:
        return pending
    # 南边和北共享都吃不下的最后一块地，放到北边自己的塔，交接区才空得出来。
    if 'ground' in have and _leftover_ground_goes_north(towers, goods):
        north = towers['spot_03']
        if _unclear(north) is None:
            return place('ground', 'spot_03', north)
    if 'cap' in have:
        north = towers['spot_03']
        if (_unclear(north) is None and _accepts(north, 'cap')):
            return place('cap', 'spot_03', north)
    return None


def choose(towers: Mapping[str, Tower], goods: Goods = Goods(),
           approaches: Optional[Mapping] = None, climbed: bool = False,
           at_north: bool = False) -> Choice:
    """下一件。towers 要有 ORDER 里的四个点。"""
    approaches = approaches or {}

    def at(spot, key):
        return _pose(approaches, spot, key)

    def look(name, why):
        return Choice(LOOK, name, why, at(name, 'look'))

    def work(name, why_prefix=''):
        """这个点现在就能做的下一件。缺货就返回 None，让调用方去看下一个点，不要干等。"""
        tower = towers.get(name, Tower(seen=True))
        # 对方车就在这座共享塔上。开过去正好碰上他们往上盖，这趟换一座。
        if name in SHARED and tower.blocked:
            return None
        if _done(tower):
            return None
        why = _unclear(tower)
        if why:
            if name in SHARED and not tower.seen and not tower.clipped and not tower.blocked:
                why = '共享区动手前先看一眼'
            return look(name, why_prefix + why)
        # 整摞优先：一次拿走两块，少跑一趟。没有整摞才一块一块拿
        if tower.ground == 0 and tower.sky_owner == SKY_NONE and goods.ground_stack >= 1:
            return Choice(PLACE_GROUND, name, why_prefix + '0 层地，一次拿两块地',
                          at(name, 'place'), tower.ground, tower.sky_owner, count=2)
        if tower.ground == 1 and tower.sky_owner == SKY_NONE and goods.cap_stack >= 1:
            return Choice(PLACE_SKY, name, why_prefix + '1 层地，一次拿地和天空',
                          at(name, 'place'), tower.ground, tower.sky_owner, count=2)
        need_sky = tower.ground >= 2
        # 只盖一块天空时不拆地天摞。摞上的那块天空不算能单拿的。
        lone_sky = goods.sky - goods.cap_stack
        if need_sky and lone_sky < 1:
            return None
        if not need_sky and goods.ground < 1:
            return None
        action = PLACE_SKY if need_sky else PLACE_GROUND
        detail = '盖天空' if need_sky else '再放一块地'
        return Choice(action, name, why_prefix + f'{tower.ground} 层地，{detail}', at(name, 'place'),
                      tower.ground, tower.sky_owner)

    if not _l2_done(towers):
        for name in OWN_BUILD_ORDER:
            tower = towers.get(name, Tower(seen=True))
            if tower.sky_owner == SKY_OTHERS:
                if name in L2_OURS:
                    if _unclear(tower):
                        return look(name, '确认 L2 自己塔的塔顶')
                    return Choice(FLIP, name, '翻回 L2 自己塔的天空', at(name, 'place'),
                                  tower.ground, tower.sky_owner)
                continue
            step = work(name)
            if step is not None:
                return step
        pending = '、'.join(n for n in L2_OURS if not _done(towers.get(n, Tower())))
        return Choice(WAIT, None, f'{pending} 尚未封顶，等自己的塔能用的货')

    north_shared = towers['spot_02']
    if north_shared.sky_owner == SKY_OTHERS and not north_shared.blocked:
        # 红地还要放在南边自己的塔上：先放下，再跑去北共享。
        if _south_own_before_north(towers, goods):
            step = work('spot_07', '先把红地放在南边自己的塔上，再去北共享：')
            if step is not None:
                return step
        why = _unclear(north_shared)
        if why:
            return look('spot_02', '北共享对方天空，但' + why)
        north_own = towers['spot_03']
        # 人还在南边：先把交接区的货放到北边自己的塔，再翻，不要专程空手跑一趟。
        # 人已经在北边：先翻再离开。回去拿货、放完再回来翻，多跑一整趟。
        own_stack_first = (_unclear(towers['spot_07']) is None and towers['spot_07'].ground == 0
                           and towers['spot_07'].sky_owner == SKY_NONE and goods.ground_stack >= 1)
        if (not at_north and not own_stack_first and _unclear(north_own) is None
                and not _done(north_own) and north_own.sky_owner == SKY_NONE
                and north_own.ground < 2):
            step = work('spot_03', '北共享要翻，先把交接区的货放到北边自己的塔上：')
            if step is not None:
                return step
        if not own_stack_first:
            return Choice(FLIP, 'spot_02', '北共享有对方天空，先翻，再去观察区拿块',
                          at('spot_02', 'place'), north_shared.ground, north_shared.sky_owner)

    for name in ORDER:
        tower = towers[name]
        if name in SHARED and tower.blocked:
            continue
        if tower.sky_owner != SKY_OTHERS:
            continue
        if name == SOUTH_SHARED and _south_own_before_north(towers, goods):
            step = work('spot_07', '先把南边自己的这块地放下，再去南共享：')
            if step is not None:
                return step
        why = _unclear(tower)
        if why:
            return look(name, '对方天空，但' + why)
        if name == SOUTH_SHARED and _stack_before_south_flip(towers, goods):
            continue
        # 从北边回来翻南共享：自己的南塔只差天空、交接区有单块天空，先盖上再翻。
        if name == SOUTH_SHARED:
            own = towers['spot_07']
            lone_sky = goods.sky - goods.cap_stack
            if (_unclear(own) is None and not _done(own) and own.ground >= 2
                    and own.sky_owner == SKY_NONE and lone_sky >= 1):
                step = work('spot_07', '翻南共享之前，先把天空盖到自己的南塔：')
                if step is not None:
                    return step
        return Choice(FLIP, name, '对方天空，翻转可以插队', at(name, 'place'),
                      tower.ground, tower.sky_owner)

    # 先看交接区有什么货，再给这批货找塔。不要先指定一座塔要天空，再跑去找天空。
    placed = _place_from_goods(towers, goods, at, look)
    if placed is not None:
        return placed
    if not _l1_ready(towers):
        return Choice(WAIT, None, f'{_still_open(towers)}，这趟货对不上，等下一趟')
    return Choice(STOP, None, '五色石已被抢走，自己半边两座已封顶，停下')


def close_look(remembered: Tower, ground: int, has_sky: bool, clipped: bool) -> Tower:
    """预备位这一眼。贴太近会少看一层，所以层数只增不减。顶部出视场就维持记账。

    多出来的天空、而且不是我们放的，记成对方的。少看掉的天空不算别人拆了。
    """
    if clipped:
        return remembered
    sky = remembered.sky_owner
    if has_sky and sky != SKY_OURS:
        sky = SKY_OTHERS
    return Tower(max(remembered.ground, int(ground)), sky, True, False, False)


def carry_choice(towers: Mapping[str, Tower], kind: str,
                 approaches: Optional[Mapping] = None, count: int = 1,
                 avoid: Optional[str] = None) -> Optional[Choice]:
    """手里已经拿着。kind 是 ground / sky / cap（地+天空一摞）。按同样的优先级找一座现在要这块的塔。"""
    approaches = approaches or {}

    def at(spot, key):
        return _pose(approaches, spot, key)

    names = OWN_BUILD_ORDER
    if _l2_done(towers):
        names = ('spot_07', SOUTH_SHARED, 'spot_03', 'spot_02')
        south = towers[SOUTH_SHARED]
        if _almost_done(south) and (
                (south.ground == 1 and (kind == 'cap' or (kind == 'ground' and count == 1)))
                or (south.ground >= 2 and kind == 'sky')):
            names = (SOUTH_SHARED, 'spot_07', 'spot_03', 'spot_02')

    load = 'stack' if kind == 'ground' and count == 2 else kind
    split = None
    for name in names:
        tower = towers.get(name, Tower(seen=True))
        if name == avoid or _unclear(tower) or tower.sky_owner != SKY_NONE:
            continue
        if _accepts(tower, load):
            action = PLACE_SKY if kind in ('sky', 'cap') else PLACE_GROUND
            amount = 2 if load in ('stack', 'cap') else 1
            return Choice(action, name, f'改放手里的货到 {name}', at(name, 'place'),
                          tower.ground, tower.sky_owner, count=amount)
        if kind == 'cap' and tower.ground >= 2:
            partial = Choice(PLACE_SKY, name, f'{name} 已经两层，先盖天空，地留着',
                             at(name, 'place'), tower.ground, tower.sky_owner)
            if name in MANDATE:
                return partial
            if split is None:
                split = partial
    return split


def on_arrival(planned: Choice, towers: Mapping[str, Tower], goods: Goods = Goods(),
               approaches: Optional[Mapping] = None, climbed: bool = False):
    """放 / 翻之前开到跟前再看一眼。和出发时不一样就重选，返回 (新的下一件, 说明)。没变说明是空的。

    这一眼看不清（出视场、被挡），而且记账和出发时一样，按出发时的计划放。
    记账已经多了一层、或者多了天空，这一眼看丢了也要改计划。
    """
    if planned.action not in (PLACE_GROUND, PLACE_SKY, FLIP):
        return planned, ''
    tower = towers[planned.spot]
    same = tower.ground == planned.assumed_ground and tower.sky_owner == planned.assumed_sky
    if _unclear(tower):
        if same or not tower.seen:
            return planned, ''
        tower = replace(tower, clipped=False, blocked=False)
        towers = dict(towers)
        towers[planned.spot] = tower
    if tower.ground == planned.assumed_ground and tower.sky_owner == planned.assumed_sky:
        return planned, ''
    new = choose(towers, goods, approaches, climbed=climbed)
    note = (f'开到 {planned.spot} 跟前：出发时 {planned.assumed_ground} 层 / {planned.assumed_sky}，'
            f'现在 {tower.ground} 层 / {tower.sky_owner}，改做 {new.line()}')
    return new, note


def after_changed_look(planned: Choice, new: Choice, towers: Mapping[str, Tower],
                       approaches: Optional[Mapping] = None, held_n: int = 0,
                       avoid: Optional[str] = None):
    """跟前这一眼改了计划之后怎么走。返回 (怎么走, 下一件)。

    翻转的检查在拿块之前，held_n 是 0。空手就执行新计划。
    当成手里有地再放，电控会报手里没有方块。
    已经拿着：同一摞换塔去放；对不上就按手里的块重选。
    怎么走：empty / same / carry / stuck。
    """
    if held_n <= 0:
        return 'empty', new
    if planned.action == PLACE_SKY and planned.count == 2:
        held, n = 'cap', 2
    elif planned.action == PLACE_SKY:
        held, n = 'sky', 1
    else:
        held, n = 'ground', planned.count
    same = (
        (held == 'ground' and new.action == PLACE_GROUND and new.count == n)
        or (held == 'sky' and new.action == PLACE_SKY and new.count == 1)
        or (held == 'cap' and new.action == PLACE_SKY and new.count == 2))
    if same:
        return 'same', new
    alt = carry_choice(towers, held, approaches, n, avoid=avoid)
    if alt is None:
        return 'stuck', None
    return 'carry', alt


def apply_choice(towers: Dict[str, Tower], goods: Goods, choice: Choice):
    """假场上推进一步（随机脚本用）。等货 = TR 把缺的那块送来。返回新的 goods。"""
    if choice.action == LOOK:
        towers[choice.spot] = replace(towers[choice.spot], seen=True, clipped=False, blocked=False)
    elif choice.action == PLACE_GROUND:
        n = choice.count
        towers[choice.spot] = replace(
            towers[choice.spot], ground=towers[choice.spot].ground + n)
        goods = replace(
            goods, ground=goods.ground - n,
            ground_stack=max(0, goods.ground_stack - (1 if n == 2 else 0)))
    elif choice.action == PLACE_SKY and choice.count == 2:
        towers[choice.spot] = replace(
            towers[choice.spot], ground=towers[choice.spot].ground + 1, sky_owner=SKY_OURS)
        goods = replace(goods, sky=goods.sky - 1, cap_stack=max(0, goods.cap_stack - 1))
    elif choice.action == PLACE_SKY:
        towers[choice.spot] = replace(towers[choice.spot], sky_owner=SKY_OURS)
        goods = replace(goods, sky=goods.sky - 1)
    elif choice.action == FLIP:
        towers[choice.spot] = replace(towers[choice.spot], sky_owner=SKY_OURS)
    return goods


def mark_unseen(towers: Dict[str, Tower]):
    """停机重试恢复：场上可能变了，四个点都要再看。"""
    for name in list(towers):
        towers[name] = replace(towers[name], seen=False)


# ── 把「下一件」编成压进栈的步骤。正常放完不退远看 ──

FINE_XY = 0.01
FINE_YAW = math.radians(1.0)

# 交接区示教拿取点，跟 two_towers_sim.yaml 一样（间隙 15 cm、从东边拿）
TRANSFER_SINGLE = (-2.675, -1.55, math.pi)
TRANSFER_STACK = (-3.125, -1.15, math.pi)


def bind_points(approaches):
    """规划用的点名 → (x, y, theta)。"""
    look = approaches['transfer']['look']
    gate = approaches['transfer']['gate']
    see = approaches['transfer'].get('see') or look
    points = {
        'transfer_look': (float(look['x']), float(look['y']), float(look['theta'])),
        'transfer_see': (float(see['x']), float(see['y']), float(see['theta'])),
        'transfer_gate': (float(gate['x']), float(gate['y']), float(gate['theta'])),
        'transfer_single': TRANSFER_SINGLE,
        'transfer_stack': TRANSFER_STACK,
    }
    stair = approaches.get('l2_stair') or {}
    for key in ('south', 'west', 'foot', 'land'):
        if key in stair:
            p = stair[key]
            points[f'l2_stair_{key}'] = (float(p['x']), float(p['y']), float(p['theta']))
    for name, spot in approaches.get('spots', {}).items():
        for key in ('lane', 'pre', 'place', 'look', 'west'):
            if key not in spot:
                continue
            p = spot[key]
            points[f'{name}_{key}'] = (float(p['x']), float(p['y']), float(p['theta']))
    # 南边自己的放置位朝向南共享。放南共享之前先在这儿转身看一眼，蓝方已经垫了就不用开进去再折返。
    own = approaches.get('spots', {}).get('spot_07', {}).get('place')
    if own:
        yaw = math.atan2(-2.7 - float(own['y']), 0.0 - float(own['x']))
        points['spot_07_glance'] = (float(own['x']), float(own['y']), yaw)
    return points


def _closer_to_the_field(stacks):
    """多摞都在时，先只看最靠近内场（x 更大，离西边外沿更远）的那摞。

    先拿外面那摞要开到北栏杆边上。里面那块拿走之后，外面的再算进货里。
    里面只是一块天、外面是地天一摞时，这块天开局没处放，地天摞仍算能拿的货。
    """
    on_pad = [s for s in stacks if s[0].x < -2.5]
    if len(on_pad) <= 1:
        return stacks
    inner = max(s[0].x for s in on_pad)
    def near(stack):
        return stack[0].x >= -2.5 or stack[0].x >= inner - 0.05
    kept = [s for s in stacks if near(s)]
    inners = [s for s in on_pad if s[0].x >= inner - 0.05]
    if inners and all(tuple(b.kind for b in s) == ('sky',) for s in inners):
        kept.extend(s for s in stacks
                    if s not in kept and tuple(b.kind for b in s) == ('ground', 'sky'))
    return kept


def trusted_transfer_blocks(blocks, clipped: bool):
    """顶部出了视场时，单独一块地可能是没看全的地天摞。

    按这块地去拿，夹爪抓的是最上面那块，会把天揭下来，底下的地留在交接区。
    """
    if not clipped:
        return list(blocks)
    from ..flow.pick_pose import GROUND, composition, group_stacks
    kept = []
    for stack in group_stacks(list(blocks)):
        if composition(stack) == (GROUND,):
            continue
        kept.extend(stack)
    return kept


def pickable_goods(blocks):
    """交接区现在拿得到的。地块压在天空块下面时只能先拿天空，不算进地块。

    拿取整段会擦到栏杆或旁边块的那几摞先去掉：这趟当它们还不存在，先拿擦不到的。
    剩下的先拿靠近内场的，贴外沿的留到下一趟。
    """
    from ..flow.pick_pose import GROUND, SKY, blocks_we_can_reach, composition, group_stacks
    blocks = blocks_we_can_reach(list(blocks))
    blocks = [b for stack in _closer_to_the_field(group_stacks(blocks)) for b in stack]
    ground = sky = ground_stack = cap_stack = 0
    for stack in group_stacks(list(blocks)):
        comp = composition(stack)
        if comp == (GROUND,):
            ground += 1
        elif comp == (GROUND, GROUND):
            ground += 2
            ground_stack += 1
        elif comp == (SKY,):
            sky += 1
        elif comp == (GROUND, SKY):
            sky += 1
            cap_stack += 1
    return Goods(ground, sky, ground_stack, cap_stack)


def _nearest_nominal(block):
    """示教点用离这块最近的那个。单块被拿走以后，剩下的地块还在一摞的位置上。"""
    # 示教点是车的停车位置，方块在车头前方 gripper 0.575 m
    spots = {
        'transfer_single': (TRANSFER_SINGLE[0] - 0.575, TRANSFER_SINGLE[1]),
        'transfer_stack': (TRANSFER_STACK[0] - 0.575, TRANSFER_STACK[1]),
    }
    name = min(spots, key=lambda n: math.hypot(block.x - spots[n][0], block.y - spots[n][1]))
    return name


def _pick_how(blocks, kind, count=1):
    """放地 / 放天空时，交接区 DETECT 找什么、用哪个示教点、拿几块。找不到返回 None。"""
    from ..flow.pick_pose import GROUND, SKY, composition, group_stacks
    stacks = group_stacks(list(blocks))
    singles = [s for s in stacks if composition(s) == (GROUND,)]
    doubles = [s for s in stacks if composition(s) == (GROUND, GROUND)]
    skies = [s for s in stacks if composition(s) == (SKY,)]
    topped = [s for s in stacks if composition(s) == (GROUND, SKY)]
    if kind == 'ground' and count == 2:
        if doubles:
            return 'stack', _nearest_nominal(doubles[0][0]), 2
        return None
    if kind == 'ground':
        if singles:
            return 'ground', _nearest_nominal(singles[0][0]), 1
        if doubles:
            return 'stack', _nearest_nominal(doubles[0][0]), 1
    if kind == 'sky' and count == 2:
        if topped:
            return 'groundsky', _nearest_nominal(topped[0][0]), 2
        return None
    if kind == 'sky':
        if skies:
            return 'sky', _nearest_nominal(skies[0][0]), 1
        return None
    return None


def _see_yaw(approaches):
    """观察校准点的朝向。从北边回来先转到这个角，到了就不用再转一次。"""
    return bind_points(approaches)['transfer_see'][2]


def _yaw_toward_spot(name, approaches):
    """站在观察校准点，转向这座塔。南北共享的层数从这儿转一下就看得到。"""
    see = bind_points(approaches)['transfer_see']
    place = approaches.get('spots', {}).get(name, {}).get('place')
    if not place:
        return see[2]
    grip = float(approaches.get('gripper_offset', 0.575))
    yaw = float(place['theta'])
    cx = float(place['x']) + grip * math.cos(yaw)
    cy = float(place['y']) + grip * math.sin(yaw)
    return math.atan2(cy - see[1], cx - see[0])


def _depart_yaw(choice, approaches):
    """拿完货回观察点用的朝向：上楼朝楼梯口，放货朝放置路线的第一段。"""
    points = bind_points(approaches)
    if choice.spot in L2_OURS and 'l2_stair_foot' in points:
        return points['l2_stair_foot'][2]
    for step in _drive_to_place(choice, approaches, with_check=False, from_north=False):
        if getattr(step, 'point', None) in points:
            return points[step.point][2]
    return _see_yaw(approaches)


def _fine(point):
    from ..flow.steps import Nav
    return Nav(point, FINE_XY, FINE_YAW)


def _align(point):
    from ..flow.steps import Realign, Relocalize
    return [_fine(point), Relocalize(), Realign(point, FINE_XY, FINE_YAW)]


def _drive_to_place(choice, approaches, with_check=False, from_north=False,
                    on_west=False, for_flip=False):
    """从过路点开到放置位。with_check 时在预备位再看一眼。

    from_north：人在北共享东边。先收到西侧再走，不要把南边的 spot_01_west 当第一个点，
    那条直线穿 L2。
    on_west：人已经在 spot_03，西侧走廊上。不用再往东收到北共享。
    for_flip：翻转这一路的校准都改不动位姿，预备位不再停。
    """
    from ..flow.steps import Detect, Nav, Relocalize
    steps = []
    points = bind_points(approaches)
    if on_west and choice.spot == 'spot_03':
        steps.append(Nav(f'{choice.spot}_pre'))
        if with_check:
            steps.append(Detect(choice.spot, 'any', f'{choice.spot}_place', 'check'))
        steps.append(_fine(f'{choice.spot}_place'))
        return steps
    if from_north and 'spot_02_west' in points:
        steps.append(Nav('spot_02_west'))
        steps.append(Relocalize())
        if choice.spot == 'spot_03':
            steps += [*_align(f'{choice.spot}_pre')]
            if with_check:
                steps.append(Detect(choice.spot, 'any', f'{choice.spot}_place', 'check'))
            steps.append(_fine(f'{choice.spot}_place'))
            return steps
        if choice.spot in ('spot_01', 'spot_07') and 'l2_stair_west' in points:
            steps.append(Nav('l2_stair_west'))
    elif choice.spot in ('spot_02', 'spot_03') and 'l2_stair_south' in points:
        # 人在南边时不能直线拉到北边。先在走廊南端转朝北。
        steps.append(Nav('l2_stair_south'))
        # spot_03 的预备位就在这条朝北的线上。楼梯口朝向一样，再停一次是北行中断。
        if choice.spot != 'spot_03':
            steps.append(Nav('l2_stair_west'))
    if f'{choice.spot}_west' in points:
        steps.append(Nav(f'{choice.spot}_west'))
        # 西侧往往刚转完弯，定位还漂着。先咬住再横穿，免得直线把车拉开。
        # 翻转这一段实测改不动，停一下只是空等。
        if not for_flip:
            steps.append(Relocalize())
    if for_flip and choice.spot == SOUTH_SHARED:
        # west 已退出台沿；沿南侧空廊到最终翻转位，省去 lane 和 pre 的两次刹停。
        steps.append(_fine(f'{choice.spot}_place'))
        return steps
    # spot_03 的 lane 在楼梯南边。人已经朝北到了楼梯口，再回去转一圈是倒车。
    if f'{choice.spot}_lane' in points and choice.spot != 'spot_03':
        steps.append(Nav(f'{choice.spot}_lane'))
    # 从南边去 spot_03，人已经朝北，预备位在正前方。对齐那一下校准改不动，只是北行中间停一下。
    # 翻转同理：预备位的校准和再对准都在死区里。
    if for_flip or (choice.spot == 'spot_03' and not from_north):
        steps.append(Nav(f'{choice.spot}_pre'))
    else:
        steps += [*_align(f'{choice.spot}_pre')]
    if with_check:
        steps.append(Detect(choice.spot, 'any', f'{choice.spot}_place', 'check'))
    steps.append(_fine(f'{choice.spot}_place'))
    return steps


def _approach_and_place(choice, approaches, layer, with_check, from_north=False, on_west=False):
    """从过路点开到放置位再放下。"""
    from ..flow.steps import Action
    return (_drive_to_place(choice, approaches, with_check, from_north, on_west)
            + [Action('PLACE', layer)])


def planner_subflows():
    """由规划器生成的 CALL 与这里的注册表配套，父任务负责离开楼梯口以后的路线。"""
    from ..flow.steps import Action, Nav
    return {'descend_l2': [Nav('l2_stair_land'), Action('CLIMB_DOWN')]}


def compile_choice(choice, approaches, blocks, holding=False, on_platform=False,
                   from_north=False, on_west=False, from_south=False):
    """下一件 → 步骤列表。编不出来返回空列表（比如要放地，但地块都压在天空块下面）。

    holding：这块已经在手里（跟前改了计划）。直接去放，不再回交接区抓一次。
    """
    from ..flow.steps import Action, Call, Detect, Nav, Relocalize, Wait, WaitAfterCorrection
    if choice.action in (MUSTIKA, STOP):
        return []

    def up(*, from_transfer=False):
        # 人已经在楼梯西边。直接开到楼梯口再换上台沿，不先朝北开到走廊南端再折回来。
        # 人还在北共享或南共享的东边时，直线去楼梯口穿 L2。先收到西侧，再沿西走廊过来。
        steps = []
        points = bind_points(approaches)
        if not from_transfer and from_north and 'spot_02_west' in points:
            steps.append(Nav('spot_02_west'))
            if 'l2_stair_west' in points:
                steps.append(Nav('l2_stair_west'))
        elif not from_transfer and from_south and 'spot_01_west' in points:
            steps.append(Nav('spot_01_west'))
            if 'l2_stair_west' in points:
                steps.append(Nav('l2_stair_west'))
        steps += [Nav('l2_stair_foot'), Action('CLIMB_L2')]
        return steps

    def down():
        """先开回上楼的落点，再换回楼梯口。不要在台面上任意位置直接瞬移。"""
        return [Call('descend_l2')]

    def to_see():
        """回观察校准点。北共享东边先收到西侧，直线会穿 L2。不要再绕走廊南端。

        从北边回来时，西侧那个点就转到观察点的朝向，南下的一路不用再到观察点转一次。
        """
        # 北共享东边先收到西侧，直线会穿 L2。人已经在 spot_03 就在西侧，不用再往东拐。
        steps = [Nav('spot_02_west', yaw=_see_yaw(approaches))] if from_north else []
        steps.append(Nav('transfer_see'))
        return steps

    # up() 使用出发时的 L1 位置。deck() 拼接的后半段从换位后的楼层出发。
    north_on_destination = from_north and choice.spot not in L2_OURS
    west_on_destination = on_west and choice.spot not in L2_OURS

    def deck(steps):
        """人在台上要去 L1，先换下来。人在楼下要去 L2，先换上去。"""
        if on_platform and choice.spot not in L2_OURS:
            # 先回到 L2 台沿再换下去。下楼落点朝向观察点，下一步直接平移过去。
            steps = down() + steps
        if not on_platform and choice.spot in L2_OURS:
            steps = up() + steps
        return steps

    if choice.action == CLIMB:
        # 先沿西侧开到楼梯口（还在 L1 上）。上楼本身不爬，电控直接换位。
        return deck(up())
    if choice.action == WAIT:
        return deck([*to_see(), Detect('transfer_red', 'any', 'transfer_stack', 'supply'), Wait(3)])
    if choice.action == LOOK and choice.spot in SHARED:
        # 南北共享的层数在观察点转身就进视场。不用再开到塔跟前的观察位。
        yaw = _yaw_toward_spot(choice.spot, approaches)
        steps = []
        if from_north and 'spot_02_west' in bind_points(approaches):
            steps.append(Nav('spot_02_west', yaw=_see_yaw(approaches)))
        steps += [Nav('transfer_see', yaw=yaw), Wait(2)]
        return deck(steps)
    if choice.action == LOOK:
        steps = []
        points = bind_points(approaches)
        # 人在北共享东边。先收到西侧。spot_01_west 在南边，直开穿 L2。
        if north_on_destination and 'spot_02_west' in points:
            steps.append(Nav('spot_02_west'))
        elif choice.spot in ('spot_02', 'spot_03') and 'spot_01_west' in points:
            steps.append(Nav('spot_01_west'))
            if 'l2_stair_west' in points:
                steps.append(Nav('l2_stair_west'))
        if f'{choice.spot}_west' in points:
            steps.append(Nav(f'{choice.spot}_west'))
            steps.append(Relocalize())
        if f'{choice.spot}_lane' in bind_points(approaches):
            steps.append(Nav(f'{choice.spot}_lane'))
        steps += [Nav(f'{choice.spot}_look'), Relocalize(), Wait(2)]
        return deck(steps)
    if choice.action == FLIP:
        # 翻要停在放置位，夹爪才够得着天空块。停在 2.3 m 的观察位上翻，画面不会变。
        # 人还在南边时，先在观察点转向这座共享塔看一层数，再开过去。
        steps = []
        if choice.spot in SHARED and not from_north:
            yaw = _yaw_toward_spot(choice.spot, approaches)
            steps += [Nav('transfer_see', yaw=yaw),
                      Detect(choice.spot, 'any', f'{choice.spot}_place', 'check')]
        steps += _drive_to_place(choice, approaches, from_north=north_on_destination, on_west=west_on_destination,
                                 for_flip=True) + [Action('FLIP_SKY')]
        if choice.spot == 'spot_02':
            # 翻完人在北共享东边。先收到西侧，再直接去观察点，不绕楼梯口。
            steps += [Nav('spot_02_west', yaw=_see_yaw(approaches)), Nav('transfer_see')]
        return deck(steps)
    if choice.action not in (PLACE_GROUND, PLACE_SKY):
        return []
    layer = (choice.assumed_ground or 0) + 1
    if holding:
        # 自己的塔层数是刚放的。共享区对方会在路上垫一层，手里拿着也要再看一眼。
        return deck(_approach_and_place(choice, approaches, layer,
                                        with_check=choice.spot in SHARED,
                                        from_north=north_on_destination, on_west=west_on_destination))
    how = _pick_how(blocks, 'sky' if choice.action == PLACE_SKY else 'ground', choice.count)
    if how is None:
        return []
    what, nominal, count = how
    # 人在台上、手里没货：先下楼再去交接区。放到 L2 的，拿完再换上台面，不开上台阶。
    steps = down() if on_platform else []
    steps += [
        *to_see(), Relocalize(), WaitAfterCorrection(1),
        Detect('transfer_red', what, nominal, 'pick'),
        # 观察点已经在开口里、拿取预备位东边。直接开过去，不再先往东北拐到入口。
        *_align('@pick_pre'),
        _fine('@pick'),
        Action('PICK', count),
    ]
    # 自己的塔层数是刚放上去记的。共享区对方会加块，放之前仍要看一眼。
    place = _approach_and_place(choice, approaches, layer, with_check=choice.spot in SHARED)
    if choice.spot in L2_OURS:
        # 远侧单块的拿取位到楼梯口是斜线，擦过楼梯西南角。先回到开口里的观察点朝东，再开过去。
        steps.append(Nav('transfer_see', yaw=_depart_yaw(choice, approaches)))
        # 拿取后已从交接区开口离开；上一件任务的共享区位置在拿取之前。
        steps += up(from_transfer=True)
        steps += place
    else:
        first = next((s.point for s in place if getattr(s, 'point', None)), None)
        if choice.spot in SHARED:
            # 拿完已经回到观察点。转向共享塔看层数，再沿放置路线开，不必先挪到南塔跟前。
            yaw = _yaw_toward_spot(choice.spot, approaches)
            steps += [Nav('transfer_see', yaw=yaw),
                      Detect(choice.spot, 'any', f'{choice.spot}_place', 'check')]
            steps += place
        elif first == 'l2_stair_south':
            # 走廊南端离观察点只有几厘米，朝向也一样。不要先开到观察点再挪过去。
            steps += place
        else:
            # 先拿近处的一摞，倒车没有遮挡。直接回观察点，朝向用下一步，
            # 不再停在观察点和预备点中间的入口。
            steps.append(Nav('transfer_see', yaw=_depart_yaw(choice, approaches)))
            steps += place
    return steps

"""随机场上状态跑规划器，把每一步打在这个终端里。不起 Gazebo。每次用系统随机数，结果不一样。

    python3 -m rc2027_decision.plan_random        # 两份
    python3 -m rc2027_decision.plan_random 5      # 五份
    ros2 run rc2027_decision plan_random -- 5
"""

import random
import sys

from .world.field_model import SKY_OTHERS
from .world.planner import (CLIMB, FLIP, L2_OURS, MUSTIKA, ORDER, PLACE_GROUND, PLACE_SKY,
                            STOP, WAIT, Goods, Tower,
                            apply_choice, choose, load_approaches, on_arrival)

# 跟 towers_plan2.yaml 一样，不随机。第 1 趟开局就在，拿空以后才送第 2 趟
TRIP1 = Goods(ground=3, sky=0, ground_stack=1)   # 2 地一摞 + 1 地
TRIP2 = Goods(ground=1, sky=2, cap_stack=1)       # 地 + 天一摞 + 1 天


L1_SHARED = ('spot_01', 'spot_02')


def blue_shared(rng):
    """L1 共享区只有三种：一层地、两层地、两层地加对方天空。自己的塔不在这里随机。"""
    kind = rng.choice((1, 2, 'full'))
    if kind == 'full':
        return Tower(ground=2, sky_owner=SKY_OTHERS, seen=True)
    return Tower(ground=kind, seen=True)


def blue_build(tower):
    """蓝方再盖一块。一层→两层，两层→对方天空。已经是对方天空就不再盖。"""
    if tower.sky_owner == SKY_OTHERS:
        return None
    if tower.ground < 2:
        return Tower(ground=tower.ground + 1, seen=True)
    return Tower(ground=2, sky_owner=SKY_OTHERS, seen=True)


def one_run(seed, approaches, index):
    """从一份随机状态推到五色石，返回打印的行。seed 只为了能照着再跑一遍。"""
    rng = random.Random(seed)
    towers = {name: Tower(seen=True) for name in ORDER + L2_OURS}
    towers['spot_01'] = blue_shared(rng)
    towers['spot_02'] = blue_shared(rng)
    goods = TRIP1
    trip2 = True
    climbed = False
    lines = [f'── 第 {index} 份（种子 {seed}）──  第 1 趟：2 地一摞 + 1 地']
    for name in ORDER:
        t = towers[name]
        lines.append(f'    {name}: {t.ground} 层地 天空={t.sky_owner}'
                     f'{" 出视场" if t.clipped else ""}{" 被挡" if t.blocked else ""}'
                     f'{"" if t.seen else " 没看清"}')
    for step in range(1, 41):
        choice = choose(towers, goods, approaches, climbed=climbed)
        built = None
        if choice.spot in L1_SHARED and choice.action in (PLACE_GROUND, PLACE_SKY, FLIP):
            built = blue_build(towers[choice.spot])
        if built is not None:
            towers[choice.spot] = built
            choice, note = on_arrival(choice, towers, goods, approaches)
            lines.append(f'  {step:2d}. 蓝方续盖。{note}')
        else:
            lines.append(f'  {step:2d}. {choice.line()}')
        if choice.action == CLIMB:
            climbed = True
            continue
        if choice.action in (MUSTIKA, STOP):
            break
        if choice.action == WAIT:
            if not trip2:
                lines.append('  两趟都送完了还缺块，停下')
                break
            trip2 = False
            goods = Goods(
                ground=goods.ground + TRIP2.ground, sky=goods.sky + TRIP2.sky,
                ground_stack=goods.ground_stack + TRIP2.ground_stack,
                cap_stack=goods.cap_stack + TRIP2.cap_stack)
            lines.append('      TR 第 2 趟送到：地 + 天一摞，加 1 块天')
            continue
        goods = apply_choice(towers, goods, choice)
    else:
        lines.append('  40 步还没到五色石')
    return lines


def main(args=None):
    """跑若干份。份数从参数来，默认 2。每份的种子是系统随机的，打在标题里。"""
    if args is None:
        args = sys.argv[1:]
    args = [a for a in args if a != '--']
    count = int(args[0]) if args else 2
    approaches = load_approaches()
    sysrand = random.SystemRandom()
    for i in range(1, count + 1):
        seed = sysrand.randrange(1, 10 ** 9)
        print('\n'.join(one_run(seed, approaches, i)))
        print()


if __name__ == '__main__':
    main()

"""Flow 的步骤类型和文本解析——纯 Python，不依赖 ROS，host 上单测。

学华工 stm32_control 的 Flow：一个流程 = 一串步骤，流程可以 CALL 别的流程（压栈），
赛前改流程只改 yaml 里的文字，不改代码。跟华工不一样的是导航在上位机（point_navigator），
所以多了 NAV / RELOCALIZE 两种步骤；华工的"导航"是发给下位机的一个动作码。

一行一个步骤（或者用 `;` 隔开写在一行里，跟华工动态流程的文本一样），`#` 后面是注释：

    NAV <点名>                 走到录好的点，用 point_navigator 的默认到位门限（3 cm）
    NAV <点名> fine            用 tolerances 里叫 fine 的门限（拿 / 放前最后一步，1 cm / 1°）
    NAV <点名> <xy_m> [<yaw_deg>]  直接给门限
    ALIGN <点名>               "精确到位"= NAV 点名 → RELOCALIZE → NAV 点名 fine（展开成三步）
    RELOCALIZE                 停稳后 ICP 精修（/field_localizer/relocalize）
    WAIT <秒>
    ACTION <动作名> [参数]      发动作码给电控，等它回 DONE（动作名见 rc2027_interface 的动作码表）
    CALL <流程名> [THEN <标签>]  压栈执行子流程；子流程走完回到父流程下一步，或者跳到标签处
                               （华工的 on_child_done_next）
    DETECT <区域> <找什么> <示教拿取点> [AS <名字>]
                               停稳后调雷达方块检测看一个区域（spot_xx / transfer_red ...），挑出离示教拿取点
                               最近、类型对得上的一摞（找什么：ground / stack / sky / groundsky / any），
                               算出拿取位姿存成 @<名字>（默认 @pick）、预备位姿 @<名字>_pre（往后退一段，
                               最后直着进）。离示教点太远（限幅）就失败。后面用 NAV @pick_pre、ALIGN @pick
    VERIFY <区域> <地块层数> [sky]
                               放完、退开以后看一眼建造点：层数（和有没有天空块）对上才算完成，对不上就失败停住。
                               放的过程中别看（夹爪举着方块从旁边过来，会被当成悬空的东西）
    LABEL <标签>               不是步骤，只给 THEN 当跳转目标

为什么拿放前要 ALIGN：point_navigator 到点后不会自己再动；relocalize 只改"车以为自己在哪"，
实际还偏着的 1～3 cm 要再发一次同一个目标、而且用更严的门限才会走过去（0002 10-04 助教那条）。
"""

from dataclasses import dataclass
import math
from typing import List, Mapping, Optional, Tuple


class FlowSyntaxError(ValueError):
    """流程文本写错了（带上是哪个流程、第几行）。"""


@dataclass(frozen=True)
class Nav:
    """走到一个录好的点。门限是 None 时用 point_navigator 的节点参数。"""

    point: str
    xy_tolerance: Optional[float] = None    # m
    yaw_tolerance: Optional[float] = None   # rad
    yaw: Optional[float] = None             # 设了就不用点里录的朝向


@dataclass(frozen=True)
class Realign(Nav):
    """前一步 ICP 有可能改变定位时，重新对准刚到过的精确点。"""


@dataclass(frozen=True)
class Relocalize:
    """停稳后调一次 ICP 精修。"""


@dataclass(frozen=True)
class Wait:
    """原地等几秒。"""

    seconds: float


@dataclass(frozen=True)
class WaitAfterCorrection(Wait):
    """观察点的 ICP 修正可能使旧检测帧过期时，等待刷新。"""


@dataclass(frozen=True)
class Action:
    """发一个动作码给电控，等它做完。"""

    name: str
    param: int = 0


@dataclass(frozen=True)
class Detect:
    """看一个区域，按检测到的方块算拿取位姿，存到 @slot / @slot_pre。"""

    region: str
    what: str
    nominal: str          # 示教的拿取点（锚点 + 限幅基准）
    slot: str = 'pick'


DETECT_WHAT = ('ground', 'stack', 'sky', 'groundsky', 'any')


@dataclass(frozen=True)
class Verify:
    """看一眼建造点，层数对上才算完成。"""

    region: str
    ground_layers: int
    sky: bool = False


def slot_names(step):
    """DETECT 会写哪些动态点名。"""
    return ('@' + step.slot, '@' + step.slot + '_pre')


@dataclass(frozen=True)
class Call:
    """压栈执行子流程。next_index 是子流程走完后父流程从第几步接着走（None = 下一步）。"""

    flow: str
    next_index: Optional[int] = None


Step = object  # Nav / Relocalize / Wait / Action / Call


def describe(step):
    """一个步骤的简短文字（日志、状态话题用）。"""
    if isinstance(step, Nav):
        if step.xy_tolerance is None:
            return f'NAV {step.point}'
        yaw = '' if step.yaw_tolerance is None else f' {math.degrees(step.yaw_tolerance):.1f}°'
        return f'NAV {step.point} {step.xy_tolerance:.3f}m{yaw}'
    if isinstance(step, Relocalize):
        return 'RELOCALIZE'
    if isinstance(step, Wait):
        return f'WAIT {step.seconds:g}'
    if isinstance(step, Action):
        return f'ACTION {step.name} {step.param}' if step.param else f'ACTION {step.name}'
    if isinstance(step, Call):
        return f'CALL {step.flow}'
    if isinstance(step, Detect):
        return f'DETECT {step.region} {step.what} {step.nominal} AS {step.slot}'
    if isinstance(step, Verify):
        return f'VERIFY {step.region} {step.ground_layers}{" sky" if step.sky else ""}'
    return repr(step)


DEFAULT_TOLERANCES = {'fine': (0.01, math.radians(1.0))}


def split_lines(text):
    """把流程文本拆成 (行号, 内容)：去注释、按换行和 `;` 拆、去空行。行号从 1 开始。"""
    out = []
    for lineno, line in enumerate(text.splitlines(), start=1):
        line = line.split('#', 1)[0]
        for part in line.split(';'):
            part = part.strip()
            if part:
                out.append((lineno, part))
    return out


def _number(token, what, where):
    try:
        return float(token)
    except ValueError:
        raise FlowSyntaxError(f'{where}: {what}应该是数字，写的是 "{token}"') from None


def _parse_nav(args, tolerances, where):
    if len(args) not in (1, 2, 3):
        raise FlowSyntaxError(f'{where}: NAV <点名> [fine | <xy_m> [<yaw_deg>]]')
    point = args[0]
    if len(args) == 1:
        return Nav(point)
    if len(args) == 2 and args[1] in tolerances:
        xy, yaw = tolerances[args[1]]
        return Nav(point, xy, yaw)
    xy = _number(args[1], '位置门限（m）', where)
    yaw = math.radians(_number(args[2], '朝向门限（度）', where)) if len(args) == 3 else None
    if xy <= 0 or xy > 0.5 or (yaw is not None and not 0 < yaw <= math.pi):
        raise FlowSyntaxError(f'{where}: 门限不合理（xy 要在 0～0.5 m，朝向 0～180°）')
    return Nav(point, xy, yaw)


def parse_flow(text, tolerances: Mapping[str, Tuple[float, float]] = None, name='?'):
    """把一个流程的文本解析成步骤列表。

    Args:
        text: 流程文本（见模块说明），也可以是字符串列表（yaml 里一行一个）。
        tolerances: 门限档位 {名字: (xy_m, yaw_rad)}，默认只有 fine。
        name: 流程名，只用于报错信息。

    Returns:
        步骤列表。ALIGN 已经展开，LABEL 已经去掉，THEN 已经换成步骤序号。

    Raises:
        FlowSyntaxError: 写错了，信息里有流程名和行号。
    """
    if tolerances is None:
        tolerances = DEFAULT_TOLERANCES
    if not isinstance(text, str):
        text = '\n'.join(str(line) for line in text)

    steps: List[object] = []
    labels = {}
    pending_then = []  # (步骤序号, 标签, where)
    for lineno, part in split_lines(text):
        where = f'流程 {name} 第 {lineno} 行 "{part}"'
        words = part.split()
        keyword, args = words[0].upper(), words[1:]
        if keyword == 'NAV':
            steps.append(_parse_nav(args, tolerances, where))
        elif keyword == 'ALIGN':
            if len(args) != 1:
                raise FlowSyntaxError(f'{where}: ALIGN <点名>')
            if 'fine' not in tolerances:
                raise FlowSyntaxError(f'{where}: ALIGN 要用 fine 门限，tolerances 里没有')
            xy, yaw = tolerances['fine']
            steps += [Nav(args[0]), Relocalize(), Nav(args[0], xy, yaw)]
        elif keyword == 'RELOCALIZE':
            if args:
                raise FlowSyntaxError(f'{where}: RELOCALIZE 不带参数')
            steps.append(Relocalize())
        elif keyword == 'WAIT':
            if len(args) != 1:
                raise FlowSyntaxError(f'{where}: WAIT <秒>')
            seconds = _number(args[0], '秒数', where)
            if seconds < 0:
                raise FlowSyntaxError(f'{where}: 秒数不能是负的')
            steps.append(Wait(seconds))
        elif keyword == 'ACTION':
            if len(args) not in (1, 2):
                raise FlowSyntaxError(f'{where}: ACTION <动作名> [参数]')
            param = 0
            if len(args) == 2:
                try:
                    param = int(args[1])
                except ValueError:
                    raise FlowSyntaxError(f'{where}: 动作参数要是整数') from None
            steps.append(Action(args[0].upper(), param))
        elif keyword == 'CALL':
            if len(args) == 1:
                steps.append(Call(args[0]))
            elif len(args) == 3 and args[1].upper() == 'THEN':
                pending_then.append((len(steps), args[2], where))
                steps.append(Call(args[0]))
            else:
                raise FlowSyntaxError(f'{where}: CALL <流程名> [THEN <标签>]')
        elif keyword == 'DETECT':
            if len(args) == 5 and args[3].upper() == 'AS':
                slot = args[4]
            elif len(args) == 3:
                slot = 'pick'
            else:
                raise FlowSyntaxError(f'{where}: DETECT <区域> <找什么> <示教拿取点> [AS <名字>]')
            what = args[1].lower()
            if what not in DETECT_WHAT:
                raise FlowSyntaxError(f'{where}: 找什么只有 {" / ".join(DETECT_WHAT)}')
            if slot.startswith('@') or args[2].startswith('@'):
                raise FlowSyntaxError(f'{where}: AS 的名字和示教点不要带 @')
            steps.append(Detect(args[0], what, args[2], slot))
        elif keyword == 'VERIFY':
            if len(args) not in (2, 3) or (len(args) == 3 and args[2].lower() != 'sky'):
                raise FlowSyntaxError(f'{where}: VERIFY <区域> <地块层数> [sky]')
            try:
                layers = int(args[1])
            except ValueError:
                raise FlowSyntaxError(f'{where}: 地块层数要是整数') from None
            if not 0 <= layers <= 3:
                raise FlowSyntaxError(f'{where}: 地块层数 0～3')
            steps.append(Verify(args[0], layers, len(args) == 3))
        elif keyword == 'LABEL':
            if len(args) != 1:
                raise FlowSyntaxError(f'{where}: LABEL <标签>')
            if args[0] in labels:
                raise FlowSyntaxError(f'{where}: 标签 {args[0]} 重复')
            labels[args[0]] = len(steps)
        else:
            raise FlowSyntaxError(
                f'{where}: 不认识 "{words[0]}"（只有 NAV ALIGN RELOCALIZE WAIT ACTION CALL '
                'DETECT VERIFY LABEL）')

    for index, label, where in pending_then:
        if label not in labels:
            raise FlowSyntaxError(f'{where}: 没有标签 {label}')
        steps[index] = Call(steps[index].flow, labels[label])
    return steps

"""FlowRegistry：按名字存流程 + 赛前检查——纯 Python，不依赖 ROS。

流程文件（yaml）长这样：

    tolerances:              # 可选，门限档位；默认只有 fine = 1 cm / 1°
      fine: {xy: 0.01, yaw_deg: 1.0}
    main: two_towers         # 开始时跑哪个流程
    retry_flows:             # Retry 成功（RESUMED <zone>）以后清栈，从这里对应的流程重新开始
      start: from_start
      l1: from_l1
    flows:
      two_towers: |
        CALL climb_l1
        ALIGN T
        ...
      climb_l1:              # 也可以写成列表，一行一个
        - NAV stairs_red
        - ACTION CLIMB_L1
    points:                  # 可选：流程自带的点（仿真演示用），点文件里有同名的以点文件为准
      transfer_look: {x: -2.2, y: 0.2, theta: -2.356}

流程的先后顺序全写在流程里，点文件（teach_point 录的 yaml）只管"点名 → 坐标"——
teach_point 存点时按键名排序（sort_keys），不能靠点文件里的顺序当路线。
"""

import math

from .steps import (Action, Call, DEFAULT_TOLERANCES, Detect, FlowSyntaxError, Nav, parse_flow,
                    slot_names)


class FlowRegistry:
    """所有流程 + 主流程名 + 各 zone 的 Retry 后流程。"""

    def __init__(self, flows, main=None, retry_flows=None, points=None):
        """flows: {流程名: [步骤]}（已解析）；points: 流程文件里自带的点 {名字: (x, y, yaw)}。"""
        self.flows = dict(flows)
        self.main = main
        self.retry_flows = dict(retry_flows or {})
        self.points = dict(points or {})

    @classmethod
    def from_dict(cls, data):
        """从流程 yaml 解析出来的 dict 建。写错了抛 FlowSyntaxError。"""
        if not isinstance(data, dict) or 'flows' not in data:
            raise FlowSyntaxError('流程文件要有 flows: 一节')
        tolerances = dict(DEFAULT_TOLERANCES)
        for name, value in (data.get('tolerances') or {}).items():
            try:
                tolerances[name] = (float(value['xy']), math.radians(float(value['yaw_deg'])))
            except (KeyError, TypeError, ValueError):
                raise FlowSyntaxError(
                    f'tolerances.{name} 要写成 {{xy: 米, yaw_deg: 度}}') from None
        flows = {}
        for name, text in (data['flows'] or {}).items():
            if text is None:
                text = ''
            flows[str(name)] = parse_flow(text, tolerances, name=str(name))
        points = {}
        for name, p in (data.get('points') or {}).items():
            try:
                points[str(name)] = (float(p['x']), float(p['y']), float(p['theta']))
            except (KeyError, TypeError, ValueError):
                raise FlowSyntaxError(f'points.{name} 要写成 {{x: 米, y: 米, theta: 弧度}}') from None
        return cls(flows, data.get('main'), data.get('retry_flows'), points)

    def get(self, name):
        """按名字取流程的步骤列表，没有抛 KeyError。"""
        return self.flows[name]

    def validate(self, points=None, actions=None):
        """赛前检查，返回问题列表（空 = 没问题）。

        Args:
            points: 录好的点名集合；None = 不查点（比如 host 上没有点文件）。
            actions: 动作码表里的动作名集合；None = 不查动作。
        """
        errors = []
        written = {name for steps in self.flows.values() for step in steps
                   if isinstance(step, Detect) for name in slot_names(step)}
        if self.main is not None and self.main not in self.flows:
            errors.append(f'main: 没有流程 {self.main}')
        for zone, name in self.retry_flows.items():
            if name not in self.flows:
                errors.append(f'retry_flows.{zone}: 没有流程 {name}')
        for name, steps in self.flows.items():
            for i, step in enumerate(steps):
                where = f'{name}#{i}'
                if isinstance(step, Call) and step.flow not in self.flows:
                    errors.append(f'{where}: CALL 的流程 {step.flow} 不存在')
                if isinstance(step, Call) and step.next_index is not None and \
                        step.next_index > len(steps):
                    errors.append(f'{where}: THEN 跳到 {step.next_index}，超出流程长度')
                if isinstance(step, Nav) and step.point.startswith('@'):
                    if step.point not in written:
                        errors.append(f'{where}: {step.point} 没有 DETECT 写它')
                elif points is not None and isinstance(step, Nav) and step.point not in points:
                    errors.append(f'{where}: 点 {step.point} 没录（点文件里没有）')
                if points is not None and isinstance(step, Detect) and \
                        step.nominal not in points:
                    errors.append(f'{where}: 示教拿取点 {step.nominal} 没录')
                if actions is not None and isinstance(step, Action) and step.name not in actions:
                    errors.append(f'{where}: 动作 {step.name} 不在动作码表里')
        errors += self._find_cycles()
        return errors

    def _find_cycles(self):
        """CALL 成环（a 调 b、b 又调 a）会无限压栈，提前查出来。"""
        errors = []
        state = {}  # 名字 -> 1 正在访问 / 2 访问完

        def visit(name, path):
            state[name] = 1
            for step in self.flows.get(name, []):
                if not isinstance(step, Call) or step.flow not in self.flows:
                    continue
                if state.get(step.flow) == 1:
                    cycle = path[path.index(step.flow):] + [step.flow]
                    errors.append('CALL 成环: ' + ' -> '.join(cycle))
                elif step.flow not in state:
                    visit(step.flow, path + [step.flow])
            state[name] = 2

        for name in self.flows:
            if name not in state:
                visit(name, [name])
        return errors

    def used_points(self, root=None):
        """从 root（默认 main 和所有 Retry 后流程）出发会走到的点名，按第一次出现的顺序。"""
        roots = [root] if root else [self.main] + list(self.retry_flows.values())
        seen_flows, out = set(), []

        def visit(name):
            if name in seen_flows or name not in self.flows:
                return
            seen_flows.add(name)
            for step in self.flows[name]:
                if isinstance(step, Nav) and not step.point.startswith('@') and \
                        step.point not in out:
                    out.append(step.point)
                elif isinstance(step, Call):
                    visit(step.flow)

        for name in roots:
            if name:
                visit(name)
        return out

"""Flow 引擎：栈 + "做完一步再给下一步"——纯 Python，不依赖 ROS，host 上单测。

学华工 stm32_control 的 ActionHandler：栈顶是正在跑的流程和它走到第几步；CALL 压栈，子流程
走完出栈，父流程走下一步（或者 CALL ... THEN 标签 指定的那一步）。华工靠 STM32 回 ack 推进，
这里靠 ROS 节点（flow_runner）执行完一个步骤后调 step_done() 推进。

引擎自己不执行任何步骤，只回答"现在该做哪一步"。CALL 在引擎里就处理掉了（压栈 / 出栈），
不会交给节点执行；节点只会拿到 NAV / RELOCALIZE / WAIT / ACTION。

Retry（0003 决定 6，field_localizer 的 /field_localizer/retry_status）：
    PAUSED <zone>        → 暂停（节点停掉当前步骤）
    RESUMED <zone>       → 清栈，从 retry_flows[zone] 重新开始（车被人搬回 Retry Zone 了，
                           原来走到哪一步已经没有意义）
    RETRY_FAILED <zone>  → 失败停住，等操作员重摆车再 retry
"""

from dataclasses import dataclass
from typing import List, Optional

from .steps import Call, describe

IDLE = 'IDLE'          # 还没开始，或者操作员停了
RUNNING = 'RUNNING'
PAUSED = 'PAUSED'      # Retry 进行中
DONE = 'DONE'          # 主流程走完
FAILED = 'FAILED'      # 某一步失败 / Retry 失败，停住等人


@dataclass
class _Frame:
    flow: str
    steps: List[object]
    index: int = 0
    return_to: Optional[int] = None   # 出栈后父流程从第几步接着走


@dataclass(frozen=True)
class StepRef:
    """交给节点执行的一个步骤。path 形如 'main#3 > place_near#1'，日志里用来定位。"""

    step: object
    path: str

    def __str__(self):
        """'main#3 > place#1 NAV N'。"""
        return f'{self.path} {describe(self.step)}'


class FlowEngine:
    """流程栈。所有方法都是同步的，节点在一个线程里调用。"""

    def __init__(self, registry, max_depth=16):
        """registry: FlowRegistry；max_depth: 栈最多几层（防止写错流程无限压栈）。"""
        self.registry = registry
        self.max_depth = max_depth
        self.state = IDLE
        self.reason = ''          # FAILED / IDLE 的原因
        self._stack: List[_Frame] = []

    # ── 查询 ──
    def current(self) -> Optional[StepRef]:
        """现在该执行的步骤；不在 RUNNING 时返回 None。"""
        if self.state != RUNNING or not self._stack:
            return None
        top = self._stack[-1]
        return StepRef(top.steps[top.index], self._path())

    def _path(self):
        return ' > '.join(f'{f.flow}#{f.index}' for f in self._stack)

    def describe_state(self):
        """一行状态，给 ~/status 话题。"""
        ref = self.current()
        if ref is not None:
            return f'{self.state} {ref}'
        if self.state == PAUSED and self._stack:
            return f'PAUSED（在 {self._path()}）'
        return f'{self.state} {self.reason}'.strip()

    # ── 控制 ──
    def run_steps(self, name, steps) -> Optional[StepRef]:
        """清栈，跑这一串已经解析好的步骤。规划器每选一件就压进来。"""
        self._stack = [_Frame(name, list(steps))]
        self.state = RUNNING
        self.reason = ''
        return self._settle()

    def start(self, flow_name=None) -> Optional[StepRef]:
        """清栈，从 flow_name（默认主流程）开始。返回第一步（流程是空的就直接 DONE）。"""
        name = flow_name or self.registry.main
        if name not in self.registry.flows:
            return self._fail(f'没有流程 {name}')
        self._stack = [_Frame(name, self.registry.get(name))]
        self.state = RUNNING
        self.reason = ''
        return self._settle()

    def stop(self, reason='操作员停止'):
        """清栈回到 IDLE。"""
        self._stack = []
        self.state = IDLE
        self.reason = reason

    def step_done(self) -> Optional[StepRef]:
        """当前步骤做完了，返回下一步（None = 走完了 / 不在 RUNNING）。"""
        if self.state != RUNNING or not self._stack:
            return None
        self._stack[-1].index += 1
        return self._settle()

    def step_failed(self, why):
        """当前步骤失败：停住，原因里带上是哪一步。v1 不自动重试，等人处理。"""
        if self.state != RUNNING:
            return
        ref = self.current()
        self._fail(f'{ref}: {why}' if ref else why)

    # ── Retry ──
    def on_retry_status(self, status, zone='') -> Optional[StepRef]:
        """field_localizer 的 retry_status。返回 RESUMED 后要执行的第一步（其它情况 None）。

        还没开始（IDLE）时什么都不做：开局前操作员调 retry 不应该让车自己跑起来。
        """
        if self.state == IDLE:
            return None
        if status == 'PAUSED':
            if self.state == RUNNING:
                self.state = PAUSED
            return None
        if status == 'RETRY_FAILED':
            self._fail(f'Retry 失败（zone={zone}），重新摆车再 retry')
            return None
        if status == 'RESUMED':
            name = self.registry.retry_flows.get(zone)
            if name is None:
                return self._fail(f'Retry 成功了，但没配 zone={zone} 的 Retry 后流程（retry_flows）')
            return self.start(name)
        return None

    # ── 内部 ──
    def _fail(self, why):
        self.state = FAILED
        self.reason = why
        return None

    def _settle(self) -> Optional[StepRef]:
        """处理掉栈顶的 CALL 和走完的流程，停在一个要节点执行的步骤上。"""
        while self._stack:
            top = self._stack[-1]
            if top.index >= len(top.steps):
                done = self._stack.pop()
                if not self._stack:
                    self.state = DONE
                    self.reason = f'{done.flow} 走完'
                    return None
                self._stack[-1].index = done.return_to
                continue
            step = top.steps[top.index]
            if isinstance(step, Call):
                if len(self._stack) >= self.max_depth:
                    return self._fail(f'{self._path()}: 压栈超过 {self.max_depth} 层（CALL 成环？）')
                if step.flow not in self.registry.flows:
                    return self._fail(f'{self._path()}: 没有流程 {step.flow}')
                return_to = step.next_index if step.next_index is not None else top.index + 1
                self._stack.append(_Frame(step.flow, self.registry.get(step.flow),
                                          return_to=return_to))
                continue
            return StepRef(step, self._path())
        return None

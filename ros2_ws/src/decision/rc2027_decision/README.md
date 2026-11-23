# rc2027_decision

决策层。现在只有 **Flow 引擎 v1**（2026-10-04）：架构学华工 `stm32_control` 的 Flow / FlowRegistry
（栈 + 做完一步再给下一步 + 运行时可以注入一段流程文本），内容按今年规则重写。Python（组长定的分层：决策用 Python）。

## 组成

| 文件 | 作用 |
|---|---|
| `rc2027_decision/flow/steps.py` | 步骤类型 + 流程文本解析（纯 Python） |
| `rc2027_decision/flow/registry.py` | 流程文件 → FlowRegistry；赛前检查（CALL 的流程在不在、点录没录、动作在不在码表、CALL 成环） |
| `rc2027_decision/flow/engine.py` | 压栈引擎：CALL 压栈、走完出栈、`THEN` 跳转、Retry 状态处理（纯 Python） |
| `rc2027_decision/flow/pick_pose.py` | 按雷达检测到的方块算拿取位姿 + 限幅（纯 Python） |
| `rc2027_decision/flow_runner_node.py` | ROS 外壳：真的去执行 NAV / RELOCALIZE / WAIT / ACTION |
| `rc2027_decision/world/field_model.py` | 世界模型：多次雷达观测 + 自己放过的 → 每个建造点现在几层（纯 Python） |
| `rc2027_decision/field_state_node.py` | 世界模型的 ROS 外壳，发 `/field_state` |
| `rc2027_decision/mock_action_executor_node.py` | 假电控：收 `/action_cmd`，按码表的时长回 `/action_state` |
| `config/flows/route_demo.yaml` | 按点顺序跑（route runner），配 `docs/field_data/sim_red.yaml` |
| `config/flows/two_towers_plan2.yaml` | BR 搭两座塔"方案 2：近点先封顶"的**骨架**（点没录、动作码是占位） |
| `config/flows/two_towers_sim.yaml` | **搭两座塔端到端仿真**（方案 2，配 `sim_tower` 和场景 `towers_plan2.yaml`） |
| `config/flows/transfer_pick_demo.yaml` | 交接区"TR 粗放、BR 雷达找块再捡"仿真演示（自带点，配场景 `transfer_trip1.yaml`） |

## 流程怎么写

```yaml
main: two_towers           # ~/start 跑哪个
retry_flows:               # Retry 成功（RESUMED <zone>）后清栈，从这里重新开始
  start: two_towers
  l1: after_l1_retry
tolerances:                # 可选，默认 fine = 1 cm / 1°
  fine: {xy: 0.01, yaw_deg: 1.0}
flows:
  two_towers: |
    CALL climb_l1
    CALL pick_stack
    ALIGN tower_near       # = NAV tower_near → RELOCALIZE → NAV tower_near fine
    ACTION PLACE 1
  pick_stack: |
    ALIGN transfer_stack
    ACTION PICK 2
```

| 步骤 | 说明 |
|---|---|
| `NAV <点名> [fine \| <xy_m> [<yaw_deg>]]` | point_navigator 的 `navigate_to_point`，ARRIVED 才算完成 |
| `ALIGN <点名>` | "精确到位"，展开成 `NAV 点` → `RELOCALIZE` → `NAV 点 fine`。拿 / 放前用 |
| `RELOCALIZE` | `/field_localizer/relocalize`；车没停稳就 0.3 s 后再试（最多 3 s），被验收拒绝只报警、接着走 |
| `WAIT <秒>` | ROS 时钟 |
| `ACTION <动作名> [参数]` | 发 `ActionCommand`，等同一个 `cmd_seq` 的 DONE；FAILED / 30 s 没回算失败。动作名见 `rc2027_interface/config/action_codes.yaml` |
| `VERIFY <区域> <地块层数> [sky]` | 放完、退开以后看一眼建造点：层数（和有没有天空块）对上才算完成，3 s 内对不上就失败停住。**放的过程中别看**（夹爪举着方块会被当成悬空的东西） |
| `CALL <流程> [THEN <标签>]` | 压栈；子流程走完回父流程下一步，或者跳到 `LABEL <标签>` 处（华工的 `on_child_done_next`） |
| `DETECT <区域> <找什么> <示教拿取点> [AS <名字>]` | 调 `/block_detector/detect` 看一个区域，挑出离示教拿取点最近、类型对得上的一摞（`ground` 单块地块 / `stack` 2 块地块一摞 / `sky` / `groundsky` 地 + 天 / `any`），算出拿取位姿 `@pick`、预备位姿 `@pick_pre`（再往后退 0.3 m，最后一段直着进）。离示教点 > 35 cm 或 > 20° 不去；没找到就 0.5 s 后再看，3 s 还不行算失败 |

一行一步，也可以用 `;` 隔开；`#` 后是注释。流程文件可以带 `points:`（仿真演示用），点文件里同名的点覆盖它。

**拿取位姿怎么算**（`DETECT`）：方块四面一样，朝向只测得到模 90°，车从四个面里挑跟示教拿取点朝向最接近的那一面去拿；
拿取位姿 = 方块中心沿那个方向往外退 `gripper_offset`（0.575 m：车头雷达伸到 0.25 m，离方块面留 15 cm，2026-10-07）。
抓整摞时按最下面那块定位。中心不可靠（只看到一部分面）的不挑。参数：`gripper_offset`、`approach_distance`、
`max_pick_shift`、`max_pick_yaw_deg`、`detect_timeout`。以后 URDF 加了 `gripper_link` 就从 TF 算**路线顺序写在流程里**，点文件（teach_point 录的）只管点名 → 坐标
（teach_point 存点按名字排序，不能拿点文件顺序当路线）。

**往已有的塔上放，先 `DETECT` 那一摞再对着它放**（2026-10-05 搭塔仿真）：`DETECT spot_03 ground far_place AS put` →
`ALIGN @put_pre` → `NAV @put fine` → `ACTION PLACE 2`。按建造点中心去放的话，塔基一偏，上一层就跟下一层对不齐（2026-10-05 间隙只有 2.5 cm 时还会顶上塔基）、
车被挤歪，上一层放偏出界；对着检测到的塔基停，间隙始终是设计值，上一层也跟下一层对齐。

**拿 / 放的最后一段不要在方块跟前 relocalize**（2026-10-05 仿真）：雷达装在车头、前倾，车顶到方块 / 塔跟前时视野被挡掉
约 70%，前后方向几乎没约束，Point-LIO 静止时都在漂，那里做 ICP 也不可信。做法：`ALIGN 预备位姿`（退 0.3 m，视野正常）→
`NAV 拿取位姿 fine` 直接开进去（见 `transfer_pick_demo.yaml`）。

为什么要 `ALIGN`：导航器到点后不会自己再动，relocalize 只改"车以为自己在哪"；实际还偏着的 1～3 cm 要用更严的门限
再发一次同一个目标才会走掉（普通 3 cm 门限下偏 2.5 cm 已经算到了）。

## 用法（仿真）

```bash
# 先起仿真（带定位 + 导航），再：
ros2 launch rc2027_decision flow.launch.py use_sim_time:=true
ros2 service call /flow_runner/start std_srvs/srv/Trigger          # 跑主流程（route_demo 是 p1→p2→p3）
ros2 topic echo /flow_runner/status
ros2 topic pub --once /flow_runner/run std_msgs/msg/String "{data: route_precise}"   # 跑别的流程
ros2 topic pub --once /flow_runner/run std_msgs/msg/String "{data: 'NAV p2; WAIT 1; NAV p1'}"  # 临时流程
ros2 service call /flow_runner/stop std_srvs/srv/Trigger
```

launch 参数：`flows`、`points`（默认 `/workspace/docs/field_data/sim_red.yaml`）、`autostart`、`mock_actions`、
`time_scale`（假电控时长系数，0 = 立刻完成）、`fail_actions`（如 `"['PLACE']"`，测失败分支）。

启动时先做赛前检查，流程里有没录的点 / 不存在的流程或动作就**直接报错退出**。

## 失败和 Retry

- 任何一步失败（导航 STUCK / TIMEOUT、动作 FAILED / 超时）：停住，`~/status` 显示 `FAILED main#3 > place#1 NAV N: STUCK`。
  v1 不自动重试
- `/field_localizer/retry_status`：`PAUSED <zone>` 停掉当前步骤；`RESUMED <zone>` **清栈**，从 `retry_flows[zone]` 重新开始；
  `RETRY_FAILED` 停住。还没 `~/start` 时不理 Retry（开局前调 retry 不会让车自己跑）
- 某一步失败停住后，操作员把车搬回 Retry Zone 调 retry，成功了就从 Retry 后流程接着跑

## 世界模型 `/field_state`（2026-10-06）

`field_state` 节点（`flow.launch.py` 默认带上，`field_state:=false` 关掉）把 block_detector 看到的
（每秒一次的定时扫 + `DETECT` / `VERIFY` 的结果，都在 `/block_detector/regions`）和自己做的
（`/action_cmd` + `/action_state` 里的 PICK / PLACE）合并成每个建造点的结论，发
`/field_state`（`rc2027_perception_msgs/FieldState`，transient local，2 Hz；结论变了打一行日志）：

| 规则 | 为什么 |
|---|---|
| 开局 10 个建造点都当空的（`SOURCE_PRIOR`） | 规则如此，不用先看一眼 |
| 层数变多要**连续 2 次**看到同样的，变少要**连续 4 次**（`confirm_up` / `confirm_down`） | 一次看错不算；正常比赛里塔只会变高 |
| 区域里有 UNKNOWN（车 / 人 / 举着的方块）的那次观测不用，只标 `obstructed` | block_detector 这时报的层数不可信（中心那摞是 UNKNOWN 时报 0 层） |
| `top_maybe_clipped` 的观测只能往上改 | 顶部出了视场，层数是"至少" |
| 层数**变少**只认"看得清"的观测：车头朝向建造点（±40°）、距离 1.8～5 m；位姿查不到也不认 | 天空块最高、又比地块往里缩 7.5 cm，背对着 / 太近时最先看不见（2026-10-06 仿真误报一次"变少了"） |
| 覆盖率 < 0.4 的"空的"不算数 | 没看清不等于空 |
| 离车 0.8 m 以内的建造点不看（`self_ignore_radius`） | 车顶到塔跟前雷达被挡一大半、夹爪举着方块（0002 10-05） |
| PLACE L 做完：按车的位姿 + `gripper_offset` 找夹爪落在哪个建造点，记"第 L 层起是我们放的、应该一共几块"；下一次看到正好这么多块就直接确认 | 自己刚放的不用等 2 次 |
| 天空块是不是我们放的 → `sky_owner`（OURS / OTHERS） | 雷达看不出红蓝，塔归天空块朝上的颜色；不是我们放的大概率是对方的（0002 10-04） |
| 塔基位置 `base_x/y/yaw`（滑动平均，跳 > 5 cm 直接换） | 往上放要对着塔基放 |

Retry（`RESUMED`）只清"车上拿着几块"，场上的塔不变。交接区最近一次看到的 RegionState 原样放在 `transfer` 里。

```bash
ros2 topic echo /field_state --field spots     # 看每个建造点
```

还没有的：规划器（按 `/field_state` 挑下一个子流程）；对方车停在建造点上一直挡着时，结论就停在挡之前（看 `age` / `obstructed`）。

## 单测（host 上直接跑，不需要 ROS）

```bash
cd ros2_ws/src/decision/rc2027_decision && python3 -m pytest test/test_flow_*.py test/test_field_model.py
```

## 待做（顺序见 docs/decisions/0002 最上面"压缩前进度"）

- [ ] 在仿真里跑通 route_demo / route_precise（用户测）
- [ ] 仿真跑 `transfer_pick_demo`（用户测）
- [ ] `wait_goods` 换成真的"货到了"判断（DETECT 交接区有没有要的那一摞）
- [x] 世界模型 `/field_state`（2026-10-06，host 单测 + 容器里假数据冒烟过；**没在仿真里跑过**）
- [x] 规则规划器（纯函数 `world/planner.py`，2026-10-07）：顺序 spot_07 → spot_01 → spot_03 → spot_02（北共享最后）。南共享若只差天空或一地加天空，先做完再动 spot_07。对方天空翻转可插队。放之前对一眼，别人放了就重选。`python3 -m rc2027_decision.plan_random` 随机跑两遍。还没接到 flow_runner 里替掉写死的流程
- [ ] 规划器接到流程上：子流程结束时按 `/field_state` 压下一步；Retry 恢复后按优先级把没看清的点看一遍
- [ ] 动作码表定稿（机械 / 电控），串口桥接 `/action_cmd`、`/action_state`

# 重复上下 L2，再回定位观察点

这是独立测试工具，不运行比赛规划，不拿放方块。新增命令 `ros2 run rc2027_utils l2_cycle_test`。2026-11-19 编写，按用户约定，本次未执行编译或测试；下面由用户运行验证。

## 每轮做什么

初始要求：车在 L1 西侧安全区域，定位与导航已启动，手里没有货。工具先去 `transfer.see`；不要从任意场中位置启动，它不做全场绕障。

1. 在观察点朝楼梯方向转身，去楼梯口。
2. 执行 `CLIMB_L2`；仿真按已知台沿 relatch，真实模式用 ICP 验收新楼层。
3. 在 L2 停 5 秒，检查高度和定位健康。
4. 回到台沿 `land`，执行 `CLIMB_DOWN`。
5. 下楼锁定后停 5 秒，检查高度。
6. 走回 `transfer.see`，先停 5 秒检查高度、位置、朝向，**此时不先 ICP**。
7. 在观察点做一次 ICP，再停 5 秒复查。
8. 通过则开始下一轮。默认 10 轮，全部通过后结束。

任何导航/动作失败、服务拒绝/超时、有效 TF 不足、超出高度容差或定位健康异常，立即结束，不自动重试、不自动回 Retry Zone、不继续下一次上下楼。失败时发零底盘速度并请求取消本工具的导航；上下楼动作自身能否中断取决于电控协议，工具没有臆造动作中止码。保留现场供分析。

正常高度容差默认 8 cm；到观察点后持续位置/朝向偏差门限为导航容差的两倍（默认 10 cm / 6°）。这是定位估计相对已知点的检查，不是独立真值精度验收。仿真请同时保存 `ground_truth_check` 日志。每轮原始里程计 z 也记下来：即使 map 高度被保持住，仍能看出底层是否继续漂。

## 编译（用户执行）

以下命令都在 `rc2027_br` 容器内执行。源码应已映射至 `/opt/ros2_ws/src`，工程根目录为 `/workspace`。

```bash
source /opt/ros/humble/setup.bash
source /opt/ros2_ws/install/setup.bash
cd /opt/ros2_ws
colcon build --packages-select rc2027_navigation rc2027_decision rc2027_utils
source /opt/ros2_ws/install/setup.bash
```

定位与决策包包含上次高度修复。编完后旧节点需重启，不能只换测试命令。

## 仿真推荐：一键启动全部前置节点

2026-11-19 补充：直接运行 `l2_cycle_test` 不会打开 Gazebo/RViz，也不会启动定位。现场三批记录都在第 0 轮初始定位超时；检查时没有相关仿真/定位进程。不能只重复执行测试节点。

编译后，在配置好 DISPLAY 的容器终端执行：

```bash
bash /workspace/ros2_ws/src/rc2027_utils/scripts/run_l2_cycle_sim.sh 1
```

先通过 1 轮，再把末尾改成 `20`。脚本不编译，自动 source ROS、外部定位工作区、业务工作区和仿真工作区，显式开启 Gazebo、RViz、定位、导航、sim_tower、真值核对和测试执行器，不启动比赛规划。等待初始定位最多120秒，与服务调用12秒超时分开；每5秒显示可见节点。检测到已有场次则拒绝启动，不清理别人的节点。

启动日志保存在 `/tmp/rc2027_l2_launch_XXXXXXXX/` 的 `sim.log`、`tower.log`、`gt.log`、`cycle.log`，结果仍在 `/tmp/rc2027_l2_cycles/`。结束/Ctrl-C只清理这批启动的进程组；启动节点提前退出时打印对应日志末尾。没有图形环境（DISPLAY为空）会直接提示，不再只报定位超时。此改动尚未由主线编译或运行，交用户验证。

## 仿真备选：分别开终端

不要同时运行 `run_two_towers_sim.sh`、`flow_runner` 或另一个测试控制器。工具发现 `flow_runner` 节点会拒绝开始。

终端 1：

```bash
export ROS_DOMAIN_ID=2
source /opt/external_ws/install/setup.bash
source /opt/ros2_ws/install/setup.bash
source /workspace/sim_ws/install/setup.bash
ros2 launch rc2027_sim_bridge sim_mapping.launch.py start_zone:=l1
```

终端 2：只开现有模拟动作执行器，不开 Flow 和 mock_action_executor，也不需要方块检测。

```bash
export ROS_DOMAIN_ID=2
source /opt/ros2_ws/install/setup.bash
ros2 run rc2027_utils sim_tower --ros-args -p random_field:=false
```

终端 3：独立真值核对。

```bash
export ROS_DOMAIN_ID=2
source /opt/ros2_ws/install/setup.bash
ros2 run rc2027_utils ground_truth_check
```

终端 4：先试 1 轮，成功后再改 20 轮。

```bash
export ROS_DOMAIN_ID=2
source /opt/ros2_ws/install/setup.bash
ros2 run rc2027_utils l2_cycle_test --ros-args \
  --params-file /opt/ros2_ws/install/rc2027_utils/share/rc2027_utils/config/l2_cycle_test.yaml \
  -p use_sim_time:=true -p cycles:=1
```

连续测时把最后的 `cycles:=1` 换成 `cycles:=20`。运行结束可重发同一命令再来一批；不需要重新编译。首次未通过先分析该轮，不让故障自动滚成后续几十轮。

## 输出

每批独立目录：`/tmp/rc2027_l2_cycles/<启动时间>/`，启动时间使用容器本地时区。

- `trace.csv`：轮次、阶段、经过秒数、ROS 时间、map 位姿、目标台面高度、高度偏差、原始 lidar_odom 的 z。
- `summary.json`：运行参数、每轮通过/失败、用时、最大高度偏差；失败时记录停止阶段与原因。

重点对比 `l2_after_lock`、`l1_after_lock`、`return_to_see`、`see_before_icp`、`see_after_icp`。如果锁定后先稳再掉，trace 会保留这段；如果 ICP 拒绝，不能把这轮算通过。原始里程计 z 在 reset 后归零属于预期，比较同一个锁定阶段内的变化，不把跨 reset 的差当成漂移。

## 换真实底盘后复用

复制 `l2_cycle_test.yaml` 成单独的底盘配置，修改楼梯口、台沿和观察点（`approaches_path`），调整导航/动作超时、容差，并填写真实动作码映射（`action_codes_path`）。默认文件引用的是当前红方仿真点，不直接作为实车示教点。

真实模式显式覆盖：

```bash
ros2 run rc2027_utils l2_cycle_test --ros-args \
  --params-file /你的路径/新底盘上下楼测试.yaml \
  -p mode:=hardware -p surface_offset:=0.0 -p use_sim_time:=false -p cycles:=1
```

此时只能开真实导航、定位和电控执行器，不能开 sim_tower 或 mock_action_executor。真实模式在上下楼前通知换层，动作成功后必须 ICP 通过，再恢复正常高度门限；**不调用 relatch，不把定位的 x/y 强制写到仿真落点**。楼梯、机构和电控动作尚未完成实车验证，这个模式目前只是可复用的测试接口；若台沿视野不足以让 ICP 通过，需要先解决观测与换层定位方案，不能放宽验收强行算成功。

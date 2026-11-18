#!/bin/bash
# 一条命令跑搭两座塔仿真。各节点的输出都打在这个终端里（带前缀），不另开终端。
#
#   bash /workspace/ros2_ws/src/rc2027_decision/scripts/run_two_towers_sim.sh
#
# 结束（流程走完，或者 Ctrl-C）时把这次拉起来的进程停掉，避免下次启动锁到旧里程计上。

FLOW=${1:-/workspace/ros2_ws/src/rc2027_decision/config/flows/two_towers_sim.yaml}
LOG=/tmp/rc2027_sim_run

# 容器里若先 source 过 /workspace/install，ros2 run 会用到那份旧包：
# 没有规划器、没有蓝方塔、交接区空了也不按新规则送第二趟。这里固定用 /opt/ros2_ws。
set +u
source /opt/ros/humble/setup.bash
source /opt/external_ws/install/setup.bash
source /opt/ros2_ws/install/setup.bash
if [ -f /workspace/sim_ws/install/setup.bash ]; then
  source /workspace/sim_ws/install/setup.bash
fi
strip_prefix() {
  local var=$1
  local val=${!var-}
  [ -n "$val" ] || return 0
  local cleaned
  cleaned=$(printf '%s' "$val" | tr ':' '\n' | grep -v '/workspace/install' | paste -sd: -)
  printf -v "$var" '%s' "$cleaned"
  export "$var"
}
strip_prefix AMENT_PREFIX_PATH
strip_prefix CMAKE_PREFIX_PATH
strip_prefix PYTHONPATH
strip_prefix PATH
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-2}
echo "本次用 /opt/ros2_ws：规划器 + 随机蓝方（ROS_DOMAIN_ID=$ROS_DOMAIN_ID）"

# 只杀 gzserver 时，ros2 launch 还在，point_navigator / field_localizer 会留到下一场。
# 两个 navigate_to_point 一起接目标，TF 时间戳对打，车一动定位就飞。
stack_lines() {
  ps -eo pid,stat,cmd \
    | grep -E '[s]im_mapping.launch.py|[b]lock_detector.launch.py|[f]low.launch.py|[g]zserver|[g]zclient|[p]ointlio_mapping|[p]oint_navigator|[f]ield_localizer|[l]io_to_base|[c]loud_crop|[r]viz2|[s]im_tower|[f]low_runner|[f]ield_state|[b]lock_detector|[g]round_truth_check|[r]obot_state_publisher' \
    | grep -v '<defunct>' || true
}
stop_stack() {
  pkill -TERM -f '[s]im_mapping.launch.py|[b]lock_detector.launch.py|[f]low.launch.py' >/dev/null 2>&1 || true
  sleep 1
  pkill -TERM -f '[g]zserver|[g]zclient|[p]ointlio_mapping|[p]oint_navigator|[f]ield_localizer|[l]io_to_base|[c]loud_crop|[r]viz2|[s]im_tower|[f]low_runner|[f]ield_state|[b]lock_detector|[g]round_truth_check|[r]obot_state_publisher' \
    >/dev/null 2>&1 || true
  sleep 1
  pkill -KILL -f '[s]im_mapping.launch.py|[b]lock_detector.launch.py|[f]low.launch.py|[g]zserver|[g]zclient|[p]ointlio_mapping|[p]oint_navigator|[f]ield_localizer|[l]io_to_base|[c]loud_crop|[r]viz2|[r]obot_state_publisher' \
    >/dev/null 2>&1 || true
}

if stack_lines | grep -q '[g]zserver'; then
  echo "已经有 gzserver 在跑。先停掉再开，不然定位会锁到旧里程计上、车一动就飞。"
  stack_lines
  exit 1
fi
leaks=$(stack_lines)
if [ -n "$leaks" ]; then
  echo "上一场的 launch 还在（gzserver 死了，导航和定位没死）。停掉再开："
  echo "$leaks"
  stop_stack
  leaks=$(stack_lines)
  if [ -n "$leaks" ]; then
    echo "还没停干净："
    echo "$leaks"
    exit 1
  fi
fi

rm -rf "$LOG"
mkdir -p "$LOG"
cleanup() {
  echo ""
  echo "停掉这次仿真"
  stop_stack
}
trap cleanup EXIT INT TERM

prefix() { sed -u "s/^/[$1] /"; }
run() {
  local tag=$1
  shift
  stdbuf -oL -eL "$@" > >(tee "$LOG/$tag.log" | prefix "$tag") 2>&1 &
}

echo "起仿真 start_zone:=l1"
run sim ros2 launch rc2027_sim_bridge sim_mapping.launch.py start_zone:=l1

echo "等 Gazebo 生成 rc2027_br"
for _ in $(seq 1 90); do
  grep -Fq 'Successfully spawned entity [rc2027_br]' "$LOG/sim.log" && break
  sleep 1
done
if ! grep -Fq 'Successfully spawned entity [rc2027_br]' "$LOG/sim.log"; then
  echo "未收到车辆生成成功日志。查看 $LOG/sim.log 中的 spawn_entity 和 gzserver。"
  exit 1
fi
echo "等 map -> base_footprint"
ready=0
for _ in $(seq 1 40); do
  if timeout 8 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | grep -q Translation; then
    ready=1
    break
  fi
done
if [ "$ready" != 1 ]; then
  echo "还没有 map -> base_footprint，停下。看上面 [sim] 的日志。"
  exit 1
fi

# 本脚本固定从 L1 开局。优先检查本场节点刚写的 map 车脚锁定值；
# 同一行后半段还有 odom 的高度，不能拿它作比较。
lock_z=$(sed -nE 's/.*已锁定：车在 map 里 \([^,]+, [^,]+, z ([-0-9.]+), yaw .*/\1/p' "$LOG/sim.log" | head -n 1)
if [[ -n "$lock_z" ]]; then
  echo "本场 L1 开局实际锁定高度：$lock_z m"
  if [[ "$lock_z" != 0.650 ]]; then
    echo "预期 L1 锁定 0.650 m，实际 $lock_z m；请核对仿真偏移和定位安装副本。" >&2
    exit 1
  fi
else
  # 日志格式不匹配时再查参数。服务发现或应答超时只表示没有读数。
  offset=''
  for attempt in 1 2 3; do
    if reply=$(LC_ALL=C timeout 12 ros2 param get /field_localizer z_check_offset 2>>"$LOG/parameter.log"); then
      if [[ "$reply" =~ Double[[:space:]]value[[:space:]]is:[[:space:]]([-+0-9.eE]+)[[:space:]]*$ ]]; then
        offset=${BASH_REMATCH[1]}
        break
      fi
    fi
    echo "第 $attempt 次未取得定位参数：$reply" | tee -a "$LOG/parameter.log" >&2
  done
  if [[ -z "$offset" ]]; then
    echo "没有读到开局锁定高度或参数应答，无法核验；见 $LOG/sim.log 和 parameter.log。" >&2
    exit 1
  fi
  echo "field_localizer 实际高度参数：$offset"
  if ! LC_ALL=C awk -v z="$offset" 'BEGIN {exit !(z+0 == 0.05)}'; then
    echo "预期仿真高度偏移 0.05，实际 $offset；请核对安装副本。" >&2
    exit 1
  fi
fi
echo "起检测、sim_tower、flow"
run det ros2 launch rc2027_perception block_detector.launch.py sim:=true
run tower ros2 run rc2027_utils sim_tower --ros-args -p random_field:=true -p seed:="${SEED:-0}"
run gt ros2 run rc2027_utils ground_truth_check
run flow ros2 launch rc2027_decision flow.launch.py use_sim_time:=true mock_actions:=false \
  planner:=true "strategy:=${STRATEGY:-full}" "flows:=$FLOW"
sleep 8

if ! grep -q '规划器接入栈' "$LOG/flow.log"; then
  echo "规划器没接上，停。上面 [flow] 应有「规划器接入栈」。"
  exit 1
fi
if ! grep -q '随机场' "$LOG/tower.log"; then
  echo "蓝方塔没摆上，停。上面 [tower] 应有「随机场」。"
  exit 1
fi

echo "开始流程 $FLOW"
ros2 service call /flow_runner/start std_srvs/srv/Trigger
echo "等流程结束"
for _ in $(seq 1 450); do
  if grep -qE '流程: (DONE|FAILED)' "$LOG/flow.log" 2>/dev/null; then
    grep -E '流程: (DONE|FAILED)' "$LOG/flow.log" | tail -1
    exit 0
  fi
  sleep 2
done
echo "15 分钟还没走完。日志在 $LOG"
exit 1

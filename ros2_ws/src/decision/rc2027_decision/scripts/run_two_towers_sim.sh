#!/bin/bash
# 一条命令跑搭两座塔仿真。各节点的输出都打在这个终端里（带前缀），不另开终端。
#
#   bash /workspace/ros2_ws/src/decision/rc2027_decision/scripts/run_two_towers_sim.sh
#
# 结束（流程走完，或者 Ctrl-C）时把这次拉起来的进程停掉，避免下次启动锁到旧里程计上。

FLOW=${1:-/workspace/ros2_ws/src/decision/rc2027_decision/config/flows/two_towers_sim.yaml}
LOG=/tmp/rc2027_sim_run

# 容器里若先 source 过 /workspace/install，ros2 run 会用到那份旧包：
# 没有规划器、没有蓝方塔、交接区空了也不按新规则送第二趟。这里固定用 /opt/ros2_ws。
set +u
source /opt/ros/humble/setup.bash
source /opt/external_ws/install/setup.bash
if [[ ! -f /opt/ros2_ws/install/setup.bash ]]; then
  echo "业务工作区尚未安装，请先在 /opt/ros2_ws 编译。" >&2
  exit 1
fi
source /opt/ros2_ws/install/setup.bash || exit 1
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

# 目录迁移后，旧 --symlink-install 安装可能仍指向迁移前的源码。
required_files=(
  'rc2027_sim_bridge:launch/sim_mapping.launch.py'
  'rc2027_navigation:launch/localization.launch.py'
  'rc2027_perception:config/cloud_crop.yaml'
)
missing_install=0
for entry in "${required_files[@]}"; do
  package=${entry%%:*}
  relative=${entry#*:}
  if ! prefix_dir=$(ros2 pkg prefix "$package" 2>/dev/null); then
    echo "未找到已安装的 ROS 包：$package" >&2
    missing_install=1
    continue
  fi
  installed_file="$prefix_dir/share/$package/$relative"
  if [[ ! -f "$installed_file" ]]; then
    echo "安装文件缺失或链接已断：$installed_file" >&2
    missing_install=1
  fi
done
if (( missing_install )); then
  echo "目录迁移后请用新的 build 和 install 目录重新编译业务工作区。" >&2
  exit 1
fi

# 只杀 gzserver 时，ros2 launch 还在，point_navigator / field_localizer 会留到下一场。
# 两个 navigate_to_point 一起接目标，TF 时间戳对打，车一动定位就飞。
stack_lines() {
  # 只匹配可执行文件和 ros2 子命令。录包进程的参数里也会有导航节点的话题名。
  ps -eo pid=,stat=,args= | awk -v kind="${1:-all}" '
    function basename(path) {
      sub(/^.*\//, "", path)
      return path
    }
    $2 !~ /^Z/ {
      index_cmd = 3
      executable = basename($index_cmd)
      if (executable ~ /^python([0-9.]+)?$/) {
        index_cmd++
        executable = basename($index_cmd)
      }
      verb = $(index_cmd + 1)
      target = $(index_cmd + 3)
      launch = executable == "ros2" && verb == "launch" &&
        target ~ /^(sim_mapping|block_detector|flow)[.]launch[.]py$/
      if (executable == "ros2" && verb == "run") {
        executable = target
      }
      node = executable ~ /^(gzserver|gzclient|pointlio_mapping|point_navigator|field_localizer|lio_to_base|cloud_crop|rviz2|sim_tower|flow_runner|field_state|block_detector|ground_truth_check|robot_state_publisher)$/
      if ((kind != "node" && launch) || (kind != "launch" && node)) {
        print $0
      }
    }'
}

signal_stack() {
  local signal=$1 kind=$2
  while read -r pid _; do
    kill -s "$signal" "$pid" 2>/dev/null || true
  done < <(stack_lines "$kind")
}

stop_stack() {
  signal_stack TERM launch
  sleep 1
  signal_stack TERM node
  sleep 1
  signal_stack KILL all
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
bag_pid=
cleanup() {
  if [[ -n "$bag_pid" ]]; then
    kill -INT "$bag_pid" 2>/dev/null
    wait "$bag_pid"
    bag_pid=
  fi
  echo ""
  echo "停掉这次仿真"
  stop_stack
}
trap cleanup EXIT
trap 'exit 130' INT TERM

prefix() { sed -u "s/^/[$1] /"; }
run() {
  local tag=$1
  shift
  stdbuf -oL -eL "$@" > >(tee "$LOG/$tag.log" | prefix "$tag") 2>&1 &
}

echo "起仿真 start_zone:=l1"
drive_args=()
if [[ "${FIRST_LEG_DRIVE:-}" == legacy ]]; then
  drive_args+=("xacro_overrides:=sim_drive_plugin:=libgazebo_ros_planar_move.so")
fi
run sim ros2 launch rc2027_sim_bridge sim_mapping.launch.py start_zone:=l1 "${drive_args[@]}"
sim_pid=$!

echo "等 Gazebo 生成 rc2027_br"
for _ in $(seq 1 90); do
  grep -Fq 'Successfully spawned entity [rc2027_br]' "$LOG/sim.log" && break
  kill -0 "$sim_pid" 2>/dev/null || break
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
if [[ -n "${FIRST_LEG_DRIVE:-}" ]]; then
  case "$FIRST_LEG_DRIVE" in
    smooth|legacy) ;;
    *) echo "FIRST_LEG_DRIVE 取 smooth 或 legacy"; exit 1 ;;
  esac
  leg_dir=$(mktemp -d "/tmp/rc2027-first-leg-${FIRST_LEG_DRIVE}-XXXXXX") || exit 1
  echo "首段录包和结果保存在 $leg_dir"
  topics=(/cmd_vel /point_navigator/status /odom /imu/data /cloud_crop/custom /aft_mapped_to_init /tf /tf_static /ground_truth/odom /clock)
  ros2 bag record -o "$leg_dir/bag" "${topics[@]}" >"$leg_dir/record.log" 2>&1 &
  bag_pid=$!
  for _ in $(seq 1 60); do
    if grep -q '随机场' "$LOG/tower.log" &&
       [[ $(grep -c 'Subscribed to topic' "$leg_dir/record.log") -ge ${#topics[@]} ]]; then
      break
    fi
    sleep 1
  done
  if ! grep -q '随机场' "$LOG/tower.log" ||
     [[ $(grep -c 'Subscribed to topic' "$leg_dir/record.log") -lt ${#topics[@]} ]]; then
    echo "场景或首段录包未就绪，查看 $leg_dir/record.log"
    exit 1
  fi
  goal=$(python3 - <<'GOAL'
import json
import math
from rc2027_decision.world.planner import bind_points, load_approaches

x, y, yaw = bind_points(load_approaches())['transfer_see']
print(json.dumps({'pose': {'header': {'frame_id': 'map'}, 'pose': {
    'position': {'x': x, 'y': y},
    'orientation': {'z': math.sin(yaw / 2), 'w': math.cos(yaw / 2)}}}}))
GOAL
  ) || exit 1
  sleep 2
  timeout 120 ros2 action send_goal /navigate_to_point rc2027_navigation_msgs/action/NavigateToPoint "$goal" | tee "$leg_dir/goal.log"
  status=${PIPESTATUS[0]}
  sleep 2
  cp "$LOG/sim.log" "$LOG/gt.log" "$leg_dir/"
  if [[ "$status" == 0 ]] && grep -Eq '^[[:space:]]*outcome: ARRIVED[[:space:]]*$' "$leg_dir/goal.log"; then
    echo "首段验证通过，车辆已到 transfer_see。录包和日志在 $leg_dir"
    exit 0
  fi
  echo "首段验证失败，查看 $leg_dir/goal.log" >&2
  exit 1
fi
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

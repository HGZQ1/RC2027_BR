#!/usr/bin/env bash
# 容器内：bash /workspace/ros2_ws/src/rc2027_utils/scripts/run_l2_cycle_sim.sh 1
# 不编译。只清理本次启动的进程组，不杀用户的其他场次。
set -eo pipefail
CYCLES=${1:-1}
if ! [[ "$CYCLES" =~ ^[1-9][0-9]*$ ]]; then
  echo '轮数必须是正整数，如：run_l2_cycle_sim.sh 20' >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
source /opt/external_ws/install/setup.bash
source /opt/ros2_ws/install/setup.bash
if [ -f /workspace/sim_ws/install/setup.bash ]; then
  source /workspace/sim_ws/install/setup.bash
fi
# 避免旧的 /workspace/install 抢到同名包。
for variable in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PATH; do
  value=${!variable-}
  cleaned=''
  IFS=: read -r -a entries <<< "$value"
  for entry in "${entries[@]}"; do
    case "$entry" in /workspace/install|/workspace/install/*) continue ;; esac
    cleaned=${cleaned:+$cleaned:}$entry
  done
  export "$variable=$cleaned"
done
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-2}
# 只检查，不擅自停别的场次。排除僵尸进程。
existing=$(ps -eo pid,stat,args | awk '
  $2 !~ /^Z/ && /sim_mapping.launch.py|\/gzserver|\/gzclient|\/pointlio_mapping|\/field_localizer( |$)|\/point_navigator( |$)|\/flow_runner( |$)|\/sim_tower( |$)|\/l2_cycle_test( |$)/ && !/awk/ {print}' )
if [ -n "$existing" ]; then
  echo '已有仿真或控制节点，请先在原终端 Ctrl-C，避免重复控制：' >&2
  echo "$existing" >&2
  exit 1
fi
if [ -z "${DISPLAY:-}" ]; then
  echo 'DISPLAY 为空，Gazebo/RViz 无法显示。请按 docker/README.md 配置图形环境。' >&2
  exit 1
fi
LOG=$(mktemp -d /tmp/rc2027_l2_launch_XXXXXXXX)
pids=()
cleanup() {
  trap - EXIT INT TERM
  echo "停止本批进程；日志保留：$LOG"
  for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  sleep 2
  for pid in "${pids[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
start() {
  local tag=$1
  shift
  setsid "$@" > "$LOG/$tag.log" 2>&1 &
  pids+=("$!")
  echo "$tag 已启动，PID=$!，日志 $LOG/$tag.log"
}
echo "本次 ROS_DOMAIN_ID=$ROS_DOMAIN_ID，轮数=$CYCLES"
start sim ros2 launch rc2027_sim_bridge sim_mapping.launch.py \
  start_zone:=l1 rviz:=true gui:=true localization:=true navigation:=true
start tower ros2 run rc2027_utils sim_tower --ros-args -p random_field:=false
start gt ros2 run rc2027_utils ground_truth_check
start cycle ros2 run rc2027_utils l2_cycle_test --ros-args \
  --params-file /opt/ros2_ws/install/rc2027_utils/share/rc2027_utils/config/l2_cycle_test.yaml \
  -p use_sim_time:=true -p startup_timeout:=120.0 -p cycles:="$CYCLES"
cycle_pid=${pids[3]}
# 实时展示测试进度；完整日志留在文件里。
setsid tail --pid="$cycle_pid" -n +1 -F "$LOG/cycle.log" &
pids+=("$!")
while kill -0 "$cycle_pid" 2>/dev/null; do
  for i in 0 1 2; do
    if ! kill -0 "${pids[$i]}" 2>/dev/null; then
      tag=sim
      [ "$i" = 1 ] && tag=tower
      [ "$i" = 2 ] && tag=gt
      echo "$tag 提前退出，日志末尾：" >&2
      tail -n 35 "$LOG/$tag.log" >&2
      exit 1
    fi
  done
  sleep 1
done
set +e
wait "$cycle_pid"
result=$?
set -e
if [ "$result" -ne 0 ]; then
  echo '本批失败，仿真启动日志末尾：' >&2
  tail -n 35 "$LOG/sim.log" >&2
fi
exit "$result"

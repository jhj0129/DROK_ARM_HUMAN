#!/usr/bin/env bash
set -Eeo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MON_LOG=/tmp/drok_human_pick_monitor.log
MON_PID=""
MON_STARTED=0
cleanup() {
  if [ "$MON_STARTED" = "1" ] && [ -n "$MON_PID" ]; then
    kill "$MON_PID" 2>/dev/null || true
    wait "$MON_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

source /opt/ros/humble/setup.bash
[ -f "$ROOT/install/setup.bash" ] || { echo "[ERROR] 먼저 bash build.sh"; exit 1; }
source "$ROOT/install/setup.bash"

for IFACE in can10 can11; do
  ip link show "$IFACE" >/dev/null 2>&1 || { echo "[ERROR] $IFACE 없음"; exit 1; }
  ip link show "$IFACE" | grep -q "UP" || { echo "[ERROR] $IFACE DOWN"; exit 1; }
done

SESSION="$HOME/.cache/drok_arm_human/session_reference.yaml"
[ -f "$SESSION" ] || { echo "[ERROR] 먼저 ./go_humanarm_home.sh 를 실행하세요."; exit 1; }

BAD="$(ros2 node list 2>/dev/null | grep -E 'moveit_to_rmd_bridge|joystick_to_rmd_control|drokck' || true)"
[ -z "$BAD" ] || { echo "[ERROR] 다른 CAN writer 실행 중:"; echo "$BAD"; exit 1; }

if ! ros2 topic list 2>/dev/null | grep -qx /humanarm/raw_motor_deg; then
  ros2 run humanarm_mode humanarm_motor_monitor </dev/null >"$MON_LOG" 2>&1 &
  MON_PID=$!
  MON_STARTED=1
  for _ in $(seq 1 80); do
    ros2 topic list 2>/dev/null | grep -qx /humanarm/raw_motor_deg && break
    sleep 0.1
  done
fi

ros2 topic list 2>/dev/null | grep -qx /humanarm/raw_motor_deg || { echo "[ERROR] RAW feedback start failed"; exit 1; }
ros2 run humanarm_mode humanarm_pick_60cm

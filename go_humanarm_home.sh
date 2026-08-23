#!/usr/bin/env bash
set -Eeo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEP="$ROOT/.deps/DROK_ARM_IK"
MON_LOG=/tmp/drok_human_raw_monitor.log
BRIDGE_LOG=/tmp/drok_human_legacy_bridge.log
MON_PID=""
BRIDGE_PID=""
MON_STARTED=0

cleanup() {
  if [ -n "$BRIDGE_PID" ]; then
    kill -- "-$BRIDGE_PID" 2>/dev/null || true
    sleep 0.5
    kill -9 -- "-$BRIDGE_PID" 2>/dev/null || true
    wait "$BRIDGE_PID" 2>/dev/null || true
  fi
  if [ "$MON_STARTED" = "1" ] && [ -n "$MON_PID" ]; then
    kill "$MON_PID" 2>/dev/null || true
    wait "$MON_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

source /opt/ros/humble/setup.bash
if [ ! -f "$ROOT/install/setup.bash" ]; then
  echo "[ERROR] 먼저 bash build.sh 를 실행하세요."
  exit 1
fi
source "$ROOT/install/setup.bash"

for IFACE in can10 can11; do
  ip link show "$IFACE" >/dev/null 2>&1 || { echo "[ERROR] $IFACE 없음"; exit 1; }
  ip link show "$IFACE" | grep -q "UP" || { echo "[ERROR] $IFACE DOWN"; exit 1; }
done

if ! ros2 topic list 2>/dev/null | grep -qx /humanarm/raw_motor_deg; then
  ros2 run humanarm_mode humanarm_motor_monitor </dev/null >"$MON_LOG" 2>&1 &
  MON_PID=$!
  MON_STARTED=1
  for _ in $(seq 1 80); do
    ros2 topic list 2>/dev/null | grep -qx /humanarm/raw_motor_deg && break
    sleep 0.1
  done
fi

ros2 topic list 2>/dev/null | grep -qx /humanarm/raw_motor_deg || { echo "[ERROR] RAW feedback 시작 실패"; tail -50 "$MON_LOG" || true; exit 1; }

echo
echo "========================================================================"
echo " DROK: POWER-ON/SAGGED -> LEGACY HOME -> HUMANARM HOME"
echo "========================================================================"
echo "전원 ON 직전 기준: J6=Legacy 방향, J7=FULL OPEN"
echo "J2/J3가 중력으로 처져 있어도 정상입니다."
echo

ros2 run humanarm_mode humanarm_capture_boot_reference

read -r -p "실행하려면 MOVE_ARM 입력: " CONFIRM
[ "$CONFIRM" = "MOVE_ARM" ] || { echo "[CANCEL]"; exit 0; }

setsid ros2 launch drok_real_arm_bridge real_arm_bridge.launch.py dry_run:=false default_max_speed:=30 >"$BRIDGE_LOG" 2>&1 &
BRIDGE_PID=$!
for _ in $(seq 1 100); do
  ros2 topic list 2>/dev/null | grep -qx /joint_states && break
  sleep 0.1
done
ros2 topic list 2>/dev/null | grep -qx /joint_states || { echo "[ERROR] bridge startup failed"; tail -100 "$BRIDGE_LOG"; exit 1; }

DROK_ARM_IK_ROOT="$DEP" ros2 run humanarm_mode humanarm_legacy_home_once

kill -- "-$BRIDGE_PID" 2>/dev/null || true
sleep 1
kill -9 -- "-$BRIDGE_PID" 2>/dev/null || true
wait "$BRIDGE_PID" 2>/dev/null || true
BRIDGE_PID=""

ros2 run humanarm_mode humanarm_session_control human-home
ros2 run humanarm_mode humanarm_session_control status

#!/usr/bin/env bash
set -Eeo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEP="$ROOT/.deps/DROK_ARM_IK"
PIN="26be1ec8480b0b44a26221fec2810bd5a38951be"

source /opt/ros/humble/setup.bash

mkdir -p "$ROOT/.deps"
if [ ! -d "$DEP/.git" ]; then
  git clone https://github.com/jhj0129/DROK_ARM_IK.git "$DEP"
fi

git -C "$DEP" fetch origin --tags
git -C "$DEP" checkout --detach "$PIN"

ln -sfn ../.deps/DROK_ARM_IK/src/drok_arm_kinematics "$ROOT/src/drok_arm_kinematics"
ln -sfn ../.deps/DROK_ARM_IK/src/drok_real_arm_bridge "$ROOT/src/drok_real_arm_bridge"

cd "$ROOT"
if command -v rosdep >/dev/null 2>&1; then
  rosdep install --from-paths src --ignore-src -r -y
fi

colcon build --symlink-install
chmod +x "$ROOT/build.sh" "$ROOT/go_humanarm_home.sh" "$ROOT/pick_60cm.sh"

echo
echo "[OK] Build complete"
echo "source $ROOT/install/setup.bash"
echo "./go_humanarm_home.sh"
echo "./pick_60cm.sh"

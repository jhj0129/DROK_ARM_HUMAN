#!/usr/bin/env python3
import importlib.util
import math
import os
from pathlib import Path

import rclpy


def main():
    dep_root = os.environ.get("DROK_ARM_IK_ROOT")
    if not dep_root:
        raise RuntimeError("DROK_ARM_IK_ROOT 환경변수가 없습니다.")
    core_path = Path(dep_root) / "tools/interactive_box_ik_grasp_v11.py"
    if not core_path.exists():
        raise RuntimeError(f"dependency tool 없음: {core_path}")

    spec = importlib.util.spec_from_file_location("drok_core", core_path)
    core = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(core)

    rclpy.init()
    node = core.RealFeedbackNode()
    try:
        if not node.wait_for_feedback(8.0):
            raise RuntimeError("Legacy /joint_states feedback 없음")
        node.refresh()
        if node.current_q is None:
            raise RuntimeError("현재 joint feedback 없음")

        start_q = node.current_q.copy()
        target_q = core.HOME_Q.copy()
        target_q[0] = float(start_q[0])  # J1 HOLD
        target_q[5] = float(start_q[5])  # J6 HOLD: fixed absolute J6 home 사용 금지

        print("\n" + "="*76)
        print(" SAGGED -> LEGACY HOME | J1/J6 HOLD")
        print("="*76)
        print("Start  :", core.format_q_deg(start_q))
        print("Target :", core.format_q_deg(target_q))
        print(f"J1 HOLD = {math.degrees(target_q[0]):+.3f} deg")
        print(f"J6 HOLD = {math.degrees(target_q[5]):+.3f} deg")
        print("="*76)

        exe = core.DirectArmRmdExecutor(node)
        try:
            ok = exe.move_poly5(target_q, 6.0, "SAGGED -> LEGACY HOME | J1/J6 HOLD")
            print("LEGACY HOME RESULT =", ok)
            if not ok:
                raise RuntimeError("Legacy Home arrival failed")
        finally:
            exe.close()
    finally:
        node.destroy_node()
        rclpy.shutdown()

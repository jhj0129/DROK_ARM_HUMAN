#!/usr/bin/env python3
import math
import os
import re
import subprocess
import time

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

from .common import (
    ARM_JOINTS,
    RAW_INDEX,
    human_home_targets,
    load_session,
    load_yaml,
    move_gripper,
    move_raw_targets,
    read_raw,
)

TARGET_X_M = 0.60
REACH_DURATION_SEC = 12.0
RETURN_DURATION_SEC = 12.0
GRIP_DURATION_SEC = 4.0
GRIP_HOLD_SEC = 1.0


def rotation_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]], dtype=float)
    ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]], dtype=float)
    rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]], dtype=float)
    return rz @ ry @ rx


def rotation_axis(axis, angle):
    a = np.asarray(axis, dtype=float)
    n = np.linalg.norm(a)
    if n < 1e-12:
        raise RuntimeError("zero rotation axis")
    x, y, z = a / n
    k = np.array([[0,-z,y],[z,0,-x],[-y,x,0]], dtype=float)
    return np.eye(3)*math.cos(angle) + (1-math.cos(angle))*np.outer(a/n, a/n) + math.sin(angle)*k


def transform(xyz, rot):
    t = np.eye(4)
    t[:3,:3] = rot
    t[:3,3] = np.asarray(xyz, dtype=float)
    return t


def geometry_path():
    return os.path.join(get_package_share_directory("humanarm_mode"), "config", "robot_geometry.yaml")


def fk(q):
    with open(geometry_path(), "r", encoding="utf-8") as f:
        chain = yaml.safe_load(f)["kinematic_chain"]
    t = np.eye(4)
    qi = 0
    for joint in chain:
        xyz = joint["origin"]["xyz"]
        rpy = joint["origin"]["rpy"]
        t = t @ transform(xyz, rotation_rpy(float(rpy[0]), float(rpy[1]), float(rpy[2])))
        if joint["type"] in ("revolute", "continuous"):
            t = t @ transform([0,0,0], rotation_axis(joint["axis"], float(q[qi])))
            qi += 1
    if qi != 6:
        raise RuntimeError("FK movable joint count mismatch")
    return t


def matrix_to_rpy(r):
    pitch = math.asin(max(-1.0, min(1.0, -float(r[2,0]))))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(r[2,1], r[2,2])
        yaw = math.atan2(r[1,0], r[0,0])
    else:
        roll = 0.0
        yaw = math.atan2(-r[0,1], r[1,1])
    return roll, pitch, yaw


def orientation_error_deg(target_rotation, actual_rotation):
    r_err = target_rotation.T @ actual_rotation
    value = (float(np.trace(r_err)) - 1.0) * 0.5
    value = max(-1.0, min(1.0, value))
    return math.degrees(math.acos(value))


def tolerance_settings():
    cfg = load_yaml("control_tolerance.yaml")
    ik = cfg["ik"]
    completion = cfg["completion"]
    return {
        "ik_position_m": float(ik["position_tolerance_m"]),
        "ik_orientation_rad": float(ik["orientation_tolerance_rad"]),
        "joint_deg": float(completion["joint_tolerance_deg"]),
        "tcp_position_m": float(completion["tcp_position_tolerance_m"]),
        "tcp_orientation_deg": float(completion["tcp_orientation_tolerance_deg"]),
        "settle_sec": float(completion["settle_time_sec"]),
    }


def mapping_arrays():
    m = load_yaml("humanarm_mapping.yaml")
    return (
        np.asarray(m["kinematic_mapping"]["model_home_rad"], dtype=float),
        np.asarray(m["kinematic_mapping"]["legacy_raw_home_deg"], dtype=float),
    )


def raw_to_model_q(raw, session, model_home, legacy_raw_home):
    q = np.zeros(6, dtype=float)
    for i, joint in enumerate(ARM_JOINTS[:5]):
        q[i] = model_home[i] + math.radians(float(raw[RAW_INDEX[joint]]) - legacy_raw_home[i])
    q[5] = model_home[5] + math.radians(
        float(raw[RAW_INDEX["J6"]]) - float(session["j6_boot_legacy_raw_deg"])
    )
    return q


def model_q_to_raw(q, session, model_home, legacy_raw_home):
    targets = {}
    for i, joint in enumerate(ARM_JOINTS[:5]):
        targets[joint] = legacy_raw_home[i] + math.degrees(float(q[i]) - model_home[i])
    targets["J6"] = float(session["j6_boot_legacy_raw_deg"]) + math.degrees(float(q[5]) - model_home[5])
    return targets


def solve_full_pose(target_xyz, target_rpy, seed_q, tol):
    args = [
        "ros2", "run", "drok_arm_kinematics", "solve_ik_pose", geometry_path(),
        *(f"{float(v):.12f}" for v in target_xyz),
        *(f"{float(v):.12f}" for v in target_rpy),
        *(f"{float(v):.12f}" for v in seed_q),
    ]
    env = os.environ.copy()
    env["DROK_IK_MODE"] = "full"
    env["DROK_IK_POSITION_TOLERANCE_M"] = f"{tol['ik_position_m']:.12g}"
    env["DROK_IK_ORIENTATION_TOLERANCE_RAD"] = f"{tol['ik_orientation_rad']:.12g}"
    result = subprocess.run(args, env=env, text=True, capture_output=True)
    print(result.stdout)
    if result.returncode != 0:
        if result.stderr:
            print(result.stderr)
        raise RuntimeError("IK solver failed. Motor command was NOT started.")
    match = re.search(r"JOINT_RESULT=([^\n\r]+)", result.stdout)
    if not match:
        raise RuntimeError("IK JOINT_RESULT parse failed")
    values = [float(x.strip()) for x in match.group(1).split(",")]
    if len(values) != 6:
        raise RuntimeError("IK result joint count != 6")
    return np.asarray(values, dtype=float)


def completion_report(title, q_target, q_actual, tol):
    target_t = fk(q_target)
    actual_t = fk(q_actual)
    joint_error_deg = np.abs(np.degrees(q_actual - q_target))
    position_error_m = float(np.linalg.norm(actual_t[:3,3] - target_t[:3,3]))
    orientation_error = orientation_error_deg(target_t[:3,:3], actual_t[:3,:3])

    joint_pass = bool(np.all(joint_error_deg <= tol["joint_deg"]))
    position_pass = position_error_m <= tol["tcp_position_m"]
    orientation_pass = orientation_error <= tol["tcp_orientation_deg"]
    passed = joint_pass and position_pass and orientation_pass

    print("\n" + "="*76)
    print(f" {title}")
    print("="*76)
    for i, joint in enumerate(ARM_JOINTS):
        flag = "PASS" if joint_error_deg[i] <= tol["joint_deg"] else "FAIL"
        print(f"{joint}: error={joint_error_deg[i]:.3f} deg / {tol['joint_deg']:.3f} deg  {flag}")
    print(f"TCP position    : {position_error_m*1000.0:.3f} mm / {tol['tcp_position_m']*1000.0:.1f} mm")
    print(f"TCP orientation : {orientation_error:.3f} deg / {tol['tcp_orientation_deg']:.1f} deg")
    print(f"Post-settle     : {tol['settle_sec']:.2f} sec minimum (trajectory endpoint hold is longer)")
    print("RESULT          : " + ("PASS" if passed else "FAIL"))
    print("="*76)
    return passed


def print_raw(title, raw):
    print("\n" + "="*76)
    print(f" {title}")
    print("="*76)
    for j in ("J1","J2","J3","J4","J5","J6","J7"):
        print(f"{j}: {raw[RAW_INDEX[j]]:+10.4f} deg")
    print("="*76)


def main():
    print("\n" + "="*76)
    print(" HUMANARM IK PICK - ARM_BASE_LINK X = 0.60 m")
    print("="*76)
    print("GRIPPER OPEN -> full-pose DLS IK -> GRASP -> HUMANARM HOME")
    print("Arm trajectory: 1 kHz, 9th-order smootherstep")
    print("="*76)

    session = load_session()
    model_home, legacy_raw_home = mapping_arrays()
    mapping = load_yaml("humanarm_mapping.yaml")
    close_travel = float(mapping["session"]["j7_close_travel_deg"])
    tol = tolerance_settings()

    print("\nDROK HumanArm tolerance:")
    print(f"  IK position       : {tol['ik_position_m']*1000.0:.3f} mm")
    print(f"  IK orientation    : {math.degrees(tol['ik_orientation_rad']):.3f} deg")
    print(f"  Joint completion  : +/- {tol['joint_deg']:.3f} deg")
    print(f"  TCP position      : +/- {tol['tcp_position_m']*1000.0:.1f} mm")
    print(f"  TCP orientation   : +/- {tol['tcp_orientation_deg']:.1f} deg")

    raw_start = read_raw()
    print_raw("START RAW", raw_start)

    print("\n>>> STEP 1 / 4 : GRIPPER FULL OPEN")
    j7_open = float(session["j7_open_raw_deg"])
    move_gripper(float(raw_start[RAW_INDEX["J7"]]), j7_open, GRIP_DURATION_SEC, "GRIPPER -> FULL OPEN")
    raw_start = read_raw()
    print_raw("AFTER GRIPPER OPEN", raw_start)

    q_start = raw_to_model_q(raw_start, session, model_home, legacy_raw_home)
    t_start = fk(q_start)
    xyz_start = t_start[:3,3].copy()
    rpy_start = matrix_to_rpy(t_start[:3,:3])
    print(f"\nCurrent TCP XYZ [m] = [{xyz_start[0]:+.4f}, {xyz_start[1]:+.4f}, {xyz_start[2]:+.4f}]")
    print("Current TCP RPY [deg] = [" + ", ".join(f"{math.degrees(v):+.2f}" for v in rpy_start) + "]")

    target_xyz = np.asarray([TARGET_X_M, float(xyz_start[1]), float(xyz_start[2])], dtype=float)
    print("\n" + "="*76)
    print(" FULL-POSE DLS IK")
    print("="*76)
    print(f"Target TCP = [{target_xyz[0]:+.4f}, {target_xyz[1]:+.4f}, {target_xyz[2]:+.4f}] m")
    q_target = solve_full_pose(target_xyz, rpy_start, q_start, tol)
    solved = fk(q_target)[:3,3]
    print(f"Solved TCP XYZ [m] = [{solved[0]:+.5f}, {solved[1]:+.5f}, {solved[2]:+.5f}]")
    print(f"FK position error = {np.linalg.norm(solved-target_xyz)*1000.0:.3f} mm")

    raw_target = model_q_to_raw(q_target, session, model_home, legacy_raw_home)
    print("\n>>> STEP 2 / 4 : MOVE TO OBJECT")
    move_raw_targets(raw_start, raw_target, REACH_DURATION_SEC, "HUMANARM HOME -> X=0.60m IK TARGET", joints=ARM_JOINTS)
    raw_at_object = read_raw()
    print_raw("AT OBJECT", raw_at_object)

    q_at_object = raw_to_model_q(raw_at_object, session, model_home, legacy_raw_home)
    if not completion_report("REACH COMPLETION CHECK", q_target, q_at_object, tol):
        raise RuntimeError("Reach tolerance failed. Gripper close was NOT started.")

    print("\n>>> STEP 3 / 4 : GRASP OBJECT")
    j7_close = j7_open + close_travel
    print(f"Session J7 FULL OPEN = {j7_open:+.3f} deg")
    print(f"J7 grasp target      = {j7_close:+.3f} deg")
    move_gripper(float(raw_at_object[RAW_INDEX["J7"]]), j7_close, GRIP_DURATION_SEC, "GRIPPER -> CLOSE / GRASP")
    time.sleep(GRIP_HOLD_SEC)

    print("\n>>> STEP 4 / 4 : RETURN TO HUMANARM HOME")
    raw_before_return = read_raw()
    home_raw = human_home_targets(session)
    move_raw_targets(raw_before_return, home_raw, RETURN_DURATION_SEC, "IK TARGET -> HUMANARM HOME", joints=ARM_JOINTS)
    raw_final = read_raw()
    print_raw("FINAL RAW", raw_final)

    q_final = raw_to_model_q(raw_final, session, model_home, legacy_raw_home)
    raw_home_vector = list(raw_final)
    for joint in ARM_JOINTS:
        raw_home_vector[RAW_INDEX[joint]] = float(home_raw[joint])
    q_home_target = raw_to_model_q(raw_home_vector, session, model_home, legacy_raw_home)
    completion_report("HOME COMPLETION CHECK", q_home_target, q_final, tol)

    print("\n" + "="*76)
    print(" PICK SEQUENCE COMPLETE")
    print("="*76)
    print("TCP reached: ARM_BASE_LINK X = 0.600 m")
    print("Gripper: FULL OPEN -> GRASP -> CLOSED/HOLDING")
    print("Arm: RETURNED TO HUMANARM HOME")
    print("="*76)

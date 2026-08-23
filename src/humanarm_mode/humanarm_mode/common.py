#!/usr/bin/env python3
import math
import socket
import struct
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

RAW_INDEX = {"J1": 0, "J2": 1, "J2_SLAVE": 2, "J3": 3, "J4": 4, "J5": 5, "J6": 6, "J7": 7}
ARM_JOINTS = ["J1", "J2", "J3", "J4", "J5", "J6"]
MOTOR = {
    "J1": ("can10", 0x141, 36.0),
    "J2": ("can10", 0x142, 36.0),
    "J3": ("can10", 0x144, 36.0),
    "J4": ("can11", 0x141, 1.0),
    "J5": ("can11", 0x142, 1.0),
    "J6": ("can11", 0x143, 6.0),
    "J7": ("can11", 0x144, 6.0),
}

COMMAND_HZ = 1000.0
DYNAMIC_SPEED_MARGIN = 1.18
MIN_PROTOCOL_SPEED = {"J1": 18, "J2": 18, "J3": 18, "J4": 4, "J5": 3, "J6": 9, "J7": 9}
MAX_PROTOCOL_SPEED = {"J1": 1200, "J2": 1200, "J3": 1200, "J4": 300, "J5": 300, "J6": 600, "J7": 600}
COUNT_SEND_STEP = {"J1": 1, "J2": 1, "J3": 1, "J4": 1, "J5": 1, "J6": 1, "J7": 1}
FINAL_SETTLE_SEC = 0.35
SESSION_FILE = Path.home() / ".cache/drok_arm_human/session_reference.yaml"


def config_path(name: str) -> Path:
    return Path(get_package_share_directory("humanarm_mode")) / "config" / name


def load_yaml(name: str):
    with config_path(name).open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def load_session():
    if not SESSION_FILE.exists():
        raise RuntimeError(
            f"HumanArm session reference가 없습니다: {SESSION_FILE}\n"
            "먼저 go_humanarm_home.sh를 실행하세요."
        )
    with SESSION_FILE.open("r", encoding="utf-8") as f:
        doc = yaml.safe_load(f) or {}
    for key in ("j6_boot_legacy_raw_deg", "j6_home_raw_deg", "j7_open_raw_deg"):
        if key not in doc:
            raise RuntimeError(f"session_reference.yaml에 {key}가 없습니다.")
    return doc


def save_boot_session(raw):
    mapping = load_yaml("humanarm_mapping.yaml")
    j6_delta = float(mapping["session"]["j6_legacy_to_human_delta_deg"])
    doc = {
        "captured_unix_time": time.time(),
        "j6_boot_legacy_raw_deg": float(raw[RAW_INDEX["J6"]]),
        "j6_home_raw_deg": float(raw[RAW_INDEX["J6"]]) + j6_delta,
        "j7_open_raw_deg": float(raw[RAW_INDEX["J7"]]),
        "legacy_j6_human_deg": -j6_delta,
    }
    SESSION_FILE.parent.mkdir(parents=True, exist_ok=True)
    with SESSION_FILE.open("w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, sort_keys=False)
    return doc


class RawFeedback(Node):
    def __init__(self, node_name="humanarm_feedback_reader"):
        super().__init__(node_name)
        self.raw = None
        self.create_subscription(Float64MultiArray, "/humanarm/raw_motor_deg", self._cb, 20)

    def _cb(self, msg):
        if len(msg.data) >= 8:
            self.raw = list(msg.data[:8])


def read_raw(timeout_sec=5.0):
    rclpy.init()
    node = RawFeedback()
    try:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and node.raw is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.raw is None:
            raise RuntimeError("/humanarm/raw_motor_deg feedback가 없습니다. raw monitor를 실행하세요.")
        return node.raw.copy()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def smooth9(tau):
    t = max(0.0, min(1.0, float(tau)))
    return 126.0*t**5 - 420.0*t**6 + 540.0*t**7 - 315.0*t**8 + 70.0*t**9


def smooth9_d1(tau):
    t = max(0.0, min(1.0, float(tau)))
    return 630.0 * t**4 * (1.0 - t)**4


def wait_until(deadline):
    while True:
        remaining = deadline - time.perf_counter()
        if remaining <= 0.0:
            return
        if remaining > 0.00080:
            time.sleep(remaining - 0.00045)
        elif remaining > 0.00030:
            time.sleep(0)
        else:
            pass


def raw_to_counts(joint, raw_deg):
    _, _, gear = MOTOR[joint]
    return int(round(float(raw_deg) * gear / 0.01))


def open_sockets(joints):
    sockets = {}
    for iface in sorted({MOTOR[j][0] for j in joints}):
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.bind((iface,))
        sockets[iface] = sock
    return sockets


def close_sockets(sockets):
    for sock in sockets.values():
        sock.close()


def send_counts(sockets, joint, counts, max_speed):
    iface, motor_id, _ = MOTOR[joint]
    speed = int(max(1, min(65535, int(max_speed))))
    payload = struct.pack("<BBHi", 0xA4, 0x00, speed, int(counts))
    frame = struct.pack("=IB3x8s", motor_id, 8, payload)
    sockets[iface].send(frame)


def dynamic_speed(joint, delta_raw_deg, duration_sec, tau):
    _, _, gear = MOTOR[joint]
    output_dps = abs(float(delta_raw_deg)) * smooth9_d1(tau) / max(float(duration_sec), 1.0e-6)
    requested = int(math.ceil(output_dps * gear * DYNAMIC_SPEED_MARGIN + 0.5))
    return min(MAX_PROTOCOL_SPEED[joint], max(MIN_PROTOCOL_SPEED[joint], requested))


def monotonic_count(count, previous, direction):
    if previous is None:
        return count
    if direction > 0:
        return max(count, previous)
    if direction < 0:
        return min(count, previous)
    return previous


def move_raw_targets(start_raw, targets, duration_sec, label, joints=None, print_timing=True):
    joints = list(joints or targets.keys())
    start = {j: float(start_raw[RAW_INDEX[j]]) for j in joints}
    delta = {j: float(targets[j]) - start[j] for j in joints}
    start_count = {j: raw_to_counts(j, start[j]) for j in joints}
    target_count = {j: raw_to_counts(j, targets[j]) for j in joints}
    direction = {
        j: (1 if target_count[j] > start_count[j] else -1 if target_count[j] < start_count[j] else 0)
        for j in joints
    }

    print("\n" + "="*76)
    print(f" {label}")
    print("="*76)
    for j in joints:
        print(f"{j}: {start[j]:+10.3f} -> {float(targets[j]):+10.3f} deg | delta={delta[j]:+9.3f}")
    print(f"9th-order smootherstep : {duration_sec:.2f} sec")
    print(f"command rate           : {COMMAND_HZ:.1f} Hz")
    print("="*76)

    sockets = open_sockets(joints)
    last_sent = {j: None for j in joints}
    periods = []
    try:
        samples = max(2, int(round(duration_sec * COMMAND_HZ)))
        dt = duration_sec / samples
        t0 = time.perf_counter()
        prev_t = t0

        for j in joints:
            send_counts(sockets, j, start_count[j], MIN_PROTOCOL_SPEED[j])
            last_sent[j] = start_count[j]

        for i in range(1, samples + 1):
            tau = i / samples
            wait_until(t0 + i*dt)
            now = time.perf_counter()
            periods.append(now - prev_t)
            prev_t = now
            blend = smooth9(tau)
            for j in joints:
                raw_cmd = start[j] + blend * delta[j]
                count = monotonic_count(raw_to_counts(j, raw_cmd), last_sent[j], direction[j])
                is_final = i == samples
                if not is_final and abs(count - last_sent[j]) < COUNT_SEND_STEP[j]:
                    continue
                send_counts(sockets, j, count, dynamic_speed(j, delta[j], duration_sec, tau))
                last_sent[j] = count

        settle_end = time.perf_counter() + FINAL_SETTLE_SEC
        while time.perf_counter() < settle_end:
            for j in joints:
                send_counts(sockets, j, target_count[j], MIN_PROTOCOL_SPEED[j])
            time.sleep(0.02)
    finally:
        close_sockets(sockets)

    if print_timing and periods:
        ms = [x*1000.0 for x in periods]
        print("\n" + "="*76)
        print(" 1 kHz STREAM TIMING")
        print("="*76)
        print(f"Average period : {sum(ms)/len(ms):.4f} ms")
        print(f"Maximum period : {max(ms):.4f} ms")
        print(f"> 1.5 ms       : {sum(1 for x in ms if x > 1.5)} / {len(ms)}")
        print(f"> 2.0 ms       : {sum(1 for x in ms if x > 2.0)} / {len(ms)}")
        print("="*76)


def move_gripper(current_raw, target_raw, duration_sec=4.0, label="GRIPPER MOVE"):
    current_raw = float(current_raw)
    target_raw = float(target_raw)
    if abs(target_raw - current_raw) < 0.05:
        print(f"\n[{label}] already at target: {current_raw:+.3f} deg")
        return
    start_raw = [0.0] * 8
    start_raw[RAW_INDEX["J7"]] = current_raw
    move_raw_targets(start_raw, {"J7": target_raw}, duration_sec, label, joints=["J7"], print_timing=False)


def human_home_targets(session):
    home = load_yaml("humanarm_home.yaml")["humanarm_home_raw_deg"]
    return {
        "J1": float(home["J1"]),
        "J2": float(home["J2"]),
        "J3": float(home["J3"]),
        "J4": float(home["J4"]),
        "J5": float(home["J5"]),
        "J6": float(session["j6_home_raw_deg"]),
    }

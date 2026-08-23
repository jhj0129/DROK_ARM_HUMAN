#!/usr/bin/env python3
import select
import socket
import struct
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

CAN_FRAME_FMT = "=IB3x8s"
MOTORS = [
    ("can10", 0x141), ("can10", 0x142), ("can10", 0x143), ("can10", 0x144),
    ("can11", 0x141), ("can11", 0x142), ("can11", 0x143), ("can11", 0x144),
]
INDEX = {key: i for i, key in enumerate(MOTORS)}


def signed_int48_le(data6):
    value = int.from_bytes(bytes(data6), byteorder="little", signed=False)
    if value & (1 << 47):
        value -= 1 << 48
    return value


def parse_angle(iface, motor_id, data):
    if iface == "can10":
        return signed_int48_le(data[1:7]) * (0.01 / 36.0)
    if motor_id in (0x141, 0x142):
        return int.from_bytes(bytes(data[4:8]), byteorder="little", signed=True) * 0.01
    return signed_int48_le(data[1:7]) * (0.01 / 6.0)


class HumanArmRawMonitor(Node):
    def __init__(self):
        super().__init__("humanarm_motor_monitor")
        self.pub = self.create_publisher(Float64MultiArray, "/humanarm/raw_motor_deg", 10)
        self.values = [0.0] * 8
        self.valid = [False] * 8
        self.sockets = {}
        for iface in ("can10", "can11"):
            s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            s.setblocking(False)
            s.bind((iface,))
            self.sockets[iface] = s
        self.timer = self.create_timer(0.01, self.update)
        self.get_logger().info("HumanArm raw monitor: can10/can11, 100 Hz")

    def destroy_node(self):
        for s in self.sockets.values():
            try:
                s.close()
            except Exception:
                pass
        super().destroy_node()

    def update(self):
        req = bytes([0x92, 0, 0, 0, 0, 0, 0, 0])
        for iface, motor_id in MOTORS:
            frame = struct.pack(CAN_FRAME_FMT, motor_id, 8, req)
            try:
                self.sockets[iface].send(frame)
            except OSError:
                pass

        deadline = time.perf_counter() + 0.006
        while time.perf_counter() < deadline:
            readable, _, _ = select.select(list(self.sockets.values()), [], [], 0.0005)
            if not readable:
                continue
            for sock in readable:
                iface = next(k for k, v in self.sockets.items() if v is sock)
                try:
                    frame = sock.recv(16)
                except BlockingIOError:
                    continue
                if len(frame) < 16:
                    continue
                can_id, dlc, payload = struct.unpack(CAN_FRAME_FMT, frame)
                rid = can_id & 0x7FF
                lookup = rid
                if iface == "can11":
                    if rid == 0x241:
                        lookup = 0x141
                    elif rid == 0x242:
                        lookup = 0x142
                key = (iface, lookup)
                if key not in INDEX or payload[0] != 0x92:
                    continue
                try:
                    self.values[INDEX[key]] = parse_angle(iface, lookup, payload)
                    self.valid[INDEX[key]] = True
                except Exception:
                    continue

        if all(self.valid):
            msg = Float64MultiArray()
            msg.data = list(self.values)
            self.pub.publish(msg)


def main():
    rclpy.init()
    node = HumanArmRawMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

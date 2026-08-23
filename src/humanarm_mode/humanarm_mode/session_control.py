#!/usr/bin/env python3
import argparse

from .common import RAW_INDEX, human_home_targets, load_session, move_raw_targets, read_raw


def status():
    session = load_session()
    targets = human_home_targets(session)
    raw = read_raw()
    print("\n" + "="*70)
    print(" HUMANARM SESSION STATUS")
    print("="*70)
    for j in ("J1", "J2", "J3", "J4", "J5", "J6"):
        print(f"{j}: RAW={raw[RAW_INDEX[j]]:+10.4f} | HOME_ERR={raw[RAW_INDEX[j]]-targets[j]:+9.4f} deg")
    print(f"J7: RAW={raw[RAW_INDEX['J7']]:+10.4f} | from FULL OPEN={raw[RAW_INDEX['J7']]-float(session['j7_open_raw_deg']):+9.4f} deg")
    print("="*70)


def human_home():
    session = load_session()
    raw = read_raw()
    targets = human_home_targets(session)
    move_raw_targets(raw, targets, 12.0, "LEGACY/CURRENT -> HUMANARM HOME", joints=list(targets.keys()))


def j6_home():
    session = load_session()
    raw = read_raw()
    move_raw_targets(raw, {"J6": float(session["j6_home_raw_deg"])}, 6.0, "J6 -> HUMANARM HOME", joints=["J6"])


def j6_legacy():
    session = load_session()
    raw = read_raw()
    move_raw_targets(raw, {"J6": float(session["j6_boot_legacy_raw_deg"])}, 6.0, "J6 -> LEGACY", joints=["J6"])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=["status", "human-home", "j6-home", "j6-legacy"])
    args = parser.parse_args()
    {"status": status, "human-home": human_home, "j6-home": j6_home, "j6-legacy": j6_legacy}[args.command]()

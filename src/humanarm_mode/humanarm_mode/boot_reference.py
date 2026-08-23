#!/usr/bin/env python3
from .common import read_raw, save_boot_session


def main():
    raw = read_raw()
    session = save_boot_session(raw)
    print("\n" + "="*70)
    print(" HUMANARM POWER-ON SESSION")
    print("="*70)
    print(f"J6 boot Legacy RAW  = {session['j6_boot_legacy_raw_deg']:+.6f} deg")
    print(f"J6 HumanArm Home RAW= {session['j6_home_raw_deg']:+.6f} deg")
    print("                      (Legacy -> CLOCKWISE 90 deg)")
    print(f"J7 FULL OPEN RAW     = {session['j7_open_raw_deg']:+.6f} deg")
    print("="*70)

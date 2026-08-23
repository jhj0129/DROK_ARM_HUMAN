#!/usr/bin/env python3
from pathlib import Path
import sys


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: patch_ik_solver_tolerance.py <solve_ik_pose.cpp>")
        return 2

    path = Path(sys.argv[1]).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(path)

    text = path.read_text(encoding="utf-8")

    pos_marker = "DROK_IK_POSITION_TOLERANCE_M"
    ori_marker = "DROK_IK_ORIENTATION_TOLERANCE_RAD"

    if pos_marker in text and ori_marker in text:
        print(f"[OK] IK tolerance env patch already present: {path}")
        return 0

    old_pos = "    options.position_tolerance = 1.0e-5;"
    new_pos = '''    const char * position_tolerance_environment =\n      std::getenv("DROK_IK_POSITION_TOLERANCE_M");\n\n    options.position_tolerance =\n      position_tolerance_environment == nullptr ?\n      1.0e-5 :\n      parseDouble(\n      position_tolerance_environment,\n      "DROK_IK_POSITION_TOLERANCE_M");'''

    old_ori = "      options.orientation_tolerance = 1.0e-5;"
    new_ori = '''      const char * orientation_tolerance_environment =\n        std::getenv("DROK_IK_ORIENTATION_TOLERANCE_RAD");\n\n      options.orientation_tolerance =\n        orientation_tolerance_environment == nullptr ?\n        1.0e-5 :\n        parseDouble(\n        orientation_tolerance_environment,\n        "DROK_IK_ORIENTATION_TOLERANCE_RAD");'''

    if old_pos not in text:
        raise RuntimeError("position_tolerance insertion point not found")
    if old_ori not in text:
        raise RuntimeError("orientation_tolerance insertion point not found")

    text = text.replace(old_pos, new_pos, 1)
    text = text.replace(old_ori, new_ori, 1)
    path.write_text(text, encoding="utf-8")

    print(f"[OK] Patched IK solver tolerance environment support: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

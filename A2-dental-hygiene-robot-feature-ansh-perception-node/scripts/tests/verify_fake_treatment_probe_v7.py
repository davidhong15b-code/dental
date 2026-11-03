#!/usr/bin/env python3
"""Verify the new fake-only station look-ahead exists in source AND binary."""
from pathlib import Path
from shutil import which
import subprocess
import sys
root = Path(__file__).resolve().parents[2]
identifier = b"DENTAL_FAKE_APPROACH_LOOKAHEAD_V7"
source = root / "src/dental_motion/src/station_motion_node.cpp"
installed = root / "install/dental_motion/lib/dental_motion/station_motion_node"
checks = ((source, "source"), (installed, "binary"))
errors = []
for path, label in checks:
    ok = path.is_file() and identifier in path.read_bytes()
    print(("PASS" if ok else "FAIL") + f" {label}: {path}")
    if not ok:
        errors.append(str(path))
if errors:
    print("FAIL: source and installed station executable differ or build failed")
    sys.exit(1)
print("PASS: v7 fake inspection look-ahead present in installed station executable")

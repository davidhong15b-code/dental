#!/usr/bin/env python3
"""Offline source/install consistency check; does not contact a robot.

Use after colcon build and sourcing the project overlay. The binary build ID
prevents silently testing an older compiled motion implementation.
"""
import os
from pathlib import Path
import subprocess
import sys

# Each executable has an independent version: modifying one need not force
# rebuilding unrelated C++ executables. Feature markers match the v3 transfer
# planner and the v4 deterministic reinspection planner.
MOTION_BUILD_ID = b'DENTAL_MVP_BUILD_ID=consolidated_humble_v2_20261103'
STATION_BUILD_ID = b'DENTAL_MVP_BUILD_ID=station_revisit_v4_20261103'


def main():
    repo = Path(__file__).resolve().parents[2]
    motion_src = repo / 'src/dental_motion/src/motion_node.cpp'
    station_src = repo / 'src/dental_motion/src/station_motion_node.cpp'
    try:
        prefix = Path(subprocess.check_output(
            ['ros2', 'pkg', 'prefix', 'dental_motion'], text=True).strip()).resolve()
    except (OSError, subprocess.CalledProcessError) as exc:
        print('FAIL: dental_motion is not discoverable:', exc, file=sys.stderr)
        return 1
    expected = (repo / 'install/dental_motion').resolve()
    print('Repository:', repo)
    print('ROS package prefix:', prefix)
    if prefix != expected:
        print('FAIL: ROS resolves dental_motion outside this repository. Source scripts/source_ros.sh.',
              file=sys.stderr)
        return 1
    checks = [
        (motion_src, prefix / 'lib/dental_motion/motion_node',
         MOTION_BUILD_ID, b'FAKE PREAPPROACH yaw %zu/4 %s'),
        (station_src, prefix / 'lib/dental_motion/station_motion_node',
         STATION_BUILD_ID, b'REVISIT_inspect: planning to saved first-pass inspection joints'),
    ]
    ok = True
    for src, exe, build_id, feature in checks:
        if not src.is_file() or not exe.is_file():
            print('FAIL: source or binary missing:', src, exe, file=sys.stderr)
            ok = False
            continue
        sb = src.read_bytes()
        bb = exe.read_bytes()
        for label, needle, blob in [
            ('source build ID', build_id, sb),
            ('binary build ID', build_id, bb),
            ('source feature', feature, sb),
            ('binary feature', feature, bb),
        ]:
            exists = needle in blob
            print(('PASS' if exists else 'FAIL'), src.name, label)
            ok = ok and exists
    if not ok:
        print('FAIL: current installed executable does not match the intended source.',
              file=sys.stderr)
        print('Stop all launches; rebuild dental_motion; re-source; repeat verification.',
              file=sys.stderr)
        return 1
    print('PASS: installed motion and station executables contain expected compiled code.')
    print('IMPORTANT: this cannot prove an already-running process is the new binary.')
    print('Stop existing launches, then relaunch from a fresh terminal and check startup build IDs.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

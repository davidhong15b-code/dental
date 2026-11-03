#!/usr/bin/env python3
"""Check current v5 home-first code is installed, not merely present in src/."""
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
FILES=(
    ('src/dental_motion/src/station_motion_node.cpp',
     'install/dental_motion/lib/dental_motion/station_motion_node',
     b'DENTAL_REAL_HOME_V5_STATION'),
    ('src/dental_motion/src/motion_node.cpp',
     'install/dental_motion/lib/dental_motion/motion_node',
     b'DENTAL_REAL_HOME_V5_MOTION'),
    ('src/dental_supervisor/src/supervisor_node.cpp',
     'install/dental_supervisor/lib/dental_supervisor/supervisor_node',
     b'DENTAL_REAL_HOME_V5_SUPERVISOR'),
)

def main():
    ok=True
    for source,binary,marker in FILES:
        for role,file in [('source',ROOT/source),('binary',ROOT/binary)]:
            found=file.is_file() and marker in file.read_bytes()
            print(('PASS' if found else 'FAIL'),role,file.relative_to(ROOT))
            ok &= found
    print('PASS v5 compiled in all three nodes' if ok else
          'FAIL: rebuild dental_motion AND dental_supervisor and restart the launch')
    return 0 if ok else 2
if __name__=='__main__':raise SystemExit(main())

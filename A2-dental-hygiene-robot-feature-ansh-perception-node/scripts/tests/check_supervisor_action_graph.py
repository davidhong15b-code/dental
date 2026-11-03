#!/usr/bin/env python3
"""Read-only ROS 2 graph check; refuses ambiguous supervisor action-server counts."""
import re
import subprocess
import sys

ACTION = "/dental/supervisor/run_cycle"


def main():
    try:
        proc = subprocess.run(["ros2", "action", "info", ACTION],
                              capture_output=True, text=True, timeout=20,
                              check=False)
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        print(f"FAIL: cannot query ROS 2 action graph: {exc}", file=sys.stderr)
        return 2
    combined = proc.stdout + "\n" + proc.stderr
    print(combined.strip())
    if proc.returncode:
        print(f"FAIL: ros2 action info exited with {proc.returncode}", file=sys.stderr)
        return 2
    match = re.search(r"Action servers:\s*(\d+)", combined)
    if match is None:
        print("FAIL: action-server count missing (cannot verify uniqueness)", file=sys.stderr)
        return 2
    count = int(match.group(1))
    if count != 1:
        print(f"FAIL: expected exactly ONE RunCycle server, found {count}; do not send a goal", file=sys.stderr)
        return 2
    print("PASS: exactly one RunCycle action server is advertised")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

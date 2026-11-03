#!/usr/bin/env python3
"""Offline AST sanity check for common ROS 2 launch list-construction mistakes.

Run from repository root: python3 scripts/tests/check_launch_structure.py
No ROS environment or robot is required.
"""
import ast
import sys
from pathlib import Path


def check_file(path: Path) -> int:
    try:
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    except (SyntaxError, OSError) as exc:
        print(f"FAIL {path}: {exc}")
        return 1
    errors = 0
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == "append" and len(node.args) != 1):
            print(f"FAIL {path}:{node.lineno}: append() has {len(node.args)} positional arguments")
            errors += 1
    if not errors:
        print(f"PASS {path}")
    return errors


def main() -> int:
    repo = Path(__file__).resolve().parents[2]
    launch_dir = repo / "scripts" / "launch"
    files = sorted(launch_dir.rglob("*.launch.py"))
    if not files:
        print(f"FAIL: no launch files found in {launch_dir}")
        return 1
    errors = sum(check_file(path) for path in files)
    if errors:
        print(f"FAIL: {errors} launch-file error(s) found")
        return 1
    print(f"PASS: {len(files)} launch files parsed; no invalid list.append() calls")
    return 0


if __name__ == "__main__":
    sys.exit(main())

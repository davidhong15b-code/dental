# MTRN4231 Real UR5e Commissioning v5 — overlay

This overlays the current, verified **v4 fake MVP**; it is NOT a complete repository.
The first no-contact fake cycle remains unchanged; real execution stays locked.

Review the full instructions: `docs/REAL_UR5E_READY_V5.md`.

## Install away from the lab

```bash
cd ~/Desktop/4231/A2-dental-hygiene-robot
git status --short
# Commit/stash any local modifications to these same files before overwrite.
unzip -o ~/Downloads/A2_dental_real_ready_v5.zip -d .
source scripts/source_ros.sh --underlays-only
colcon build --symlink-install \
  --packages-select dental_motion dental_supervisor dental_bringup \
  --event-handlers console_direct+
source install/local_setup.bash
python3 scripts/tests/verify_real_home_build.py
python3 scripts/tests/preflight_real_offline.py --mode preview
```

## Real-motion safety gate

Do not execute until the controller, measured active TCP/pendant pose,
robot-specific UR5e kinematics, overhead and wrist camera extrinsics,
ArUco layout, full attached tool, collision scene and smooth home transfer are
independently verified in the lab.

The six supplied values are interpreted as a pendant **Base-frame TCP pose**
`p[-0.58853, -0.13330, 0.22700, 2.221, 2.221, 0]`, not joint angles.
The robot's six-joint home must be captured once at the lab, and the real
supervisor will command that approved joint configuration first.

The fake MVP remains validated from the previous run; neither C++ code here
nor this ZIP has been executed with ROS 2 or hardware in the current environment.

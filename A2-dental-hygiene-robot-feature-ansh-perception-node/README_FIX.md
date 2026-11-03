# v5 fake MoveIt launch list.append() correction

## Root cause
`scripts/launch/fake_ur_moveit.launch.py` incorrectly passed 11 `DeclareLaunchArgument(...)`
objects to `declared_arguments.append(...)`. Python's `list.append` takes one object;
`extend([...])` is required for multiple launch declarations.

The corrected file changes only that list construction. It does not change MoveIt
configuration, collision geometry, controller settings, C++ nodes or motion guards.

## Apply
Stop existing ROS launches, then from the project root:

```bash
cp scripts/launch/fake_ur_moveit.launch.py /tmp/fake_ur_moveit.launch.py.before_v5_fix
unzip -o ~/Downloads/A2_dental_v5_launch_append_fix.zip -d .
python3 scripts/tests/check_launch_structure.py
python3 -m py_compile scripts/launch/fake_ur_moveit.launch.py scripts/launch/mvp_fake.launch.py
```

Expected check: `PASS: 3 launch files parsed; no invalid list.append() calls`.
The check is offline and does not require ROS or a robot.

Re-run your existing fake `ros2 launch scripts/launch/mvp_fake.launch.py ...` command.
There is no `colcon build` step for a pure Python launch-file correction.

This is a structural launch check, not a full ROS runtime or real-hardware safety test.

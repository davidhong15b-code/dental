# ROS 2 Humble compatibility and stale motion binary guard

This overlay is based on the *previously supplied consolidated rework*, which was
based on the repository ZIP supplied by the user. Apply the consolidated rework
first if it has not yet been extracted, then apply this overlay. This does NOT
change collision geometry, motion limits, tool touch links, robot safety gates,
or treatment trajectory planning. These are build-verification and diagnostic fixes.

Modified files:
- `scripts/tests/preflight_mvp_fake.py`: typed ActionClient `.server_is_ready()`
  on Humble instead of missing `Node.get_action_names_and_types()`.
- `scripts/tests/run_mvp_cycle.py`: early aborts print the actual last phase and
  returned result instead of an unhelpful completion assertion.
- `src/dental_motion/src/motion_node.cpp` and `station_motion_node.cpp`: start-up
  build IDs for tracing code running in ROS; no planning logic change.
- `scripts/tests/verify_mvp_build.py`: checks the compiled executable against
  the exact expected build ID, correct package prefix and compiled feature log.

After stopping the launch, from the project root:

```bash
unzip -o ~/Downloads/A2_dental_MVP_humble_build_diagnostics.zip -d .
source scripts/source_ros.sh --underlays-only
colcon build --symlink-install --packages-select dental_motion --cmake-clean-cache
source install/local_setup.bash
python3 scripts/tests/verify_mvp_build.py
```

The verifier must PASS before launching. Launch from a new terminal and confirm
both nodes log `DENTAL_MVP_BUILD_ID=consolidated_humble_v2_20261103`.

Then test preflight and run cycle, if preflight passes. If the fake preapproach
still fails, capture the four `FAKE PREAPPROACH candidate ...` messages and
associated MoveIt collision details. Do not raise joint limits or suppress
collisions. Real hardware is out of scope until separate commissioning.

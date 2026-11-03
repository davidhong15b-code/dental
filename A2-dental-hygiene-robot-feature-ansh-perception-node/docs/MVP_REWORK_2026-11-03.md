# MTRN4231 dental MVP — consolidated motion-planning rework (3 Nov 2026)

## Scope and evidence

This patch is based on the **actual uploaded repository**, not a guessed earlier revision. The real UR5e and ROS 2 Humble cannot be run in the patch-authoring environment, so **the C++ changes are uncompiled here**. Treat this as a testable source update, not a validated hardware release.

Your logs already prove the following **in fake hardware**: the collision workcell loads; `dental_model` and four tooth TF frames resolve; MoveIt executes inspection; the mock inspector reports one stain and the target manager selects ID 1001. The remaining issue is treatment planning from a poor inspected wrist branch. Inspection also sometimes selects unnecessarily long joint configurations because OMPL samples different IK solutions.

## Changes

1. `station_motion_node.cpp` now evaluates the existing four downward-facing fake inspection camera yaw choices and **chooses the lowest joint-L1 valid trajectory**, rather than the first one. The existing per-step, per-joint, total-travel and collision guards remain intact. Real station motion unchanged.
2. `motion_node.cpp` now tries four **fake-only** tool orientations rotating around the target surface normal for the first PREAPPROACH. For each, it checks distance, MoveIt collision-free planning, joint travel, joint jumps and duration. It chooses the shortest passing plan. The model-frame TCP target is unchanged; the configured brush-offset vector is rotated with the tool. It does **not** allow collision with the table, jaw or forearm; does not change real treatment IK.
3. Each previously planned treatment segment now requires **current joint positions to match the first trajectory waypoint within 0.03 rad for every joint** before it can execute. A mismatch aborts instead of sending a stale planned path. This applies in both fake and real modes.
4. `scripts/tests/run_mvp_cycle.py` now checks action status, zero treated count, actual feedback for inspection, both completed hover passes, repeated inspection and return home. A residual stain remains an **ABORTED** action (because the robot did not clean); the CLI reports a successful *workflow test* only if all those stages occurred.
5. `scripts/tests/preflight_mvp_fake.py` is a **read-only** graph, TF and workcell readiness check. It sends **no movement**.
6. Fake launch defaults to `use_real_perception=false`, so image-only live nodes do not start unexpectedly. Fake station and treatment execution also **reject action goals if the verified collision workcell is missing**.
7. Real station/treatment action servers now support **planning without execution** when measured workcell and calibration attestations are present but `execute_real_motion=false`. They still return a non-success action result at the execution gate; they do not command the controller. Do not run a supervised cycle expecting actual camera movement in plan-only mode.
8. `scripts/launch/mvp_real.launch.py` no longer broadcasts a non-verified reference wrist-camera TF by default. The reference is only published with `publish_verified_reference_wrist_tf:=true` after proving the real mounting is identical. Never publish it in parallel with calibration camera wrapper TF publishers.

## Apply only to the existing repo

Stop the fake/real launches. From `~/Desktop/4231/A2-dental-hygiene-robot`:

```bash
# Consider saving local changes before overwriting these source files.
git status --short
unzip -o ~/Downloads/A2_dental_MVP_consolidated_rework.zip -d .

# Fresh terminal: do not source an old workspace setup file.
source scripts/source_ros.sh --underlays-only
colcon build --symlink-install --packages-select dental_motion --cmake-clean-cache
source install/local_setup.bash
ros2 pkg executables dental_motion
```

The updated script paths are **regular files**, not symlinks into the backup repo:

```bash
readlink -f scripts/tests/run_mvp_cycle.py
python3 -m py_compile scripts/tests/*.py scripts/launch/mvp_real.launch.py
```

If compilation fails, stop and send **the first compiler error**. Do not continue to test older installed binaries.

## Fake end-to-end: single integrated test after build

Ensure **no real UR driver** is connected in ROS_DOMAIN_ID 50. Use a new terminal:

```bash
cd ~/Desktop/4231/A2-dental-hygiene-robot
source scripts/source_ros.sh
ros2 launch scripts/launch/mvp_fake.launch.py \
  start_fake_driver:=true execute_fake_motion:=true \
  confirm_fake_hardware:=true use_mock_inspection:=true \
  use_real_perception:=false mock_clear_after:=-1 \
  load_workcell:=true launch_rviz:=true
```

In another terminal, after the collision workcell is loaded:

```bash
cd ~/Desktop/4231/A2-dental-hygiene-robot
source scripts/source_ros.sh
python3 scripts/tests/preflight_mvp_fake.py
python3 scripts/tests/run_mvp_cycle.py --timeout 600
```

Expected first phases: `MOVING_TO_inspect`, `AT_inspect`, `INSPECTING pass=1`, `SCAN_COMPLETE findings=1`, `MOTION_REQUEST`, `PLANNING_PREAPPROACH`. Expected final outcome with `mock_clear_after=-1`: two actual `HOVER_STROKE_COMPLETE` phases, final reinspection, `AT_home`, and ROS action **ABORTED** with a residual stain message. The test script then prints `PASS: TWO hover passes ... NO physical cleaning claimed`.

A failed planning stage must remain a failure. **Do not** increase travel bounds, disable the workcell, add the forearm to `touch_links`, change the virtual fixture to hide a collision, or manually move the real robot to satisfy these tests.

## If the integrated fake cycle still fails

Record the FIRST specific `motion_node` failure and `move_group` contact message. In particular:

- If all four PREAPPROACH candidates fail, the **simulated table, jaw and initial inspection posture are kinematically incompatible** with a no-contact hover at those chosen positions; a more suitable *measured/configured* virtual starting pose or fixture location is required. Do not pretend software can solve infeasible geometry.
- If a later `HOVER` or `SWEEP` has no local IK, the fully planned segment sequence needs a continuous Cartesian/IK path rework before attempting real execution. Do not increase limits to push through an IK branch switch.
- If actual/planned start error exceeds 0.03 rad, halt and diagnose controller or joint-state synchronization.
- If `/dental/workcell/ready` is absent, inspect `apply_workcell.py` and MoveIt's get/apply scene service; do not override readiness.

## Real hardware is a distinct commissioning step

The reference calibration is only a **starting point**, never a measured lab TF. The real workcell must have the actual overhead base transform, marker layout relative to the jaw, bracket-specific wrist optical transform, jaw/table obstacle dimensions, tool envelope and brush TCP. Confirm active controller, actual UR5e kinematics, robot mode, emergency stops and operator clearance. First run with execution **disabled** and review the plan with the demonstrator. Only proceed to supervised low-speed motion once all frames, distances, orientations and collision objects are physically verified. A successful fake run does **not** certify any real robot path.

The current software still has engineering limitations: separate motion action servers can command the same MoveGroup; camera extrinsic sign and tool-normal convention need independent measurement; there is no force sensing or contact-cleaning controller. Until interlocks, trajectory validation and physical workcell commissioning are complete, treat the real MVP as **plan-only/supervised no-contact testing**, not an autonomous production robot.

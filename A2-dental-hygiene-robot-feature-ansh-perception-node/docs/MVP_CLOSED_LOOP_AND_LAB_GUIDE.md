# MTRN4231 — MVP Closed-Loop UR5e Integration and Lab Commissioning

## Scope and implemented contract

This increment replaces the `dental_supervisor` reject-all scaffold with a real ROS 2 C++
`/dental/supervisor/run_cycle` action orchestrator. It preserves the team's existing
`InspectTeeth`, `DentalFindingArray`, `DentalTarget` and `TreatTarget` contracts.
The tool controller stays **a teammate-owned placeholder**; it is neither launched
nor called by the MVP. A toothbrush is assumed to be passively mounted.

Current Increment 3 `TreatTarget` can plan/execute a **no-contact** tool0 preapproach,
hover and retract sequence on **fake hardware only**. It always aborts with
`success=false` even when positioning finishes. The supervisor specifically checks
its `EXECUTED_RETRACT` feedback *and* final no-cleaning result, records that a
positioning pass occurred, then requests a NEW inspection.

**This is not verified plaque removal.** `RunCycle.treated_count=0` remains correct.
If the last scan still sees targets, the cycle aborts at the configured pass cap.
If a later scan reports no findings after positioning, the supervisor STILL refuses
to claim successful cleaning. An initial scan with no findings can return success.

## Data-flow diagram

```text
                       /dental/supervisor/run_cycle
                                   |
                          [supervisor C++]
                                   |
              INSPECT (InspectTeeth action; fresh sensor data)
                                   v
         [teammate's dental_perception OR EXPLICIT synthetic adapter]
                     |                          |
        /dental/perception/findings       InspectTeeth result
                     |                          |
         [target_manager C++] <---- scan stamp matched to result
                     |
         /dental/targets/selected (base_link, transformed normal)
                     |
                 [supervisor] --- verifies matching stamp/ID
                     |
             TreatTarget (tool_id = brush)
                     |
             [dental_motion C++ / MoveIt2]
                     |
        controller /joint_trajectory_controller (FAKE UR5e)
                     |
           EXECUTED_RETRACT + action result
                     |
           SUPERVISOR requests NEW inspection
                     |
            Repeat up to max_positioning_passes
```

Supervisor: one cycle at a time; rejects concurrent cycles; watches for fresh
joint states before/throughout motion; action timeouts and cancellation;
requires measured workcell readiness if `load_workcell:=true`; never accepts
unmatched/stale selected targets. A bare message arriving on the findings topic
is not considered an inspection result: the perception node must implement the
on-demand `InspectTeeth` action and publish the **identical timestamped** findings.

## Install and build (Ubuntu 22.04, ROS 2 Humble)

From the repository root, after successfully applying the patch:

```bash
cd ~/Desktop/4231/A2-dental-hygiene-robot
source scripts/source_ros.sh --underlays-only
colcon build --symlink-install --packages-up-to \
  dental_interfaces dental_target_manager dental_supervisor dental_motion dental_bringup
source install/local_setup.bash
ros2 interface show dental_interfaces/action/RunCycle
```

All terminals must use **the same ROS_DOMAIN_ID=50** and underlays. `scripts/source_ros.sh`
sets these in the team's existing configuration. Build may warn if the project
install workspace was already sourced as an underlay; use the `--underlays-only`
option and a clean terminal when needed.

The first integrated launch is `scripts/launch/mvp_fake.launch.py`. It starts:
- UR ROS driver with `use_fake_hardware:=true`, UR5e and unscaled joint trajectory controller;
- MoveIt and RViz with the existing verified URDF/SRDF;
- C++ target manager, C++ supervisor, and dental motion action;
- an **obviously synthetic** inspection action adapter (replaceable);
- optional measured workcell scene loader (default OFF).

**If your fake driver is already running**, use `start_fake_driver:=false` to avoid
starting two controller managers. If robot description or controllers are unavailable,
use the original two-terminal fake driver + MoveIt launch as proven in Increment 3
and start application nodes via `ros2 launch dental_bringup nodes.launch.py`.

### First: plan-only wiring check

Close previously running duplicate `move_group`, `motion_node`, supervisor,
target manager and mock inspector processes before starting the integrated launch.

Terminal 1:

```bash
source scripts/source_ros.sh
ros2 launch ./scripts/launch/mvp_fake.launch.py \
  execute_fake_motion:=false \
  max_passes:=2
```

Terminal 2:

```bash
source scripts/source_ros.sh
ros2 action list -t
ros2 control list_controllers
python3 scripts/tests/run_mvp_cycle.py
```

`RunCycle` should reach an inspection, selected target, and planned motion,
then ABORT because **plan-only is not executed**. Do not proceed until the target
manager logs show the selected ID and the motion planner reports bounded joint
paths. Check `/joint_states` and base_link/tool0 TF.

### Second: verified fake-UR5e execution (no physical robot)

Only after checking the driver is **fake hardware** and the RViz plan is clear:

```bash
ros2 launch ./scripts/launch/mvp_fake.launch.py \
  execute_fake_motion:=true \
  confirm_fake_hardware:=true \
  motion_velocity_scaling:=0.15 \
  motion_acceleration_scaling:=0.15 \
  max_passes:=2
```

(Stop the previous integrated launch first.) In a second terminal:

```bash
python3 scripts/tests/run_mvp_cycle.py --timeout 180
```

Expected flow: `INSPECTING pass=1` -> `SCAN_COMPLETE` -> `AWAITING_TF_SELECTED_TARGET`
-> `MOTION_REQUEST` -> `MOTION_EXECUTING_*` -> `MOTION_EXECUTED_RETRACT`
-> `POSITION_ONLY_COMPLETE` -> `INSPECTING pass=2` -> another execution ->
`INSPECTING pass=3` -> residual finding -> terminate at pass cap.

The synthetic adapter generates one reachable candidate approximately 150 mm
below the **current** tool0, with upward normal. It **does not** project onto a tooth;
this is only for debugging perception/motion/supervisor interfaces. It keeps
reporting a finding, since no cleaning occurs. An optional `mock_clear_after:=2`
returns an empty result on scan 3 for testing the empty-findings branch; this
**does not prove cleaning** and `RunCycle` still will not claim it did.

### Teammate's real perception integration (still with FAKE UR5e)

1. Launch the **actual camera** (RGB, aligned depth, CameraInfo), its extrinsic
   calibration transform, and the teammate's `dental_perception` node/action.
2. Start `mvp_fake.launch.py use_mock_inspection:=false`, plus optional measured
   workcell as below. Do not start the mock inspector alongside real perception.
3. Verify `ros2 action info /dental/perception/inspect` shows the teammate action.
4. Request `RunCycle`. Each request must trigger a fresh actual image acquisition,
   and result.findings must equal the published `/dental/perception/findings`
   including the **same header timestamp**.
5. Inspect perception's frame ID, camera intrinsics, depth units, point/normals;
   validate using a physical fiducial. Measure and publish camera->base extrinsics;
   never silently use the most-recent transform if the teeth may have moved.
6. If the jaw moves between scans, reacquire all target positions each time;
   pre-existing targets do not remain valid.

The target manager currently selects **one highest-confidence eligible target
per inspection**. Multiple targets therefore require successive scans/passes;
it does not process a batch of findings from a single scan.

## Measured workcell and RViz

A planning scene loader is included, but accurate physical coordinates are not
known yet. It **refuses to load** until the YAML calibration is explicitly set
and all required dimensions/poses are measured:

`src/dental_bringup/config/workcell.template.yaml`

Start by copying it to `src/dental_bringup/config/workcell.lab.yaml` (keep that
local calibration file out of the team repo until validated). Measure, in
**base_link metres**:

- table centre and bounding box, including thickness;
- jaw collision envelope and location, plus STL units, mesh origin, scale and orientation;
- passive brush centre and conservative envelope relative to `tool0`;
- optionally the brush CAD mesh origin, scale and pose relative to `tool0`;
- RGB-D camera optical-frame extrinsic transform relative to `base_link`.

The loader synchronously applies table/jaw collision boxes and a `tool0`-attached
brush box to MoveIt via `/apply_planning_scene`, and publishes CAD STL visual
markers for jaw and brush. **The STL markers are visual-only:** the collision
shapes are conservative boxes, not mesh-to-mesh dental geometry. Verify their
physical correspondence. MoveIt's planning scene is not a complete safety model
or a simulation of contact forces.

Once actual calibration has been verified, set `calibrated: true` and launch:

```bash
ros2 launch ./scripts/launch/mvp_fake.launch.py \
  execute_fake_motion:=false \
  load_workcell:=true \
  workcell_config:=/ABSOLUTE/PATH/TO/workcell.lab.yaml
```

The supervisor now refuses `RunCycle` until `/dental/workcell/ready` is true.
Inspect the resulting collision scene *before* enabling fake execution.

Use `base_link` consistently for target positions and scene object locations.
`move_group` may internally use `world` as its planning frame; the motion node
already computes `base_link <- tool0` FK correctly. For camera-based targets,
publish the actual **measured** base->camera optical transform via TF2; e.g.

```bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
ros2 run tf2_ros tf2_echo base_link tool0
ros2 topic echo /dental/targets/selected --once
```

Use `tf2_ros static_transform_publisher` **only if** the camera is rigidly
mounted and those extrinsics have been measured; a moving camera instead needs
live transforms. Check `ros2 run tf2_ros static_transform_publisher --help`
for your installed syntax. Never use guessed values to force TF lookups to pass.

RViz recommended: set Fixed Frame to `base_link` or `world` with valid TF;
show RobotModel, MotionPlanning / Planning Scene, MarkerArray on
`/dental/targets/markers`, Marker on `/dental/workcell/jaw_mesh`, Marker on
`/dental/workcell/brush_mesh`, and TF. Confirm the brush model follows tool0
and that target arrows point outwards from the real teeth. Watch **actual**
UR joint states, not only RViz planned-trajectory ghost animations.

## Lab readiness checklist (do not skip)

1. **Software:** build and run the fake integrated 2-pass test with CV mock,
   then with teammate's actual camera action and real image/depth data.
2. **Frames:** robot base, camera optical frame, jaw and tool0 calibrated,
   metric units verified, no mirror/axis inversion. Validate normals physically.
3. **Model:** real jaw, table, robot and passive brush measured and represented
   in the collision scene; correct URDF end-effector TCP measured and checked.
4. **Controller:** joint states current; exactly one MoveIt instance and UR
   controller manager; correct active trajectory controller; emergency stop
   and operator protective stop tested under lab procedures.
5. **Safety:** no person near robot, reduced speed, protective clearances,
   planned approach orientation, motion bounds and joint limits reviewed.
   **No motion to contact tooth using this increment.**
6. **Negative cases:** disconnect perception, remove TF, publish empty scan,
   return stale scan, cancel action, deactivate controller in fake mode,
   and verify the supervisor rejects or halts instead of using stale targets.
7. **Evidence:** capture full terminal phase log, `/joint_states` and
   `/dental/targets/selected`, and RViz scene including calibrated meshes.

### Important real-UR5e execution boundary

The verified Increment 3 `motion_node` still **rejects robot_mode=real** and the
single-command launcher **always uses fake hardware**. It would be unsafe to
circumvent those guards merely by pointing its MoveIt configuration at the
physical arm. Before real execution, the team must integrate the correct real
UR5e controller/scaling and safety state, measured brush **tip TCP** (not only
`tool0`), actual tool/camera/jaw calibration, a genuine collision scene,
operator-supervised safe pose and approach angles, and validate controlled
physical motion separately. Passive brush mounting eliminates tool-switching
control; it does **not** eliminate TCP calibration, collisions, force or contact
risks. Contact/cleaning success and nonzero treated_count must be implemented
only after physical cleaning and reinspection are validated.

## End-effector teammate handoff

`src/dental_tool_control/src/tool_control_node.cpp` remains the existing
**reject-all ChangeTool scaffold**. It is not called by this passive-brush MVP.
The teammate can later implement it using `ChangeTool.action`; they must own
real sensor/actuator feedback and tool pose calibration. No simulated tool
selection is claimed to represent actual hardware.

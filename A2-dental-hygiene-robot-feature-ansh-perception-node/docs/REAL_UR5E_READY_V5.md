# MTRN4231 dental robot — v5 lab commissioning and home-first real MVP

## Status and purpose

**Baseline preserved:** fake UR5e passed 2 guarded no-contact hover passes,
3 inspections, return-home, correctly reported residual stain. The fake C++
path is unchanged except for an optional RViz config argument; do not replace
its simulation configuration with the physical workcell template.

**Real:** no physical movement is authorized or tested by these files. The
six numbers supplied by the operator are a **TCP pose** in pendant `base`,
not joint angles. They are treated as XYZ millimetres + rotation vector radians:

`p[-0.58853, -0.13330, 0.22700, 2.221, 2.221, 0.0]`

The ROS URDF `base_link` convention differs by a 180° yaw from UR pendant
`base`. Do not negate XYZ by hand. Capture controller-reported TCP in its
`base` frame and compare directly with this requested pose.

## Installation and build away from the lab

From project root, back up local edits, then extract the complete ZIP into
this **existing** repository. It is an overlay, not a fresh Git repository.

```
cd ~/Desktop/4231/A2-dental-hygiene-robot
git status --short
cp -a src/dental_motion/src /tmp/dental_motion_backup_v5
unzip -o ~/Downloads/A2_dental_real_ready_v5.zip -d .
source scripts/source_ros.sh --underlays-only
colcon build --symlink-install --packages-select dental_motion dental_supervisor dental_bringup \
  --event-handlers console_direct+
source install/local_setup.bash
python3 scripts/tests/verify_real_home_build.py
python3 scripts/tests/preflight_real_offline.py --mode preview
```

Test that the fake MVP still passes in a separate terminal (use your previous
known-good launch command). Do not switch real execution on for an offline test.

### RViz configuration — planning scene, both CV feeds, both CAD overlays

```
source scripts/source_ros.sh
python3 scripts/rviz/prepare_dual_cv_rviz.py \
  --base-config "$(ros2 pkg prefix ur_moveit_config)/share/ur_moveit_config/rviz/view_robot.rviz" \
  --output scripts/rviz/dental_real_v5.rviz
```

The resulting RViz contains separate **Image** displays:
`/dental/perception/overhead_preview` and
`/dental/perception/rgbd_preview`, plus marker overlays
`/dental/workcell/jaw_mesh`, `/dental/workcell/brush_mesh`. These are
not live until the cameras publish them and the workcell scene is loaded.
Ensure RobotModel, TF, MotionPlanning scene collision objects remain enabled.

## Lab-day order — never skip these gates

### A. Real driver and calibration file (physical execution disabled)

Extract robot-specific kinematics according to UR's driver documentation.
The SAME YAML must be supplied to the driver and our real MoveIt launch.
Start UR driver in its own terminal, using its supported arguments:

```
ros2 launch ur_robot_driver ur_control.launch.py --show-args
# Example only: replace BOTH values, confirm driver version's argument names
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e \
  robot_ip:=<REAL_ROBOT_IP> kinematics_params_file:=/absolute/ur5e_calibration.yaml \
  launch_rviz:=false
```

Verify controller state with `ros2 control list_controllers` and check the
**real** hardware, joint states and robot mode. External Control URCap must
be running. The MoveIt config expects a real trajectory controller (normally
`scaled_joint_trajectory_controller`); match the active controller, don't
blindly activate conflicting controllers. Never run fake and real drivers
in the same ROS domain or as simultaneous competing publishers.

### B. One-time pendant home capture — extra step required

Using the pendant, safely move the physical robot to the intended TCP home,
confirm the selected feature is **Base**, the active TCP is correct, and
there is physical clearance. Activate the controller's read-only
`tcp_pose_broadcaster` if installed so it publishes `base -> tool0_controller`.

```
python3 scripts/tests/capture_home_reference.py \
  --requested calibration/home.requested.yaml \
  --tcp-frame tool0_controller \
  --output calibration/home.capture.yaml
```

Examine the reported position/orientation discrepancies and captured six
joint angles. After verifying robot-specific FK, mounting, clearance and
active TCP, convert that capture into an approved *joint target*:

```
python3 scripts/tests/commission_home_from_capture.py \
  --capture calibration/home.capture.yaml \
  --output calibration/home.approved.yaml \
  --confirmed-pendant-base --confirmed-active-tcp \
  --verified-robot-kinematics --verified-clearance
```

The converter checks the live controller TCP agrees with the requested pose
within **8 mm / 5°** and verifies all six joint names/values. These are
software acceptance bounds, not mechanical safety proof; tighten with
real measurement results. It cannot generate measured joints from the
requested Cartesian pose while away from the robot. Do not fabricate a
joint solution.

### C. Cameras and workcell

1. Edit `calibration/cameras.real.yaml`: real `base_link -> overhead_cam_optical`
   extrinsic and `tool0 -> rgbd_color_optical_frame` hand-eye transform,
   including quaternion in ROS optical conventions; set `calibrated:true`
   only after verified (avoid the sample fake static TF).
2. Calibrate/check overhead camera intrinsics, IDs/marker size and
   `marker_layout_calibrated:true` in
   `src/dental_perception/config/perception.real.yaml`. Verify measured
   camera intrinsics used match the actual camera resolution. The default
   overhead matrix is historical reference data, not proof of calibration.
3. Configure the RealSense wrist RGB-D serial, check aligned depth, metric
   projection and stable dark-region detections. There must be no offline
   findings or synthetic TF on the real robot.
4. Edit `calibration/workcell.real.yaml`: measured table, jaw and **complete
   mounted tool** collision boxes, conservative padding, optional jaw and
   brush STL overlays with actual scale/origins; set `calibrated:true` after
   visually checking physical registration. Optional course wall and ceiling
   planes remain **disabled** until their placement and frame are verified;
   those source numbers were from a UR10e example, not ground truth.
5. Brush tip offset on launch must be measured relative to **tool0**, not
   blindly taken from active pendant TCP. The jaw mesh is visual only;
   MoveIt collision primitives are separately validated by read-back.

### D. Preview / RViz with real motion impossible

No supervisor is started and the station/treatment actions cannot execute
physical trajectories when defaults are used:

```
python3 scripts/tests/preflight_real_offline.py --mode preview
source scripts/source_ros.sh
ros2 launch scripts/launch/mvp_real.launch.py \
  launch_rviz:=true execute_real_motion:=false enable_real_cycle:=false \
  load_workcell:=true \
  workcell_config:=$(pwd)/calibration/workcell.real.yaml \
  cameras_config:=$(pwd)/calibration/cameras.real.yaml \
  rviz_config_file:=$(pwd)/scripts/rviz/dental_real_v5.rviz
```

**During initial camera calibration,** `workcell.real.yaml` is uncalibrated:
use `load_workcell:=false` instead. Once measured, use true to see its
collision boxes and CAD overlays. The real driver is started **separately**.

```
python3 scripts/tests/check_dual_cv_previews.py --timeout 20
ros2 run tf2_ros tf2_echo base_link overhead_cam_optical
ros2 run tf2_ros tf2_echo tool0 rgbd_color_optical_frame
ros2 topic echo /dental/perception/model_localized --once
```

A CV image topic appearing is not proof that the inspection action returns
valid metric tooth positions. Verify action, depth, timestamps and TF.

### E. Static execution gate

```
python3 scripts/tests/preflight_real_offline.py --mode execution \
  --home calibration/home.approved.yaml \
  --cameras calibration/cameras.real.yaml \
  --workcell calibration/workcell.real.yaml \
  --perception src/dental_perception/config/perception.real.yaml \
  --kinematics /absolute/ur5e_calibration.yaml
```

This does not authorize motion; it checks commissioning files only. Validate
physical stop buttons, speed sliders, safety planes, control mode, robot
calibration, real home-to-inspect reachability and smoothness **in planning**
with the operator before real execution. The first real supervised action
is `MoveToStation(home)`. Its joint target uses the approved measured
configuration and its path is fully collision-checked. The station then
verifies endpoint joints before publishing `real_home_ready=true`.
`TreatTarget` itself checks that latch (so bypassing the supervisor with a
direct action request does not bypass home-first validation).

### F. Explicit, supervised NO-CONTACT motion only after lab approval

**Not to run unattended.** Confirm each physical condition and observe a
planning preview before running this with a demonstrator and E-stop ready:

```
ros2 launch scripts/launch/mvp_real.launch.py \
  robot_kinematics_file:=/absolute/ur5e_calibration.yaml \
  home_config:=$(pwd)/calibration/home.approved.yaml \
  cameras_config:=$(pwd)/calibration/cameras.real.yaml \
  workcell_config:=$(pwd)/calibration/workcell.real.yaml \
  perception_config:=$(pwd)/src/dental_perception/config/perception.real.yaml \
  rviz_config_file:=$(pwd)/scripts/rviz/dental_real_v5.rviz \
  load_workcell:=true \
  brush_tip_offset_x_m:=<MEASURED_X> \
  brush_tip_offset_y_m:=<MEASURED_Y> \
  brush_tip_offset_z_m:=<MEASURED_Z> \
  confirm_real_calibration:=true confirm_real_workcell:=true \
  execute_real_motion:=true enable_real_cycle:=true
```

Then from a second terminal run the read-only `preflight_real_live.py --require-scene`, then the supervised `run_mvp_cycle.py` action
client (after first verifying action servers, TF, workcell and real controller).
It uses NO-CLEANING, NO-CONTACT motion with a passive brush identifier;
residual staining is not proof of tool performance. Supervisor must require
`MOVING_TO_home -> AT_home -> MOVING_TO_inspect` before any inspection.
If no collision-free path to commissioned home exists from the live current
joint state, the run **must fail safely** and the operator must reposition
manually using approved procedures. Home-first is not a guarantee that an
arbitrary unknown robot position is safe to traverse automatically.

**Further validation needed:** This ZIP has Python/static checks but was not
C++-compiled in ROS Humble nor tested on a real UR5e. Do not treat the
implementation as a certified safety controller.

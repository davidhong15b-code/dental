# MTRN4231 — unified dental MVP: simulation and lab commissioning

**Status:** implemented source; Python and offline CV checks passed; **ROS 2 Humble C++ build and hardware execution have NOT been run in this environment**. Never call this a validated robot-motion pipeline until the lab checks pass. The demo should run with *no contact*, using documented, measured camera and brush geometry. Do not change physical controller or robot safety configuration without demonstrator approval.

## Architecture and action flow

```
[Overhead USB V4L2] -> model_localizer_node -> dynamic dental_model TF -> tooth_1..4 static TF
[Wrist RealSense USB SDK] -> perception_node (InspectTeeth) -> DentalFindingArray
 -> target_manager_node (camera frame -> base_link, MarkerArray visualization)
 -> supervisor_node (RunCycle):
       MoveToStation(inspect) [records initial home joints]
       InspectTeeth -> target_manager -> TreatTarget(brush)
       MoveToStation(inspect) -> InspectTeeth -> possible TreatTarget (2nd pass)
       MoveToStation(inspect) -> InspectTeeth (3rd scan)
       MoveToStation(home) -> final result
 -> motion_node: planned bounded pre-approach, hover, lateral strokes, retract
 -> station_motion_node: model-relative inspection, return to recorded starting joints
```

**Outcome semantics:** Return-home after a failed treatment stage is NOT automatically attempted (a controller fault might make recovery hazardous). A persistent physical black mark is expected after non-contact passes; it triggers a second pass, then ends with `Residual stain ... returned home` and `success=false` (meaning not cleaned). `treated_count=0` intentionally, because hovering cannot prove cleaning. A clear image after a hover may finish successfully but still does not prove the motion removed a stain. The simulation-only switch `mock_clear_after:=1` fakes a cleared image on inspection 2; do not confuse this with physical cleaning.

## What comes from the linked calibration repo

Reference: https://github.com/phantom16361/ur5e-camera-calib/blob/main/reference/REFERENCE.md (26 Oct 2026)

- LifeCam Studio 1920×1080: fx=1388.8, fy=1382.2, cx=919.3, cy=574.4, distortion [-0.0033,-0.0237,-0.0005,-0.0070,0.0]; lock focus/exposure as in `setup_overhead_cam.sh`.
- Wrist `tool0 -> camera_link`: xyz [-0.01615,-0.09938,0.02493], xyzw [-0.499904,0.502214,-0.500659,-0.497209]. During calibration the RealSense ROS wrapper supplies the internal optical TF.
- Wrist `tool0 -> camera_color_optical_frame`: xyz [-0.03113,-0.09924,0.02460], xyzw [-0.002221,-0.002155,-0.003637,0.999989]. Used directly in normal perception **only after verifying identical physical mounting**.
- Reference overhead `base_link -> overhead_cam_optical` xyz [1.26842,0.06553,0.85374], xyzw [0.660371,0.680313,-0.223169,-0.226450] is **from someone else's swivel station and MUST NOT be used to move the real robot**. File `calibration/reference/REFERENCE_PREVIEW_ONLY.yaml` labels it as reference-only. Our 60 mm markers replace their 56 mm markers.
- No checkerboard or ArUco grid needed for C2 when hand-eye is already valid. Actual script `overhead_calib.py` takes `--dict DICT_ARUCO_ORIGINAL --size 0.060 --ids 0,1,2,3 --station dental_bench`; capture `c`, solve `o`, save `s`.

## Tonight: simulation mode

The fake workcell is deliberately synthetic; RGB stain presence is obtained via 2D CV on your actual `wrist_01.png`, but its 3D tooth centre is a **synthetic TF**. It does not claim spatial localization of the photograph.

```bash
cd ~/Desktop/4231/A2-dental-hygiene-robot
source scripts/source_ros.sh --underlays-only
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to \
  dental_interfaces dental_perception dental_motion dental_supervisor \
  dental_target_manager dental_bringup
source install/local_setup.bash
ros2 launch scripts/launch/mvp_fake.launch.py \
  execute_fake_motion:=true confirm_fake_hardware:=true
```

Ensure *only fake* UR driver is running; do not run this alongside the actual UR driver. If the fake controller is not active, do not send the cycle goal. The fake workcell exists at a virtual `[0.62,0.12,0.02]` metre model origin and is loaded only after `--allow-simulation`. `mock_clear_after:=-1` means the image still shows the mark after both hover sweeps. To demonstrate a simulated clear state, use `mock_clear_after:=1` and clearly label it as a scripted scene change.

After the terminal says MoveIt and the workcell are ready:

```bash
source scripts/source_ros.sh
ros2 control list_controllers
ros2 run tf2_ros tf2_echo base_link tooth_1
python3 scripts/tests/run_mvp_cycle.py --timeout 600
```

Track `[SUPERVISOR] MOVING_TO_inspect`, `[STATION] PLANNING_inspect`, `INSPECTING`, `SCAN_COMPLETE`, `MOTION_PLANNING_SWEEP_A`, `EXECUTED_RETRACT`, `AT_inspect`, `MOVING_TO_home`. A missing/unsafe IK, scene collision, excessive travel or controller fault must abort. The default fake initial joint posture may need to be set to a reachable bent-elbow start at the demonstrator's direction; **the code does not invent or override initial joint angles**.

In RViz set `Fixed Frame=base_link` and add `TF`, `RobotModel`, `MarkerArray` `/dental/targets/markers`, and two Image displays `/dental/perception/overhead_preview` and `/dental/perception/rgbd_preview`. MoveIt planning-scene display shows the simulated table, jaw and attached brush collision envelopes. There is no fabricated video of physical stain removal.

## Tomorrow: real UR5e, cameras and C2 calibration

1. With robot movement disabled, confirm physical emergency stop, clear workcell, brush/tool attachment, and active controller. Start the real UR driver using the lab robot IP and its extracted kinematics. Verify `/joint_states` ~500 Hz and `tf2_echo base_link tool0`.
2. Stop `dental_perception` before camera calibration (the devices cannot be opened twice). Start temporary RealSense ROS camera wrapper (`ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true`) and overhead `./setup_overhead_cam.sh --launch` from the `ur5e-camera-calib` repository.
3. Publish `tool0 -> camera_link` reference hand-eye TF **only if same bracket**. Verify `tf2_echo base_link camera_color_optical_frame` before captures.
4. In the calibration tool directory:

```bash
python3 overhead_calib.py --dict DICT_ARUCO_ORIGINAL \
  --size 0.060 --ids 0,1,2,3 --station dental_bench
```

With fixture stationary, use approved freedrive to take 2–3 stationary captures per ID (`c`), move arm clear, solve (`o`), check RMS and leave-one-out errors, then save (`s`). Do not use the other team's C2 static transform. Keep `overhead_dental_bench.yaml` and captures JSON.

5. Measure four ArUco **centres and decoded yaw angles in the jaw CAD frame**, including heights and plate origin alignment. Edit `calibration/marker_layout.template.yaml` and set `physically_measured: true`, then import:

```bash
python3 scripts/cameras/prepare_demo_calibration.py \
  --overhead ~/Desktop/4231/ur5e-camera-calib/overhead_dental_bench.yaml \
  --layout calibration/marker_layout.template.yaml \
  --output-dir calibration/session
```

The script checks C2 parent/child frame, quaternion, RMS (<4 px), four LOO errors (<10 mm), true 60 mm square and marker separation. These are **preliminary screening thresholds**, not sufficient evidence for tooth-level accuracy. Verify the physical tooth frames from multiple fixture positions, compare projected tooth centres with the actual tooth top within a few millimetres, and use the workcell's operator signoff procedure.

6. Stop `usb_cam`, RealSense ROS wrapper, and `tool0->camera_link` publisher. Normal C++ camera nodes now own the USB cameras. Keep the *new* calibrated `base_link->overhead_cam_optical` transform from `calibration/session/overhead_tf.sh`; the real launch publishes the direct `tool0 -> rgbd_color_optical_frame` reference transform, only if mounting verified. Never publish two parents for the same child frame. A C2 script frame named differently from `overhead_cam_optical` must be corrected at the source before importing.
7. Complete a **MEASURED** real workcell file from `src/dental_bringup/config/workcell.template.yaml`: table top, jaw collision envelope at its actual robot-base pose, and mounted brush centre/dimensions relative to `tool0`. Measure brush tip offset from `tool0` accurately and transfer to the real launch parameters. The collision scene must be refreshed whenever the jaw fixture is moved.
8. Check `ros2 control list_controllers` and confirm the controller MoveIt is configured to use (real launch prefers `scaled_joint_trajectory_controller`) is actually active. Ensure `robot_kinematics_file` matches the real UR driver. **Initially use plan-only**:

```bash
source scripts/source_ros.sh
ros2 launch scripts/launch/mvp_real.launch.py \
  robot_kinematics_file:=$HOME/ur5e_kinematics.yaml \
  perception_config:=$PWD/calibration/session/perception.real.yaml \
  workcell_config:=$PWD/src/dental_bringup/config/workcell.template.yaml
```

This launch is intentionally blocked from real execution until the measured workcell and brush tip are configured, independent camera/model/frame cross-checks pass, and a demonstrator approves the non-contact trajectory. An accepted real-motion run requires `execute_real_motion:=true confirm_real_calibration:=true confirm_real_workcell:=true`, plus **measured nonzero** `brush_tip_offset_x_m/y_m/z_m` and a workcell scene that publishes `/dental/workcell/ready: true`. Never turn those switches on to silence a missing calibration or collision error.

9. With calibration validated and safety approval, run the same `python3 scripts/tests/run_mvp_cycle.py --timeout 600`. For physical black marks under a no-contact stroke, the expected final outcome is **two hover passes then residual stain reported after returning home**.

## Required physical measurements (cannot be inferred from the reference repo)

- **Fixed CAD geometry:** marker centres x,y,z and yaw for IDs 0–3 relative to `dental_model` origin; mounting must be rigid. ArUco square =60 mm; outer card=80 mm. Confirm printed-tooth surfaces and top Z=31 mm nominal from `Jaw_model.stl`.
- **Wrist mount:** confirm exact RealSense model, camera serial and unchanged camera bracket/tool0 coordinate; measure the brush tip offset and collision envelope for the attached tool. Same optical frame convention does **not** imply the same hand-eye extrinsic.
- **Workbench and robot:** table upper surface and dimensions in `base_link`, measured jaw placement/orientation, obstacle envelope, actual home joint posture. UR MoveIt must use the robot's same kinematic calibration and active controller. The model must not move after the workcell scene is applied unless that scene is updated.
- **Overhead:** confirm 1920×1080 LifeCam settings and focus, stable `/dev/dental_overhead` symlink, generated C2 pose and precision. The other team's C2 pose is not valid for your bench.

## Remaining caveats / integration test needed

- No ROS 2 Humble headers/driver/controller exist in the authoring environment, so C++ build and actual robot paths are **not yet compiled or motion-validated**. Treat these changes as a candidate implementation and resolve any compiler output before the demo.
- Positioning fallback to CAD-plane projection is disabled by default. Only enable `allow_cad_plane_fallback: true` after verifying measured model-to-optical TF and flat tooth-top planes. The result is a ray-plane intersection, **not measured depth**. Contact is forbidden.
- Current coordinator bounds *two passes total for the selected blemish*, not two per tooth if several teeth are marked. Multi-target prioritisation would require an expanded per-tooth ledger. Your one-mark MVP fits this contract.
- The demonstration does not perform brushing, remove black marks or independently confirm any cleaning. It demonstrates recognition, bounded hover motion and verification. Persistent mark triggers residual-failure result, as designed.

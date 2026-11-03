# MTRN4231 dental MVP — Real UR5e commissioning: stage 1

**Purpose:** capture and verify inputs for a safe real-home-first workflow and show both annotated CV streams in RViz. No file in this stage commands motion or enables real motion. The previously verified two-pass fake cycle is unchanged.

## 1. Resolve the requested HOME representation

Requested six values are provisionally interpreted as **UR TCP pose** (millimetres, axis-angle radians):

```
p[-0.58853, -0.13330, 0.22700, 2.221, 2.221, 0.0]
```

NOT as six joint angles. **Critical:** the pendant `Base` is the ROS `base` frame, not `base_link`; the two share an origin but differ by 180 degrees about Z. Thus requested X/Y in ROS `base_link` are approximately **+0.58853/+0.13330 m**, near the fake model at +0.620/+0.120 m. Use TF for full pose conversion including rotation; do not just negate X/Y in C++. Base feature and **active TCP name** must be checked on the *actual robot pendant*. A UR TCP pose does not select one unique 6-joint configuration, and `tool0` is not necessarily the active TCP. The tool geometry/offset must be measured independently. The conversion for its rotation quaternion is approximately `[0.70710675, 0.70710675, 0, 0.00031217]` (xyzw) but MUST NOT be used to execute until calibrated.

For a deterministic home move on real hardware, first have the operator jog/position the robot safely to the intended home via the pendant at low speed with correct TCP and Base frame, and record the actual joints and `base_link -> tool0` geometry. Do not attempt ROS automatic homing before that. Record using:

```
cd ~/Desktop/4231/A2-dental-hygiene-robot
source scripts/source_ros.sh
python3 scripts/tests/capture_home_reference.py \
  --requested calibration/home.requested.yaml \
  --output calibration/home.capture.yaml
```

The script only listens to `/joint_states` and ROS `base_link -> tool0` TF, and keeps the requested pose in **pendant `base`**. It needs at least six recent stationary joint-state observations, and refuses to overwrite an existing capture. If the real driver exposes a **live active TCP** frame such as `tool0_controller` through `tcp_pose_broadcaster`, use `--tcp-frame tool0_controller` to compare the requested pose in **`base`** coordinates directly against controller measurements. Confirm that broadcaster and configured active TCP are correct; do not use a fake/static frame. **Do not specify `tool0` unless the UR pendant's active TCP is really tool0.**

The output is marked `UNAPPROVED_CAPTURE_ONLY` and all approvals remain false. A matching pose requires confirmation of actual pendant readout, Base frame, active TCP, robot-specific calibration file, joint limits, jaw/table/brush geometry, and a collision-free route from the real starting configuration.

## 2. Separate the robot home from the cycle-start joints

Current MVP station code records the joint configuration at the first inspection request as *cycle home*. It **does not implement** the above requested real home. Do not enable `execute_real_motion` on the existing station node on that assumption.

Stage 2 motion change, AFTER approving the captured home, will:

1. introduce a verified six-joint home target loaded from commissioned YAML, plus an independent FK/active-TCP consistency check;
2. require `MoveToStation(home)` as the **first** supervisor action, *before* inspection, and again at the cycle end;
3. block all cycle motion if no approved home exists or no valid collision-checked path from the current measured joints exists;
4. use a fixed named home joint branch, with path-cost/joint-delta gates and actual-state checks; keep `PLAN_ONLY` default for physical hardware;
5. separate final-approach linear Cartesian planning from general no-contact transfers, after calibration.

**Important:** "always move home first" does not mean forcing motion through a collision. Infeasible or unsafe start-to-home motion must abort for operator intervention.

## 3. Real planning scene and RViz geometry

Existing `src/dental_bringup/config/workcell.template.yaml` already blocks real execution with `calibrated: false` and null dimensions. Use `calibration/workcell.real.measurements.template.yaml` as an additional measurement checklist, not a runnable real workcell. Measure in the **base_link** frame:

- tabletop box center and dimensions, plus other physically relevant fixtures/obstacles;
- jaw collision envelope (allow sensible safety clearance) and actual STL origin/orientation/scale;
- complete mounted end-effector collision envelope and `tool0` extrinsic, not just the brush bristles;
- camera mounts/extrinsics, checked against the true robot model and measured joint state.

Existing STL dimensions in the attached repository (raw mesh coordinates):

| STL | Raw extents | Expected metres at scale 0.001 (if STL authored in mm) |
|---|---|---|
| `docs/Jaw_model.stl` | 140 x 100 x 31 | 0.140 x 0.100 x 0.031 |
| `docs/EE_4_Brush_tool.stl` | 36 x 36 x 60 | 0.036 x 0.036 x 0.060 |

The **fake brush collision proxy** is only 0.020 x 0.020 x 0.030 m; it is not a safe real-robot collision model for the entire 36 x 36 x 60 mm CAD brush, let alone the rest of the end effector. The mesh's CAD origin is also not automatically at the actual tool0 origin. Rebuild the real attached collision geometry from measurements; do not just copy its fake `touch_links`.

`apply_workcell.py` supports `jaw.visual_stl` and `brush.visual_stl` marker overlays with origin, quaternion and scale. These are **visual only**; MoveIt collision checking uses separately defined measured primitives. Make `visual_stl` an **absolute file path** when populating this template, because relative paths can resolve differently when launched from an installed package.

For accurate workcell display in RViz: use Fixed Frame `base_link` (or confirmed `world` connected to base_link); enable `RobotModel`, `TF`, `MotionPlanning` scene geometry; add two `Marker` displays on `/dental/workcell/jaw_mesh` and `/dental/workcell/brush_mesh` after their calibrated visualization fields are configured. Confirm the collision boxes align with, and conservatively bound, the measured CAD and hardware. Only a photograph alone cannot establish 3D registration.

**Do not copy the UR10e example collision-plane coordinates from the course guide.** Add actual lab table/walls/stands, and revalidate camera visibility and motion reachability.

## 4. Dual live camera/CV previews in RViz

The existing C++ nodes already publish annotated **sensor_msgs/Image** streams:

- overhead ArUco tracking: `/dental/perception/overhead_preview`;
- wrist RGB-D/dark-mark tracking: `/dental/perception/rgbd_preview`;
- additional wrist debug output: `/dental/perception/debug`;
- model tracking state: `/dental/perception/model_localized`.

After bringing up physical cameras (only **one** process should own each USB camera), verify:

```
source scripts/source_ros.sh
python3 scripts/tests/check_dual_cv_previews.py --timeout 20
```

Add **two separate `rviz_default_plugins/Image` displays** to your normal MoveIt RViz: one for each `*_preview` topic. Use **Image**, not **Camera**, because these existing CV preview topics currently have no paired CameraInfo publisher under matching image-topic namespaces. Keep the MotionPlanning and TF displays visible. If the two camera nodes publish no preview, the Image panels cannot show live video: check USB enumeration, `rgbd_serial`, camera input mode, V4L2 resolution, librealsense SDK support and active image topic names before continuing.

The fake MVP with `use_mock_inspection:=true` uses a stored photo inside an action server, not the live C++ camera nodes. For live previews, launch the *real perception nodes* using the matched calibrated config, without competing mock InspectTeeth action servers; leave robot execution disabled during this stage.

Important mismatch to reconcile: `perception.real.yaml` expects the overhead optical frame `overhead_cam_optical`, while some earlier fake configs use `overhead_color_optical_frame`. The real camera TF must be measured and named consistently with the live image headers, depth alignment, and calibrated hand-eye transform. Do not publish a static transform simply to make an RViz error disappear.

The readiness script only verifies both preview streams have fresh timestamps and pixels; pass `--require-localization` once the marker layout and extrinsics are actually calibrated.

## 5. Reference guidance

The uploaded 2025 *MoveIt! Tips and Tricks* guide recommends realistic collision planes, considering near-singular configurations, appropriate planners and caches for repeated movements. The current logs already show RRTConnect and a working two-pass fake sequence. Do not replace the planner merely for novelty; instead design and validate dedicated motion modes (transfer vs short controlled hover) and persistent approved home/inspect configurations. The guide's real-world caution that joint targets alone do not make stochastic planning paths repeatable is pertinent.

## 6. Exit criteria for stage 1

- [ ] Pendant confirms requested pose is in Base frame, with correct active TCP.
- [ ] Stable joint-angle snapshot obtained while robot is **already** safely at home.
- [ ] `base_link -> tool0` and `base -> tool0_controller` are consistent with robot-specific UR calibration, the active pendant TCP and physical pose.
- [ ] Real table/jaw/complete end-effector geometry measured and visually aligned in RViz.
- [ ] Real camera IDs/intrinsics/TF names verified, both annotated previews fresh.
- [ ] `robot_mode=real` plan-only workflow validated, scaled controller inspected, real execution remains disabled.

When these checks pass, integrate the commissioned home as a first-action, fail-closed C++ station target. No file in this stage is proof of safety or permission to execute a physical robot.

## 7. Optional launch argument for a persistent dual-image RViz layout

`OPTIONAL_rviz_config_arg.patch` adds one `rviz_config_file` launch argument to existing fake MoveIt, fake MVP and real MVP launch files. The default is **unchanged** (`ur_moveit_config/rviz/view_robot.rviz`). It does not change the real or fake motion code and has no controller effects. Apply only if `git apply --check` succeeds:

```
cd ~/Desktop/4231/A2-dental-hygiene-robot
git apply --check OPTIONAL_rviz_config_arg.patch
git apply OPTIONAL_rviz_config_arg.patch
```

Make a dental RViz config without replacing the original MoveIt layout:

```
source scripts/source_ros.sh
python3 scripts/rviz/prepare_dual_cv_rviz.py \
  --base-config "$(ros2 pkg prefix ur_moveit_config)/share/ur_moveit_config/rviz/view_robot.rviz" \
  --output scripts/rviz/dental_full.rviz
```

Add to an **existing validated** fake or real MVP launch invocation:

```
rviz_config_file:=$(pwd)/scripts/rviz/dental_full.rviz
```

The two Image displays open with the full MoveIt view. Adjust the dock panels in RViz and save the config after a dry run. The dual previews are annotated 2D RGB views; they are NOT a 3D depth point cloud. Future geometric wrist depth visualization requires a registered depth/point-cloud publisher and camera calibration.

For first camera-only testing without starting any robot-motion nodes:

```
source scripts/source_ros.sh
ros2 launch dental_perception perception.launch.py \
  config:=$(pwd)/src/dental_perception/config/perception.real.yaml
```

This works only if both camera backends are actually available: V4L2 for the overhead camera and librealsense2 for the wrist RGB-D camera, with correct USB device path/serial and negotiated resolutions. The file currently disables calibrated model pose and wrist metric projection; preview-only operation does not imply permission to execute robot motion.

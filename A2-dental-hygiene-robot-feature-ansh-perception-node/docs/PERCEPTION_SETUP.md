# Dental perception — camera and image modes

This is the standard `dental_perception` package for the MTRN4231 dental hygiene robot.
**No separate ROS camera publisher is required in camera mode.** The nodes open the
USB devices in C++ and publish only ROS TF, action results and diagnostic previews.

## 1. Folder map (the files you need)

| File | Role | Normally edited? |
|---|---|---|
| `src/dental_perception/config/perception.yaml` | **ALL settings and measured lab geometry** | **YES** |
| `src/dental_perception/launch/perception.launch.py` | Start both C++ nodes | No |
| `src/dental_perception/src/model_localizer_node.cpp` | Overhead USB/image acquisition, IDs 0–3, pose and 4 tooth TFs | No |
| `src/dental_perception/src/perception_node.cpp` | RGB-D USB/image acquisition, 2D preview, registered-depth stains, `InspectTeeth` | No |
| `src/dental_perception/samples/` | Two overhead and two wrist RGB example images | Replace/add more files |
| `scripts/tests/test_perception_samples.py` | Offline 2D computer-vision smoke test | No |
| `scripts/cameras/inspect_usb_cameras.sh` | Discover video devices | No |
| `scripts/cameras/setup_overhead_udev.sh` | One-time stable `/dev/dental_overhead` symlink | No |
| `scripts/cameras/calibrate_overhead.py` | Camera intrinsics calibration with checkerboard | No |

**Only `perception.yaml` normally needs lab-specific edits.** Do not put measured
marker sizes/coordinates inside C++ constants.

## 2. First use: try your supplied images (no cameras required)

This configuration is already the default:

```yaml
model_localizer:
  ros__parameters:
    overhead_input_mode: image
    overhead_images: [samples/overhead_01.png, samples/overhead_02.png]
perception:
  ros__parameters:
    rgbd_input_mode: image
    rgbd_images: [samples/wrist_01.png, samples/wrist_02.png]
    # rgbd_depth_images: [samples/wrist_depth_01.png, samples/wrist_depth_02.png]
```

In ROS 2 Humble, omit `rgbd_depth_images` when no registered depth files are available; an empty YAML array (`[]`) can cause parameter declaration to fail. The C++ node uses an empty list by default.

The real settings file has additional parameters; preserve them. Paths can be
package-relative, as above, or absolute paths such as `/home/user/photos/scan.png`.
The nodes alternate through the listed images every second.

Build on Ubuntu 22.04 / ROS 2 Humble:

```bash
cd ~/Desktop/4231/A2-dental-hygiene-robot
source scripts/source_ros.sh --underlays-only
sudo apt update
sudo apt install -y libopencv-dev libopencv-contrib-dev \
  ros-humble-cv-bridge v4l-utils python3-opencv python3-numpy
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to dental_perception
source install/local_setup.bash
ros2 launch dental_perception perception.launch.py
```

**Note**: the bundled photos are RGB only. Marker IDs and a **2D stain preview**
work immediately. No calibrated depth, jaw-to-robot pose or metric 3D findings
will be fabricated from those photos. A `/dental/perception/inspect` action
request will fail clearly in RGB-only image mode, by design.

For a quick independent check, which does not need ROS:

```bash
python3 scripts/tests/test_perception_samples.py --output-dir /tmp/dental_previews
```

The test verifies all four sample images against `DICT_ARUCO_ORIGINAL` and
reports dark pixels inside a marker-aligned jaw crop. This is **not** a clinical
stain classifier; lighting and print artifacts can cause false positives.

To view running C++ node results (open RViz2 and add two Image displays):

- `/dental/perception/overhead_preview`: detected IDs outlined on overhead image
- `/dental/perception/rgbd_preview`: 2D dark candidates circled on sample RGB

Or inspect numeric IDs:

```bash
ros2 topic echo /dental/perception/marker_ids --once
ros2 topic echo /dental/perception/model_localized --once
```

Expected IDs: 0, 1, 2, 3 (order may vary). `model_localized: false` is **correct**
with uncalibrated sample images; it does not mean the markers were missed.

## 3. Lab: plug in both cameras

The two camera switches are independent. Change in **one** file
`src/dental_perception/config/perception.yaml`:

```yaml
model_localizer:
  ros__parameters:
    overhead_input_mode: camera
perception:
  ros__parameters:
    rgbd_input_mode: camera
```

Any combination is valid: `image/image`, `camera/image`, `image/camera`,
`camera/camera`. Both live camera processes run inside the perception C++ nodes.
An advanced legacy `ros` mode is also supported if an external camera driver is
later needed, but is not the standard setup.

### 3.1 Overhead USB (UVC/V4L2)

1. Plug in both cameras.
2. `bash scripts/cameras/inspect_usb_cameras.sh`.
3. Identify the **correct overhead capture endpoint** (e.g. `/dev/video2`).
4. Run `bash scripts/cameras/setup_overhead_udev.sh /dev/video2`. It prints the
   exact rule and asks for confirmation before installing. Camera serial is
   preferred; otherwise it uses the physical USB port path.
5. Confirm `ls -l /dev/dental_overhead`. Keep the camera in the same port if
   the rule was port-bound.

The overhead C++ code opens `/dev/dental_overhead` using OpenCV's V4L2 backend.
It checks that negotiated resolution matches the configured calibration.

### 3.2 Wrist RGB-D camera

The current `camera` backend supports an **Intel RealSense** device using its
C++ `librealsense2` SDK. A USB camera's `videoN` endpoint is NOT sufficient for
registered, metric depth unless the vendor SDK supplies aligned depth and
intrinsics. Install the SDK and its device permission rules following the vendor
instructions, verify with `rs-enumerate-devices`, and rebuild `dental_perception`.
Set `rgbd_serial` if more than one RealSense is connected.

If your lab depth camera is **not** a RealSense, the C++ SDK adapter must be
changed for that model. Do not assume RealSense will work for an unknown RGB-D camera.

See `docs/PERCEPTION_MODEL_TF_LAB.md` to calibrate and publish camera-relative dental/tooth TF with the UR5e switched off.

## 4. **LAB MEASUREMENT CHECKLIST — edit these before 3D robot operation**

All numerical values go in `src/dental_perception/config/perception.yaml`.
Fields are explicitly marked `LAB MEASUREMENT` or `LAB CALIBRATION`.

1. **Overhead camera intrinsics**: at the exact selected capture resolution,
   checkerboard-calibrate `overhead_camera_matrix` (fx/fy/cx/cy) and
   `overhead_distortion`. Use `scripts/cameras/calibrate_overhead.py` and set
   `overhead_intrinsics_calibrated: true` only after calibration passes.
2. **Printed fiducials**: measure side length of the actual square black marker
   (not the paper/card boundary) in metres: `marker_size_m`.
3. **Marker centres**: measure each ID 0,1,2,3 in the *jaw STL coordinate system*
   with all axes in metres: `marker_centres_xyz_m`. The marker cards shown in
   your photos are outside the printed plate; do NOT use the STL bounding-box
   corners as their positions without measurement.
4. **Marker rotations**: measure marker local top relative to jaw X/Y axes,
   in radians `marker_yaw_rad`. Visually similar marker cards can have distinct
   rotations. **ID 0 is not automatically the world origin.**
5. **Calibrated overhead camera extrinsics**: a valid TF from robot `base_link`
   to `overhead_color_optical_frame` (not provided by udev). Supply this from
   a measured fixed-camera-to-robot calibration.
6. **Mounted wrist camera extrinsics**: a valid TF from robot `tool0` to
   `rgbd_color_optical_frame` from measured hand-eye/calibrated mount position.
7. **Printed model**: STL-derived tooth top frames are currently at CAD X
   `[-21,-7,+7,+21] mm`, Y `0`, Z `+31 mm`. Verify actual print, alignment,
   model origin and tooth surface extents before motion.
8. **Real lighting / RGB-D**: tune depth limits, tooth region sizes,
   reprojection error and `dark_threshold` only from actual lab data.
9. **RGB-D SDK**: confirm model, supported stream profiles, calibration, and
   depth-to-RGB registration. Don't substitute raw arbitrary USB video for depth.

After (1)-(4), set `marker_layout_calibrated: true`. Lab measurements and
extrinsics must be validated against fixed points before enabling any motion.

## 5. 3D offline images (optional)

To exercise the existing 3D `InspectTeeth` action from saved images, supply **true
registered depth** for each RGB image, at the same resolution/order, plus the
recorded RGB intrinsics. 16-bit PNG stores millimetres; use a supported
OpenCV-readable 32-bit floating-point format for metres (e.g. TIFF). Set
`rgbd_image_intrinsics_calibrated: true` after measuring the intrinsics.

`allow_offline_findings` defaults to `false`; keep it **false** for any real
robot workflow. This option is only for a separately isolated simulation
with valid TF and complete RGB-D images. Even if enabled, the 3D analyzer
requires measured tooth TF, measured tooth depth and calibrated model pose.

## 6. Existing pipeline and limitations

These nodes maintain the existing interfaces:

- `model_localizer_node` → `dental_model` + `tooth_1`..`tooth_4` TF and `model_pose`
- `perception_node` → `/dental/perception/inspect` (`InspectTeeth.action`)
- Results → `/dental/perception/findings` → current target manager → supervisor

The existing motion node currently supports **fake-hardware, no-contact**
pre-approach/hover/retract, not a physical brushing stroke or a move-to-
inspection-pose. The current MVP also needs explicit cleaning pass counting
per tooth. **Do not interpret a successful image preview as real UR5e readiness.**

Use `scripts/launch/mvp_fake.launch.py` if you need the fake-UR full stack; its
`perception_config` now points to the standard `perception.yaml`. Do not enable
`use_mock_inspection` and `use_real_perception` at the same time (both would
own the same `/dental/perception/inspect` action server). With default image
mode only, real perception will intentionally not complete the motion cycle.

## 7. Build / test limits

The sample-image detector has been tested against the supplied PNG files and
all four frames identify IDs 0–3 and at least one candidate dark area. The
full ROS 2 Humble compilation, camera SDK initialization, calibrated TF, and
real-robot execution must be validated on your Ubuntu machine.

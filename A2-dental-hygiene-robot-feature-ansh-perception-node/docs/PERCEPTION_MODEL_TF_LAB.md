# Dental model + tooth TF calibration (UR5e OFF)

This is part of the standard `dental_perception` package, not a separate USB perception implementation.

## Important: rigid fiducial geometry

All four square ArUco cards and the printed jaw must be affixed to a **single rigid fixture**.
Moving only the jaw while leaving markers on the table will invalidate the marker-to-jaw
transform. A flat movable carrier or model holder with measured card mounts is ideal.

`Jaw_model.stl` has bounds X -70..+70 mm, Y -50..+50 mm, Z 0..31 mm.
Its four tooth top-frame centres are (-21, 0, 31), (-7, 0, 31),
(+7, 0, 31) and (+21, 0, 31), all in CAD mm.

### Step 1: one-time measurements, all in millimetres

- Measure the **detected ArUco square's outer edge**, as shown by the outline in
  `/dental/perception/overhead_preview`; **not** the entire black carrier/card.
- Establish CAD +X along the teeth from tooth_1 to tooth_4. CAD +Z is up.
  The CAD origin is the **centre of the bottom face** of the 140 x 100 mm jaw base.
- Measure each marker square centre's X, Y from that CAD origin and Z above
  the jaw base bottom. If the marker is flush on the same table as the bottom
  of the model, Z = 0; otherwise measure any height difference.
- Measure each marker's rotation about CAD +Z: positive anticlockwise from
  **CAD +X to the cyan arrow** shown on the marker in the preview. That arrow
  identifies the marker's decoded canonical +X, so yaw need not be guessed
  from the printed marker's pattern.
- Once the square edge, all four centres and all four orientations are known,
  convert mm to metres and degrees to radians before copying to the YAML.

For markers IDs 0,1,2,3, fill:

```yaml
    marker_size_m: 0.0 # replace with black fiducial square edge in metres
    marker_centres_xyz_m: [x0,y0,z0, x1,y1,z1, x2,y2,z2, x3,y3,z3] # metres
    marker_yaw_rad: [yaw0, yaw1, yaw2, yaw3] # radians
    marker_layout_calibrated: true # only after measuring ALL values
```

### Step 2: camera intrinsic calibration

Select the overhead camera's final stream resolution. Do not change focus,
resolution or zoom after calibrating. A checkerboard calibration is independent
of UR robot power. Stop perception.launch.py while capturing the checkerboard.
Count checkerboard *internal* corners, and measure one square's side in mm.

```bash
source scripts/source_ros.sh
ls -l /dev/dental_overhead
python3 scripts/cameras/calibrate_overhead.py \
  --device /dev/dental_overhead \
  --width 640 --height 480 \
  --columns 9 --rows 6 --square-mm <ACTUAL_SQUARE_MM> \
  --samples 25 --output overhead_intrinsics.txt
```

Change `--columns` and `--rows` to match the actual checkerboard; 9x6 and
640x480 are examples. Capture the board at different image positions and tilts,
not 25 consecutive identical frames. Copy the measured K, D and calibration
flag to `perception.yaml` *only after checking RMS and resolution*.

### Step 3: Camera-relative PnP, no robot or base TF

```yaml
    overhead_input_mode: camera
    model_pose_reference: camera
    overhead_intrinsics_calibrated: true
    marker_layout_calibrated: true
    publish_offline_model_pose: false
```

In separate terminals:

```bash
source scripts/source_ros.sh
ros2 launch dental_perception perception.launch.py
```

```bash
source scripts/source_ros.sh
ros2 topic echo /dental/perception/model_localized --once
ros2 topic echo /dental/perception/model_pose --once
ros2 run tf2_ros tf2_echo overhead_color_optical_frame dental_model
ros2 run tf2_ros tf2_echo dental_model tooth_1
```

Camera-relative pose mode publishes `overhead_color_optical_frame -> dental_model`
using only observed camera images and measured geometry. It does **not** claim
any transform relative to `base_link` and **cannot authorize robot motion**.
In RViz use `overhead_color_optical_frame` as the Fixed Frame. Use TF + Axes
for the model and tooth frames. Check x/y/z, Z up from the model, and rotate
or translate the complete fixture and verify the TF follows it.

### Step 4: later, once the robot is available

Physically calibrate the fixed transform from `base_link` to the overhead
optical frame, switch `model_pose_reference: base`, then validate tooth positions
with known reference points and MoveIt collision geometry before motion.
Never substitute an arbitrary identity base-camera TF.

### Notes

- Image-only mode (saved RGB photos) stays preview-only by default; do not turn
  on `publish_offline_model_pose` to pretend photos describe a live lab pose.
- Only **one** localizer should publish `dental_model` at once. Never run both
  camera-relative and base-relative localizers simultaneously.
- If a PnP pose fails, first verify ArUco arrow orientations, X/Y sign,
  fiducial square dimensions, true centre positions, and camera intrinsics.

# Perception teammate handoff

**Start point:** `src/dental_perception/src/perception_node.cpp` (C++, ROS 2 Humble).

**Build dependencies:** CMake/ament, `rclcpp`, `rclcpp_action`, `sensor_msgs`,
`dental_interfaces`. Add OpenCV `cv_bridge`, `image_transport`,
`message_filters` and calibration packages when needed, and reflect them in
`CMakeLists.txt` / `package.xml`.

## Inputs: actual camera topics

- `rgb_topic`: default `/camera/color/image_raw` (`sensor_msgs/msg/Image`)
- `depth_topic`: default `/camera/aligned_depth_to_color/image_raw` (`sensor_msgs/msg/Image`)
- `camera_info_topic`: default `/camera/color/camera_info` (`sensor_msgs/msg/CameraInfo`)

The defaults are placeholders for **topic names**, not fake data sources.
Remap/parametrize them to the actual RGB-D device or a camera-enabled simulator.
Synchronize RGB, depth, and camera calibration; confirm depth encoding and
registration rather than assuming pixel alignment.

## Outputs

- Publish `dental_interfaces/msg/DentalFindingArray` on
  `/dental/perception/findings` after each actual completed inspection.
- Implement `/dental/perception/inspect` (`InspectTeeth.action`) using actual
  camera frames. Do not return success if acquisition/calibration fails.
- All positions/normals expressed in one valid frame, with acquisition time
  in array header. Position metres, normal unit length, confidence [0,1].
- Identifiers are only guaranteed unique within an inspection.
- `issue_type`: start with PLAQUE and STAIN; only label what the detector can
  meaningfully distinguish on the provided dental model.

## Acceptance criteria

1. Package builds and runs as C++ node with ROS 2 Humble.
2. Camera publisher connects; actual RGB, registered depth and intrinsics used.
3. Detects issue regions on observed tooth surfaces, or returns an empty
   array when a **real completed inspection** finds none.
4. Produces measured 3D surface coordinates (metres) and estimated normals;
   validate with a physically known point or geometrical reference.
5. Uses meaningful `header.frame_id` and acquisition timestamp. Downstream
   TF2 transforms must succeed with real camera extrinsics.
6. Supports an action request for inspect/reinspect when integrated with the
   supervisor, with cancellation and failure handling.
7. Demonstrates no stale findings on failed or interrupted acquisition.
8. Document actual sensor model, topic names, calibration and required launch.

Perception does **not** directly command the robot, choose tools or
compute force-controlled treatment trajectories.

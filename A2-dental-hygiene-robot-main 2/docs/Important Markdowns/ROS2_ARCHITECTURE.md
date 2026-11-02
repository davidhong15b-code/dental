# ROS 2 initialization architecture (MTRN4231)

**This is a C++ package / message / action / launch scaffold,**

## Package ownership (Tentative)

| Package | Owner | Future development |
| --- | --- | --- |
| dental_interfaces | shared | Contracts below |
| dental_perception | Pradheep | Actual camera RGB-D -> image processing and 3D findings |
| dental_target_manager | Ansh | selection, TF2, reachability/geometry checks |
| dental_supervisor | Ansh | state machine and retry/inspection workflow |
| dental_motion | Ansh | MoveIt planning and execution for UR5e |
| dental_tool_control | Ansh | physical tool selection + calibrated TCP |
| dental_bringup | shared | C++ node launch and mode parameters |

## Defined message/action contracts

- `/dental/perception/findings`: `dental_interfaces/msg/DentalFindingArray`.
  Every published message represents a **completed actual inspection**. `header.stamp`
  reflects sensor acquisition, `frame_id` is the camera optical frame (or another
  accurately calibrated and named frame). A successful no-findings inspection may
  send an empty array; lack of any message is *not* an empty inspection.
- `/dental/perception/inspect`: `dental_interfaces/action/InspectTeeth`.
  On-demand acquisition action. Current implementation rejects all goals until implemented.
  Its result should agree with the inspection findings topic.
- `/dental/targets/selected`: `dental_interfaces/msg/DentalTarget`.
  Contains a selected measured finding, with position and outward surface normal
  **both transformed into** `header.frame_id` (typically `base_link`). Publish only
  after successful TF2 transform and validations.
- `/dental/supervisor/run_cycle`: `dental_interfaces/action/RunCycle`.
- `/dental/motion/treat_target`: `dental_interfaces/action/TreatTarget`.
- `/dental/tools/change_tool`: `dental_interfaces/action/ChangeTool`.

## Coordinate and interpretation contract for perception

- Positions: metres, frame = `DentalFindingArray.header.frame_id`.
- Outward surface normals: unit vectors in the same frame (typically camera optical).
- Confidence: [0, 1].
- `id`: inspection-local finding ID; no guarantees across inspections.
- `tooth_id`: optional, use empty string if unknown.
- `issue_type`: value defined in `DentalFinding.msg`.
- Using real synchronized RGB, registered depth and appropriate camera intrinsics.
  A raw 2D image alone cannot produce metric 3D positions.
- Camera optical axis conventions should follow the installed ROS optical frame;
  the robot manager handles TF2 camera -> base transform once calibrated.
- A normal alone does not define the full tool orientation or a safe contact path.
  Contact angle, tool approach, TCP calibration and force/motion limits belong
  to the motion/tool implementation.

## Fake vs real UR5e

The *existing* robot driver and MoveIt launch process remains separate from
`ros2 launch dental_bringup nodes.launch.py robot_mode:=...`:

- Fake UR5e: `use_fake_hardware:=true` with `joint_trajectory_controller`,
  paired with existing `scripts/launch/fake_ur_moveit.launch.py` adapted
  to use that controller.
- Real UR5e: driver connected to real robot and controller configured for its
  MoveIt adapter (existing setup uses `scaled_joint_trajectory_controller`).
  Calibrate the real robot and tools before any later execution; current
  `dental_motion` rejects all action requests.
- For a fully virtual camera pipeline, add a camera-capable simulation and
  publish `sensor_msgs/Image` and `CameraInfo` on the configured real topics.
  Otherwise a physical RGB-D camera can be used with either fake or real UR5e.

## Branch workflow (tentative)

Suggested follow-ons:

- `feature/pradheep-perception`
- `feature/ansh-target-manager-tf2`
- `feature/ansh-supervisor-state-machine`
- `feature/ansh-ur5e-motion`
- `feature/ansh-tool-control`


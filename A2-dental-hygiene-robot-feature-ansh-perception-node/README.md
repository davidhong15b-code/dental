## Unified dental MVP (camera localisation → hover sweeps → reinspection → home)

See **[docs/MVP_COMMISSIONING.md](docs/MVP_COMMISSIONING.md)** for the new
simulation + real UR5e instructions, source-derived camera reference values,
non-contact safety gates, marker-layout measurements and day-of-demo procedure.
The new action is `MoveToStation`, implemented by `station_motion_node`, and
existing perception/supervisor/target manager/TreatTarget nodes remain in use.

**NOT hardware validated:** offline image/structure checks pass, but new ROS 2 C++
changes must be compiled and commissioned on Ubuntu before they can move the
real UR5e. Physical camera reference values are provisional until verified.

---

# Dental Hygiene Product Tester

**MTRN4231 — Robotics Group Project**

## Overview

This project aims to develop a vision-guided autonomous robotic system for automated dental hygiene product testing using a Universal Robots UR5e robotic arm and a 3D-printed dental model.

The system is intended to support repeatable testing of dental cleaning tools and procedures by identifying simulated plaque or staining, planning appropriate cleaning actions, executing them using a robotic end effector, and evaluating the results through visual re-inspection.

## Proposed System Workflow

1. **Inspect:** Capture images of the dental model using a camera.
2. **Detect:** Identify and localise simulated plaque or staining.
3. **Plan:** Select the appropriate cleaning tool and generate a collision-free robotic trajectory.
4. **Execute:** Position the UR5e and perform the cleaning operation.
5. **Verify:** Re-inspect the treated region and evaluate cleaning performance.
6. **Repeat:** Apply additional cleaning where required.

## Key Objectives

- Develop a ROS 2-based modular robotic control system.
- Integrate camera-based detection and localisation.
- Implement robotic motion planning using the UR5e.
- Develop and control a custom end-effector/tool-changing mechanism.
- Incorporate visual feedback and closed-loop cleaning correction.
- Quantitatively evaluate cleaning performance, including cleaning efficiency.

## Technology Stack

- **Robot:** Universal Robots UR5e
- **Middleware:** ROS 2 Humble
- **Programming:** Python and C++
- **Motion Planning:** MoveIt 2 (planned)
- **Perception:** Camera-based computer vision
- **End Effector:** Custom cleaning-tool mechanism and controller

## Repository Structure

- `src/` — ROS 2 packages for perception, planning, control, interfaces and launch files.
- `firmware/end_effector/` — Microcontroller software for the custom end effector.
- `docs/` — System architecture, hardware design, calibration and testing documentation.

## ROS 2 Environment Setup

### Prerequisites

This project uses **Ubuntu 22.04 with ROS 2 Humble**. The following dependencies are required:

- ROS 2 Humble
- MoveIt 2
- Universal Robots ROS 2 Driver (`ur_robot_driver`, `ur_moveit_config`)
- RViz 2 and `ros2_control`
- Colcon build tools

MoveIt 2 and the UR driver may be installed from source in separate workspaces.

### Workspace Configuration

Before running the project, source ROS 2 and any required dependency workspaces:

```bash
source /opt/ros/humble/setup.bash
source /path/to/ws_moveit2/install/local_setup.bash
source /path/to/ws_ur_driver/install/local_setup.bash
```

Replace the paths with the locations of your own installations. **Source MoveIt before the UR driver** when using this workspace arrangement.

Verify that the packages are available:

```bash
ros2 pkg prefix moveit_ros_planning_interface
ros2 pkg prefix ur_robot_driver
ros2 pkg prefix ur_moveit_config
```

The `scripts/source_ros.sh` file provides an example of a working environment configuration. It is specific to Ansh's development machine and is not required for other setups.

### ROS Domain ID

ROS 2 uses `ROS_DOMAIN_ID=50` to separate communication between independent ROS systems.

```bash
export ROS_DOMAIN_ID=50
```

Set the domain ID in each terminal before launching ROS 2 nodes.

### Running the UR5e

**Fake UR5e — Terminal 1 (Driver)**

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=192.168.56.101 \
  use_fake_hardware:=true \
  initial_joint_controller:=joint_trajectory_controller \
  launch_rviz:=false
```

**Fake UR5e — Terminal 2 (MoveIt + RViz)**

```bash
ros2 launch ./scripts/launch/fake_ur_moveit.launch.py \
  ur_type:=ur5e \
  launch_rviz:=true
```

The custom `fake_ur_moveit.launch.py` selects the correct trajectory controller for fake hardware. It was tested with our source-built UR driver configuration; compatibility should be checked with other installations.

**Real UR5e**

Use `use_fake_hardware:=false` and the actual robot IP when launching the driver. The physical robot uses `scaled_joint_trajectory_controller` and the original `ur_moveit_config` launch file.

### Verification

After launching the driver, check:

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

For fake hardware, confirm that `joint_trajectory_controller` is active and that MoveIt can successfully **Plan & Execute** a trajectory in RViz.

## Development Status

**Initial setup — implementation in progress.**

System modules, interfaces, dependencies, and launch instructions will be documented as development progresses.

## Contributors

Ansh Nimbalkar
David Hong
Charchit Bhatnagar
Pradheep

## Perception camera setup

See [docs/PERCEPTION_SETUP.md](docs/PERCEPTION_SETUP.md) for the standard C++ camera/image pipeline, lab measurements, udev camera setup, image-mode smoke test and hardware safety boundaries. All measured settings belong in `src/dental_perception/config/perception.yaml`.

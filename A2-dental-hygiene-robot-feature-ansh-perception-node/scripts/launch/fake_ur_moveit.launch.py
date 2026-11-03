# Copyright (c) 2021 PickNik, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Author: Denis Stogl

import os

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    _publish_robot_description_semantic = LaunchConfiguration("publish_robot_description_semantic")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")
    # Optional, isolated C++ planning test: disabled for normal team workflows.
    launch_motion_smoke_test = LaunchConfiguration("launch_motion_smoke_test")
    smoke_execute = LaunchConfiguration("smoke_execute")
    smoke_dz = LaunchConfiguration("smoke_dz")
    launch_dental_motion = LaunchConfiguration("launch_dental_motion")
    launch_station_motion = LaunchConfiguration("launch_station_motion")
    mvp_hover = LaunchConfiguration("enable_mvp_hover_strokes")
    robot_mode = LaunchConfiguration("robot_mode")
    mode = robot_mode.perform(context)
    if mode not in ("real", "fake"):
        raise RuntimeError("robot_mode must be fake or real")
    real_mode = mode == "real"
    confirm_real_calibration = LaunchConfiguration("confirm_real_calibration")
    confirm_real_workcell = LaunchConfiguration("confirm_real_workcell")
    execute_real_motion = LaunchConfiguration("execute_real_motion")
    brush_tip_offset = [float(LaunchConfiguration(f"brush_tip_offset_{axis}_m").perform(context))
                        for axis in ("x","y","z")]
    robot_kinematics_file = LaunchConfiguration("robot_kinematics_file")
    real_home_csv = LaunchConfiguration("real_home_joint_positions_csv")
    fake_probe_enabled = LaunchConfiguration("fake_treatment_probe_enabled")

    execute_fake_motion = LaunchConfiguration("execute_fake_motion")
    confirm_fake_hardware = LaunchConfiguration("confirm_fake_hardware")
    motion_velocity_scaling = LaunchConfiguration("motion_velocity_scaling")
    motion_acceleration_scaling = LaunchConfiguration("motion_acceleration_scaling")

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = (robot_kinematics_file if robot_kinematics_file.perform(context)
                         else PathJoinSubstitution(
                             [FindPackageShare(description_package), "config", ur_type, "default_kinematics.yaml"]))
    physical_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "visual_parameters.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "srdf", moveit_config_file]
            ),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    publish_robot_description_semantic = {
        "publish_robot_description_semantic": _publish_robot_description_semantic
    }

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )

    robot_description_planning = {
        "robot_description_planning": load_yaml(
            str(moveit_config_package.perform(context)),
            os.path.join("config", str(moveit_joint_limits_file.perform(context))),
        )
    }

    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")

    # Real UR driver normally uses scaled JTC, fake UR driver unscaled JTC.
    # Verify the active controller against ros2 control list_controllers BEFORE execution.
    controllers_yaml["scaled_joint_trajectory_controller"]["default"] = real_mode
    controllers_yaml["joint_trajectory_controller"]["default"] = not real_mode

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 2.0,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        # Fake UR5e uses the UNscaled JTC. Keep MoveIt's execution watchdog
        # enabled: otherwise a stuck controller result can hang indefinitely.
        "trajectory_execution.execution_duration_monitoring": True,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            publish_robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
            warehouse_ros_config,
        ],
    )

    # rviz with moveit configuration
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "rviz", "view_robot.rviz"]
    )
    configured_rviz_path = LaunchConfiguration("rviz_config_file").perform(context)
    rviz_config_file = configured_rviz_path if configured_rviz_path else default_rviz_config
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
            warehouse_ros_config,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    # Servo node for realtime control
    servo_yaml = load_yaml("ur_moveit_config", "config/ur_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
        ],
        output="screen",
    )

    # MoveGroupInterface does NOT automatically inherit move_group's node-local
    # parameters. Pass the exact same URDF, SRDF, kinematics, and limits here.
    # This intentionally does not alter existing nodes or perception interfaces.
    motion_smoke_test_node = Node(
        package="dental_motion",
        executable="motion_smoke_test",
        condition=IfCondition(launch_motion_smoke_test),
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            {
                "use_sim_time": use_sim_time,
                "robot_mode": robot_mode,
                "planning_group": "ur_manipulator",
                "pose_link": "tool0",
                "dz": ParameterValue(smoke_dz, value_type=float),
                "execute": ParameterValue(smoke_execute, value_type=bool),
            },
        ],
    )

    # Increment 3 action server. MoveGroupInterface needs the SAME URDF/SRDF
    # node parameters that move_group uses; ROS 2 parameters are node-local.
    # Disabled unless explicitly requested, preserving teammates' launches.
    dental_motion_node = Node(
        package="dental_motion",
        executable="motion_node",
        condition=IfCondition(launch_dental_motion),
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            {
                "use_sim_time": use_sim_time,
                "robot_mode": robot_mode,
                "planning_group": "ur_manipulator",
                "robot_base_frame": "base_link",
                "pose_link": "tool0",
                "execute_fake_motion": ParameterValue(execute_fake_motion, value_type=bool),
                "confirm_fake_hardware": ParameterValue(confirm_fake_hardware, value_type=bool),
                "fake_velocity_scaling": ParameterValue(motion_velocity_scaling, value_type=float),
                "fake_acceleration_scaling": ParameterValue(motion_acceleration_scaling, value_type=float),
                "enable_mvp_hover_strokes": ParameterValue(mvp_hover, value_type=bool),
                "max_first_move_m": 0.35,
                "preapproach_distance_m": 0.15,
                "hover_distance_m": 0.09,
                "max_total_planned_duration_sec": 120.0,
                "execute_real_motion": ParameterValue(execute_real_motion,value_type=bool),
                "confirm_real_calibration": ParameterValue(confirm_real_calibration,value_type=bool),
                "confirm_real_workcell": ParameterValue(confirm_real_workcell,value_type=bool),
                "brush_tip_offset_tool_m": brush_tip_offset,
            },
        ],
    )

    station_motion_node = Node(
        package="dental_motion", executable="station_motion_node", output="screen",
        condition=IfCondition(launch_station_motion),
        parameters=[robot_description,robot_description_semantic,
                    robot_description_kinematics,robot_description_planning,
                    {"robot_mode":robot_mode,"robot_base_frame":"base_link",
                     "planning_group":"ur_manipulator","pose_link":"tool0",
                     "rgbd_optical_frame":"rgbd_color_optical_frame",
                     "execute_fake_motion":ParameterValue(execute_fake_motion,value_type=bool),
                     "confirm_fake_hardware":ParameterValue(confirm_fake_hardware,value_type=bool),
                     "execute_real_motion":ParameterValue(execute_real_motion,value_type=bool),
                     "confirm_real_calibration":ParameterValue(confirm_real_calibration,value_type=bool),
                     "confirm_real_workcell":ParameterValue(confirm_real_workcell,value_type=bool),
                     "velocity_scaling":ParameterValue(motion_velocity_scaling,value_type=float),
                     "real_home_joint_positions_csv":real_home_csv.perform(context),
                     "fake_treatment_probe_enabled":ParameterValue(fake_probe_enabled,value_type=bool),
                     "fake_treatment_probe_tooth_frame":"tooth_1",
                     "fake_treatment_probe_preapproach_m":0.15,
                     "fake_treatment_probe_first_move_m":0.35,
                     "fake_treatment_probe_brush_offset_tool_m":brush_tip_offset}])

    # Give move_group a moment to expose its services and action servers.
    nodes_to_start = [
        move_group_node,
        rviz_node,
        servo_node,
        TimerAction(period=3.0, actions=[motion_smoke_test_node, dental_motion_node,
                                        station_motion_node]),
    ]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Robot model of the used UR robot.",
            choices=[
                "ur3",
                "ur5",
                "ur10",
                "ur3e",
                "ur5e",
                "ur7e",
                "ur10e",
                "ur12e",
                "ur16e",
                "ur8long",
                "ur15",
                "ur18",
                "ur20",
                "ur30",
            ],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_description",
            description="Description package with robot URDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_robot_description_semantic",
            default_value="True",
            description="Whether to publish the SRDF description on topic /robot_description_semantic.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="ur_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom moveit config.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_joint_limits_file",
            default_value="joint_limits.yaml",
            description="MoveIt joint limits that augment or override the values from the URDF robot_description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path where the warehouse database should be stored",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Make MoveIt to use simulation time. This is needed for the trajectory planing in simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_servo", default_value="true", description="Launch Servo?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_motion_smoke_test", default_value="false",
            description="Launch the optional guarded dental_motion C++ MoveIt test?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "smoke_execute", default_value="false",
            description="Execute the smoke trajectory (fake hardware only, after verification)?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "smoke_dz", default_value="0.025",
            description="Bounded planning-frame Z displacement in metres.",
        )
    )

    # Multiple declarations require extend([...]); append() accepts one item.
    declared_arguments.extend([
        DeclareLaunchArgument("launch_station_motion",default_value="false"),
        DeclareLaunchArgument("robot_mode",default_value="fake"),
        DeclareLaunchArgument("robot_kinematics_file",default_value="",description="UR kinematic YAML, same as actual UR driver"),
        DeclareLaunchArgument("enable_mvp_hover_strokes",default_value="false"),
        DeclareLaunchArgument("execute_real_motion",default_value="false"),
        DeclareLaunchArgument("brush_tip_offset_x_m",default_value="0.0"),
        DeclareLaunchArgument("brush_tip_offset_y_m",default_value="0.0"),
        DeclareLaunchArgument("brush_tip_offset_z_m",default_value="0.0"),
        DeclareLaunchArgument("confirm_real_calibration",default_value="false"),
        DeclareLaunchArgument("confirm_real_workcell",default_value="false"),
    ])
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_dental_motion", default_value="false",
            description="Launch Increment 3 TreatTarget action server (fake-only positioning demo)?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_treatment_probe_enabled", default_value="false",
            description="Fake mock-only treatment look-ahead during first inspection planning.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "execute_fake_motion", default_value="false",
            description="Enable fake hardware pre-approach/hover/retract execution (not cleaning)?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "confirm_fake_hardware", default_value="false",
            description="Operator asserts that the UR driver is verified to use fake hardware.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "motion_velocity_scaling", default_value="0.05",
            description="Fake-hardware motion velocity scaling [0.01, 0.20]; default 0.05.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "motion_acceleration_scaling", default_value="0.05",
            description="Fake-hardware motion acceleration scaling [0.01, 0.20]; default 0.05.",
        )
    )

    declared_arguments.append(DeclareLaunchArgument("real_home_joint_positions_csv", default_value=""))
    declared_arguments.append(DeclareLaunchArgument("rviz_config_file", default_value=""))
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

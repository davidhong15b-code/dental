"""Single-command fake UR5e + MoveIt + closed-loop dental MVP bringup.

This file ALWAYS requests use_fake_hardware=true. It NEVER launches a real driver.
The mock inspection backend is explicitly synthetic; replace with teammate's
InspectTeeth action server for live camera tests.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    here = os.path.dirname(os.path.realpath(__file__))
    fake_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ur_robot_driver'), 'launch', 'ur_control.launch.py'])),
        condition=IfCondition(LaunchConfiguration('start_fake_driver')),
        launch_arguments={
            'ur_type': 'ur5e', 'robot_ip': '192.168.56.101',
            'use_fake_hardware': 'true',
            'initial_joint_controller': 'joint_trajectory_controller',
            'launch_rviz': 'true',
        }.items())
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(here, 'fake_ur_moveit.launch.py')),
        launch_arguments={
            'ur_type': 'ur5e',
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'launch_servo': 'false',
            'launch_motion_smoke_test': 'false',
            'launch_dental_motion': 'true',
            'execute_fake_motion': LaunchConfiguration('execute_fake_motion'),
            'confirm_fake_hardware': LaunchConfiguration('confirm_fake_hardware'),
            'motion_velocity_scaling': LaunchConfiguration('motion_velocity_scaling'),
            'motion_acceleration_scaling': LaunchConfiguration('motion_acceleration_scaling'),
        }.items())
    manager = Node(package='dental_target_manager', executable='target_manager_node',
                   output='screen', parameters=[{'robot_base_frame': 'base_link'}])
    supervisor = Node(package='dental_supervisor', executable='supervisor_node',
                      output='screen', parameters=[{
                          'max_positioning_passes': ParameterValue(LaunchConfiguration('max_passes'), value_type=int),
                          'motion_timeout_sec': 100.0,
                          'max_joint_state_age_sec': 2.0,
                          'require_workcell_ready': ParameterValue(LaunchConfiguration('load_workcell'), value_type=bool),
                      }])
    mock_inspector = Node(
        package='dental_bringup', executable='mock_inspection_action.py',
        condition=IfCondition(LaunchConfiguration('use_mock_inspection')),
        output='screen', arguments=['--clear-after', LaunchConfiguration('mock_clear_after')])
    workcell = Node(
        package='dental_bringup', executable='apply_workcell.py',
        condition=IfCondition(LaunchConfiguration('load_workcell')),
        arguments=['--config', LaunchConfiguration('workcell_config')], output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('start_fake_driver', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('execute_fake_motion', default_value='false'),
        DeclareLaunchArgument('confirm_fake_hardware', default_value='false'),
        DeclareLaunchArgument('motion_velocity_scaling', default_value='0.15'),
        DeclareLaunchArgument('motion_acceleration_scaling', default_value='0.15'),
        DeclareLaunchArgument('max_passes', default_value='2'),
        DeclareLaunchArgument('use_mock_inspection', default_value='true'),
        DeclareLaunchArgument('mock_clear_after', default_value='-1'),
        DeclareLaunchArgument('load_workcell', default_value='false'),
        DeclareLaunchArgument('workcell_config', default_value=PathJoinSubstitution([
            FindPackageShare('dental_bringup'), 'config', 'workcell.template.yaml'])),
        fake_driver,
        moveit,
        TimerAction(period=5.0, actions=[manager, supervisor, mock_inspector, workcell]),
    ])

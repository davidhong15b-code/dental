"""Single-command fake UR5e + MoveIt + closed-loop dental MVP bringup.

This file ALWAYS requests use_fake_hardware=true. It NEVER launches a real driver.
The mock inspection backend is explicitly synthetic; replace with teammate's
InspectTeeth action server for live camera tests.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    here = os.path.dirname(os.path.realpath(__file__))
    # The UR driver and MoveIt both declare `launch_rviz`. Scope the UR driver's
    # launch arguments so its `launch_rviz=false` does NOT accidentally override
    # the MoveIt RViz setting for the entire parent launch.
    fake_driver = GroupAction(scoped=True, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'), 'launch', 'ur_control.launch.py'])),
            condition=IfCondition(LaunchConfiguration('start_fake_driver')),
            launch_arguments={
                'ur_type': 'ur5e', 'robot_ip': '192.168.56.101',
                'use_fake_hardware': 'true',
                'initial_joint_controller': 'joint_trajectory_controller',
                'launch_rviz': 'false',
            }.items())
    ])
    # Synthetic tooth offsets from the CAD jaw: the entire tree is fake-hardware only.
    # Without these frames, the simulated image inspector cannot resolve its
    # tooth_1 target despite a valid base_link -> dental_model transform.
    virtual_teeth = [
        Node(package='tf2_ros', executable='static_transform_publisher',
             output='screen', arguments=[
                 '--x', str(x), '--y', '0', '--z', '0.031',
                 '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                 '--frame-id', 'dental_model', '--child-frame-id', f'tooth_{index}'])
        for index, x in enumerate((-0.021, -0.007, 0.007, 0.021), start=1)
    ]
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(here, 'fake_ur_moveit.launch.py')),
        launch_arguments={
            'ur_type': 'ur5e',
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'rviz_config_file': LaunchConfiguration('rviz_config_file'),
            'launch_servo': 'false',
            'launch_motion_smoke_test': 'false',
            'launch_dental_motion': 'true',
            'launch_station_motion': 'true',
            'enable_mvp_hover_strokes':'true',
            'robot_mode':'fake',
            'execute_fake_motion': LaunchConfiguration('execute_fake_motion'),
            'confirm_fake_hardware': LaunchConfiguration('confirm_fake_hardware'),
            'fake_treatment_probe_enabled': LaunchConfiguration('use_mock_inspection'),
            'motion_velocity_scaling': LaunchConfiguration('motion_velocity_scaling'),
            'motion_acceleration_scaling': LaunchConfiguration('motion_acceleration_scaling'),
        }.items())
    manager = Node(package='dental_target_manager', executable='target_manager_node',
                   output='screen', parameters=[{'robot_base_frame': 'base_link'}])
    supervisor = Node(package='dental_supervisor', executable='supervisor_node',
                      output='screen', parameters=[{
                          'max_positioning_passes': ParameterValue(LaunchConfiguration('max_passes'), value_type=int),
                          'motion_timeout_sec': 180.0,
                          'max_joint_state_age_sec': 2.0,
                          'require_workcell_ready': ParameterValue(LaunchConfiguration('load_workcell'), value_type=bool),
                      'enable_mvp_cycle': True,
                      }])
    live_localizer = Node(
        package='dental_perception', executable='model_localizer_node',
        name='model_localizer', output='screen',
        condition=IfCondition(LaunchConfiguration('use_real_perception')),
        parameters=[LaunchConfiguration('perception_config')])
    live_inspector = Node(
        package='dental_perception', executable='perception_node',
        name='perception', output='screen',
        condition=IfCondition(LaunchConfiguration('use_real_perception')),
        parameters=[LaunchConfiguration('perception_config'), {'enable_inspection_action': False}])
    mock_inspector = Node(
        package='dental_bringup', executable='mock_inspection_action.py',
        condition=IfCondition(LaunchConfiguration('use_mock_inspection')),
        output='screen', arguments=['--clear-after', LaunchConfiguration('mock_clear_after'),
                  '--tooth-frame','tooth_1',
                  '--wrist-sample',PathJoinSubstitution([
                      FindPackageShare('dental_perception'),'samples','wrist_01.png'])])
    workcell = Node(
        package='dental_bringup', executable='apply_workcell.py',
        condition=IfCondition(LaunchConfiguration('load_workcell')),
        arguments=['--config', LaunchConfiguration('workcell_config'), '--allow-simulation'], output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('start_fake_driver', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config_file', default_value=''),
        DeclareLaunchArgument('execute_fake_motion', default_value='false'),
        DeclareLaunchArgument('confirm_fake_hardware', default_value='false'),
        DeclareLaunchArgument('motion_velocity_scaling', default_value='0.15'),
        DeclareLaunchArgument('motion_acceleration_scaling', default_value='0.15'),
        DeclareLaunchArgument('max_passes', default_value='2'),
        DeclareLaunchArgument('use_mock_inspection', default_value='true'),
        DeclareLaunchArgument('use_real_perception', default_value='false'),
        DeclareLaunchArgument('perception_config', default_value=PathJoinSubstitution([
            FindPackageShare('dental_perception'), 'config', 'perception.yaml'])),
        DeclareLaunchArgument('mock_clear_after', default_value='-1'),
        DeclareLaunchArgument('load_workcell', default_value='true'),
        DeclareLaunchArgument('workcell_config', default_value=PathJoinSubstitution([
            FindPackageShare('dental_bringup'), 'config', 'workcell.simulation.yaml'])),
        fake_driver,
        # SIMULATION ONLY: the following pose is a virtual jaw fixture. It is
        # not a calibrated TF for any lab UR5e.
        Node(package='tf2_ros',executable='static_transform_publisher',output='screen',
             arguments=['--x','0.62','--y','0.12','--z','0.02',
                        '--qx','0','--qy','0','--qz','0','--qw','1',
                        '--frame-id','base_link','--child-frame-id','dental_model']),
        # REFERENCE wrist optical transform for virtual camera geometry only.
        Node(package='tf2_ros',executable='static_transform_publisher',output='screen',
             arguments=['--x','-0.03113','--y','-0.09924','--z','0.02460',
                        '--qx','-0.002221','--qy','-0.002155',
                        '--qz','-0.003637','--qw','0.999989',
                        '--frame-id','tool0','--child-frame-id','rgbd_color_optical_frame']),
        # Reference C2 from ANOTHER LAB STATION, for RViz illustration ONLY.
        # This deliberately does not enter our real-hardware launch.
        Node(package='tf2_ros',executable='static_transform_publisher',output='screen',
             arguments=['--x','1.26842','--y','0.06553','--z','0.85374',
                        '--qx','0.660371','--qy','0.680313',
                        '--qz','-0.223169','--qw','-0.226450',
                        '--frame-id','base_link','--child-frame-id','overhead_color_optical_frame']),
        *virtual_teeth,
        moveit,
        TimerAction(period=5.0, actions=[manager, supervisor, mock_inspector,
                                       live_localizer, live_inspector]),
        # MoveIt can advertise /apply_planning_scene before its scene monitor is
        # responsive. The loader retries, reads back and verifies the collision
        # objects, and only then publishes /dental/workcell/ready.
        TimerAction(period=9.0, actions=[workcell]),
    ])

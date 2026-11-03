"""Application-only launch (external UR driver and MoveIt are already started).

Motion must be launched via scripts/launch/fake_ur_moveit.launch.py to receive
identical URDF/SRDF/kinematics; don't start a bare motion_node here.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_mode', default_value='fake',
                              description='Compatibility argument. Actual driver mode is set in driver launch.'),
        DeclareLaunchArgument('enable_perception', default_value='false'),
        DeclareLaunchArgument('perception_config', default_value=PathJoinSubstitution([
            FindPackageShare('dental_perception'), 'config', 'perception.yaml'])),
        DeclareLaunchArgument('enable_tool_control_placeholder', default_value='false'),
        DeclareLaunchArgument('max_passes', default_value='2'),
        Node(package='dental_target_manager', executable='target_manager_node', output='screen'),
        Node(package='dental_supervisor', executable='supervisor_node', output='screen',
             parameters=[{'max_positioning_passes': ParameterValue(LaunchConfiguration('max_passes'), value_type=int)}]),
        Node(package='dental_perception', executable='model_localizer_node', output='screen',
             name='model_localizer', condition=IfCondition(LaunchConfiguration('enable_perception')),
             parameters=[LaunchConfiguration('perception_config')]),
        Node(package='dental_perception', executable='perception_node', output='screen',
             name='perception', condition=IfCondition(LaunchConfiguration('enable_perception')),
             parameters=[LaunchConfiguration('perception_config')]),
        # Interface-only placeholder: teammate owns hardware implementation.
        Node(package='dental_tool_control', executable='tool_control_node', output='screen',
             condition=IfCondition(LaunchConfiguration('enable_tool_control_placeholder'))),
    ])

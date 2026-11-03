"""MTRN4231 UR5e real-robot commissioning / guarded MVP launcher.

Does not start a robot driver. Default: visualization + live CV previews,
NO physical motion, NO supervisor. The robot must have the external control
program running and a matching, robot-specific kinematics calibration.
"""
import math
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
REQUESTED = [-0.58853, -0.13330, 0.22700, 2.221, 2.221, 0.0]


def _true(v):
    return str(v).strip().lower() in ('true', '1', 'yes')


def _vec(v, n, name):
    if not isinstance(v, list) or len(v) != n or any(
            not isinstance(x, (int, float)) or isinstance(x, bool) or
            not math.isfinite(x) for x in v):
        raise RuntimeError(f'{name} must contain {n} finite numeric elements')
    return [float(x) for x in v]


def _load_config(path, label):
    p = Path(path).expanduser()
    if not p.is_file():
        raise RuntimeError(f'{label} missing: {path}')
    value = yaml.safe_load(p.read_text())
    if not isinstance(value, dict):
        raise RuntimeError(f'{label} must contain a YAML mapping')
    return value


def _prepare(context, *args, **kwargs):
    get = lambda x: LaunchConfiguration(x).perform(context)
    execute = _true(get('execute_real_motion'))
    supervisor = _true(get('enable_real_cycle'))
    if supervisor and not execute:
        raise RuntimeError('enable_real_cycle requires explicitly enabled, commissioned real motion')

    # Real motion never inherits synthetic camera TF or a mock workcell.
    home_file = get('home_config')
    home_csv = ''
    if Path(home_file).is_file():
        home = _load_config(home_file, 'home configuration')
        if home.get('approved_for_automatic_home_motion') is True:
            if home.get('robot_joint_names') != JOINTS or home.get('requested_pose_frame') != 'base':
                raise RuntimeError('Approved home joint ordering or Base frame is wrong')
            _vec(home.get('requested_tcp_xyz_m_rotation_vector_rad'), 6, 'home requested TCP')
            expected = _vec(home['requested_tcp_xyz_m_rotation_vector_rad'], 6, 'requested TCP')
            if any(abs(a-b)>1e-6 for a,b in zip(expected, REQUESTED)):
                raise RuntimeError('Home file does not match the requested pendant TCP pose')
            joints = _vec(home.get('measured_joint_positions_rad'), 6, 'home joints')
            if any(abs(x)>2*math.pi for x in joints):
                raise RuntimeError('Home joints out of bounded UR range')
            if home.get('verified_active_tcp') is not True or home.get('verified_robot_kinematics') is not True:
                raise RuntimeError('Home not commissioned against active TCP and robot kinematics')
            if home.get('observed_tcp_position_error_m', 1.0)>0.008 or home.get('observed_tcp_orientation_error_rad', 1.0)>0.0873:
                raise RuntimeError('Requested pendant home and measured controller TCP disagree')
            home_csv = ','.join(format(v,'.12g') for v in joints)
    if execute:
        if not home_csv:
            raise RuntimeError('Real movement locked: first capture & approve home from pendant; no fallback IK')
        if not all(_true(get(flag)) for flag in ('confirm_real_calibration','confirm_real_workcell')):
            raise RuntimeError('Real movement requires camera/tool and workcell operator confirmations')
        kin = Path(get('robot_kinematics_file'))
        if not kin.is_file():
            raise RuntimeError('Real movement requires robot-specific UR kinematics YAML')
        cell = _load_config(get('workcell_config'), 'measured workcell')
        if cell.get('simulation') is True or cell.get('calibrated') is not True:
            raise RuntimeError('Real movement locked: measured workcell calibrated=true required')
        live_cfg = _load_config(get('perception_config'), 'live perception')
        over = live_cfg.get('model_localizer',{}).get('ros__parameters',{})
        wrist = live_cfg.get('perception',{}).get('ros__parameters',{})
        if (over.get('marker_layout_calibrated') is not True or
            over.get('overhead_input_mode')!='camera' or
            wrist.get('rgbd_input_mode')!='camera' or
            wrist.get('allow_offline_findings') is not False):
            raise RuntimeError('Real movement requires calibrated ArUco layout and two live camera inputs')
        cams = _load_config(get('cameras_config'), 'camera calibration')
        if cams.get('calibrated') is not True:
            raise RuntimeError('Real movement locked: both camera TFs must be calibrated')
        if not _true(get('load_workcell')):
            raise RuntimeError('Real movement requires planning scene loading')

    actions = [SetLaunchConfiguration('real_home_joint_positions_csv', home_csv)]
    cameras_path = Path(get('cameras_config'))
    if cameras_path.is_file():
        cams = _load_config(str(cameras_path), 'camera config')
        if cams.get('calibrated') is True:
            for entry, parent, child in (
                    ('overhead', 'base_link', 'overhead_cam_optical'),
                    ('wrist', 'tool0', 'rgbd_color_optical_frame')):
                cfg = cams.get(entry, {})
                if cfg.get('parent_frame') != parent or cfg.get('optical_frame') != child:
                    raise RuntimeError(f'{entry}: camera parent/optical frame mismatch')
                xyz = _vec(cfg.get('translation_xyz_m'),3,f'{entry} translation')
                quat = _vec(cfg.get('quaternion_xyzw'),4,f'{entry} rotation')
                norm = math.sqrt(sum(x*x for x in quat))
                if abs(norm-1.0)>0.005:
                    raise RuntimeError(f'{entry} quaternion not normalized')
                actions.append(Node(package='tf2_ros',executable='static_transform_publisher',
                    name='dental_'+entry+'_calibrated_tf', output='screen', arguments=[
                        '--x', str(xyz[0]),'--y',str(xyz[1]),'--z',str(xyz[2]),
                        '--qx',str(quat[0]),'--qy',str(quat[1]),
                        '--qz',str(quat[2]),'--qw',str(quat[3]),
                        '--frame-id',parent,'--child-frame-id',child]))
        elif execute:
            raise RuntimeError('Camera extrinsics not calibrated')
    elif execute:
        raise RuntimeError('Camera configuration missing')
    return actions


def generate_launch_description():
    here = Path(__file__).resolve().parent
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(here/'fake_ur_moveit.launch.py')),
        launch_arguments={
            'ur_type':'ur5e', 'robot_mode':'real',
            'launch_rviz':LaunchConfiguration('launch_rviz'),
            'rviz_config_file':LaunchConfiguration('rviz_config_file'),
            'launch_servo':'false', 'launch_motion_smoke_test':'false',
            'launch_dental_motion':'true', 'launch_station_motion':'true',
            'enable_mvp_hover_strokes':'true',
            'execute_fake_motion':'false', 'confirm_fake_hardware':'false',
            'execute_real_motion':LaunchConfiguration('execute_real_motion'),
            'confirm_real_calibration':LaunchConfiguration('confirm_real_calibration'),
            'confirm_real_workcell':LaunchConfiguration('confirm_real_workcell'),
            'robot_kinematics_file':LaunchConfiguration('robot_kinematics_file'),
            'real_home_joint_positions_csv':LaunchConfiguration('real_home_joint_positions_csv'),
            'brush_tip_offset_x_m':LaunchConfiguration('brush_tip_offset_x_m'),
            'brush_tip_offset_y_m':LaunchConfiguration('brush_tip_offset_y_m'),
            'brush_tip_offset_z_m':LaunchConfiguration('brush_tip_offset_z_m'),
            'motion_velocity_scaling':'0.05',
            'motion_acceleration_scaling':'0.05',
        }.items())
    return LaunchDescription([
        DeclareLaunchArgument('execute_real_motion',default_value='false'),
        DeclareLaunchArgument('enable_real_cycle',default_value='false'),
        DeclareLaunchArgument('launch_rviz',default_value='true'),
        DeclareLaunchArgument('rviz_config_file',default_value=''),
        DeclareLaunchArgument('confirm_real_calibration',default_value='false'),
        DeclareLaunchArgument('confirm_real_workcell',default_value='false'),
        DeclareLaunchArgument('robot_kinematics_file',default_value=''),
        DeclareLaunchArgument('brush_tip_offset_x_m',default_value='0.0'),
        DeclareLaunchArgument('brush_tip_offset_y_m',default_value='0.0'),
        DeclareLaunchArgument('brush_tip_offset_z_m',default_value='0.0'),
        DeclareLaunchArgument('perception_config',default_value=PathJoinSubstitution([
            FindPackageShare('dental_perception'),'config','perception.real.yaml'])),
        DeclareLaunchArgument('workcell_config',default_value=str(here.parent.parent/'calibration'/'workcell.real.yaml')),
        DeclareLaunchArgument('home_config',default_value=str(here.parent.parent/'calibration'/'home.real.yaml')),
        DeclareLaunchArgument('cameras_config',default_value=str(here.parent.parent/'calibration'/'cameras.real.yaml')),
        DeclareLaunchArgument('load_workcell',default_value='false'),
        DeclareLaunchArgument('real_home_joint_positions_csv',default_value=''),
        OpaqueFunction(function=_prepare),
        moveit,
        TimerAction(period=5.0,actions=[
            Node(package='dental_perception',executable='model_localizer_node',
                 name='model_localizer',output='screen',
                 parameters=[LaunchConfiguration('perception_config')]),
            Node(package='dental_perception',executable='perception_node',
                 name='perception',output='screen',
                 parameters=[LaunchConfiguration('perception_config')]),
            Node(package='dental_target_manager',executable='target_manager_node',output='screen',
                 condition=IfCondition(LaunchConfiguration('enable_real_cycle'))),
            Node(package='dental_supervisor',executable='supervisor_node',output='screen',
                 condition=IfCondition(LaunchConfiguration('enable_real_cycle')),
                 parameters=[{'enable_mvp_cycle':True,'max_positioning_passes':2,
                    'require_home_before_inspection':True,
                    'motion_timeout_sec':180.0,'require_workcell_ready':True}]),
            Node(package='dental_bringup',executable='apply_workcell.py',output='screen',
                 condition=IfCondition(LaunchConfiguration('load_workcell')),
                 arguments=['--config',LaunchConfiguration('workcell_config')]),
        ]),
    ])

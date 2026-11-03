#!/usr/bin/env python3
"""Static commissioning gate; no ROS imports, no motion or network access."""
import argparse
import math
from pathlib import Path
import yaml

HOME=[-0.58853,-0.13330,0.227,2.221,2.221,0.0]
JOINTS=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
        'wrist_1_joint','wrist_2_joint','wrist_3_joint']

def load(path):
    p=Path(path)
    if not p.is_file():raise ValueError(f'missing config: {p}')
    d=yaml.safe_load(p.read_text())
    if not isinstance(d,dict):raise ValueError(f'YAML mapping required: {p}')
    return d

def vec(x,n,label,positive=False):
    if not isinstance(x,list) or len(x)!=n or any(not isinstance(v,(int,float)) or isinstance(v,bool) or not math.isfinite(v) for v in x):
        raise ValueError(f'{label}: {n} finite values required')
    if positive and any(v<=0 for v in x):raise ValueError(label+' must be positive')
    return x

def check(args):
    root=Path(args.root).resolve()
    home=load(args.home)
    pose=vec(home.get('requested_tcp_xyz_m_rotation_vector_rad'),6,'pendant pose')
    if any(abs(a-b)>1e-6 for a,b in zip(pose,HOME)):
        raise ValueError('Requested home pose changed')
    print('PASS requested pendant Base home reference (metres + rotation vector)')
    cams=load(args.cameras)
    cell=load(args.workcell)
    perception=load(args.perception)
    if args.mode=='preview':
        print('PASS preview-only mode: live cameras may still require USB/SDK configuration')
        print('NOT APPROVED for any real robot motion')
        return
    if home.get('approved_for_automatic_home_motion') is not True or home.get('robot_joint_names')!=JOINTS:
        raise ValueError('Commissioned home approval/joint order not recorded')
    joints=vec(home.get('measured_joint_positions_rad'),6,'measured joints')
    if any(abs(x)>2*math.pi for x in joints):raise ValueError('Home joint outside bounded range')
    if (home.get('requested_pose_frame')!='base' or not home.get('verified_active_tcp')
            or not home.get('verified_robot_kinematics') or not home.get('operator_verified_clearance')):
        raise ValueError('Home not verified using real controller active TCP + robot kinematics')
    if home.get('observed_tcp_position_error_m',1)>0.008 or home.get('observed_tcp_orientation_error_rad',1)>0.0873:
        raise ValueError('Home captured TCP does not match requested pose')
    print('PASS measured approved six-joint home')
    if cams.get('calibrated') is not True:
        raise ValueError('Both cameras extrinsics not calibrated')
    for tag, p, c in (('overhead','base_link','overhead_cam_optical'),('wrist','tool0','rgbd_color_optical_frame')):
        data=cams.get(tag,{})
        if data.get('parent_frame')!=p or data.get('optical_frame')!=c:
            raise ValueError(tag+' TF frame mismatch')
        vec(data.get('translation_xyz_m'),3,tag+' xyz')
        q=vec(data.get('quaternion_xyzw'),4,tag+' quaternion')
        if abs(math.sqrt(sum(x*x for x in q))-1)>0.005:
            raise ValueError(tag+' TF quaternion not normalized')
    print('PASS camera extrinsics present')
    p1=perception.get('model_localizer',{}).get('ros__parameters',{})
    p2=perception.get('perception',{}).get('ros__parameters',{})
    if p1.get('marker_layout_calibrated') is not True or p2.get('allow_offline_findings') is not False:
        raise ValueError('Real localization must be marker-calibrated and use no offline findings')
    if p2.get('rgbd_input_mode')!='camera' or p1.get('overhead_input_mode')!='camera':
        raise ValueError('Real camera inputs required')
    print('PASS perception calibration flags')
    if cell.get('simulation') is True or cell.get('calibrated') is not True or cell.get('planning_frame')!='base_link':
        raise ValueError('Real measured collision workcell not approved')
    for key in ('table','jaw','brush'):
        if key not in cell:raise ValueError('Missing '+key)
    vec(cell['table'].get('center_xyz_m'),3,'table xyz')
    vec(cell['table'].get('size_xyz_m'),3,'table size',True)
    vec(cell['jaw'].get('collision_center_xyz_m'),3,'jaw xyz')
    vec(cell['jaw'].get('collision_size_xyz_m'),3,'jaw size',True)
    vec(cell['brush'].get('center_tool0_xyz_m'),3,'brush xyz')
    vec(cell['brush'].get('size_xyz_m'),3,'brush size',True)
    if cell['brush'].get('touch_links')!=['tool0','flange']:
        raise ValueError('Review brush touch_links, default physical mounting only')
    print('PASS measured workcell collision dimensions')
    if not args.kinematics or not Path(args.kinematics).is_file():
        raise ValueError('Missing robot-specific kinematics YAML')
    print('PASS robot kinematics file provided; confirm the driver uses the SAME file')
    print('STATIC PREFLIGHT PASS. This DOES NOT validate reachability, sensor accuracy, controllers or physical safety.')

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',default='.')
    p.add_argument('--mode',choices=['preview','execution'],default='preview')
    p.add_argument('--home',default='calibration/home.real.yaml')
    p.add_argument('--cameras',default='calibration/cameras.real.yaml')
    p.add_argument('--workcell',default='calibration/workcell.real.yaml')
    p.add_argument('--perception',default='src/dental_perception/config/perception.real.yaml')
    p.add_argument('--kinematics',default='')
    args=p.parse_args()
    try:check(args)
    except Exception as e:
        print('FAIL:',e)
        return 2
    return 0
if __name__=='__main__':raise SystemExit(main())

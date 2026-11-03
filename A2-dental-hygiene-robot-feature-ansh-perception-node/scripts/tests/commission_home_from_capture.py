#!/usr/bin/env python3
"""Convert one read-only real UR5e home capture into a gated commissioning YAML.

No robot commands. Requires active TCP pose from tcp_pose_broadcaster in
pendant Base coordinates and explicit operator declarations.
"""
import argparse
import math
from pathlib import Path
import yaml

JOINTS = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
          'wrist_1_joint','wrist_2_joint','wrist_3_joint']
REQUESTED = [-0.58853,-0.13330,0.22700,2.221,2.221,0.0]

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture',required=True)
    p.add_argument('--output',default='calibration/home.approved.yaml')
    p.add_argument('--confirmed-pendant-base',action='store_true')
    p.add_argument('--confirmed-active-tcp',action='store_true')
    p.add_argument('--verified-robot-kinematics',action='store_true')
    p.add_argument('--verified-clearance',action='store_true')
    a=p.parse_args()
    if not all((a.confirmed_pendant_base,a.confirmed_active_tcp,
                a.verified_robot_kinematics,a.verified_clearance)):
        p.error('A physical operator must verify pendant Base, active TCP, robot kinematics and clearance')
    src=Path(a.capture)
    if not src.is_file():p.error('Capture file missing')
    c=yaml.safe_load(src.read_text())
    if c.get('status')!='UNAPPROVED_CAPTURE_ONLY' or c.get('requested_frame')!='base':
        p.error('Capture must be raw unapproved pose in pendant Base frame')
    if c.get('observed_active_tcp_frame')!='tool0_controller':
        p.error('Expected live tool0_controller from active TCP pose broadcaster')
    xyz=c.get('requested_in_pendant_reference_frame_to_active_tcp',{}).get('position_m')
    rotvec=c.get('requested_rotation_vector_rad')
    if not isinstance(xyz,list) or not isinstance(rotvec,list) or len(xyz)!=3 or len(rotvec)!=3:
        p.error('Malformed captured requested pose')
    if any(abs(float(x)-y)>1e-7 for x,y in zip(xyz+rotvec,REQUESTED)):
        p.error('Capture target does not match requested home')
    positions=c.get('measured_joint_positions_rad')
    if c.get('robot_joint_names')!=JOINTS or not isinstance(positions,list) or len(positions)!=6:
        p.error('Wrong robot joint names or count')
    if any(not isinstance(x,(int,float)) or not math.isfinite(x) or abs(x)>2*math.pi for x in positions):
        p.error('Invalid measured joint configuration')
    if c.get('observed_requestedframe_to_active_tcp') is None:
        p.error('No independent live TCP geometry observed')
    dp=c.get('tcp_position_error_m')
    da=c.get('tcp_angle_error_rad')
    if not isinstance(dp,(int,float)) or not isinstance(da,(int,float)) or dp>0.008 or da>0.0873:
        p.error('Controller TCP differs from requested pendant TCP by >8mm or >5deg')
    if dp<0 or da<0 or not math.isfinite(dp) or not math.isfinite(da):
        p.error('Invalid pose discrepancies')
    out=Path(a.output)
    if out.exists():p.error('Refusing overwrite of existing home configuration')
    out.parent.mkdir(parents=True,exist_ok=True)
    cfg={
        'approved_for_automatic_home_motion':True,
        'requested_pose_frame':'base',
        'requested_tcp_xyz_m_rotation_vector_rad':REQUESTED,
        'robot_joint_names':JOINTS,
        'measured_joint_positions_rad':[float(x) for x in positions],
        'verified_active_tcp':True,
        'verified_robot_kinematics':True,
        'observed_tcp_position_error_m':float(dp),
        'observed_tcp_orientation_error_rad':float(da),
        'capture_file':str(src.resolve()),
        'operator_verified_clearance':True,
        'warning':'This validates the home target only; path and obstacles still require MoveIt + operator review.'
    }
    out.write_text(yaml.safe_dump(cfg,sort_keys=False))
    print('Commissioned home reference saved (no robot motion):',out)
    print('Re-check the live initial-to-home trajectory in MoveIt, do not blindly execute.')
    return 0
if __name__=='__main__':raise SystemExit(main())

#!/usr/bin/env python3
"""Read-only ROS robot-state snapshot at operator-positioned home.

Does NOT send robot goals. Only run with the physical arm already placed by a
trained operator at the intended home, with the pendant displaying Base-frame
TCP coordinates. The tool0 transform is NOT automatically the active UR TCP.

Outputs a PROVISIONAL YAML snapshot (approval flags remain false), allowing a
later validated motion planner to use a deterministic home joint configuration.
"""
import argparse
import datetime
import math
import os
from pathlib import Path
import tempfile
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
import yaml

JOINTS = (
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
)


def finite_triplet(values, name):
    if not isinstance(values, list) or len(values) != 3 or not all(
            isinstance(x, (float, int)) and math.isfinite(x) for x in values):
        raise ValueError(name + ': expected three finite numbers')
    return [float(x) for x in values]


def pose_from_transform(t: TransformStamped):
    x = t.transform.translation
    q = t.transform.rotation
    values = [x.x, x.y, x.z, q.x, q.y, q.z, q.w]
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Nonfinite TF pose')
    norm = math.sqrt(sum(v * v for v in values[3:]))
    if abs(norm - 1.) > 0.01:
        raise ValueError('TF quaternion not normalized')
    return dict(position_m=values[:3], quaternion_xyzw=values[3:])


def rotvec_to_quaternion(r):
    theta = math.sqrt(sum(v*v for v in r))
    if theta < 1e-12:
        return [0., 0., 0., 1.]
    scale = math.sin(theta/2.) / theta
    return [v*scale for v in r] + [math.cos(theta/2.)]


def angular_error(q1, q2):
    dot = abs(sum(a*b for a,b in zip(q1, q2)))
    return 2.0 * math.acos(min(1., max(-1., dot)))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--requested', default='calibration/home.requested.yaml')
    p.add_argument('--output', default=None, help='Provisional YAML output path (will not overwrite)')
    p.add_argument('--base-frame', default='base_link')
    p.add_argument('--tool0-frame', default='tool0')
    p.add_argument('--tcp-frame', default=None,
                   help='OPTIONAL live frame of the actual ACTIVE UR TCP (often tool0_controller '
                        'when tcp_pose_broadcaster is active); not presumed equal to tool0')
    p.add_argument('--timeout', type=float, default=20.)
    p.add_argument('--stable-spread-rad', type=float, default=0.005)
    args = p.parse_args()
    if args.timeout <= 0 or args.stable_spread_rad <= 0:
        p.error('timeout and stable-spread-rad must be positive')
    cfg = yaml.safe_load(Path(args.requested).read_text())
    if cfg.get('representation') != 'ur_tcp_pose_m_rotation_vector_rad':
        raise ValueError('Requested file does not declare a native UR TCP pose')
    xyz = finite_triplet(cfg.get('position_m'), 'position_m')
    rotvec = finite_triplet(cfg.get('rotation_vector_rad'), 'rotation_vector_rad')
    reference_frame = cfg.get('frame')
    if reference_frame not in ('base', 'base_link'):
        raise ValueError('Reference pose frame must be explicit base or base_link')
    if reference_frame == 'base_link':
        print('WARNING: requested pose is in base_link, not the pendant Base frame!')
    expected_quat = rotvec_to_quaternion(rotvec)
    expected_pose = dict(position_m=xyz, quaternion_xyzw=expected_quat)

    rclpy.init()
    node = Node('dental_home_snapshot_readonly')
    buffer = Buffer()
    listener = TransformListener(buffer, node)  # Keep listener alive.
    samples = []

    def on_joint(msg):
        if len(msg.position) != len(msg.name):
            return
        named = dict(zip(msg.name, msg.position))
        if not all(k in named for k in JOINTS):
            return
        values = [float(named[k]) for k in JOINTS]
        if not all(math.isfinite(v) for v in values):
            return
        now_ns = node.get_clock().now().nanoseconds
        stamp_ns = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
        if stamp_ns == 0 or abs(now_ns-stamp_ns) > 2_000_000_000:
            return
        samples.append((time.monotonic(), values))
        del samples[:-12]

    sub = node.create_subscription(JointState, '/joint_states', on_joint,
                                   qos_profile_sensor_data)
    try:
        deadline = time.monotonic()+args.timeout
        stable = None
        tool0_tf = None
        tcp_tf = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.15)
            try:
                tool0_tf = buffer.lookup_transform(args.base_frame, args.tool0_frame, Time())
                if args.tcp_frame:
                    tcp_tf = buffer.lookup_transform(reference_frame, args.tcp_frame, Time())
            except Exception:
                continue
            recent = [(ts, q) for ts, q in samples if time.monotonic()-ts < 1.0]
            if len(recent) < 6:
                continue
            # A short window must be stationary before recording.
            window = [q for _, q in recent[-6:]]
            spread = max(max(q[j] for q in window)-min(q[j] for q in window)
                         for j in range(len(JOINTS)))
            if spread <= args.stable_spread_rad:
                stable = window[-1]
                break
        if stable is None or tool0_tf is None:
            print('FAIL: no stable, fresh joints and connected base->tool0 TF; no snapshot saved.')
            return 1
        measured_tool = pose_from_transform(tool0_tf)
        result = {
            'status': 'UNAPPROVED_CAPTURE_ONLY',
            'captured_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
            'robot_joint_names': list(JOINTS),
            'measured_joint_positions_rad': stable,
            'measured_ros_base_link_to_tool0': measured_tool,
            'requested_frame': reference_frame,
            'requested_in_pendant_reference_frame_to_active_tcp': expected_pose,
            'requested_rotation_vector_rad': rotvec,
            'active_tcp_name_at_capture': cfg.get('active_tcp_name_at_capture'),
            'operator_confirmed_base_frame_and_active_tcp': False,
            'tcp_frame_transform_verified': False,
            'planning_scene_verified': False,
            'robot_specific_kinematics_verified': False,
            'approved_for_automatic_home_motion': False,
            'notes': 'Read-only capture. Do not feed directly to a real motion node.',
        }
        print('Recorded joint state (radians, UR ordering):')
        for name, value in zip(JOINTS, stable):
            print('  %-22s %.8f' % (name, value))
        print('Measured base_link -> tool0:', measured_tool)
        if tcp_tf is not None:
            actual = pose_from_transform(tcp_tf)
            distance = math.dist(actual['position_m'], xyz)
            ang = angular_error(actual['quaternion_xyzw'], expected_quat)
            result['observed_active_tcp_frame'] = args.tcp_frame
            result['observed_requestedframe_to_active_tcp'] = actual
            result['tcp_position_error_m'] = distance
            result['tcp_angle_error_rad'] = ang
            print('Observed TCP discrepancy in %s frame: %.3f mm / %.3f deg' %
                  (reference_frame, distance*1000, math.degrees(ang)))
            print('WARNING: this is geometric evidence only; no approval is implied.')
        else:
            print('NOT VERIFIED: base->tool0 is not necessarily the active TCP shown on pendant.')
            print('Compare the pendant actual TCP (Base frame) with the requested pose,')
            print('then measure/verify tool0->active_tcp before enabling ANY automatic home move.')
        if args.output:
            output = Path(args.output)
            if output.exists():
                raise FileExistsError('Refusing to overwrite existing capture: ' + str(output))
            output.parent.mkdir(parents=True, exist_ok=True)
            # Avoid partially written YAML on interruption.
            with tempfile.NamedTemporaryFile('w', dir=output.parent, delete=False,
                                            prefix='.home_tmp_', encoding='utf-8') as f:
                tmp = Path(f.name)
                yaml.safe_dump(result, f, sort_keys=False)
            try:
                os.replace(tmp, output)
            finally:
                tmp.unlink(missing_ok=True)
            print('PROVISIONAL snapshot written:', output)
        else:
            print('No file written; provide --output to preserve this snapshot.')
        print('No motion commands issued. ALL execution approvals remain false.')
        return 0
    finally:
        node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
"""Read-only preflight for the fake dental MVP. Does not send robot goals.

Checks the *running ROS graph* after mvp_fake.launch.py has started.
Passing this test is a prerequisite, not evidence that any trajectory is safe.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from dental_interfaces.action import RunCycle, InspectTeeth, TreatTarget, MoveToStation
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=20.0)
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error('--timeout must be positive')
    rclpy.init()
    node = Node('dental_fake_preflight')
    tf = Buffer()
    listener = TransformListener(tf, node)
    readiness = {'ready': False}
    node.create_subscription(
        Bool, '/dental/workcell/ready',
        lambda msg: readiness.update(ready=bool(msg.data)),
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
    # ROS 2 Humble's rclpy Node does not implement get_action_names_and_types().
    # Typed ActionClients verify actual server availability without sending a goal.
    action_clients = {
        '/dental/supervisor/run_cycle': ActionClient(
            node, RunCycle, '/dental/supervisor/run_cycle'),
        '/dental/perception/inspect': ActionClient(
            node, InspectTeeth, '/dental/perception/inspect'),
        '/dental/motion/treat_target': ActionClient(
            node, TreatTarget, '/dental/motion/treat_target'),
        '/dental/motion/move_to_station': ActionClient(
            node, MoveToStation, '/dental/motion/move_to_station'),
    }
    required_tf = [
        ('base_link', 'tool0'),
        ('base_link', 'dental_model'),
        ('base_link', 'tooth_1'),
        ('base_link', 'tooth_2'),
        ('base_link', 'tooth_3'),
        ('base_link', 'tooth_4'),
        ('tool0', 'rgbd_color_optical_frame'),
    ]
    try:
        deadline = time.monotonic() + args.timeout
        absent = sorted(action_clients)
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            absent = sorted(name for name, client in action_clients.items()
                            if not client.server_is_ready())
            disconnected = []
            for parent, child in required_tf:
                try:
                    tf.lookup_transform(parent, child, Time())
                except Exception:
                    disconnected.append(f'{parent} -> {child}')
            if readiness['ready'] and not absent and not disconnected:
                print('PASS: workcell ready; four typed action servers and seven TF paths present.')
                print('Simulation readiness verified. This DOES NOT prove motion safety or real calibration.')
                return 0
        print('FAIL: fake MVP graph is incomplete; do not run the cycle.', file=sys.stderr)
        print('workcell ready:', readiness['ready'], file=sys.stderr)
        print('missing actions:', absent, file=sys.stderr)
        print('missing TF:', disconnected, file=sys.stderr)
        return 1
    finally:
        for client in action_clients.values():
            client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())

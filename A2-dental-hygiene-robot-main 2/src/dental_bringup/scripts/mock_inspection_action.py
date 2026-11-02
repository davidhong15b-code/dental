#!/usr/bin/env python3
"""EXPLICITLY SYNTHETIC InspectTeeth adapter for integration testing ONLY.

Matches the teammate's public InspectTeeth action + findings topic contract.
Produces an artificial target near the CURRENT tool0 in base_link: not a camera
measurement, not a cleaning detector, never launch on physical UR5e.
"""
import argparse
import sys
import math
import threading
import rclpy
from rclpy.utilities import remove_ros_args
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from dental_interfaces.action import InspectTeeth
from dental_interfaces.msg import DentalFinding, DentalFindingArray

class MockInspector(Node):
    def __init__(self, clear_after):
        super().__init__('diagnostic_mock_inspector')
        self._lock = threading.Lock()
        self._n = 0
        self._clear_after = clear_after
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self._pub = self.create_publisher(DentalFindingArray, '/dental/perception/findings', 10)
        self._server = ActionServer(
            self, InspectTeeth, '/dental/perception/inspect',
            execute_callback=self.on_inspect,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.get_logger().warning('SYNTHETIC INSPECTION ONLY. Not camera-derived. Do not use with real robot.')

    def on_inspect(self, handle):
        with self._lock:
            self._n += 1
            n = self._n
        array = DentalFindingArray()
        array.header.frame_id = 'base_link'
        array.header.stamp = self.get_clock().now().to_msg()
        if self._clear_after < 0 or n <= self._clear_after:
            try:
                pose = self._tf.lookup_transform('base_link', 'tool0', Time())
                xyz = pose.transform.translation
                if not all(math.isfinite(x) for x in (xyz.x, xyz.y, xyz.z)):
                    raise ValueError('nonfinite tool0 transform')
            except Exception as exc:
                handle.abort()
                result = InspectTeeth.Result()
                result.success = False
                result.message = f'MOCK only: TF base_link <- tool0 unavailable: {exc}'
                return result
            issue = DentalFinding()
            issue.id = 1000 + n
            issue.issue_type = DentalFinding.ISSUE_PLAQUE
            issue.tooth_id = 'SYNTHETIC_NOT_A_TOOTH'
            issue.confidence = 0.95
            issue.position.x = xyz.x
            issue.position.y = xyz.y
            issue.position.z = xyz.z - 0.150
            issue.surface_normal.x = 0.0
            issue.surface_normal.y = 0.0
            issue.surface_normal.z = 1.0
            array.findings = [issue]
        # Contract: publish exact same stamped findings returned by InspectTeeth.
        self._pub.publish(array)
        result = InspectTeeth.Result()
        result.success = True
        result.message = f'SYNTHETIC scan #{n}: {len(array.findings)} targets; not real CV'
        result.findings = array
        if handle.is_cancel_requested:
            handle.canceled()
            result.success = False
            result.message = 'Canceled'
        else:
            handle.succeed()
        self.get_logger().warning(result.message)
        return result

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--clear-after', type=int, default=-1,
                        help='If >=0, return empty array on scan N+1 (test fixture only)')
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    rclpy.init()
    node = MockInspector(args.clear_after)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

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
from pathlib import Path
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
    def __init__(self, clear_after, tooth_frame="", wrist_sample=""):
        super().__init__('diagnostic_mock_inspector')
        self._lock = threading.Lock()
        self._n = 0
        self._clear_after = clear_after
        self._tooth_frame = tooth_frame
        self._wrist_sample = wrist_sample
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self._pub = self.create_publisher(DentalFindingArray, '/dental/perception/findings', 10)
        self._server = ActionServer(
            self, InspectTeeth, '/dental/perception/inspect',
            execute_callback=self.on_inspect,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.get_logger().warning('SIMULATION-ONLY: tooth pose is synthetic; RGB sample dark-region classifier is real. NEVER use on UR hardware.')

    def on_inspect(self, handle):
        with self._lock:
            self._n += 1
            n = self._n
        array = DentalFindingArray()
        array.header.frame_id = 'base_link'
        array.header.stamp = self.get_clock().now().to_msg()
        detected = True
        if self._wrist_sample:
            try:
                import cv2
                import numpy as np
                image = cv2.imread(self._wrist_sample)
                if image is None:
                    raise ValueError('Missing wrist sample')
                dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
                if hasattr(cv2.aruco, 'ArucoDetector'):
                    corners, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(image)
                else:
                    corners, ids, _ = cv2.aruco.detectMarkers(image, dictionary)
                centres = {} if ids is None else {int(i): c[0].mean(axis=0)
                         for c, i in zip(corners, ids.reshape(-1))}
                if not all(i in centres for i in range(4)):
                    raise ValueError('ArUco IDs 0-3 not all visible')
                H = cv2.getPerspectiveTransform(np.float32([centres[i] for i in (2,0,1,3)]),
                                                 np.float32([[0,0],[640,0],[640,480],[0,480]]))
                crop = cv2.warpPerspective(image, H, (640,480))
                gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
                dark = cv2.threshold(gray, 95, 255, cv2.THRESH_BINARY_INV)[1]
                mask = np.zeros_like(dark)
                mask[90:390,235:405] = 255
                dark = cv2.bitwise_and(dark,mask)
                dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN,
                                        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3)))
                components, _, stats, _ = cv2.connectedComponentsWithStats(dark, 8)
                detected = any(30 <= stats[i,cv2.CC_STAT_AREA] <= 12000
                               for i in range(1,components))
            except Exception as exc:
                handle.abort()
                result = InspectTeeth.Result()
                result.success = False
                result.message = f'SIMULATED CV input invalid: {exc}'
                return result
        if detected and (self._clear_after < 0 or n <= self._clear_after):
            try:
                pose = self._tf.lookup_transform('base_link',
                    self._tooth_frame or 'tool0', Time())
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
            issue.issue_type = DentalFinding.ISSUE_STAIN
            issue.tooth_id = self._tooth_frame or 'SYNTHETIC_NOT_A_TOOTH'
            issue.confidence = 0.95
            issue.position.x = xyz.x
            issue.position.y = xyz.y
            issue.position.z = xyz.z if self._tooth_frame else xyz.z - 0.150
            issue.surface_normal.x = 0.0
            issue.surface_normal.y = 0.0
            issue.surface_normal.z = 1.0
            array.findings = [issue]
        # Contract: publish exact same stamped findings returned by InspectTeeth.
        self._pub.publish(array)
        result = InspectTeeth.Result()
        result.success = True
        result.message = f'SIMULATED scan #{n}: {len(array.findings)} stains, 2D photo classification + synthetic 3D tooth centre'
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
    parser.add_argument('--tooth-frame', default='', help='Synthetic marker-to-tooth TF (FAKE ONLY)')
    parser.add_argument('--wrist-sample', default='', help='2D photo used for real dark-mark classification (FAKE ONLY)')
    parser.add_argument('--clear-after', type=int, default=-1,
                        help='If >=0, return empty array on scan N+1 (test fixture only)')
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    rclpy.init()
    node = MockInspector(args.clear_after, args.tooth_frame, args.wrist_sample)
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

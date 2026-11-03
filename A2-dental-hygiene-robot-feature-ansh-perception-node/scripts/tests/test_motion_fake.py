#!/usr/bin/env python3
"""Diagnostic-only TreatTarget action client for Increment 3.

Generates an explicitly SYNTHETIC target near the reported fake UR5e tool0
pose. Does not publish perception findings or assert that cleaning occurred.
The action server owns execution; its default is plan-only. Before enabling
execution, independently verify the UR driver is in fake-hardware mode.

Usage (after sourcing the workspace and launching MoveIt + motion_node):
    python3 scripts/tests/test_motion_fake.py
    python3 scripts/tests/test_motion_fake.py --expect-fake-execution
"""

import argparse
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from dental_interfaces.action import TreatTarget
from dental_interfaces.msg import DentalFinding, DentalTarget


def wait_future(future, timeout):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.05)
    if not future.done():
        raise TimeoutError("Timed out waiting for ROS action response")
    return future.result()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--expect-fake-execution", action="store_true",
        help="Check that EXECUTED_RETRACT feedback was received",
    )
    args = parser.parse_args()

    rclpy.init()
    node = Node("motion_increment3_diagnostic")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    client = ActionClient(node, TreatTarget, "/dental/motion/treat_target")
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        # The robot_state_publisher must provide the live tool0 frame.
        transform = buffer.lookup_transform(
            "base_link", "tool0", Time(), timeout=Duration(seconds=10.0)
        )
        translation = transform.transform.translation
        print(
            "Current tool0 base-frame position: "
            f"({translation.x:.4f}, {translation.y:.4f}, {translation.z:.4f}) m"
        )
        goal = TreatTarget.Goal()
        goal.tool_id = "brush"  # label only: no actual tool actuation
        goal.target = DentalTarget()
        goal.target.header.frame_id = "base_link"
        goal.target.header.stamp = node.get_clock().now().to_msg()
        goal.target.finding = DentalFinding()
        goal.target.finding.id = 9001
        goal.target.finding.issue_type = DentalFinding.ISSUE_PLAQUE
        goal.target.finding.tooth_id = "synthetic_increment3_test"
        goal.target.finding.confidence = 0.95
        # Diagnostic-only target, intentionally inward from the near-extended
        # default fake UR5e pose. With +Z normal and the motion node defaults
        # (preapproach=0.10m, hover=0.06m), this gives:
        #   preapproach = current tool0 Z - 0.050m
        #   hover       = current tool0 Z - 0.090m
        # This is NOT a measured dental target and does not guarantee IK.
        goal.target.finding.position.x = translation.x
        goal.target.finding.position.y = translation.y
        goal.target.finding.position.z = translation.z - 0.150
        goal.target.finding.surface_normal.x = 0.0
        goal.target.finding.surface_normal.y = 0.0
        goal.target.finding.surface_normal.z = 1.0
        print(
            "Diagnostic Z targets with default offsets (m): "
            f"preapproach={goal.target.finding.position.z + 0.10:.4f}, "
            f"hover={goal.target.finding.position.z + 0.06:.4f}, "
            f"target={goal.target.finding.position.z:.4f}"
        )

        if not client.wait_for_server(timeout_sec=10):
            raise RuntimeError("TreatTarget server not available")
        phases = []

        def feedback(msg):
            phase = msg.feedback.phase
            phases.append(phase)
            print("Feedback:", phase)

        handle = wait_future(client.send_goal_async(goal, feedback_callback=feedback), 15)
        if not handle.accepted:
            raise RuntimeError("Diagnostic goal rejected; inspect motion_node logs")
        result_future = handle.get_result_async()
        try:
            answer = wait_future(result_future, 120)
        except TimeoutError as exc:
            # Never silently abandon an accepted robot-motion goal. Request
            # cancellation and report whether the server acknowledged it.
            print("No result after 120s. Requesting TreatTarget cancellation...")
            try:
                response = wait_future(handle.cancel_goal_async(), 10)
                print("Cancellation acknowledged:", bool(response.goals_canceling))
            except Exception as cancel_exc:
                print("Cancellation acknowledgment unavailable:", cancel_exc)
            print("WARNING: verify the controller is idle before restarting any motion test.")
            raise RuntimeError(
                "TreatTarget timed out during execution; inspect motion_node and "
                "move_group logs and the controller's actual vs desired state"
            ) from exc
        print("Action status:", answer.status, "success:", answer.result.success)
        print("Message:", answer.result.message)
        if answer.result.success:
            raise RuntimeError("Incorrect: positioning-only action must not claim cleaning succeeded")
        if answer.status != GoalStatus.STATUS_ABORTED:
            raise RuntimeError(f"Expected ABORTED (incomplete treatment), got {answer.status}")
        required = (
            ["EXECUTED_RETRACT"] if args.expect_fake_execution else ["PLANNED_RETRACT"]
        )
        if not all(item in phases for item in required):
            raise RuntimeError(f"Expected phase {required}, received {phases}")
        print("PASS: motion planning/positioning demo completed; no cleaning was claimed")
    finally:
        executor.shutdown()
        spinner.join(timeout=3)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

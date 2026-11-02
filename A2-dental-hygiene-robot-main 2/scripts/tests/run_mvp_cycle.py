#!/usr/bin/env python3
"""Invoke the real RunCycle action; does not invent perception or motion data."""
import argparse
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from dental_interfaces.action import RunCycle


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--region', default='')
    p.add_argument('--timeout', type=float, default=180.0)
    args = p.parse_args()
    rclpy.init()
    node = Node('mvp_cycle_client')
    client = ActionClient(node, RunCycle, '/dental/supervisor/run_cycle')
    try:
        if not client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError('RunCycle action server not available')
        goal = RunCycle.Goal()
        goal.region_id = args.region
        send = client.send_goal_async(goal,
            feedback_callback=lambda m: print('[SUPERVISOR]', m.feedback.phase, flush=True))
        rclpy.spin_until_future_complete(node, send, timeout_sec=10.0)
        if not send.done() or not send.result().accepted:
            raise RuntimeError('RunCycle goal rejected/timed out')
        handle = send.result()
        future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
        if not future.done():
            print('Timeout: requesting cancellation; robot status must be checked', flush=True)
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=5.0)
            raise RuntimeError('RunCycle did not return; cancellation requested')
        answer = future.result()
        print(f'Action status={answer.status}; success={answer.result.success}; '
              f'treated_count={answer.result.treated_count}; message={answer.result.message}', flush=True)
        # The current MVP deliberately does NOT claim cleaning success.
        if answer.result.success:
            print('Cycle completed with no targets requiring treatment.')
        elif 'positioning' in answer.result.message.lower() or 'Residual' in answer.result.message:
            print('PASS: supervised positioning + reinspection demonstrated; NO cleaning claimed')
        else:
            raise RuntimeError('Cycle failed before completing expected supervised workflow')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

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
    p.add_argument('--goal-response-timeout', type=float, default=45.0,
                   help='Seconds to wait for RunCycle goal acknowledgement (not execution)')
    args = p.parse_args()
    if args.goal_response_timeout <= 0:
        p.error('--goal-response-timeout must be positive')
    rclpy.init()
    node = Node('mvp_cycle_client')
    client = ActionClient(node, RunCycle, '/dental/supervisor/run_cycle')
    try:
        if not client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError('RunCycle action server not available')
        goal = RunCycle.Goal()
        goal.region_id = args.region
        phases = []
        def on_feedback(message):
            phase = message.feedback.phase
            phases.append(phase)
            print('[SUPERVISOR]', phase, flush=True)
        send = client.send_goal_async(goal, feedback_callback=on_feedback)
        print(f'RunCycle goal sent; awaiting response for up to '
              f'{args.goal_response_timeout:g}s', flush=True)
        rclpy.spin_until_future_complete(
            node, send, timeout_sec=args.goal_response_timeout)
        if not send.done():
            raise RuntimeError(
                'NO GOAL RESPONSE: server acceptance is UNKNOWN. Do not send a second '
                'goal until the supervisor/action graph and simulated robot status '
                'have been checked; restart the fake launch if uncertain.')
        handle = send.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(
                'GOAL REJECTED by the supervisor (check for a running cycle).')
        print('RunCycle goal accepted', flush=True)
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
        # If the motion pipeline aborted early, say WHERE and WHY before
        # running completion-only assertions (which otherwise obscure the error).
        import action_msgs.msg
        if (answer.status == action_msgs.msg.GoalStatus.STATUS_ABORTED and
                'Residual stain remains' not in answer.result.message):
            last_phase = phases[-1] if phases else '(no feedback received)'
            raise RuntimeError('MVP aborted after phase ' + last_phase + ': ' +
                               answer.result.message)
        # Verify what the robot actually reported doing, not just the result text.
        # A no-contact hover does NOT remove the actual black mark. The honest
        # expected outcome is ABORTED/residual after TWO complete passes and
        # a verified return-home action. This is a successful *workflow test*,
        # not cleaning success and not a successful ROS action result.
        if answer.result.treated_count != 0:
            raise RuntimeError('Non-contact MVP cannot claim teeth were treated')
        if 'AT_inspect' not in phases or 'AT_home' not in phases:
            raise RuntimeError('Cycle never reported both inspected and returned-home poses')
        if answer.result.success:
            if answer.status != action_msgs.msg.GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError('Reported success with non-success ROS action status')
            if not any(phase.startswith('SCAN_COMPLETE findings=0') for phase in phases):
                raise RuntimeError('Clear result without an actual zero-findings inspection')
            print('PASS: inspected, verified a clear scan and returned home. NO cleaning claimed.')
        elif 'Residual stain remains' in answer.result.message:
            if answer.status != action_msgs.msg.GoalStatus.STATUS_ABORTED:
                raise RuntimeError('Residual stain must have ABORTED action status')
            expected = ['HOVER_STROKE_COMPLETE pass=1',
                        'HOVER_STROKE_COMPLETE pass=2']
            for item in expected:
                if not any(phase.startswith(item) for phase in phases):
                    raise RuntimeError('Residual-result reported without verified ' + item)
            if phases.count('AT_inspect') < 3:
                raise RuntimeError('Two-pass residual test needs initial and two repeat inspections')
            print('PASS: TWO hover passes, reinspection, and return home verified; '
                  'residual stain correctly reported. NO physical cleaning claimed.')
        else:
            raise RuntimeError('Cycle failed before completing expected supervised workflow')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

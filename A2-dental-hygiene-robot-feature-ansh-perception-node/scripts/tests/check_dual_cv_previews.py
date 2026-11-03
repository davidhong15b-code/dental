#!/usr/bin/env python3
"""Read-only health check for the two annotated CV image feeds used by RViz.

Only verifies image transport and timestamps, NOT physical calibration,
localization accuracy, depth alignment, robot safety, or tool clearance.
"""
import argparse
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

TOPICS = {
    'overhead': '/dental/perception/overhead_preview',
    'wrist_rgbd': '/dental/perception/rgbd_preview',
}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--timeout', type=float, default=20.0)
    ap.add_argument('--min-frames', type=int, default=2)
    ap.add_argument('--max-age-sec', type=float, default=3.0)
    ap.add_argument('--require-localization', action='store_true',
                    help='Also require a recent /dental/perception/model_localized=true')
    args = ap.parse_args()
    if args.timeout <= 0 or args.min_frames < 1 or args.max_age_sec <= 0:
        ap.error('timeout, min-frames and max-age-sec must be positive')
    rclpy.init()
    node = Node('dental_dual_cv_preview_checker')
    images = {name: [] for name in TOPICS}
    localization = {'valid': False, 'monotonic': 0.}
    subscriptions = []

    def cb(name):
        def callback(msg):
            if msg.width <= 0 or msg.height <= 0 or not msg.data:
                return
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            clock = node.get_clock().now().nanoseconds*1e-9
            age = abs(clock-stamp)
            # An image with an old stamp must not count as live.
            if stamp <= 0 or age > args.max_age_sec:
                return
            images[name].append((time.monotonic(), msg.width, msg.height,
                                 msg.encoding, msg.header.frame_id, age))
            del images[name][:-100]
        return callback

    for name, topic in TOPICS.items():
        subscriptions.append(node.create_subscription(Image, topic, cb(name),
                                                     qos_profile_sensor_data))

    if args.require_localization:
        def localized(msg):
            localization.update(valid=bool(msg.data), monotonic=time.monotonic())
        subscriptions.append(node.create_subscription(
            Bool, '/dental/perception/model_localized', localized,
            qos_profile_sensor_data))
    try:
        deadline = time.monotonic()+args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            ok_images = all(len(history) >= args.min_frames and
                            time.monotonic()-history[-1][0] <= args.max_age_sec
                            for history in images.values())
            ok_model = not args.require_localization or (
                localization['valid'] and
                time.monotonic()-localization['monotonic'] <= args.max_age_sec)
            if ok_images and ok_model:
                break
        ok = True
        for name, history in images.items():
            if history:
                last = history[-1]
                # Rate is only a small-window estimate; don't infer camera FPS.
                rate = ((len(history)-1)/(history[-1][0]-history[0][0])
                        if len(history)>1 and history[-1][0]>history[0][0] else 0.)
                print('%s: %d frames, latest %dx%d %s, frame=%r, age=%.2fs, preview≈%.1f Hz' %
                      (name, len(history), last[1], last[2], last[3], last[4], last[5],rate))
            else:
                print(name + ': NO FRESH IMAGES')
            ok &= len(history) >= args.min_frames and bool(history) and (
                time.monotonic()-history[-1][0] <= args.max_age_sec)
        if args.require_localization:
            print('calibrated model localized (current status):', localization['valid'])
            ok &= localization['valid'] and (
                time.monotonic()-localization['monotonic'] <= args.max_age_sec)
        print('PASS: fresh RGB previews from both CV nodes' if ok else
              'FAIL: one or both CV preview feeds not currently usable')
        print('This does not validate TF calibration or safe motion.')
        return 0 if ok else 1
    finally:
        for sub in subscriptions:
            node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())

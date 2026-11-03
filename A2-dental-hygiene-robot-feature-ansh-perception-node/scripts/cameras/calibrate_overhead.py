#!/usr/bin/env python3
"""One-time, direct-V4L2 checkerboard calibration; no ROS camera driver.

This helper never launches or commands the robot. Requires python3-opencv.
Captures only on SPACE so each image is a deliberate, distinct board pose.
"""
import argparse
import time
from pathlib import Path
import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--device', default='/dev/dental_overhead')
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--columns', type=int, default=9, help='checkerboard INTERNAL corners, columns')
    ap.add_argument('--rows', type=int, default=6, help='checkerboard INTERNAL corners, rows')
    ap.add_argument('--square-mm', type=float, required=True, help='measured square side length (mm)')
    ap.add_argument('--samples', type=int, default=25)
    ap.add_argument('--output', default='overhead_intrinsics.txt')
    args = ap.parse_args()
    if args.width <= 0 or args.height <= 0 or args.columns < 3 or args.rows < 3 or args.square_mm <= 0 or args.samples < 10:
        ap.error('Invalid geometry; provide at least ten checkerboard captures')
    device = args.device
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        ap.error(f'Cannot open {device}; check udev link, permissions and other processes')
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    got_width, got_height = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if (got_width, got_height) != (args.width, args.height):
        cap.release()
        ap.error(f'Video mode negotiated {got_width}x{got_height}, not {args.width}x{args.height}')
    pattern = (args.columns, args.rows)
    obj = np.zeros((args.columns * args.rows, 3), np.float32)
    obj[:, :2] = np.mgrid[0:args.columns, 0:args.rows].T.reshape(-1, 2) * (args.square_mm / 1000.0)
    objects, images = [], []
    window = 'Overhead checkerboard | SPACE=capture | Q=quit'
    print('Capture diverse board locations, orientations and tilts. Avoid nearly identical images.')
    print(f'Looking for {args.columns}x{args.rows} INTERNAL corners; SPACE captures a pose.')
    try:
        while len(images) < args.samples:
            success, frame = cap.read()
            if not success or frame is None:
                print('Unable to acquire frame; check camera.'); break
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, pattern,
                cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
            display = frame.copy()
            refined = None
            if found:
                refined = cv2.cornerSubPix(gray, corners, (7,7), (-1,-1),
                    (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                cv2.drawChessboardCorners(display, pattern, refined, found)
            cv2.putText(display, f'Samples {len(images)}/{args.samples} | board={"FOUND" if found else "not found"}',
                        (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (30,220,30) if found else (0,0,255), 2)
            cv2.imshow(window, display)
            k = cv2.waitKey(1) & 0xff
            if k in (ord('q'), 27): break
            if k == 32 and found:
                objects.append(obj.copy()); images.append(refined.copy())
                print(f'Captured pose {len(images)}/{args.samples}')
                time.sleep(0.2)
    finally:
        cap.release(); cv2.destroyAllWindows()
    if len(images) < 10:
        raise SystemExit(f'Only {len(images)} captures. Need >=10; nothing written.')
    rms, K, distortion, rv, tv = cv2.calibrateCamera(objects, images,
        (args.width, args.height), None, None)
    if not np.isfinite(K).all() or not np.isfinite(distortion).all() or rms > 1.5:
        raise SystemExit(f'Calibration RMS {rms:.3f}px too high or invalid. Re-capture better poses.')
    coeffs = distortion.reshape(-1).tolist()
    matrix = K.reshape(-1).tolist()
    result = (f'# Resolution: {args.width}x{args.height}, RMS {rms:.4f} px, {len(images)} samples\n'
              f'overhead_camera_matrix: {matrix}\n'
              f'overhead_distortion: {coeffs}\n'
              'overhead_intrinsics_calibrated: true\n'
              '# Copy the above fields into model_localizer.ros__parameters in src/dental_perception/config/perception.yaml.\n'
              '# Verify marker sizes and base-to-overhead TF separately.\n')
    output = Path(args.output)
    if output.exists():
        raise SystemExit(f'Refusing to overwrite {output}; use a different --output')
    output.write_text(result)
    print(result)
    print(f'Saved {output}. Confirm calibration and copy values to src/dental_perception/config/perception.yaml.')


if __name__ == '__main__':
    main()

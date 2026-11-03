#!/usr/bin/env python3
"""Pure OpenCV ArUco/PnP smoke test, NOT a ROS camera or robot test.

Draws 4 synthetic ArUco tags, projects CAD-corner coordinates with known
camera intrinsics and checks PnP finds a comparable metric jaw pose.
This test is isolated from runtime perception and never publishes ROS targets.
"""
import math
import numpy as np
import cv2


def main():
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("This test needs opencv-contrib (cv2.aruco)")
    ar = cv2.aruco
    dictionary = ar.getPredefinedDictionary(ar.DICT_4X4_50)
    marker_size_m = 0.018
    centers = {
        0: (-0.060, 0.040, 0.0),
        1: (0.060, 0.040, 0.0),
        2: (0.060, -0.040, 0.0),
        3: (-0.060, -0.040, 0.0),
    }
    f, z, cx, cy = 1200.0, 0.40, 512., 384.
    pixels_per_m = f / z
    size_px = round(marker_size_m * pixels_per_m)
    img = np.full((768, 1024), 255, dtype=np.uint8)
    for marker_id, (x, y, _) in centers.items():
        # model +X projects to image right; model +Y projects to image up
        u, v = round(cx + x*pixels_per_m), round(cy - y*pixels_per_m)
        if hasattr(ar, "generateImageMarker"):
            marker = ar.generateImageMarker(dictionary, marker_id, size_px)
        else:
            marker = np.zeros((size_px, size_px), dtype=np.uint8)
            ar.drawMarker(dictionary, marker_id, size_px, marker)
        img[v-size_px//2:v-size_px//2+size_px,
            u-size_px//2:u-size_px//2+size_px] = marker

    if hasattr(ar, "ArucoDetector"):
        corners, ids, _ = ar.ArucoDetector(dictionary).detectMarkers(img)
    else:
        corners, ids, _ = ar.detectMarkers(img, dictionary)
    assert ids is not None and set(ids.flatten()) == set(range(4)), ids
    corners_by_id = {int(i): c[0] for i, c in zip(ids.flatten(), corners)}
    model_corners = []
    image_corners = []
    half = marker_size_m/2
    for marker_id in range(4):
        x, y, z_model = centers[marker_id]
        model_corners.extend([
            [x-half, y+half, z_model], [x+half, y+half, z_model],
            [x+half, y-half, z_model], [x-half, y-half, z_model]
        ])
        image_corners.extend(corners_by_id[marker_id])
    K = np.array([[f, 0, cx], [0, f, cy], [0, 0, 1]], np.float64)
    object_pts = np.asarray(model_corners, np.float32)
    image_pts = np.asarray(image_corners, np.float32)
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        object_pts, image_pts, K, np.zeros(5),
        flags=cv2.SOLVEPNP_ITERATIVE, reprojectionError=2.5, iterationsCount=100)
    assert ok and len(inliers) >= 12
    R, _ = cv2.Rodrigues(rvec)
    assert tvec[2,0] > 0
    assert abs(tvec[2,0]-z) < 0.015, tvec
    # Tooth +Z points towards overhead camera (-camera Z), not through the table.
    assert R[2,2] < -0.9, R
    expected_tooth_x = [-0.021,-0.007,0.007,0.021]
    # Missing required tag should be handled as unusable localisation by the runtime policy.
    missing = img.copy()
    marker_id = 2
    x, y, _ = centers[marker_id]
    u, v = round(cx + x*pixels_per_m), round(cy - y*pixels_per_m)
    missing[v-size_px//2:v-size_px//2+size_px,
            u-size_px//2:u-size_px//2+size_px] = 255
    if hasattr(ar, "ArucoDetector"):
        _, ids_missing, _ = ar.ArucoDetector(dictionary).detectMarkers(missing)
    else:
        _, ids_missing, _ = ar.detectMarkers(missing, dictionary)
    assert ids_missing is None or set(ids_missing.flatten()) != set(range(4))
    print("PASS: 4 marker IDs, PnP metric camera distance, tooth frame orientation")
    print("PASS: removing one marker invalidates four-marker detection")
    print("Measured distance %.4f m; teeth CAD x coords: %s" % (tvec[2,0], expected_tooth_x))


if __name__ == "__main__":
    main()

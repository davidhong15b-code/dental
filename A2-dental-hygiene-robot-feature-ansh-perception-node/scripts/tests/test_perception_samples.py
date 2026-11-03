#!/usr/bin/env python3
"""Offline sample-image smoke test; no ROS or robot required.

Checks marker IDs in all 4 photos and one dark candidate in each centred
jaw crop, using the same projective ROI and threshold as perception_node.cpp.
This validates CV diagnostics only, NOT metric poses or real robot readiness.
"""
from pathlib import Path
import argparse
import cv2
import numpy as np


SAMPLES = ("overhead_01.png", "overhead_02.png", "wrist_01.png", "wrist_02.png")


def inspect(path: Path, threshold: int, minimum_area: int, output_dir: Path | None):
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"Unable to read {path}")
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    # Ubuntu 22.04's OpenCV 4.5 has detectMarkers; newer OpenCV has ArucoDetector.
    if hasattr(cv2.aruco, "detectMarkers"):
        corners, ids, _ = cv2.aruco.detectMarkers(image, dictionary)
    else:
        corners, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(image)
    found = {} if ids is None else {
        int(marker_id): polygon[0].mean(axis=0)
        for polygon, marker_id in zip(corners, ids.reshape(-1))
    }
    expected = {0, 1, 2, 3}
    if not expected.issubset(found):
        raise AssertionError(f"{path.name}: expected 0,1,2,3; detected {sorted(found)}")

    source = np.float32([found[i] for i in (2, 0, 1, 3)])
    destination = np.float32([[0, 0], [640, 0], [640, 480], [0, 480]])
    H = cv2.getPerspectiveTransform(source, destination)
    flattened = cv2.warpPerspective(image, H, (640, 480))
    gray = cv2.cvtColor(flattened, cv2.COLOR_BGR2GRAY)
    dark = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY_INV)[1]
    mask = np.zeros_like(dark)
    mask[90:390, 235:405] = 255
    dark = cv2.bitwise_and(dark, mask)
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    count, _, stats, centres = cv2.connectedComponentsWithStats(dark, 8)
    result = [(int(stats[i, cv2.CC_STAT_AREA]), centres[i])
              for i in range(1, count)
              if minimum_area <= stats[i, cv2.CC_STAT_AREA] <= 12000]

    if output_dir is not None:
        annotated = image.copy()
        cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
        for _, centre in result:
            projected = cv2.perspectiveTransform(
                np.array([[centre]], dtype=np.float32), np.linalg.inv(H))[0][0]
            cv2.circle(annotated, tuple(np.round(projected).astype(int)), 14, (0, 0, 255), 3)
        output_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(output_dir / path.name), annotated)

    if not result:
        raise AssertionError(f"{path.name}: no dark candidate in central jaw ROI")
    print(f"PASS {path.name}: markers={sorted(found)}, 2D dark candidates={len(result)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=None,
                        help="optional directory for annotated test images")
    args = parser.parse_args()
    fixtures = Path(__file__).resolve().parents[2] / "src/dental_perception/samples"
    for sample in SAMPLES:
        inspect(fixtures / sample, 95, 30, args.output_dir)
    print("All sample-image CV tests passed. Metric pose and registered depth NOT tested.")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Stereo calibration from ChArUco image pairs.

Usage:
    python stereo_calibrate.py [image_dir] [output_yaml]

Defaults:
    image_dir  = data/calibration_images
    output     = config/calibration.yaml
"""

import cv2
import numpy as np
import glob
import sys
import os

# ── ChArUco board parameters (must match generate_board.py) ──────
SQUARES_X = 5
SQUARES_Y = 7
SQUARE_LENGTH = 0.032   # change this if you changed ur board
MARKER_LENGTH = 0.0236   # black to black
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
BOARD = cv2.aruco.CharucoBoard(
    (SQUARES_X, SQUARES_Y), SQUARE_LENGTH, MARKER_LENGTH, ARUCO_DICT
)


def detect_charuco(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
    detector = cv2.aruco.ArucoDetector(ARUCO_DICT)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None or len(ids) < 4:
        return None, None
    _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        corners, ids, gray, BOARD
    )
    return charuco_corners, charuco_ids


def main():
    image_dir = sys.argv[1] if len(sys.argv) > 1 else "data/calibration_images"
    output_path = sys.argv[2] if len(sys.argv) > 2 else "config/calibration.yaml"

    left_paths = sorted(glob.glob(os.path.join(image_dir, "left_*.png")))
    right_paths = sorted(glob.glob(os.path.join(image_dir, "right_*.png")))
    assert len(left_paths) == len(right_paths), "Mismatched left/right counts"
    assert len(left_paths) > 0, f"No images found in {image_dir}"

    all_charuco_corners_l = []
    all_charuco_corners_r = []
    all_charuco_ids_l = []
    all_charuco_ids_r = []
    image_size = None
    used = 0

    for lp, rp in zip(left_paths, right_paths):
        img_l = cv2.imread(lp)
        img_r = cv2.imread(rp)
        if image_size is None:
            image_size = (img_l.shape[1], img_l.shape[0])

        corners_l, ids_l = detect_charuco(img_l)
        corners_r, ids_r = detect_charuco(img_r)

        if corners_l is None or corners_r is None:
            print(f"  Skipping {os.path.basename(lp)} (insufficient corners)")
            continue

        # Only keep corners detected in both images.
        common_ids = np.intersect1d(ids_l.flatten(), ids_r.flatten())
        if len(common_ids) < 6:
            print(f"  Skipping {os.path.basename(lp)} ({len(common_ids)} common corners)")
            continue

        mask_l = np.isin(ids_l.flatten(), common_ids)
        mask_r = np.isin(ids_r.flatten(), common_ids)

        all_charuco_corners_l.append(corners_l[mask_l])
        all_charuco_corners_r.append(corners_r[mask_r])
        all_charuco_ids_l.append(ids_l[mask_l])
        all_charuco_ids_r.append(ids_r[mask_r])
        used += 1
        print(f"  {os.path.basename(lp)}: {len(common_ids)} common corners")

    print(f"\nUsing {used}/{len(left_paths)} image pairs for calibration.")
    if used < 5:
        print("ERROR: need at least 5 usable pairs.")
        sys.exit(1)

    # Individual camera calibration.
    obj_pts = [BOARD.getChessboardCorners()[ids.flatten()] for ids in all_charuco_ids_l]

    ret_l, M1, D1, _, _ = cv2.calibrateCamera(
        obj_pts, all_charuco_corners_l, image_size, None, None
    )
    ret_r, M2, D2, _, _ = cv2.calibrateCamera(
        obj_pts, all_charuco_corners_r, image_size, None, None
    )
    print(f"Left  RMS reprojection error: {ret_l:.4f}")
    print(f"Right RMS reprojection error: {ret_r:.4f}")

    # Stereo calibration.
    flags = cv2.CALIB_FIX_INTRINSIC
    ret, M1, D1, M2, D2, R, T, E, F = cv2.stereoCalibrate(
        obj_pts, all_charuco_corners_l, all_charuco_corners_r,
        M1, D1, M2, D2, image_size, flags=flags
    )
    print(f"Stereo RMS reprojection error: {ret:.4f}")

    # Rectification.
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        M1, D1, M2, D2, image_size, R, T, alpha=0
    )

    focal = P1[0, 0]
    baseline = abs(P2[0, 3]) / focal
    print(f"Focal length: {focal:.1f} px")
    print(f"Baseline:     {baseline * 1000:.1f} mm")

    # Save.
    fs = cv2.FileStorage(output_path, cv2.FileStorage_WRITE)
    for name, mat in [("M1", M1), ("D1", D1), ("M2", M2), ("D2", D2),
                      ("R", R), ("T", T), ("R1", R1), ("R2", R2),
                      ("P1", P1), ("P2", P2), ("Q", Q)]:
        fs.write(name, mat)
    fs.release()
    print(f"\nCalibration saved to {output_path}")


if __name__ == "__main__":
    main()

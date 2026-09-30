#!/usr/bin/env python3
"""Stereo calibration from ChArUco image pairs.

Usage:
    python tools/stereo_calibrate.py [image_dir] [output_yaml] [board_yaml]

Defaults:
    image_dir  = data/calibration_images
    output     = config/calibration.yaml
    board_yaml = image_dir/board.yaml if it exists, otherwise config/board.yaml
"""

import cv2
import numpy as np
import glob
import sys
import os

from board import load_board

BOARD = None
DETECTOR = None


def detect_charuco(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
    charuco_corners, charuco_ids, _, marker_ids = DETECTOR.detectBoard(gray)
    if marker_ids is None or len(marker_ids) < 4 or charuco_ids is None:
        return None, None
    return charuco_corners, charuco_ids


def main():
    global BOARD, DETECTOR
    image_dir = sys.argv[1] if len(sys.argv) > 1 else "data/calibration_images"
    output_path = sys.argv[2] if len(sys.argv) > 2 else "config/calibration.yaml"
    board_path = sys.argv[3] if len(sys.argv) > 3 else os.path.join(image_dir, "board.yaml")
    if not os.path.exists(board_path):
        board_path = "config/board.yaml"
    BOARD, _, s = load_board(board_path)
    DETECTOR = cv2.aruco.CharucoDetector(BOARD)
    print(f"Board from {board_path}: {s['squares_x']} x {s['squares_y']}, "
          f"squares {s['square_mm']} mm, markers {s['marker_mm']} mm, {s['dictionary']}")

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

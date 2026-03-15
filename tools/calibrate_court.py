#!/usr/bin/env python3
"""Compute the camera-to-court 4×4 rigid transform from an ArUco marker
placed at a known position on the court.

Usage:
    python calibrate_court.py <left_image> <right_image> <calibration_yaml>

The script detects the ArUco marker, estimates its pose relative to the camera,
and writes T_camera_to_court into the calibration file.
"""

import cv2
import numpy as np
import sys

# ── Configuration ────────────────────────────────────────────────
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
MARKER_ID = 0
MARKER_SIZE = 0.20  # 20 cm physical marker side length (metres)

# Where the marker centre sits in court coordinates (metres).
# Adjust this to match where you physically place the marker.
MARKER_COURT_POSITION = np.array([3.05, 0.0, 0.0])  # centre of back line, on floor


def main():
    if len(sys.argv) < 4:
        print("Usage: calibrate_court.py <left_img> <right_img> <calibration.yaml>")
        sys.exit(1)

    left_path, right_path, cal_path = sys.argv[1], sys.argv[2], sys.argv[3]

    # Load calibration.
    fs = cv2.FileStorage(cal_path, cv2.FileStorage_READ)
    M1 = fs.getNode("M1").mat()
    D1 = fs.getNode("D1").mat()
    fs.release()

    img = cv2.imread(left_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    detector = cv2.aruco.ArucoDetector(ARUCO_DICT)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is None or MARKER_ID not in ids.flatten():
        print(f"Error: marker ID {MARKER_ID} not found in {left_path}")
        sys.exit(1)

    idx = list(ids.flatten()).index(MARKER_ID)
    marker_corners = corners[idx]

    # Estimate pose of the marker relative to the camera.
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        marker_corners.reshape(1, 4, 2), MARKER_SIZE, M1, D1
    )
    rvec = rvecs[0].flatten()
    tvec = tvecs[0].flatten()

    # Build 4×4 transform: marker frame → camera frame.
    R_marker_to_cam, _ = cv2.Rodrigues(rvec)
    T_marker_to_cam = np.eye(4)
    T_marker_to_cam[:3, :3] = R_marker_to_cam
    T_marker_to_cam[:3, 3] = tvec

    # Build 4×4 transform: court frame → marker frame.
    # Assumes the marker is flat on the floor with its Z-axis pointing up.
    T_court_to_marker = np.eye(4)
    T_court_to_marker[:3, 3] = -MARKER_COURT_POSITION

    # camera → court = inverse(marker→camera) composed with court→marker inverted,
    # but more directly: court→camera = marker→camera * court→marker,
    # so camera→court = inverse of that.
    T_court_to_cam = T_marker_to_cam @ T_court_to_marker
    T_cam_to_court = np.linalg.inv(T_court_to_cam)

    print("T_camera_to_court:")
    print(T_cam_to_court)

    # Append to calibration file.
    fs = cv2.FileStorage(cal_path, cv2.FileStorage_APPEND)
    fs.write("T_camera_to_court", T_cam_to_court)
    fs.release()
    print(f"\nSaved T_camera_to_court to {cal_path}")


if __name__ == "__main__":
    main()

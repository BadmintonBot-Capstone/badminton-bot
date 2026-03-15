#!/usr/bin/env python3
"""Generate a ChArUco calibration board image.

Usage:
    python generate_board.py [output.png]
"""

import cv2
import sys

SQUARES_X = 9
SQUARES_Y = 6
SQUARE_LENGTH = 0.030
MARKER_LENGTH = 0.023
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)

board = cv2.aruco.CharucoBoard(
    (SQUARES_X, SQUARES_Y), SQUARE_LENGTH, MARKER_LENGTH, ARUCO_DICT
)

output = sys.argv[1] if len(sys.argv) > 1 else "charuco_board.png"
img = board.generateImage((1800, 1200), marginSize=40)
cv2.imwrite(output, img)
print(f"Saved {SQUARES_X}x{SQUARES_Y} ChArUco board to {output}")

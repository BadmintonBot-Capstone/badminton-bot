"""Loads the ChArUco board description shared by generate_board.py and stereo_calibrate.py."""

import cv2
import yaml


def load_board(path="config/board.yaml"):
    """Returns (board, dictionary, settings) with the board lengths in metres."""
    with open(path) as f:
        s = yaml.safe_load(f)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, s["dictionary"]))
    board = cv2.aruco.CharucoBoard(
        (s["squares_x"], s["squares_y"]), s["square_mm"] / 1000, s["marker_mm"] / 1000, dictionary
    )
    return board, dictionary, s

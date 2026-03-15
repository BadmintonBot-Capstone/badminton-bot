#!/usr/bin/env python3
"""Capture synchronised stereo image pairs for calibration.

Requires configure_cameras.py to be run first.

Press 's' to save a pair, 'q' to quit.
"""

import PySpin
import cv2
import os
import sys
import yaml


def main():
    config_path = sys.argv[1] if len(sys.argv) > 1 else "config/camera.yaml"
    output_dir = sys.argv[2] if len(sys.argv) > 2 else "data/calibration_images"
    os.makedirs(output_dir, exist_ok=True)

    with open(config_path) as f:
        cfg = yaml.safe_load(f)

    system = PySpin.System.GetInstance()
    cam_list = system.GetCameras()

    primary_serial = cfg.get("primary_serial", "")
    if primary_serial:
        primary = cam_list.GetBySerial(primary_serial)
        secondary = cam_list.GetBySerial(cfg["secondary_serial"])
    else:
        primary = cam_list.GetByIndex(0)
        secondary = cam_list.GetByIndex(1)

    primary.Init()
    secondary.Init()

    # Start acquisition (secondary first for hardware trigger).
    secondary.BeginAcquisition()
    primary.BeginAcquisition()

    print(f"Saving to {output_dir}/  Press 's' to save, 'q' to quit.")
    pair_count = 0

    while True:
        img_p = primary.GetNextImage(1000)
        img_s = secondary.GetNextImage(1000)

        if img_p.IsIncomplete() or img_s.IsIncomplete():
            img_p.Release()
            img_s.Release()
            continue

        frame_l = cv2.cvtColor(img_p.Convert(PySpin.PixelFormat_BGR8).GetNDArray(), cv2.COLOR_RGB2BGR)
        frame_r = cv2.cvtColor(img_s.Convert(PySpin.PixelFormat_BGR8).GetNDArray(), cv2.COLOR_RGB2BGR)
        img_p.Release()
        img_s.Release()

        disp_l = cv2.resize(frame_l, (640, 400))
        disp_r = cv2.resize(frame_r, (640, 400))
        combined = cv2.hconcat([disp_l, disp_r])
        cv2.putText(combined, f"Pairs saved: {pair_count}", (10, 30),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Calibration Capture", combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite(os.path.join(output_dir, f"left_{pair_count:02d}.png"), frame_l)
            cv2.imwrite(os.path.join(output_dir, f"right_{pair_count:02d}.png"), frame_r)
            print(f"  Saved pair #{pair_count}")
            pair_count += 1

    primary.EndAcquisition()
    secondary.EndAcquisition()
    primary.DeInit()
    secondary.DeInit()
    cam_list.Clear()
    system.ReleaseInstance()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

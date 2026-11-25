# Stereo calibration log

One entry per calibration, newest first. Each records the rig, the capture settings, the data, the commands and the
results, so a calibration can be checked or redone later. The YAML files sit next to this one in `config/`.

---

## 2026-11-25: cameras on their sides, lenses from their own shots

**Status:** done, saved as `config/calibration_2026-11-25_lens.yaml`. **Not yet copied to `config/calibration.yaml`**
(that is still the 2026-10-02 calibration, also backed up as `config/calibration_before_2026-11-25.yaml`). Check it
against LEDs at tape-measured distances before relying on it.

### Rig

| | |
|---|---|
| Cameras | 2 x Blackfly S BFS-U3-23S3C, 1920 x 1200, BayerRG8 |
| Camera 1 (M1, "left", primary) | serial 25376120, drives the trigger on Line2 |
| Camera 2 (M2, "right", secondary) | serial 25376121, triggered on Line3 |
| Mounting | both cameras rotated 90 deg about their optical axes and tilted slightly up; the baseline runs along the image y axis, so the pair rectifies **vertically** (disparity along y) |
| Baseline | tape: about 62 cm; calibration: 618.3 mm |
| Trigger check | `preview --stereo` at 20 and 70 fps: every pair matched, cameras within about 4 us. 100 fps for both cameras at full resolution loses pairs (USB bandwidth, not the disk) |

### Board

`config/board.yaml` (copied into each capture folder): ChArUco 8 x 5 squares, DICT_4X4_50, squares 47.88 mm, markers
35.96 mm (calipers, 2026-09-30), 28 inner corners.

### Capture

Both tools take these settings from their own defaults, not from `camera.yaml` (which sets 50 fps, 3000 us, 12 dB):

| | |
|---|---|
| Frame rate | 20 fps |
| Exposure | 8333 us (flicker free under the 60 Hz room lights) |
| Gain | 12 dB |
| Resolution, format, white balance | from `config/camera.yaml`: 1920 x 1200, BayerRG8, red 1.28 / blue 2.08, gamma off |

| Data | Tool | Saved | Used |
|---|---|---|---|
| `data/calibration_images_2026-11-25/` | `capture_calibration` (pairs: saved when both cameras see >= 6 of the same corners) | 63 pairs | 53 pairs for R, T (10 had < 6 common corners); each camera's own corners also go into its lens fit |
| `data/calibration_lens_2026-11-25/` | `capture_calibration_lens` (each camera on its own: saved when it sees >= 12 corners) | 104 left, 86 right | 104 left, 77 right |

The pairs only cover the part of each image both cameras see, which with the cameras on their sides leaves the top of
camera 1, the bottom of camera 2 and the left strip of both without corners. The lens shots fill those in.

### Commands

```
.\build\Release\capture_calibration.exe      --config config\ --output data\calibration_images_2026-11-25
.\build\Release\capture_calibration_lens.exe --config config\ --output data\calibration_lens_2026-11-25
python tools\stereo_calibrate_lens.py data\calibration_images_2026-11-25 config\calibration_2026-11-25_lens.yaml --lens data\calibration_lens_2026-11-25
```

`tools/stereo_calibrate_lens.py` (a copy of `stereo_calibrate.py`): 1. each lens (M, D) from every corner that camera
saw, in its lens shots and its half of each pair; 2. R, T from the pairs with the lenses fixed
(`CALIB_FIX_INTRINSIC`); 3. `stereoRectify(alpha=0)`. Python OpenCV 5.0.0 (the C++ tools build against OpenCV 4.12.0;
the calibration file and lens model are the same in both).

### Results

| | Camera 1 (primary) | Camera 2 (secondary) |
|---|---|---|
| Images in the lens fit | 166 (104 lens shots, 62 pair halves) | 114 (77 lens shots, 37 pair halves) |
| Lens fit RMS | 0.216 px | 0.234 px |
| fx, fy | 2262.8, 2265.1 px | 2316.7, 2319.2 px |
| cx, cy | 938.0, 561.8 px | 987.1, 565.0 px |
| D (k1 k2 p1 p2 k3) | -0.0843 0.3476 -0.0020 -0.0028 -0.1663 | -0.0864 0.2790 -0.0015 -0.0011 0.3122 |
| Empty 192 x 200 px tiles (of 60) | 1 | 3 (thin: bottom row, right column) |

| Stereo | |
|---|---|
| Stereo RMS | 0.613 px |
| T | (-14.8, -616.0, 51.4) mm, \|T\| = 618.3 mm |
| Rotation between the cameras | 4.00 deg |
| Rectification | vertical; rectified focal length 2864.7 px |

### Checks

Triangulating the board corners of the 53 pairs (unrectified: undistort each camera, `triangulatePoints` with
[I|0] and [R|T]):

| | Pairs only (`calibration_2026-11-25.yaml`) | **Lens shots + pairs (`calibration_2026-11-25_lens.yaml`)** |
|---|---|---|
| Square size (true 47.88 mm) | 47.83 mm (-0.09%) | 47.85 mm (-0.06%) |
| Spread of square sizes | 0.66% | 0.71% |
| Board flatness (rms from best plane) | 0.61 mm | 0.59 mm |
| Epipolar residual, median / 95% | 0.57 / 2.15 px | 0.67 / 2.39 px |
| Boards at | 1.67-3.58 m | 1.67-3.56 m |

These pairs also fitted R and T, so this is a consistency check, not an independent one. Both agree inside the
shared view; the lens-shot version is the one constrained near the image edges.

Still to do: LEDs at tape-measured distances (e.g. 3 m and 5 m), near the centre and near the edges: triangulated
spacing against the 30 cm on the rig, depth against the tape.

### Other files from this session

- `config/calibration_2026-11-25.yaml`: pairs only (`tools/stereo_calibrate.py`), lens fits from the shared view
  only (20 and 24 empty tiles; k3 -0.44 / -0.04). Kept for comparison; don't use.
- `config/calibration_before_2026-11-25.yaml`: backup of the 2026-10-02 calibration.

### Compared with the 2026-10-02 calibration (side by side rig)

| | 2026-10-02 | 2026-11-25 | Change |
|---|---|---|---|
| Camera 1 fx, fy | 2321.3, 2316.6 | 2262.8, 2265.1 | -2.5%, -2.2% |
| Camera 1 cx, cy | 931.1, 623.0 | 938.0, 561.8 | +7.0, **-61.3** px |
| Camera 1 D | -0.0858 0.2468 0.0052 -0.0022 0.0675 | -0.0843 0.3476 -0.0020 -0.0028 -0.1663 | |
| Camera 2 fx, fy | 2361.3, 2356.8 | 2316.7, 2319.2 | -1.9%, -1.6% |
| Camera 2 cx, cy | 984.8, 559.9 | 987.1, 565.0 | +2.3, +5.1 px |
| Camera 2 D | -0.0588 0.1014 -0.0023 -0.0011 0.7903 | -0.0864 0.2790 -0.0015 -0.0011 0.3122 | |
| Baseline | 504.3 mm | 618.3 mm | cameras remounted |
| Rotation between the cameras | 1.89 deg | 4.00 deg | cameras remounted |

Same pixel, old model against new: camera 1's rays differ by 1.25-1.83 deg across the image (mostly the cy shift:
61 px / 2263 px = 1.55 deg), camera 2's by 0.13-0.54 deg. Distortion alone (same camera matrix) moves a pixel by
0-3 px at the edges and 9 px (camera 1) / 3 px (camera 2) in the corners; the centre is the same.

Rotating a camera does not change its lens, so differences this size mean either a lens or focus was moved when the
cameras were remounted, or the October fit was less well constrained (it had no lens shots). The focal lengths came
down by about 2% in both cameras, about the size of the 2.9% depth shortfall seen with the October calibration
(`analysis/LED_STEREO_NOTES.md`); whether that is the cause is open until the LED check above.

### What uses this file

The pair now rectifies vertically. `analysis/flight_common.py` (`rect`, `tri`), `analysis/leds_depth.py` (row gate)
and `src/core/stereo.cpp` assume a side by side pair (same row, disparity along x) and need to triangulate unrectified
(undistort each camera, `triangulatePoints` with [I|0] and [R|T], epipolar check) before using this calibration.
Results come out in camera 1's frame; a level frame (x lateral, y vertical, z level with the floor, origin midway
between the cameras) needs the "up" direction, e.g. from `solvePnP` on the board lying flat on the floor.

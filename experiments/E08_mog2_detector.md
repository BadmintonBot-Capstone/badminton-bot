# E08 MOG2 detector on the E04 clips

Status: done 2026-10-13 (left camera only)

## Question
Does jtandjung's MOG2 detector (`mog2_c8_h100`) find more flying frames than `ra`, `prev` and `tri` on the labelled E04 clips, at the same number of false blobs per frame?

## Setup
jtandjung's branch `origin/mog2_experiment` (commit 14d6845, 2026-10-11) adds OpenCV's MOG2 background subtractor to the E04 detector sweep. On his one labelled clip (60 s, one camera, 8.33 ms, 6 dB, 100 fps, players and rackets moving in view) MOG2 on the 8-bit colour image with history 100 found 76.5% [73-80] of flying frames against 65.3% [61-69] for `ra`, at about 0.3 false blobs per frame (`analysis/MOG2_SCORING_PRELIM.md` on his branch). MOG2 alone took about 10 ms per frame on his laptop CPU.

His scripts run unchanged on the left camera of the 13 labelled E04 clips (setup, clips and labels in `E04_settings_sweep.md`). They are not on this branch, so they run from a copy of his `analysis/` folder. The run is narrowed to keep the time down:

| choice | value | why |
|---|---|---|
| MOG2 history | 100 | his pick; 500 was worse at tight budgets |
| `varThreshold` | 16, 32, 64, 128 | his picks were 32 and 64 |
| MOG2 inputs | `c8` (colour), `g8` (grey) | his best, plus grey as the control |
| `mog2tri` models | off | he dropped them |
| `ra`, `prev`, `tri` | his 10 thresholds | rerun so every model is scored with the same plain yellowness |

## Protocol
1. Copy his scripts: `git archive origin/mog2_experiment analysis | tar -x -C <dir>`.
2. Per clip, left camera: `cands_mog2.py <clip> --camera left --history 100 --var 16 32 64 128 --tri-t --inputs c8 g8 --workers 3 --bench 0`. Four clips at a time.
3. Score all 13 clips together: `sweep_single.py <clips> --camera left --start 100 --yellow plain`. Flights from every clip and setting are pooled into one 5-fold cross validation by flight; budget 0.3 false blobs per frame unless stated.
4. Break the scores down by range (11 and 35 ft) from the saved `R` table.
5. Time MOG2 `apply()` on `c8` on a quiet machine: CPU (one thread) against OpenCL on the RTX 2070 (`cv::UMat`, upload and download included), same 500 frames, and count the mask pixels that differ.

## Data
Inputs: `~/Projects/baddy-bot/recordings/E04_settings_sweep/` (13 clips, left camera, `labels/left.csv`). Outputs: `sweep/left_mog2*.npz` in each clip folder and `sweep_single_left_s100_plain.npz` in `ft11_c1_e8333_g6/sweep/`.

## Results
13 clips, 85 flights, 15,961 detector variants. Flying frames found on held-out flights, colour rule allowed, by budget (false blobs per frame):

| model | 0.03 | 0.1 | 0.3 | 1 | 3 |
|---|---|---|---|---|---|
| `ra` | 51.8% | 75.0% | 84.3% | 92.2% | 97.3% |
| `prev` | 65.2% | 83.6% | 89.5% | 97.6% | 99.4% |
| `tri` | 65.5% | 79.2% | 87.5% | 97.2% | 98.8% |
| `mog2_c8_h100` | 20.6% | 53.9% | 87.2% | 91.9% | 98.5% |
| `mog2_g8_h100` | 54.3% | 78.7% | 85.0% | 93.1% | 97.4% |

By range, with 95% ranges from resampling flights:

| model | budget | all | 11 ft | 35 ft |
|---|---|---|---|---|
| `ra` | 0.1 | 75.0% [71-78] | 71.4% [65-77] | 77.1% [73-81] |
| `prev` | 0.1 | 83.6% [81-86] | 78.9% [74-83] | 86.2% [83-89] |
| `tri` | 0.1 | 79.2% [76-82] | 78.7% [71-85] | 79.5% [76-83] |
| `mog2_c8_h100` | 0.1 | 53.9% [49-58] | 74.1% [68-79] | 42.3% [39-45] |
| `mog2_g8_h100` | 0.1 | 78.7% [76-82] | 84.1% [78-89] | 75.5% [72-79] |
| `ra` | 0.3 | 84.3% [81-87] | 80.2% [74-86] | 86.6% [83-90] |
| `prev` | 0.3 | 89.5% [87-92] | 85.5% [81-89] | 91.8% [90-94] |
| `tri` | 0.3 | 87.5% [85-90] | 85.2% [79-91] | 88.8% [86-91] |
| `mog2_c8_h100` | 0.3 | 87.2% [85-89] | 87.8% [84-91] | 86.8% [84-89] |
| `mog2_g8_h100` | 0.3 | 85.0% [83-87] | 88.2% [83-92] | 83.1% [80-85] |

Settings picked at 0.3: `mog2_c8_h100` varThreshold 32 or 64, area 64 and up, yellowness at least 0.35; `tri` T6 or T8, area 16 or 64 and up, yellowness at least 0.3. With the colour rule off, `mog2_c8_h100` finds 7.2% at 0.3 and `tri` 83.2%. The current detector as it ships (`ra` T12, area 4-900, weighted yellowness at least 0.3) finds 90.8% [88-93] at 2.15 false blobs per frame.

Timing, `c8` input, `ft35_c1_e8333_g6` frames 600 to 1100, Python, one OpenCV thread, quiet machine (i7-10875H, RTX 2070 Max-Q, OpenCL from `opencl-nvidia-580xx`):

| path | median ms | 95th percentile ms |
|---|---|---|
| CPU: blur and `apply()` | 13.49 | 14.17 |
| CPU: `apply()` alone (first 100 frames) | 9.50 | |
| GPU, OpenCL: upload, blur, `apply()`, download | 5.11 | 5.94 |

GPU masks differ from the CPU's by a median of 1 pixel per frame (largest 46) out of a median of 182 foreground pixels. While it ran, the 2070 went from 10 W idle (P8) to 32 W (P0) and from 51 to 64 degrees C.

## Answer
On the E04 clips MOG2 does not beat the memoryless detectors: at 0.3 false blobs per frame colour MOG2 finds 87.2% against 87.5% for `tri` and 89.5% for `prev`, and at 0.1 it drops to 53.9% (42.3% at 35 ft) against 79.2% for `tri`. Its lead on jtandjung's clip (76.5% against 65.3% for `ra`) does not carry over to these clips.

## Next
- His clip had players, rackets and clothing moving in view; ours had one thrower. Labelling a busy rally clip and rerunning this is the way to find out whether the scene explains the difference.
- Until then `tri` stays the 3D tracker's detector. It has no background memory, which also suits the crop in E09.
- The GPU route works on the robot laptop from Python (5 ms with upload and download). The C++ build needs `"opencl"` added to the `opencv4` features in `vcpkg.json` and an OpenCV rebuild (`HAVE_OPENCL` is off in the current build).

# MOG2 vs running average detection: preliminary scoring results

Preliminary: scoring only. Timing, centroid accuracy, the yellowness histograms and the C++/GPU setup are in the
full handoff file (`MOG2_DETECTOR_HANDOFF.md`).

## What was tested

- **Clip:** `C:/Users/justi/479_recording_60s_exp8330_gain6`, one camera (`primary`), BayerRG8 1920x1200,
  100 fps, 8.33 ms exposure, 6 dB gain, 5999 frames, no dropped frames.
- **Labels:** `labels/primary.csv`, hand labelled with `analysis/label_raw.py`. 3149 flying, 1159 still,
  1656 none and 35 unsure frames, in 46 flights after frame 100.
- **Resolution:** every detector works at half resolution, where each 2x2 Bayer cell is one pixel (960x600).
  Areas below are in half-resolution pixels, and positions are in full-resolution pixels.

### Detectors

| name | what it is |
|---|---|
| `ra` | change from a running-average background, alpha 0.05, blurred (sigma 1.5) and thresholded at T grey levels. **This is the tracker's detector today** (`shuttle::Detector` in `src/shuttle_tracker.hpp`). |
| `prev` | change from the previous frame |
| `tri` | the smaller of the changes from the previous and the next frame (one frame of latency) |
| `mog2_<in>_h<H>` | OpenCV `BackgroundSubtractorMOG2`, `history=H`, `detectShadows=false`. T is `varThreshold`. |

MOG2 inputs (`<in>`):

| input | meaning |
|---|---|
| `g` | grey, blurred (sigma 1.5) |
| `gr` | grey, not blurred |
| `c` | colour (R, mean of the two G, B per Bayer cell), blurred |
| `cr` | colour, not blurred |
| `g8`, `c8` | same as `g` and `c`, but built in 8 bits all the way, with no float round trip (faster; see the comparison below) |

`mog2tri<t>_*` models (the MOG2 mask ANDed with `tri > t`) were also scored. They were dropped: `mog2tri12` tops out
around 63% at any budget, and `mog2tri6` only helps for grey at the tightest budget.

### Blob filters (applied after any detector)

- **Area:** the blob's area in half-resolution pixels must lie between `min_area` and `max_area`.
- **Colour rule:** the blob's yellowness must be at least `min_yellow`. Yellowness is
  `((R+G)/2 - B) / ((R+G)/2 + B + 1)` on the mean raw colour of the blob's pixels (the plain version). The current
  tracker instead weights each pixel by how far it changed past the threshold; that weighted version is used only
  to score the current detector.

### Scoring (`analysis/sweep_single.py`)

- **Found:** a labelled frame is found when some kept blob's bounding box, grown by 10 full-resolution px,
  contains the label.
- **False candidates:** every other kept blob on a labelled none, flying or still frame is a false candidate.
  Unsure frames and the first 100 frames are left out.
- **Budget:** the maximum average number of false candidates per frame.
- **Cross-validation:** 5-fold by flight. Each detector's settings (threshold, area range, yellowness) are picked
  on 4 folds, as the setting that finds the most flying frames within the budget, and then scored on the 5th.
  Every flight is therefore scored by settings that never saw it.
- **Ranges:** the 95% ranges in brackets come from resampling flights (bootstrap).
- **Grid searched per model:**
  - threshold: 6-50 grey levels for `ra`/`prev`/`tri`, `varThreshold` 16-512 for MOG2
  - `min_area` 2-64
  - `max_area` 50 to unlimited
  - `min_yellow` off, or 0 to 0.6
  - In total, 83,161 detector variants.
- **Warm-up:** leaving out the first 500 frames instead of 100 changed nothing by more than about 1 point, so 100
  is used.

## Results: flying frames found on held-out flights (colour rule allowed)

| model | budget 0.03 | 0.1 | 0.3 | 1 | 3 |
|---|---|---|---|---|---|
| `ra` | 31.3% | 47.0% | 65.3% | 79.7% | 88.3% |
| `prev` | 34.0% | 44.8% | 66.4% | 77.6% | 85.2% |
| `tri` | 39.4% | 48.3% | 60.6% | 73.0% | 83.6% |
| `mog2_g_h100` | 37.1% | 55.8% | 68.1% | 76.9% | 84.6% |
| `mog2_g8_h100` | 37.2% | 55.8% | 68.4% | 77.1% | 84.7% |
| `mog2_g_h500` | 31.0% | 54.0% | 65.4% | 74.4% | 81.1% |
| `mog2_g8_h500` | 31.1% | 54.1% | 65.6% | 74.5% | 81.1% |
| `mog2_gr_h100` | 19.8% | 39.7% | 64.3% | 75.5% | 84.9% |
| `mog2_gr_h500` | 12.6% | 37.5% | 48.7% | 74.3% | 83.0% |
| `mog2_c_h100` | 46.1% | 63.8% | 76.5% | 86.5% | 91.6% |
| **`mog2_c8_h100`** | **46.3%** | **64.0%** | **76.5%** | **86.5%** | **91.7%** |
| `mog2_c_h500` | 29.7% | 52.4% | 77.3% | 85.3% | 90.8% |
| `mog2_c8_h500` | 29.5% | 52.5% | 77.2% | 85.4% | 90.9% |
| `mog2_cr_h100` | 25.3% | 39.3% | 49.9% | 90.1% | 94.8% |
| `mog2_cr_h500` | 19.9% | 33.7% | 49.7% | 85.0% | 94.9% |
| `mog2tri6_c_h100` | 45.6% | 57.6% | 67.8% | 76.4% | 84.3% |
| `mog2tri6_g_h100` | 45.4% | 56.9% | 64.9% | 73.4% | 83.0% |
| `mog2tri12_c_h100` | 37.9% | 44.8% | 50.2% | 57.9% | 62.3% |

The **current detector** (`ra` T12, area 4-900, weighted yellowness ≥ 0.3, untuned) finds 84.9% [82-88] of flying
frames, but with **4.62 false candidates per frame**.

### Detail at budget 0.3

| model | flying found [95%] | still found | false/frame (measured) | settings picked |
|---|---|---|---|---|
| `ra` | 65.3% [61-69] | 17.7% | 0.26 | T8-10, area 32-900, yellow ≥ 0.4 |
| `prev` | 66.4% [62-70] | 14.8% | 0.21 | T6, area 64-900/3000, yellow ≥ 0.35 |
| `tri` | 60.6% [57-64] | 13.8% | 0.27 | T6-8, area 16-900, yellow ≥ 0.4 |
| `mog2_g8_h100` | 68.4% [64-72] | 23.5% | 0.25 | T16-32, area 32-900, yellow ≥ 0.4 |
| `mog2_c8_h100` | 76.5% [73-80] | 22.5% | 0.21 | T64, area 32-900, yellow ≥ 0.4 |
| `mog2_c8_h500` | 77.2% [74-81] | 33.1% | 0.27 | T64, area 32-900, yellow ≥ 0.4 |

"false/frame (measured)" is the actual count of false blobs on the held-out flights, not the cap. It can slightly
exceed the budget, because the budget is only enforced on the training folds.

### Colour rule off

Without the yellowness rule, nothing is usable at tight budgets. The best model at 0.3 finds 25%, and at 0.1 the
best finds 17% (`tri`). At budget 3: `prev` 71.5%, `mog2tri6_c_h100` 67.2%, `mog2_c_h100` 65.1%, `ra` 35.7%.
The scene has many moving things (players, rackets, clothing, shadows) whose blobs are shuttle-sized, and colour is
what separates the shuttle from them.

## Float vs 8-bit MOG2 input

The float input path (`c`, `g`) converts the camera's 8-bit data to float, merges, blurs, and rounds back to 8 bits
for MOG2. The 8-bit path (`c8`, `g8`) stays in 8 bits throughout. The two inputs differ by at most 2 grey levels,
on about 3-4% of pixels.

| comparison | `g` vs `g8` | `c` vs `c8` |
|---|---|---|
| blobs per varThreshold, 8-bit vs float | +0.2% to +1.3% | +0.1% to +1.1% |
| flying found, all 2520 settings: median difference (largest) | +0.10 points (+0.48) | +0.03 to +0.06 points (+0.64) |
| false/frame: median difference (largest) | +0.01 (+0.16) | +0.01 (+0.19) |
| flying frames found at T64/area 32-900/y ≥ 0.4, h100: both / float only / 8-bit only | 1781 / 1 / 3 | 2406 / 4 / 3 |

The two are interchangeable. 8-bit gives very slightly more blobs, probably because OpenCV's fixed-point 8-bit blur
rounds differently. The cross-validated results and the settings picked are the same to within 0.3 points.

### Input preparation and `apply()` cost

Python, one thread, i7-1185G7 (this laptop, **not** the robot's laptop), median ms:

| input | float preparation | 8-bit preparation | MOG2 `apply()`, float input | MOG2 `apply()`, 8-bit input |
|---|---|---|---|---|
| grey (blurred) | ~2.1 | ~1.3 | 9.28 | 9.20 |
| colour (blurred) | ~12-16 | ~4.5 | 10.66 | 10.46 |

`apply()` was measured back to back on frames 600-1600 (busier than the start of the clip).

## Takeaways so far

1. **Colour MOG2 with history 100 (`mog2_c8_h100`) is the best overall.**
   - It finds about 11 points more flying frames than `ra` at budget 0.3 (76.5% vs 65.3%; the 95% ranges don't
     overlap), about 17 points more at 0.1, and about 7 points more at 1.
   - Its settings are stable across folds: T64, area 32-900, yellowness ≥ 0.4.
2. **History 500 keeps still shuttles visible** (33% found vs 23%) but is much worse at tight budgets (52% vs 64%
   at 0.1).
3. **Grey MOG2 is only a small gain** over `ra` (+2 to +9 points). Colour is what makes MOG2 work here.
4. **The unblurred inputs (`gr`, `cr`) are worse at tight budgets.** `cr` is the best of all at budgets 1-3
   (90-95%).
5. **The 8-bit input is as accurate as the float input and much cheaper to make.** Use it.
6. **Speed:** MOG2 `apply()` alone is about 9-10 ms per frame on this CPU, against about 2 ms for `ra`'s whole
   change step. MOG2 needs a GPU (or pipelining in C++) to fit the 10 ms per frame budget at 100 fps. The details
   are in the full handoff file.

## Caveats

- **One clip, one scene, one lighting setup.** Cross-validation by flight guards against tuning to particular
  flights, but not against this scene. A second labelled clip should be scored before relying on these numbers.
- **The yellowness cut-off (about 0.4) is for this camera's raw colour,** this lighting and this (yellow) shuttle.
- **"Found" means the label is within the blob's grown box,** not that the tracker would pick that blob.

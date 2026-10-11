# Handoff: switching shuttle detection from the running average (`ra`) to MOG2

This file is for someone who has the current `ra` detector working (`shuttle::Detector` in
`src/shuttle_tracker.hpp`) and wants to try OpenCV's MOG2 background subtractor in its place. It covers:

- what was measured;
- what to use;
- how to build it in C++ on a GPU.

The scoring-only summary is in `analysis/MOG2_SCORING_PRELIM.md`. This file repeats the main tables and adds
accuracy, colour and timing.

## TL;DR

- **Use `mog2_c8_h100` with these settings** (all areas in half-resolution pixels):
  - MOG2 on an 8-bit half-resolution **colour** image, blurred with sigma 1.5
  - `history=100`, `varThreshold=64`, `detectShadows=false`
  - keep blobs with area 32-900
  - keep blobs with **plain** yellowness ≥ 0.4
- **Detection results** on held-out flights, at about 0.2-0.3 false candidates per frame:
  - It finds **76.5%** of flying frames, against **65.3%** for `ra` tuned the same way. The 95% ranges are [73-80]
    and [61-69], so they don't overlap.
  - At a tighter budget of 0.1 false per frame, it finds 64.0% against 47.0%.
- **Centroid accuracy** matches `ra`:
  - median 5.0 px from the hand label (full resolution) on flying frames, 90th percentile 9.6 px;
  - 100% of found frames are within 28 px.
- **Speed is the cost.** On a laptop CPU (i7-1185G7, Python), MOG2's `apply()` takes about 10 ms per frame on its
  own, and the whole detector takes about 22 ms per frame against 10 ms for `ra`.
- **The GPU solves the speed problem.** On this laptop's Intel iGPU, OpenCL (`cv::UMat`) ran the same `apply()` in
  about 1 ms, and its masks match the CPU's to within about 1% of the foreground pixels. The robot's RTX 2070
  should be at least as fast. The C++ setup is below.
- **Even if you stay on `ra`, change the colour rule.** Plain blob yellowness ≥ 0.4, area 32-900 and T10 take the
  current detector from 4.6 false candidates per frame to 0.26. The cost is finding 65% of flying frames instead
  of 85%.

## 1. What was tested

### Data

- **Clip:** `C:/Users/justi/479_recording_60s_exp8330_gain6`, one camera (`primary`), BayerRG8 1920x1200,
  100 fps, 8.33 ms exposure, 6 dB gain, 5999 frames, no drops.
- **Labels:** `labels/primary.csv`, made with `analysis/label_raw.py`. Each frame is labelled flying, still, none
  or unsure, with an (x, y) at full resolution. There are 3149 flying, 1159 still, 1656 none and 35 unsure frames,
  in 46 flights after frame 100.
- **Resolution:** every detector works at **half resolution**, where one 2x2 Bayer cell is one pixel (960x600).
  Areas are in half-resolution pixels. Positions and distances are in full-resolution pixels.

### Detectors

| name | change image | notes |
|---|---|---|
| `ra` | \|grey − running average\|, alpha 0.05, blur sigma 1.5, threshold T | the current C++ detector |
| `prev` | \|grey − previous grey\| | |
| `tri` | min(\|grey − previous\|, \|grey − next\|) | one frame of latency |
| `mog2_<in>_h<H>` | OpenCV `BackgroundSubtractorMOG2(history=H, varThreshold=T, detectShadows=false)` foreground mask | default learning rate; the first frame is ignored, because MOG2 marks all of it as foreground |

MOG2 inputs:

| input | what |
|---|---|
| `g` / `g8` | grey (2x2 area average), blurred with sigma 1.5. `g` goes through float, `g8` stays 8-bit. |
| `c` / `c8` | colour: per Bayer cell R, mean of the two G, B (stored in B, G, R order), blurred with sigma 1.5. `c` goes through float, `c8` stays 8-bit. |
| `gr`, `cr` | the same, not blurred |

### Blob filters

After any detector:

1. Run `connectedComponentsWithStats` (8-connected).
2. Keep blobs with `min_area` ≤ area ≤ `max_area`.
3. Keep blobs with yellowness ≥ `min_yellow`.

**Plain yellowness** is `((R+G)/2 − B) / ((R+G)/2 + B + 1)` on the **mean raw (unblurred) Bayer colour of the
blob's own pixels**. The current C++ `Detector::yellowness()` is different: it uses an 11x11 window weighted by
`diff − thresh`. All the tuned numbers here use plain yellowness. The weighted version was only used to score the
current detector as it is.

### Scoring (`analysis/sweep_single.py`)

- **Found:** some kept blob's bounding box, grown by 10 full-resolution px, contains the label.
- **False candidates:** every other kept blob on a none, flying or still frame. Unsure frames and frames before 100
  are left out.
- **Cross-validation:** 5-fold by flight. For each budget (maximum false candidates per frame), the setting that
  finds the most flying frames within the budget on 4 folds is picked, then scored on the fifth.
- **Ranges:** 95% ranges come from a bootstrap over flights.
- **Grid:**
  - T: 6-50 for the change detectors, `varThreshold` 16-512 for MOG2
  - `min_area` 2-64
  - `max_area` 50 to unlimited
  - `min_yellow` off or 0-0.6
  - 83k variants in all
- **Warm-up:** starting the counting at frame 500 instead of 100 changed nothing by more than about 1 point.

## 2. Scoring results

### Flying frames found on held-out flights (colour rule allowed)

| model | budget 0.03 | 0.1 | 0.3 | 1 | 3 |
|---|---|---|---|---|---|
| `ra` | 31.3% | 47.0% | 65.3% | 79.7% | 88.3% |
| `prev` | 34.0% | 44.8% | 66.4% | 77.6% | 85.2% |
| `tri` | 39.4% | 48.3% | 60.6% | 73.0% | 83.6% |
| `mog2_g8_h100` | 37.2% | 55.8% | 68.4% | 77.1% | 84.7% |
| `mog2_g8_h500` | 31.1% | 54.1% | 65.6% | 74.5% | 81.1% |
| `mog2_gr_h100` | 19.8% | 39.7% | 64.3% | 75.5% | 84.9% |
| **`mog2_c8_h100`** | **46.3%** | **64.0%** | **76.5%** | **86.5%** | **91.7%** |
| `mog2_c8_h500` | 29.5% | 52.5% | 77.2% | 85.4% | 90.9% |
| `mog2_cr_h100` | 25.3% | 39.3% | 49.9% | 90.1% | 94.8% |
| `mog2_cr_h500` | 19.9% | 33.7% | 49.7% | 85.0% | 94.9% |

The float inputs (`g`, `c`) are within 0.3 points of their 8-bit versions; see section 3.

**The current detector as it ships** (`ra` T12, area 4-900, weighted yellowness ≥ 0.3, not tuned) finds 84.9%
[82-88] of flying frames, but with **4.62 false candidates per frame**.

### Detail at budget 0.3

| model | flying found [95%] | still found | false/frame (measured) | settings picked |
|---|---|---|---|---|
| `ra` | 65.3% [61-69] | 17.7% | 0.26 | T8-10, area 32-900, yellow ≥ 0.4 |
| `prev` | 66.4% [62-70] | 14.8% | 0.21 | T6, area 64-900/3000, yellow ≥ 0.35 |
| `tri` | 60.6% [57-64] | 13.8% | 0.27 | T6-8, area 16-900, yellow ≥ 0.4 |
| `mog2_g8_h100` | 68.4% [64-72] | 23.5% | 0.25 | T16-32, area 32-900, yellow ≥ 0.4 |
| `mog2_c8_h100` | 76.5% [73-80] | 22.5% | 0.21 | T64, area 32-900, yellow ≥ 0.4 (every fold) |
| `mog2_c8_h500` | 77.2% [74-81] | 33.1% | 0.27 | T64, area 32-900, yellow ≥ 0.4 |

- "false/frame (measured)" is the real count of false blobs on the held-out flights, not the cap.
- At looser budgets (up to 1) the cross-validation also picked these settings for `mog2_c8_h100`:
  - T64, area 64-900, yellow ≥ 0.4
  - T32, area 32-900 or 32-3000, yellow ≥ 0.35

### Without the colour rule

**The colour rule is needed.** Without it, nothing works at tight budgets:
- At 0.3 the best model finds 25%, and at 0.1 the best finds 17%.
- At budget 3: `prev` finds 71.5%, `mog2_c_h100` 65.1% and `ra` 35.7%.

Players, rackets, clothes and shadows make many shuttle-sized moving blobs, and colour is what separates the
shuttle from them.

### Which history to use

- **History 100** reacts faster and is much better at tight budgets (64.0% against 52.5% at 0.1).
- **History 500** keeps still shuttles on the floor visible longer (33% of still frames found against 23%).

For a tracker that cares about flight, use 100.

## 3. Float vs 8-bit input

The 8-bit input makes the colour image without going through float:

```
R  = bayer[0::2, 0::2]
G  = addWeighted(bayer[0::2, 1::2], 0.5, bayer[1::2, 0::2], 0.5)
B  = bayer[1::2, 1::2]
c8 = GaussianBlur(merge(B, G, R), sigma 1.5)
```

- **Inputs:** they differ from the float path by at most 2 grey levels, on about 3-4% of pixels.
- **Blob count:** 8-bit gives +0.1% to +1.3% more blobs per varThreshold.
- **Over all 2520 scored settings:**
  - flying found differs by a median of +0.03 to +0.10 points (largest +0.64);
  - false per frame differs by a median of +0.01 (largest +0.19).
- **Frame by frame at the picked setting** (both / float only / 8-bit only):
  - `c`: 2406 / 4 / 3
  - `g`: 1781 / 1 / 3
- **Cross-validated results** are within 0.3 points, with the same picks.

**Conclusion:** the two are interchangeable, so use 8-bit.

## 4. Accuracy: centroid against the hand label

These are blob centroid to label distances in full-resolution px, on the frames each setting found (fixed
settings, all frames after 100). "In 28 px" is the share within the 28 px ring that `label.py` draws.

| variant | flying frames found | median | mean | 90% | in 28 px | still frames found | still median | still 90% | still in 28 px |
|---|---|---|---|---|---|---|---|---|---|
| current: `ra` T12 4-900 weighted y ≥ 0.3 | 2674 | 5.4 | 5.9 | 10.1 | 99.9% | 351 | 7.5 | 22.6 | 92.3% |
| `ra` T10 32-900 y ≥ 0.4 | 2048 | 5.1 | 5.7 | 9.9 | 100% | 210 | 6.4 | 9.9 | 100% |
| `prev` T6 64-900 y ≥ 0.35 | 2091 | 9.7 | 9.9 | 15.9 | 99.9% | 151 | 8.1 | 38.4 | 87.4% |
| `tri` T6 16-900 y ≥ 0.4 | 1929 | 5.7 | 6.3 | 11.0 | 100% | 158 | 8.0 | 18.1 | 100% |
| `mog2_g8_h100` T32 32-900 y ≥ 0.4 | 2050 | 5.4 | 5.9 | 10.3 | 100% | 264 | 10.0 | 20.1 | 99.2% |
| **`mog2_c8_h100` T64 32-900 y ≥ 0.4** | **2409** | **5.0** | **5.5** | **9.6** | **100%** | 261 | 8.6 | 17.6 | 99.2% |
| `mog2_c_h100` (float) T64 32-900 y ≥ 0.4 | 2410 | 5.0 | 5.5 | 9.6 | 100% | 266 | 8.9 | 17.7 | 99.2% |
| `mog2_c8_h500` T64 32-900 y ≥ 0.4 | 2432 | 5.0 | 5.5 | 9.6 | 100% | 384 | 8.1 | 22.3 | 99.2% |

Flying frames by label speed (px per frame), median error:

| variant | 0-5 | 5-15 | 15-30 | 30+ |
|---|---|---|---|---|
| `ra` T10 32-900 | 3.5 (132) | 4.9 (1313) | 6.0 (584) | 4.6 (19) |
| `prev` T6 64-900 | 5.5 (93) | 8.9 (1485) | 13.3 (506) | 16.0 (7) |
| `mog2_c8_h100` T64 32-900 | 3.6 (143) | 4.8 (1600) | 6.0 (646) | 4.8 (20) |

The numbers in brackets are frame counts.

- **MOG2 and `ra` put the centroid in the same place.** It sits at the shuttle's current position, with about 5 px
  of error, and the error grows only slowly with speed.
- **`prev` is worse.** Its blob covers both the old and the new position, so its error grows with speed: 13 px at
  15-30 px/frame.
- **Faster than 30 px/frame:** there are few frames, so those medians are noisy.
- **Labels:** the labels are hand clicks, so a few px of this error is the label's own.

## 5. Colour: shuttle blobs against everything else

This section looks at every blob that passes the area rule, before the colour rule, on counted frames.
- **Shuttle blob:** the blob containing the label (box grown by 10 px).
- **Other:** every other blob.

| variant | shuttle blobs | other blobs | shuttle median yellowness | others median | at ≥ 0.4: shuttle kept | others kept |
|---|---|---|---|---|---|---|
| `ra` T10 32-900 | 2858 | 52455 | 0.45 | 0.26 | 78.8% | 2.9% |
| `mog2_c8_h100` T64 32-900 | 3291 | 36481 | 0.45 | 0.26 | 81.1% | 3.3% |
| `mog2_g8_h100` T16 32-900 | 3140 | 46842 | 0.45 | 0.26 | 78.4% | 3.6% |

MOG2 also gives fewer other blobs overall (36k against 52k for `ra`), which is where most of its advantage at
tight budgets comes from.

### Histograms of plain yellowness

The tables below are % of each group per 0.05 bin. Bins below −0.20 and above 0.80 are clipped into the end bins,
and empty end bins are trimmed.

**`ra` T10 area 32-900:**

| yellowness | shuttle % | | other % | |
|---|---|---|---|---|
| −0.10 | 0.0 | | 0.1 | |
| −0.05 | 0.0 | | 0.4 | |
| 0.00 | 0.0 | | 1.0 | `#` |
| +0.05 | 0.0 | | 3.6 | `####` |
| +0.10 | 0.2 | | 5.4 | `#####` |
| +0.15 | 0.4 | | 10.1 | `##########` |
| +0.20 | 2.7 | `###` | 21.1 | `#####################` |
| +0.25 | 2.8 | `###` | 34.6 | `###################################` |
| +0.30 | 4.7 | `#####` | 15.5 | `################` |
| +0.35 | 10.4 | `##########` | 5.1 | `#####` |
| +0.40 | 26.5 | `###########################` | 1.9 | `##` |
| +0.45 | 43.6 | `############################################` | 0.5 | |
| +0.50 | 8.4 | `########` | 0.2 | |
| +0.55 | 0.3 | | 0.1 | |
| +0.60 | 0.0 | | 0.2 | |

**`mog2_c8_h100` T64 area 32-900:**

| yellowness | shuttle % | | other % | |
|---|---|---|---|---|
| −0.10 | 0.0 | | 0.1 | |
| −0.05 | 0.0 | | 0.2 | |
| 0.00 | 0.0 | | 0.5 | |
| +0.05 | 0.3 | | 1.5 | `#` |
| +0.10 | 0.3 | | 6.0 | `######` |
| +0.15 | 0.6 | `#` | 9.2 | `#########` |
| +0.20 | 1.9 | `##` | 28.1 | `############################` |
| +0.25 | 2.6 | `###` | 25.2 | `#########################` |
| +0.30 | 5.1 | `#####` | 19.4 | `###################` |
| +0.35 | 8.1 | `########` | 6.6 | `#######` |
| +0.40 | 28.6 | `#############################` | 2.0 | `##` |
| +0.45 | 44.2 | `############################################` | 0.6 | `#` |
| +0.50 | 7.7 | `########` | 0.5 | `#` |
| +0.55 | 0.6 | `#` | 0.1 | |

**`mog2_g8_h100` T16 area 32-900:**

| yellowness | shuttle % | | other % | |
|---|---|---|---|---|
| −0.05 | 0.0 | | 0.2 | |
| 0.00 | 0.0 | | 0.6 | `#` |
| +0.05 | 0.3 | | 1.7 | `##` |
| +0.10 | 0.4 | | 6.1 | `######` |
| +0.15 | 0.5 | `#` | 9.6 | `##########` |
| +0.20 | 2.0 | `##` | 25.9 | `##########################` |
| +0.25 | 3.8 | `####` | 25.4 | `#########################` |
| +0.30 | 6.5 | `######` | 19.5 | `####################` |
| +0.35 | 8.0 | `########` | 7.3 | `#######` |
| +0.40 | 30.3 | `##############################` | 2.3 | `##` |
| +0.45 | 39.4 | `#######################################` | 0.8 | `#` |
| +0.50 | 7.8 | `########` | 0.5 | |
| +0.55 | 0.9 | `#` | 0.1 | |

**The shuttle sits at 0.40-0.50.** Almost all other blobs sit at 0.15-0.35, because the scene is warm in raw
colour (the raw camera colour isn't white balanced). The cut-off at 0.4 sits in the gap between them. About 20%
of shuttle blobs fall below it, which is why a missed shuttle frame is usually a colour miss, not a detection
miss.

### Other colour rules tried (tuned in-sample, so treat them as optimistic)

Share of other blobs kept, at the cut-off that keeps 90% of shuttle blobs:

| variant | plain yellowness | white-balanced yellowness (grey world: R 0.85, G 1.00, B 1.57) | hue window + saturation floor (HSV on the blob's mean raw colour, hue in degrees) |
|---|---|---|---|
| `ra` T10 32-900 | 8.7% (≥ 0.35) | 9.7% | 0.6% (hue 60-90, sat ≥ 0.18) |
| `mog2_c8_h100` T64 32-900 | 11.5% (≥ 0.34) | 11.5% | 1.2% (hue 45-105, sat ≥ 0.16) |
| `mog2_g8_h100` T32 32-900 | 10.3% (≥ 0.35) | 10.2% | 1.4% (hue 40-100, sat ≥ 0.18) |

- **White balance doesn't help.** It is a monotone change, so it only moves the cut-off.
- **A hue and saturation rule looks 5-10x better at keeping the shuttle,** but the best window moves a lot between
  variants (anywhere from 65-85° up to 30-115°). It was also picked on the same data it is scored on.
- **Before relying on HSV,** add it to the cross-validated sweep (`sweep_single.py`). That needs the R, G, B
  columns, which only the `g8`/`c8` tables have so far. Then check it on a second clip.

## 6. Timing

These figures were measured in Python with OpenCV 4.x (pip) on this development laptop:
- i7-1185G7, 4 cores / 8 threads, Iris Xe, **no CUDA**;
- full clip, frames 100-5999, median ms per frame, with the 95th percentile after the slash.

**This is not the robot's laptop**, which has an RTX 2070.

Stages:
- **prep:** half-resolution grey plus the R, G, B planes used for blob colour.
- **input:** building the MOG2 input image.
- **model:** the change image, blur and threshold for `ra`/`prev`/`tri`, or `apply()` for MOG2.
- **blobs:** connected components, the area rule and colour, done in numpy.

| variant | prep | input | model | blobs | total, 1 OpenCV thread | total, 8 threads |
|---|---|---|---|---|---|---|
| current `ra` T12 4-900 weighted y ≥ 0.3 | 3.21 | 0 | 2.33 | 5.66 | 11.28 / 14.06 | 10.05 / 11.75 |
| `ra` T10 32-900 y ≥ 0.4 | 3.00 | 0 | 2.22 | 4.67 | 10.05 / 12.39 | 10.02 / 15.42 |
| `prev` T6 64-900 y ≥ 0.35 | 3.72 | 0 | 2.14 | 4.45 | 10.61 / 13.35 | 10.14 / 13.17 |
| `tri` T6 16-900 y ≥ 0.4 | 2.88 | 0 | 2.07 | 4.32 | 9.31 / 11.10 | 9.49 / 12.18 |
| `mog2_g8_h100` T32 32-900 y ≥ 0.4 | 2.93 | 1.34 | 8.61 | 4.65 | 17.62 / 21.24 | 13.92 / 18.56 |
| **`mog2_c8_h100` T64 32-900 y ≥ 0.4** | 2.94 | 4.42 | **9.96** | 4.71 | **22.17 / 26.13** | **17.52 / 21.53** |
| `mog2_c_h100` (float input) T64 32-900 | 4.16 | 7.41 | 10.04 | 4.77 | 26.41 / 32.02 | 22.76 / 29.93 |
| `mog2_c8_h500` T64 32-900 y ≥ 0.4 | 3.04 | 4.59 | 10.54 | 5.12 | 23.41 / 27.99 | 20.47 / 26.89 |

### Notes on timing

- **The budget at 100 fps is 10 ms per frame.** On this CPU, `ra` just fits and CPU MOG2 doesn't.
- **More threads help MOG2's `apply()`** (about 10 ms down to about 5 ms), but the total only drops to about 17 ms.
  The other stages are memory-bound or single-threaded Python.
- **Most of the "blobs" and "prep" time is Python and numpy overhead.**
  - In C++ with one pass over the label image, blobs should take well under 1 ms.
  - prep isn't needed at all: the colour can be read straight from the Bayer frame.
- **OpenCL check (`cv::UMat`) on this laptop's Iris Xe:**
  - MOG2 `apply()` plus download of the mask: **0.97 ms** median, against 14-15 ms on the CPU in the same run.
  - The masks differ from the CPU's by a median of 14 pixels per frame (max 172), out of about 1070 foreground
    pixels. That is float rounding in the GPU kernel.
  - A 4-channel input with a constant 4th channel gives exactly the same result as 3-channel. This matters for
    CUDA, which can only blur 1- or 4-channel images.
  - The script is `ocl_check.py`, run on frames 600-1200.

### Estimates for the robot laptop (RTX 2070), not measured

| pipeline | estimated ms per frame |
|---|---|
| MOG2 `apply()` on the GPU, including upload and download | ~1-3 |
| Python, 8-bit input, GPU MOG2 | ~14-16 (Python blob code dominates) |
| C++, 8-bit input on the CPU, GPU MOG2, C++ blobs | ~4-6 |
| C++, everything except blobs on the GPU | ~3-5 |
| C++ `ra` (today) | ~3-4 |

## 7. Building it in C++ with a GPU

There are two routes. **Try OpenCL first**: it needs no CUDA toolkit and no contrib build, the same code also
runs on the CPU, and it was verified here. Use CUDA only if OpenCL timing on the 2070 is not good enough.

### 7.1 Dependencies (vcpkg)

**OpenCL route.** Add the `opencl` feature to `opencv4` in `vcpkg.json`. The `video` module, which holds MOG2, is
built by default.

```json
{
  "name": "opencv4",
  "default-features": false,
  "features": [
    "calib3d", "highgui", "png", "fs", "thread", "intrinsics", "opencl",
    { "name": "gtk", "platform": "linux" },
    { "name": "win32ui", "platform": "windows" }
  ]
}
```

1. Check the feature exists in the pinned baseline: `vcpkg search opencv4 --featurepackages`. If it doesn't, bump
   the baseline or build OpenCV with `-DWITH_OPENCL=ON`.
2. Update the NVIDIA driver. It supplies the OpenCL runtime for the 2070, and nothing else is needed.

**CUDA route.**
1. Install the CUDA Toolkit (12.x) on the robot laptop.
2. Add `"cuda"` and `"contrib"` to the features above. `contrib` builds the `cudabgsegm`, `cudafilters` and
   `cudaarithm` modules.
3. Expect a long build (an hour or more).
4. Set the GPU architecture to the 2070's (compute 7.5, for example `CUDA_ARCH_BIN=7.5`) to cut build time.
5. After building, check that `cv::getBuildInformation()` lists `cudabgsegm` and shows `NVIDIA CUDA: YES`.

### 7.2 CMake

Add `video` to the OpenCV components. For CUDA also add `cudabgsegm`, `cudafilters` and `cudaarithm`.

```cmake
find_package(OpenCV 4 REQUIRED COMPONENTS
    core
    imgproc
    calib3d
    highgui
    objdetect
    video          # BackgroundSubtractorMOG2
    # cudabgsegm cudafilters cudaarithm   # CUDA route only
)
```

### 7.3 A MOG2 detector that drops in for `shuttle::Detector`

`src/mog2_detector.hpp` (new) produces the same `shuttle::Candidate` list in half-resolution pixels, so
`shuttle::Tracker` doesn't change. It follows the Python reference exactly:

| step | Python reference (`analysis/cands_mog2.py`, `analysis/detector_timing.py`) |
|---|---|
| 8-bit colour input | `mog_input(..., 'c8')` |
| MOG2 | `history=100, varThreshold=64, detectShadows=False`, default learning rate, first frame ignored |
| blobs | `connectedComponentsWithStats`, 8-connected |
| area rule | `32 <= area <= 900` |
| colour | plain mean of the blob's own raw Bayer R, G (two-green mean), B |

Using `cv::UMat` makes OpenCV run blur and MOG2 through OpenCL when it is available, and on the CPU when it isn't.

```cpp
// mog2 based shuttle detection for one camera, a drop in for shuttle::Detector

#pragma once

#include "shuttle_tracker.hpp"
#include <opencv2/core/ocl.hpp>
#include <opencv2/video/background_segm.hpp>

namespace shuttle {

struct MogParams {
    int    history    = 100;
    double var_thresh = 64;     // MOG2 varThreshold
    double blur       = 1.5;    // gaussian sigma on the 8 bit colour input
    int    min_area   = 32;     // half resolution pixels
    int    max_area   = 900;    // anything bigger is treated as a person
    int    near_big   = 40;
    double min_yellow = 0.4;    // plain yellowness of the blob's own pixels, -1 turns it off
};

inline MogParams load_mog_params(const std::string& path)
{
    MogParams p;
    if (!std::filesystem::exists(path)) return p;
    YAML::Node n = YAML::LoadFile(path);
    auto rd = [&](const char* k, auto& v) { if (n[k]) v = n[k].as<std::decay_t<decltype(v)>>(); };
    rd("mog_history", p.history); rd("mog_var_thresh", p.var_thresh); rd("mog_blur", p.blur);
    rd("mog_min_area", p.min_area); rd("mog_max_area", p.max_area); rd("near_big", p.near_big);
    rd("mog_min_yellow", p.min_yellow);
    return p;
}

// bayer frame in, blob candidates out
class MogDetector {
public:
    explicit MogDetector(const MogParams& p) : p_(p)
    {
        mog_ = cv::createBackgroundSubtractorMOG2(p.history, p.var_thresh, false);
    }

    const std::vector<Candidate>& step(const cv::Mat& bayer)
    {
        // one pixel per 2x2 BayerRG cell: B, mean G, R, all in 8 bits (the 'c8' input)
        col_.create(bayer.rows / 2, bayer.cols / 2, CV_8UC3);
        for (int y = 0; y < col_.rows; ++y) {
            const uchar* r0 = bayer.ptr<uchar>(2 * y);
            const uchar* r1 = bayer.ptr<uchar>(2 * y + 1);
            uchar* o = col_.ptr<uchar>(y);
            for (int x = 0; x < col_.cols; ++x, o += 3) {
                o[0] = r1[2 * x + 1];                                  // B
                o[1] = uchar((r0[2 * x + 1] + r1[2 * x] + 1) >> 1);    // G
                o[2] = r0[2 * x];                                      // R
            }
        }
        col_.copyTo(ucol_);                                  // upload when OpenCL is on
        cv::GaussianBlur(ucol_, ublur_, {0, 0}, p_.blur);
        mog_->apply(ublur_, umask_);
        umask_.copyTo(mask_);                                // download

        cands_.clear();
        big_.clear();
        if (frame_++ == 0) return cands_;                    // MOG2 marks the whole first frame as foreground

        int n = cv::connectedComponentsWithStats(mask_, labels_, stats_, cents_, 8, CV_32S);

        // plain mean raw colour of each blob's own pixels, one pass over the label image
        sums_.assign(3 * n, 0.0);
        for (int y = 0; y < labels_.rows; ++y) {
            const int* l = labels_.ptr<int>(y);
            const uchar* r0 = bayer.ptr<uchar>(2 * y);
            const uchar* r1 = bayer.ptr<uchar>(2 * y + 1);
            for (int x = 0; x < labels_.cols; ++x) {
                int k = l[x];
                if (!k) continue;
                sums_[3 * k]     += r0[2 * x];
                sums_[3 * k + 1] += 0.5 * (r0[2 * x + 1] + r1[2 * x]);
                sums_[3 * k + 2] += r1[2 * x + 1];
            }
        }

        for (int k = 1; k < n; ++k) {
            int a = stats_.at<int>(k, cv::CC_STAT_AREA);
            if (a > p_.max_area) {
                cv::Rect r(stats_.at<int>(k, cv::CC_STAT_LEFT), stats_.at<int>(k, cv::CC_STAT_TOP),
                           stats_.at<int>(k, cv::CC_STAT_WIDTH), stats_.at<int>(k, cv::CC_STAT_HEIGHT));
                big_.push_back(r + cv::Size(2 * p_.near_big, 2 * p_.near_big) - cv::Point(p_.near_big, p_.near_big));
            }
        }
        for (int k = 1; k < n; ++k) {
            int a = stats_.at<int>(k, cv::CC_STAT_AREA);
            if (a < p_.min_area || a > p_.max_area) continue;
            double rg = 0.5 * (sums_[3 * k] + sums_[3 * k + 1]) / a, b = sums_[3 * k + 2] / a;
            Candidate c{{cents_.at<double>(k, 0), cents_.at<double>(k, 1)}, a, false, (rg - b) / (rg + b + 1)};
            if (p_.min_yellow > -1 && c.yellow < p_.min_yellow) continue;
            for (auto& r : big_)
                if (r.contains(cv::Point(int(c.c.x), int(c.c.y)))) c.near_big = true;
            cands_.push_back(c);
        }
        return cands_;
    }

    const cv::Mat& mask() const { return mask_; }
    const std::vector<cv::Rect>& people() const { return big_; }

private:
    MogParams p_;
    cv::Ptr<cv::BackgroundSubtractorMOG2> mog_;
    cv::Mat col_, mask_, labels_, stats_, cents_;
    cv::UMat ucol_, ublur_, umask_;
    std::vector<double> sums_;
    std::vector<cv::Rect> big_;
    std::vector<Candidate> cands_;
    long frame_ = 0;
};

} // namespace shuttle
```

**Using it** (for example in `src/track_single.cpp`):

```cpp
#include "mog2_detector.hpp"
...
cv::ocl::setUseOpenCL(true);
std::cout << "OpenCL " << (cv::ocl::useOpenCL() ? cv::ocl::Device::getDefault().name() : "off") << "\n";
shuttle::MogDetector det(shuttle::load_mog_params(params_path));
...
auto& cands = det.step(bayer);     // instead of shuttle::Detector::step
auto  out   = tracker.step(cands);
```

**Points to watch:**

- **Candidate coordinates.** Candidates are in half-resolution pixels, as with `shuttle::Detector`. Full
  resolution is `2 * c + 0.5`, which is what the Python scoring uses.
- **Colour gating.** The class drops non-yellow blobs entirely, like `yellow_strict = true`. That is how the
  numbers above were scored: they count every kept blob as a candidate. The current tracker uses `min_yellow` only
  to block new tracks, and keeps non-yellow candidates for existing tracks. That is a different setting and was not
  scored here.
- **Tracker tuning.** The tracker's `gate`, `spawn_r` and related parameters were tuned with the old detector's
  4.6 false candidates per frame. With about 0.2 per frame and a few more misses, `coast` may need to be longer.
- **Rounding.** The G rounding (`+1 >> 1`) matches OpenCV's `addWeighted` to within 1 grey level, which section 3
  shows doesn't matter.
- **OpenCL start-up.** The first OpenCL call compiles kernels and takes a second or so. Warm up on a few frames
  before timing, and let MOG2 learn for about 100 frames (its history) before trusting it.

### 7.4 CUDA version of the GPU part

The CUDA version replaces only the upload, blur, MOG2 and download steps. CUDA's Gaussian filter takes 1- or
4-channel 8-bit images, so build the input as BGRA with a constant 4th channel. This was checked on the OpenCL
path, where it gives an identical mask, and it should behave the same on CUDA.

```cpp
#include <opencv2/cudabgsegm.hpp>
#include <opencv2/cudafilters.hpp>

// members
cv::Ptr<cv::cuda::BackgroundSubtractorMOG2> mog_ =
    cv::cuda::createBackgroundSubtractorMOG2(100, 64, false);
cv::Ptr<cv::cuda::Filter> blur_ =
    cv::cuda::createGaussianFilter(CV_8UC4, CV_8UC4, {11, 11}, 1.5);  // 11 = the kernel the CPU picks for sigma 1.5
cv::cuda::HostMem col_host_{600, 960, CV_8UC4, cv::cuda::HostMem::PAGE_LOCKED};  // pinned for fast upload
cv::cuda::GpuMat gcol_, gblur_, gmask_;
cv::cuda::Stream stream_;

// in step(): fill col_host_ (as above, with o[3] = 0 and o += 4), then
cv::Mat col = col_host_.createMatHeader();
gcol_.upload(col, stream_);
blur_->apply(gcol_, gblur_, stream_);
mog_->apply(gblur_, gmask_, -1, stream_);
gmask_.download(mask_, stream_);
stream_.waitForCompletion();
// connected components, colour and filters as above, on the CPU
```

CUDA's MOG2 is a separate implementation from the CPU one, so verify it against the Python reference (section 7.6)
before trusting the numbers above.

### 7.5 Pipelining (optional)

With the GPU doing MOG2, a single thread should fit in 10 ms (estimated 4-6 ms). If it doesn't:

1. Run two stages on two threads with a one-frame queue:
   - stage A: Bayer → colour, upload, blur, MOG2, download;
   - stage B: connected components, colour, tracker.
2. This adds one frame (10 ms) of latency.

CPU-only MOG2 in C++ is borderline even with pipelining, because `apply()` alone is about 5-10 ms.

### 7.6 Verifying the C++ port

1. **Compare against the Python tables.** Run the C++ detector over the labelled clip:
   - Read frames with the same reader as `analysis/rec_any.py`: raw BayerRG8, 1920x1200, plus the frame index.
   - Dump `frame, cx, cy, area, yellow` per kept blob.
   - Compare with the Python rows: `analysis/detector_timing.py ... --models mog2_c8_h100` writes them into
     `<clip>/sweep/detector_timing_primary.npz` (`rows_<k>`, columns: frame, cx, cy, x0, y0, w, h, area, R, G, B,
     yellow, passed).
   - **CPU path:** expect almost all blobs to match exactly.
   - **GPU path:** expect about 1% of mask pixels to differ, so a few blobs will appear or disappear at the edges.
     Flying frames found should stay within about 1 point.
2. **Score it.** Flying frames found and false per frame on the clip should be about 76% and 0.2 at T64, area
   32-900, yellow ≥ 0.4. These are in-sample numbers for that setting: section 4 shows 2409 flying frames found,
   out of about 3100 counted.
3. **Time it on the robot laptop.** Log per-stage times (input, GPU, blobs, tracker) and the 95th percentile, not
   just the mean.

## 8. Caveats

- **One clip, one scene, one lighting setup.** Cross-validation by flight guards against tuning to particular
  flights, not to this scene. **Label and score a second clip before relying on these numbers.**
  `analysis/label_raw.py` labels, and the pipeline in section 9 scores.
- **The yellowness cut-off (0.4) depends on this camera's raw colour, this lighting and this shuttle.** A
  different white balance, gain or shuttle will move it. Re-check the histogram (section 5) on new footage.
- **"Found" means the label is inside a kept blob's grown box.** It doesn't mean the tracker would pick that blob.
  The tracker on top hasn't been rescored with MOG2.
- **Timing is from a different machine, in Python.** The robot-laptop numbers are estimates. The GPU and mask
  agreement figures are from OpenCL on an Intel iGPU, not CUDA on the 2070.
- **The C++ code above is untested.** It was written against the Python reference and the existing
  `shuttle::Detector`, but not compiled here. In particular, check that `vcpkg` has the `opencl` feature at the
  pinned baseline.
- **Not tried:**
  - MOG2's other parameters (`setNMixtures`, `setBackgroundRatio`, `setVarThresholdGen`, learning rate);
  - KNN background subtraction;
  - a cross-validated hue and saturation colour rule.

## 9. Files

| file | what |
|---|---|
| `analysis/rec_any.py` | reads a recording (`load(clip)` → frames, metadata) |
| `analysis/label_raw.py` | hand labelling tool → `<clip>/labels/<camera>.csv` |
| `analysis/cands_mog2.py` | runs every detector and MOG2 input over a clip → `<clip>/sweep/<camera>_mog2*.npz` (blob tables). `--inputs c8 g8 --add` adds inputs to an existing run. |
| `analysis/sweep_single.py` | the cross-validated grid search and scoring → `<clip>/sweep/sweep_single_<camera>_s<start>_plain.npz` and a text report |
| `analysis/detector_timing.py` | per-stage timing, centroid accuracy and colour analysis of the picked variants → `<clip>/sweep/detector_timing_primary.txt` / `.npz`. Each pass is cached in `detector_timing_primary_parts/`. |
| `analysis/MOG2_SCORING_PRELIM.md` | scoring-only summary |
| `src/shuttle_tracker.hpp` | the current `ra` `Detector` and the `Tracker` |

Typical run order for a new clip (from the repo root, with `uv run --with numpy --with opencv-python --with pyyaml
python ...`):

1. `analysis/label_raw.py <clip>`
2. `analysis/cands_mog2.py <clip>`
3. `analysis/sweep_single.py <clip>`
4. `analysis/detector_timing.py <clip> --models ra mog2_c8_h100`

On a full 60 s clip, steps 2-4 take tens of minutes to a couple of hours each on a laptop. Watch memory:
`detector_timing.py` reads frames from the file instead of memory-mapping, because mapping a 13.8 GB clip on
Windows exhausted RAM.

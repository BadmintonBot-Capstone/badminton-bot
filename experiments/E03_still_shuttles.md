# E03 Still shuttles

Status: done 2026-10-01 for the centre of the view (left and right spots not recorded)

## Question
How far off is a triangulated 3D point, as a steady offset and as frame-to-frame scatter, at ranges from 11 to 30 ft?

## Setup
Two yellow shuttles taped cork up to a wooden stick, 3 ft (914.4 mm) apart tip to tip. The test compares the triangulated distance between the two shuttles with that 914.4 mm. Distances from the camera are never used, because nobody knows exactly where the camera's optical centre sits inside the lens.

The stick is held by hand. That is fine for the separation: the stick is rigid, and the hardware trigger exposes both cameras at the same instant, so every frame pair catches the stick in one position and the true separation is 914.4 mm in every frame however the stick moves.

Both cameras on the trigger, rig in the wide aisle view, 8.33 ms, 6 dB, 100 fps. Distances 11, 16, 22 and 30 ft from the rig, all in the centre of the view, held above the table line past 16 ft.

**Recalibrated first.** The first analysis showed the left and right images no longer lined up after rectification (step 4 below): one camera had turned about 0.5 degrees around its viewing axis since the 2026-09-30 calibration, most likely when the rig was moved or the cables replugged. The rig was left untouched and recalibrated as it stood (`data/calibration_images_2026-10-01/`, 65 of 66 pairs, RMS 0.23 / 0.22 / 0.29 px, baseline 503.3 mm), which now lives in `config/calibration.yaml`. All results below use it.

## Protocol
1. Per distance, one 20 s recording (`record_stereo`, 15 s countdown): hold the stick across the view for about 6 s, then turn it to point along the view, far end slightly higher so both shuttles stay visible in both cameras, and hold about 6 s.
2. `analysis/separation.py` mirrors `src/core/stereo.cpp`: each frame is debayered and rectified whole with the calibration's maps, each shuttle is found by colour, and the matching left and right points are triangulated with P1 and P2.
3. Finding the shuttle. Colour rule from sampled pixels (OpenCV HSV): hue 22 to 40, saturation over 120, brightness over 60. This keeps the shuttle (hue 27, saturation 170, brightness 90) and rejects the stick (hue 15), the yellow bins (hue 18), the wall (saturation 111) and dark plant leaves (brightness 23). At one frame per hold, shuttle coloured blobs of 60 px or more are paired across the cameras by row; each shuttle is then followed frame by frame inside a 40 px window, and tracking stops instead of jumping more than 20 px.
4. Each shuttle's point is the centroid of its blob. Centring matters: if a shuttle's blob is cut differently in the two cameras (tape, a hand, the background), the two centroids stand for different points on the shuttle and the depth moves.
5. Every frame is labelled by the stick's angle out of the image plane (across under 25 degrees, along over 35, turning between) and dropped if a shuttle moves more than 3 px since the last frame, if a shuttle's left and right rows differ by more than 3 px (a wrong lock in one camera lands on a different row), or if a shuttle's blob is more than 1.5 times bigger in one camera than the other.
6. `<clip>/separation_check.jpg` shows three used frames per hold, each shuttle zoomed in both cameras with the triangulated point marked; `separation_dropped.jpg` shows the worst dropped frames. Both were checked by eye.

```
~/.venvs/cpsc330/bin/python analysis/separation.py ../recordings/E03_still_shuttles/<clip> [--pick across 0,1]
```

## Data
`~/Projects/baddy-bot/recordings/E03_still_shuttles/`: `ft11_centre_take2`, `ft16_centre` (needs `--pick across 0,1`), `ft22_centre`, `ft22_centre_take2`, `ft30_centre`. `ft11_centre` is not used: one shuttle sat in front of the yellow bins. `ft22_centre` has the hanging plant behind one shuttle, so it was retaken one step left; both takes are reported. The recorder lost no frames in any recording.

## Results

**Calibration check.** Left and right should agree on rows after rectification. Row mismatch from about 300 to 2000 matched scene features per image pair:

| images | calibration | row mismatch (median) | change across the image |
|--------|-------------|-----------------------|-------------------------|
| calibration pairs, 2026-09-30 | 2026-09-30 | 0.5 px | none |
| test clips, 2026-09-30 18:43 | 2026-09-30 | 2 px | none |
| E02 and E03 clips, 2026-10-01 | 2026-09-30 | 3.6 to 4.9 px | -5 px left edge to +6 px right edge |
| E02 and E03 clips, 2026-10-01 | 2026-10-01 | 0.4 to 0.55 px | none |

**Separation**, true value 914.4 mm. Error is the average over the hold's frames (plus means measured too long); scatter is the frame-to-frame standard deviation, how much one frame wanders around that average.

| distance | across: frames, error, scatter | along: frames, error, scatter |
|----------|--------------------------------|-------------------------------|
| 11 ft (3.2 m) | 951, +12.0 mm, 0.8 mm | 953, +3.2 mm, 3.1 mm |
| 16 ft (4.9 m) | 588, +10.4 mm, 3.2 mm | 1080, +2.6 mm, 7.3 mm |
| 22 ft (6.7 m) | 846, +9.7 mm, 4.4 mm | 1000, +2.7 mm, 15.9 mm |
| 22 ft take 2 (6.6 to 7.1 m) | 860, +14.3 mm, 1.7 mm | 1096, +4.7 mm, 16.9 mm |
| 30 ft (8.2 to 8.7 m) | 983, +12.3 mm, 5.8 mm | 367, +2.4 mm, 29.0 mm |

Dropped by the checks: 205 frames in `ft16_centre` (the stick pointed at the camera so the shuttles overlapped and both tracks took the same merged blob; and the lit plant merging with one shuttle in one camera), 17 in `ft30_centre`, none elsewhere. Before the checks, `ft16_centre` read 596 to 1370 mm in single frames.

**Where the along scatter comes from.** Separation scatter (standard deviation, mm) split by how fast the shuttles move in the image, plus the change from one frame to the next (standard deviation of consecutive differences over the square root of 2):

| distance | still frames (under 0.3 px/frame) | moving frames (0.8 to 3 px/frame) | whole hold | frame to frame |
|----------|------------------|------------------|------------|----------------|
| 11 ft | 2.5 | 2.8 | 3.1 | 1.0 |
| 16 ft | 6.1 | 9.3 | 7.3 | 4.0 |
| 22 ft | 11.5 | 18.1 | 15.9 | 6.6 |
| 22 ft take 2 | 18.0 | 14.4 | 16.9 | 7.4 |
| 30 ft | 22.1 | 26.9 | 29.0 | 15.3 |

Still frames scatter about as much as moving ones, so speed (blur) is not the cause. Consecutive frames agree about twice as well as the whole hold, so about half the scatter is a slow drift over the hold: something that changes slowly, such as the stick's angle as the grip shifts (changing how tape and hand cover each skirt, which moves the centroid) or the stick flexing. Pure camera noise would be just as large frame to frame.

**Depth jitter of one point.** The along hold measures mostly the difference in depth between the two shuttles. Taking the scatter as depth noise, shared equally by the two shuttles and scaled by the stick's angle (48 to 58 degrees out of the image plane):

| distance | camera noise per point (frame to frame) | same as disparity noise | with the hand-held drift (whole hold) | same as disparity noise |
|----------|-------------|---------|-------------|---------|
| 11 ft (3.25 m) | 0.9 mm | 0.10 px | 2.7 mm | 0.31 px |
| 16 ft (5.26 m) | 3.8 mm | 0.17 px | 6.9 mm | 0.30 px |
| 22 ft (6.8 to 7.1 m) | 6 mm | 0.15 px | 14.1 mm | 0.36 px |
| 30 ft (8.7 m) | 13.4 mm | 0.21 px | 25.3 mm | 0.40 px |

Depth from stereo is Z = f B / d (focal length f = 2388 px, baseline B = 0.503 m, disparity d in px), so a disparity error moves depth by Z squared over f B per pixel. The disparity noise stays roughly constant with distance, which is why the depth jitter grows with the square of the distance.

## Answer
With the cameras recalibrated, a still shuttle's 3D points agree with a tape-measured length to within about 1 cm on average at every distance from 11 to 30 ft (+2 to +5 mm along the view, +10 to +14 mm across it). The camera's own noise moves one point's depth by 0.1 to 0.2 px of disparity per frame, 1 mm at 11 ft and 13 mm at 30 ft, growing with the square of the distance; a hand-held stick roughly doubles that through slow drift. Averaging frames or fitting a flight path is what brings single points down to millimetres.

The steady +12 mm across the view does not grow with distance, so it behaves like a fixed length difference on the stick (for example the skirt centres sitting a little further apart than the cork tips) and not like a depth error. His call: "chalk it up to experimentation error ~1cm is well within acceptable tolerence".

## Next
- This test measures depth differences between two shuttles. An error shared by both (everything reading 5 cm too far) cancels here, so absolute depth needs its own test.
- A camera turning in its mount ruins every 3D point. Fix the mounts so the cameras cannot turn, and run the row-mismatch check on a still frame at the start of every session; it needs no board and takes seconds.
- Left and right spots at each distance, to see whether the error changes across the image.
- E06 compares ways of centring the shuttle against these clips. In flight the shuttle smears into a streak (about 17 cm at 20 m/s and 8.33 ms), so E06 also scores centring on real flights with the flight fit.
- Expected depth jitter per frame at 13 m: about 3 cm from camera noise alone (0.2 px), about 5 cm with drift like the hand-held hold (0.35 px).
- A stick on a stand would separate the camera noise from the drift directly.

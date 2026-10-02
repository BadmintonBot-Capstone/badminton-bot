# E05 Tracker comparison

Status: pilot measured 2026-09-30; 3D physics tracker run offline on the E04 clips 2026-10-02

## Question
Which tracker variant finds the most flying and still frames, with the fewest false reports and the earliest pickup, within the per-frame time budget?

## Setup
Pilot: the old single-camera rally clip `rally1_e8333_g6` (2026-09-27, 8.33 ms, 6 dB, 100 fps, players 5 to 7 m out), frames 0 to 879 labelled by hand with `analysis/label.py`. Tracker `track_single` at commit cfcb478, with and without the yellow check.

3D physics tracker (2026-10-02): the 13 labelled E04 clips (11 ft mixed shots, 35 ft racket hits toward the rig, four camera settings), run offline in Python on the motion blobs `analysis/cands.py` saved for both cameras. Per frame, left blobs are paired with right blobs on the same rectified rows (gap at most 4 px, areas within 3x, disparity inside the room) and each pair is a 3D point. Two points in consecutive frames moving at 0.5 to 80 m/s start a track: an EKF with the drag plus gravity model (alpha and gravity's direction from the E04 flight fit for that range), measuring left x, left y and disparity in px. After that a track only takes the point inside its gate (chi square 16 on 3 values), at a lower threshold than a new track needs, so a fading shuttle is kept. Points must be yellow and 4 to 80 cm^2 at their depth. A track is reported once it holds enough points and moves fast enough, and from then on only in frames where it has a point that is still fast enough and whose mean prediction error over the last 10 frames is small enough; coasted frames are never reported (his rule from 2026-09-27: "guessing where it is from the camera feed is risky"). Nothing in the tracker uses the labels; the labels only score it.

## Protocol
1. Pilot: `track_single --replay <clip> --out <csv> --headless` per variant, then `analysis/score_frames.py <clip> <csv>`.
2. 3D tracker: `analysis/track3d.py` runs one setting on given clips; `analysis/track3d_sweep.py` runs a grid of 72 settings on all 13 clips and picks one by 5-fold cross validation by flight (most flying frames found live with at most a given number of false reports per minute, picked on four folds and scored on the fifth). The current tracker's `track.csv` is scored with the same rule: a report is on the shuttle when its blob's box, grown by 10 px, holds the label; any other report on a labelled frame is false; unsure frames are left out.
3. Grid: yellowness to start and to keep a track (0.3 and 0.25, or 0.35 and 0.3), points to confirm (5 or 8), measurement noise (5, 5, 3 px or 3, 3, 2 px), largest mean prediction error to report (none, 2.5, 1.5), slowest reported speed (0, 1.5, 2.5 m/s). Fixed from a first, wider grid where every fold picked the same: background model `tri`, 4 to 80 cm^2, confirm only above 2.5 m/s.

## Data
Pilot: `~/Projects/baddy-bot/recordings/rally1_e8333_g6/` (labels in `labels/left.csv`, tracker outputs `yellow.csv` and `cpp_track.csv`).

3D tracker: `~/Projects/baddy-bot/recordings/E04_settings_sweep/`, outputs in each clip's `sweep/track3d_<setting>.csv` and the grid scores in `track3d_grid.npz`. 85 flights, 45 at 11 ft and 40 at 35 ft, counting a flight that leaves the frame and comes back as two. Gravity and alpha come from fitting the same flights (E04), so the physics is tuned on the test data; a court calibration has to supply them for real.

## Results
Pilot, 880 labelled frames (546 flying, 317 still), 8 flights:

| variant | flying found | still found | bad reports | flights found | frames to first report (median, p90) |
|---------|-------------|-------------|-------------|---------------|-----------------|
| no colour check | 220 (40%) | 4 (1%) | 23 | 8/8 | 5, 8 |
| yellow check 0.3 | 214 (39%) | 4 (1%) | 1 | 8/8 | 6, 9 |

Yellow check, flying frames found by shuttle speed and by position in the flight:

| speed (full res px per frame) | frames | found |
|---|---|---|
| under 10 | 71 | 8% |
| 10 to 20 | 340 | 40% |
| 20 to 30 | 104 | 61% |
| 30 to 60 | 27 | 37% |

| frames into the flight | frames | found |
|---|---|---|
| 0 to 5 | 48 | 10% |
| 6 to 14 | 72 | 78% |
| 15 to 39 | 193 | 48% |
| 40 and later | 233 | 26% |

3D tracker against the current tracker, all 13 E04 clips, flying frames reported live (`figures/E04/tracker_tradeoff.png`; per setting in `E04_settings_sweep.md` and `figures/E04/detector_vs_tracker.png`):

| tracker | flying found live | found once a track confirms | false reports per minute | flights found |
|---|---|---|---|---|
| current | 22.5% | 22.5% | 4.1 | 72/85 |
| 3D, picked for at most 5 false/min | 32.1% | 37.5% | 7.7 | 59/85 |
| 3D, picked for at most 15 false/min | 45.4% | 53.0% | 14.7 | 65/85 |
| 3D, picked for at most 30 false/min | 51.8% | 60.0% | 41.4 | 70/85 |

No setting reached 5 false reports per minute, so the 5 row is the strictest one tried.

By range, picked for at most 15 false reports per minute:

| range | tracker | flying found live | flights found | frames from flight start to first report (median, p90) |
|---|---|---|---|---|
| 11 ft | current | 25 to 42% by setting | 33/45 | 11, 56 |
| 11 ft | 3D | 14 to 37% | 25/45 | 16, 55 |
| 35 ft | current | 9 to 19% | 39/40 | 91, 145 |
| 35 ft | 3D | 40 to 72% | 40/40 | 8, 25 |

What still fools the 3D tracker, from contact sheets of its worst tracks: his raised hand after a hit (skin passes the yellow test under this light), the next shuttle held by its cork in that hand, racket strings sweeping over the yellow bins, and at first his body near the middle of the 35 ft view. The body went once the model's slack dropped from 10 to 2 m/s^2 and yellowness, real size and a speed floor were added (one 35 ft clip went from about 400 false reports per minute to 29). On the reports kept, the shuttle and the false points differ mostly in yellowness (shuttle median 0.47, false 0.28), size (shuttle 6 to 54 cm^2 for the middle 90%) and speed (shuttle rarely under 2 m/s, false median 1.3 to 2.6 m/s).

Physics alone cannot reject a slow object quickly: the filter keeps re-estimating velocity, so something that is not falling only drifts from free fall by g dt^2, about 1 mm per frame, and at 8 m one point's depth is uncertain by about 16 cm. Telling a held object from a falling one there takes about 0.2 s of points.

## Answer
Pilot: the tracker is precise (1 bad report in 880 frames with the yellow check) but finds just 39% of flying frames, losing mostly the slow, late part of each flight. That fits its speed rule (median recent speed of at least 8 half-resolution px per frame to report).

3D tracker: with its gates in metres and the drag plus gravity model as the check, it reports 59 to 72% of flying frames live at 35 ft against 9 to 19% for the current tracker, and picks every flight up within a median of 8 frames instead of 91, at about 15 false reports per minute against 4. At 11 ft it is worse than the current tracker (14 to 37% against 25 to 42%): close tosses are slow, short and next to the thrower.

## Next
- 11 ft: the confirmation rules (8 points above 2.5 m/s) cut slow close tosses; try rules that scale with range.
- False reports: a longer free-fall test for slow tracks (held shuttles, hands), and the racket against the yellow bins.
- Gravity's direction and alpha from a calibration separate from the scored flights.
- Port to C++ in `track_single` and time it on two cameras.
- A rally clip with a second player, once someone is recruited.

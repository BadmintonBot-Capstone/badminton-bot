# E05 Tracker comparison

Status: pilot measured 2026-09-30, full run waits on E04 clips

## Question
Which tracker variant finds the most flying and still frames, with the fewest false reports and the earliest pickup, within the per-frame time budget?

## Setup
Pilot only: the old single-camera rally clip `rally1_e8333_g6` (2026-09-27, 8.33 ms, 6 dB, 100 fps, players 5 to 7 m out), frames 0 to 879 labelled by hand with `analysis/label.py`. Tracker `track_single` at commit cfcb478, with and without the yellow check.

## Protocol
1. `track_single --replay <clip> --out <csv> --headless` per variant.
2. `analysis/score_frames.py <clip> <csv>` against `labels/left.csv`.

## Data
`~/Projects/baddy-bot/recordings/rally1_e8333_g6/` (labels in `labels/left.csv`, tracker outputs `yellow.csv` and `cpp_track.csv`).

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

## Answer
Pilot only: the tracker is precise (1 bad report in 880 frames with the yellow check) but finds just 39% of flying frames, losing mostly the slow, late part of each flight. That fits its speed rule (median recent speed of at least 8 half-resolution px per frame to report). The yellow check costs almost nothing in found frames.

## Next
Work on finding more of each flight, starting with the speed rule (lower it, or replace it with a physics check once stereo is in). Score each change on these labels first, then on the E04 clips as the held-out test.

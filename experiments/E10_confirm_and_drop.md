# E10 Confirm sooner, drop quiet tracks

Status: done 2026-10-13

## Question
How few points can the 3D tracker confirm a track on, and does dropping a confirmed track that stops moving like a shuttle help, judged by how early and how close it predicts where each flight ends?

## Setup
The 3D physics tracker (`analysis/track3d.py`, E05) on the 13 labelled E04 clips, `tri` blobs, both cameras.

What good means, his call 2026-10-13: the robot needs an early, accurate prediction of where the shuttle is going, so the score is the predicted end point, not the share of frames reported.

- **Truth:** each labelled flight's drag plus gravity fit from E04 (`flightfit.npy`, 79 of 85 flights have one), at the last moment both cameras saw the shuttle. The floor height is not known, so this end point stands in for the landing point. The last raw stereo point of each flight is kept as a second truth; the two are a median of 3.3 cm apart at 11 ft and 2.2 cm at 35 ft (90th percentile 10.7 and 5.8 cm).
- **Prediction:** at 0.05, 0.1, 0.2, 0.3 and 0.5 s after the hit (the flight's first flying frame), the tracker's latest state sent with a report, on a track that reported this flight's shuttle, pushed forward with the same physics to the end time. A flight with no such state yet counts as a miss. Flights that end before a time are left out of it.
- **Score:** share of flights within 5, 10 and 20 cm, his thresholds, and false reports per minute as in E05.

Diagnosis that set the drop rule: with the E05 setting, 83 false reports came from 36 tracks. 52 of them are one or two reports at the start or end of a track that is otherwise on the shuttle, 26 come from 6 tracks that were never on it, and three tracks give 32. The track that sat on his raised hand in E09 was not reported in the normal tracker, because it moved too slowly to pass the report rule.

## Protocol
1. Grid, 864 settings: confirm after 2, 3, 4, 5, 6 or 8 points; drop a confirmed track after 3, 6 or 12 frames in a row without a reportable point, or never; and the E05 grid (yellowness 0.3/0.25 or 0.35/0.3, noise 5, 5, 3 or 3, 3, 2 px, mean prediction error at most 1.5, 2.5 or no limit, reported speed at least 0, 1.5 or 2.5 m/s).
2. For each confirm count, each drop rule and for the whole grid: pick a setting on four folds of flights (5-fold by flight, as in E05) for the most flights within 20 cm at 0.3 s with at most 15 false reports per minute, and score it on the fifth.
3. The E05 setting (confirm 8, never drop) is scored as it is, without picking.

```
~/.venvs/cpsc330/bin/python analysis/track3d_confirm.py ../recordings/E04_settings_sweep/ft* --workers 14
```

## Data
`~/Projects/baddy-bot/recordings/E04_settings_sweep/track3d_confirm.npz`: per setting and flight, the E05 scores and the end point error at each time against both truths.

## Results
79 flights with a fitted end point (39 at 11 ft, 40 at 35 ft). The time from the hit to the last stereo point is a median of 1.08 s at 35 ft and 0.59 s at 11 ft; flights that end before a time are left out of it. Rows other than the E05 setting are held-out scores from cross validation; the E05 setting was itself picked on these flights in E05, so its row is in-sample.

Flights predicted within 20 cm of the fitted end point, all ranges:

| setting | found live | false/min | 0.05 s | 0.1 s | 0.2 s | 0.3 s | 0.5 s | median error at 0.3 s |
|---|---|---|---|---|---|---|---|---|
| E05 setting (confirm 8, never drop) | 45.7% | 14.2 | 2.5% | 7.6% | 28.0% | 42.6% | 45.3% | 25.3 cm |
| confirm 2 | 35.0% | 27.2 | 1.3% | 5.1% | 12.0% | 32.4% | 37.5% | 40.0 cm |
| confirm 3 | 35.0% | 27.2 | 1.3% | 5.1% | 12.0% | 32.4% | 37.5% | 40.0 cm |
| confirm 4 | 34.4% | 16.1 | 3.8% | 7.6% | 14.7% | 35.3% | 42.2% | 34.1 cm |
| confirm 5 | 37.2% | 16.9 | 1.3% | 2.5% | 13.3% | 32.4% | 42.2% | 33.1 cm |
| confirm 6 | 39.9% | 15.6 | 2.5% | 6.3% | 17.3% | 36.8% | 45.3% | 31.9 cm |
| confirm 8 | 40.9% | 17.4 | 2.5% | 8.9% | 20.0% | 36.8% | 56.2% | 31.8 cm |
| never drop | 40.7% | 14.5 | 2.5% | 6.3% | 17.3% | 39.7% | 53.1% | 27.9 cm |
| drop after 3 | 39.5% | 11.1 | 2.5% | 3.8% | 20.0% | 36.8% | 42.2% | 27.4 cm |
| drop after 6 | 40.9% | 18.6 | 1.3% | 7.6% | 21.3% | 36.8% | 48.4% | 31.6 cm |
| drop after 12 | 42.9% | 14.5 | 3.8% | 6.3% | 21.3% | 35.3% | 46.9% | 26.3 cm |

At 0.3 s, within 5 and 10 cm (fitted end point; last raw stereo point in brackets):

| setting | within 5 cm | within 10 cm | within 20 cm |
|---|---|---|---|
| E05 setting | 1.5% (4.4%) | 17.6% (10.3%) | 42.6% (38.2%) |
| confirm 4 | 0.0% (1.5%) | 10.3% (7.4%) | 35.3% (33.8%) |
| confirm 8 | 0.0% (1.5%) | 14.7% (7.4%) | 36.8% (33.8%) |

E05 setting by range at 0.3 s: within 20 cm 44.8% at 11 ft and 41.0% at 35 ft; within 10 cm 31.0% and 7.7%; within 5 cm 3.4% and 0.0%.

Where the error comes from, E05 setting, latest state at 0.3 s after the hit, medians:

| range | state age | predicting ahead | position off | velocity off | end point error | with the true velocity | with the true position |
|---|---|---|---|---|---|---|---|
| 11 ft | 10 ms | 0.35 s | 2.3 cm | 0.33 m/s | 15.2 cm | 2.7 cm | 12.5 cm |
| 35 ft | 10 ms | 0.81 s | 6.0 cm (4.7 in depth) | 0.43 m/s | 25.1 cm | 6.6 cm | 18.8 cm |

"With the true velocity" keeps the tracker's position and swaps in the fitted flight's velocity at the same moment; "with the true position" does the reverse.

## Answer
Confirming sooner does not help: 2 to 6 points give fewer flights within 20 cm at 0.3 s than 8 (32 to 37% against 37 to 43%) and more false reports (27 per minute at 2 or 3). Dropping quiet tracks after 3 frames cuts false reports from 14.5 to 11.1 per minute without better predictions. The predictions themselves are the limit: 0.3 s after the hit the median end point error is 25 cm at 35 ft, within 5 cm almost never, and most of it comes from the tracker's velocity (0.43 m/s off).

## Next
- Better velocity: the EKF's velocity after about 30 points is still 0.3 to 0.4 m/s off. Candidates: fit the whole track so far with the drag plus gravity model (as `flightfit.py` does offline) instead of the filter's running estimate, and retune the filter's model noise (`sig_a` 2 m/s^2) now that alpha and gravity are fitted.
- Confirm 2 and 3 are the same setting: a track starts from 2 points and the confirm check only runs when a third arrives.
- Robot simulation (his idea, 2026-10-13): the year 1 spec, 2.36 m/s^2 and 2.95 m/s with a triangle profile, chasing the latest prediction from rings of start offsets, scored within 5, 10 and 20 cm at the deadline.

# E04 Settings sweep

Status: planned (needs E02 first, for the matched gains)

## Question
Which camera settings (exposure, gain, frame rate, binning) track real flights best at 3, 7, 10 and 13 m?

## Setup
Both cameras on the trigger, calibration from commit bf409c1. One change at a time from the baseline:

| setting | exposure | gain | fps | resolution |
|---------|----------|------|-----|-----------|
| baseline | 8.33 ms | 6 dB | 100 | full |
| short exposure | 4 ms | matched (E02) | 100 | full |
| shorter exposure | 2 ms | matched (E02) | 100 | full |
| higher frame rate | 4 ms | matched (E02) | 150 | full |
| camera binning | 8.33 ms | per E02 | 100 | 2x2 binned |

The 150 fps clips are compared with the 4 ms clips at 100 fps, because 8.33 ms does not fit in a 150 fps frame. Exposures other than 8.33 ms bring the lab flicker back, and that cost is part of what this measures.

Ranges: 3 m (clear zone), 7 and 10 m (aisle, shots kept above the table line since only the upper half of the image sees past 5 m), 13 m (corridor after turning the rig 90 degrees, toward and away shots only since it is about 1 m wide).

Shots, all solo: tosses across the view, tosses toward and away from the cameras, and solo racket hits.

## Protocol
1. Set the rig for the range. Note the range in the clip notes.
2. For each setting, record 30 s clips until there are 10 flights at that range, mixing the three shot types evenly (fewer types in the corridor).
3. Keep the order of settings the same at every range, and record the baseline again at the end of each range to catch drift in the light.
4. Label every clip in the left view with `analysis/label.py`, and the right view for one clip per range.
5. Run the current tracker (`track_single --replay <clip> --camera left`) on every clip and score it with `analysis/score_frames.py`.

About 24 clips and 330 GB raw in total.

## Data
`~/Projects/baddy-bot/recordings/E04_settings_sweep/`

## Results
One table per range, one row per setting: flying frames found, still frames found, false reports per minute, flights detected, frames from flight start to first report, ms per frame, and flight-fit scatter in 3D.

## Answer

## Next
The best setting becomes the new baseline for E05 and E06, and the ranking by range feeds the court routine (E07).

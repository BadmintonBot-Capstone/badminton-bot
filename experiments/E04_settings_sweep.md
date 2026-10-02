# E04 Settings sweep

Status: running (started 2026-10-01)

## Question
Which camera settings (exposure, gain, frame rate) track real flights best at 11, 22 and 30 ft?

## Setup
Both cameras on the trigger, rig in the wide aisle view used in E02 (past 16 ft only the upper half of the image sees over the table). Evening light with the windows dark, the light the E02 gains were matched in.

**Recalibrated after tightening the camera mounts** (2026-10-01 evening): `data/calibration_images_2026-10-01_mounts`, 63 of 66 pairs, RMS 0.22 / 0.31 / 0.50 px, focal 2422 px, baseline 504.3 mm, now `config/calibration.yaml`. Tightening turned a camera slightly sideways. The row check from E03 (rectify, match scene features, compare their rows) missed it, because a sideways turn slides features along the rows and leaves the rows matched; what it changes is the left to right gap, and with it every depth. Triangulating the board corners in each calibration pair and comparing the board's size with its true size caught it:

| calibration | board size on the 2026-10-01 pairs (before tightening) | on the pairs after tightening |
|-------------|------------------------------|------------------------------|
| 2026-10-01 | -0.02% | +7.9% |
| 2026-10-01 after tightening | -8.1% | -0.10% |

The higher stereo RMS (0.50 against 0.29) is spread evenly over the capture (0.41 to 0.45 px per quarter), so the cameras did not move during it; the corners were found a little less precisely in every pair.

Every clip is stereo with both cameras at the same setting, so each flight also gives 3D points for fitting the trajectory (alpha) and for testing the EKF once E05 has picked the detection method. His words: "we should always be doing 3d for these, so we can start characterizing our trajectory afterwards as well (finding alpha, for example) and testing the EKF after we pick the best detection method for these".

One change at a time from the baseline, gains matched to the baseline brightness in E02:

| setting | exposure | gain | fps |
|---------|----------|------|-----|
| baseline | 8.33 ms | 6 dB | 100 |
| short exposure | 4 ms | 12.3 dB | 100 |
| shorter exposure | 2 ms | 18.3 dB | 100 |
| higher frame rate | 4 ms | 12.3 dB | 150 |

The 150 fps clips are compared with the 4 ms clips at 100 fps, because 8.33 ms does not fit in a 150 fps frame. Exposures other than 8.33 ms bring the lab flicker back (E01), which is part of what this measures. Camera binning waits until `record_stereo` can set it.

## Protocol
1. Two batches in the same aisle view: 11 ft with mixed shots, then racket hits toward the rig from the 30 ft mark. The second batch replaced the 22 and 30 ft mixed-shot ranges, his call after 11 ft: hits toward the camera from far away are what the robot will mostly see, and racket hits fly at real speed. Turning the rig to the corridor was considered and dropped, since handling the rig risks moving the cameras again; the cameras sit high, so the last part of some flights near the rig leaves the view.
2. Per batch, 8 clips: the four settings in the order above, then all four again, so drift in the light or in the throwing lands on every setting alike.
3. 11 ft, 30 s clips of 5 flights: 2 tosses across the view at different heights, 1 toss along it toward the rig, 1 along it away from the rig, 1 racket hit across. The thrower stands where the left camera just sees him at its right edge and throws toward the left, so flights cross the open part of the view instead of passing in front of the thrower. Each flight starts inside both views and may leave them; throws are at normal speed, since slow throws would tilt the test toward slow shuttles.
4. 35 ft (the 30 ft mark after the rig moved back), 40 s clips of 8 racket hits toward the rig, shuttles picked up between clips, each landing short of the rig, since a shuttle hitting a camera would move it.
5. After each clip, `analysis/trail.py` draws everything yellow that moved in both cameras, coloured by time (blue early, red late). A clip is retaken if its flights did not stay in both views.
6. Label every clip in the left view with `analysis/label.py`, and the right view for one clip per range. Frames where the shuttle is visible but not flying (in the hand, on the racket, on the floor) are labelled still with a position, so tracker reports on a held shuttle are judged instead of ignored. That is most of the labelling time (clip 1: 3000 frames in 12 minutes, 1450 of them still), his call: "i want the report to cover when the shuttle was still in my hand".
7. Run the current tracker (`track_single --replay <clip> --camera left`) on every clip and score it with `analysis/score_frames.py`.

```
./build/ninja/record_stereo --out ../recordings/E04_settings_sweep/ft<range>_c<n>_e<us>_g<db>[_f150] --exposure <us> --gain <db> --fps <100|150> --seconds 30 --delay 15 --note experiment=E04 --note range_ft=<range> --note round=<1|2> --note setting=<name> --note shots=across_low,across_high,along_toward,along_away,racket_across --note calibration=calibration_images_2026-10-01_mounts
~/.venvs/cpsc330/bin/python analysis/trail.py ../recordings/E04_settings_sweep/<clip>
```

16 clips, 40 tossed or hit flights at 11 ft and 64 hits from 35 ft, about 290 GB raw (13.8 GB per 30 s at 100 fps, 20.7 GB per 30 s at 150 fps).

## Data
`~/Projects/baddy-bot/recordings/E04_settings_sweep/`, recorded 2026-10-01 23:44 to 2026-10-02 00:40, no frames dropped in any clip. Each clip folder holds `<clip>_trail.jpg`.

- 11 ft, all 8 clips (`ft11_c1` to `ft11_c8`). The first take of `ft11_c1_e8333_g6` (15 s) was deleted and retaken: only four flights fit, most passed in front of the thrower, and the thrower stood outside the left camera's view. The first take of `ft11_c3_e2000_g18.3` was retaken on his call (the racket hit was missing and both along tosses passed outside the left view). The toward toss in `ft11_c4` and `ft11_c7` only clips the left camera's right edge.
- 35 ft, clips 1 to 5 of 8 (`ft35_c1` to `ft35_c5`; clips 6 to 8 not recorded, stopped on his call). Before this batch the rig was moved about 5 ft back so whole arcs fit, which turned the 30 ft mark into about 35 ft; the two clips first saved as `ft30_` were renamed and their range note corrected. In `ft35_c1` one hit struck the ceiling and one bounced on a table, so those two are left out of the trajectory fit.
- Rows still line up after the move (row mismatch 1.0 px against 0.9 px at 11 ft). The board-size check after the move was skipped, his call: "forget the board check, i trust the rig very much".- Labels: left view of all 13 clips, labelled by hand 2026-10-02 00:49 to 02:40 (1 h 34 min; the first clip took 12 minutes, the rest 5 to 10), with the current tracker's reports loaded as starting guesses. 43 flights at 11 ft (some tosses leave the top of the frame and come back as a second flight) and 40 at 35 ft. Late in the session `ft35_c2` to `ft35_c4` got many unsure frames where the shuttle was visible (3571 of 5996 in `ft35_c4`), his words: "i got lethargic and i saw too many frames with the birdies so i just said unsure"; those frames are left out of every score.
- Label check: every flying label was compared with a moving shuttle-coloured blob in the image, and every tracker report on a frame labelled not visible was looked at. All flying labels sit on the shuttle except a few at the very end of flights leaving the frame edge (left as they are, his call). The reports on not-visible frames at 11 ft were all tracker false reports on the hand, chin, scarf or racket. At 35 ft, a landed shuttle often tumbled back into view at the bottom of the frame for 7 to 40 frames and had been labelled not visible; 13 such stretches (337 frames) were relabelled still from the image, the position taken from the moving blob followed from where the flight ended, each stretch checked by eye on a contact sheet (source `auto_bounce`; the labels before this are in `labels/left_before_bounce.csv`). Also fixed in `ft35_c2`: frames 1696 to 1697 (carried 42 px off the shuttle, now interpolated) and frame 2288 (a stray one-frame flight, now still).
- At 2 ms the trails show far more speckle over bright walls and windows than at the other settings, most likely the flicker returning plus the higher gain's noise (to be confirmed by the scoring).

## Results
One table per range, one row per setting: flying frames found, still frames found, false reports per minute, flights detected, frames from flight start to first report, ms per frame, and flight-fit scatter in 3D.

## Answer

## Next
The best setting becomes the new baseline for E05 and E06, the ranking by range feeds the court routine (E07), and the 3D flights feed the trajectory fit (alpha) and the EKF tests.

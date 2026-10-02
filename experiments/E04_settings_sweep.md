# E04 Settings sweep

Status: done 2026-10-02 (35 ft clips 6 to 8 not recorded)

## Question
Which camera settings (exposure, gain, frame rate) track real flights best at 11, 22 and 30 ft? (Run at 11 and 35 ft, see Protocol.)

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
- At 2 ms the trails show far more speckle over bright walls and windows than at the other settings. The scoring confirms the cause is noise and flicker (the typical background change doubles, 0.5 to 1.0 grey levels) and that it costs nothing in detection.
- Second label check (2026-10-02, after the trackers were run): every stretch labelled not visible where either tracker reported was looked at, 21 stretches. 19 were real false reports (racket strings over the yellow bins, hand, sleeve, the end of a light tube, chin). 2 had the shuttle visible in his hand and were relabelled still (`ft35_c2` 1174 to 1194, `ft35_c5` 1665 to 1682; source `auto_inhand`, the labels before this in `labels/left_before_inhand.csv`). His words on the cause: "yea i did that a lot".

## Analysis
Every scoring step below runs offline on the recorded clips and is redone by the scripts in `../analysis/` (README there): `cands.py` (every motion blob per frame, both cameras), `contrast.py`, `sweep.py`, `flightfit.py`, `track3d.py`, `track3d_sweep.py`, then `figures_e04.py` for the figures in `figures/E04/`. Saved results sit next to the clips: `sweep_left.npz`, `flightfit.npy`, `track3d_grid.npz`.

Anything tuned is tuned by 5-fold cross validation by flight: each flight goes into a fold with the frames before it (hand, wind up, pick up), the choice is made on four folds and scored on the fifth, so every flight is scored once by a choice that never saw it. Ranges in brackets are 95% intervals from resampling flights. A frame counts as found when a reported blob's box, grown by 10 px, holds the label (the old 30 px radius misses close shuttles, whose label sits on the cork far from the blob's centre).

1. Contrast: at every frame labelled flying, the largest change near the label against the typical change over the frame, with no detector choices.
2. Detector: is the shuttle among the frame's motion blobs, and how many other blobs come with it. 7680 variants (background model, threshold, blob size, yellowness, right camera check), tuned per setting.
3. Flight fit: each labelled flight triangulated (left blob holding the label paired with the right blob on the same rectified rows) and fitted with the EKF's model, a = g - alpha |v| v. Alpha and the direction of gravity are fitted per range, the start of each flight per flight, in pixels so near and far points weigh by their real noise; the first 3 frames of each flight are left out while the shuttle turns over.
4. Trackers: the current tracker (`track.csv`), and a 3D physics tracker built for this (E05), cross validated over 72 settings and picked for at most 15 false reports per minute.

## Results
Contrast, running average background, every labelled flying frame (`figures/E04/contrast.png`):

| range | setting | frames | shuttle change, median (grey levels) | background change, median | frames above the tracker's threshold (12) |
|---|---|---|---|---|---|
| 11 ft | 8.33 ms, 6 dB | 593 | 49 | 0.51 | 99% |
| 11 ft | 4 ms, 12.3 dB | 609 | 59 | 0.74 | 100% |
| 11 ft | 2 ms, 18.3 dB | 576 | 62 | 1.01 | 100% |
| 11 ft | 4 ms, 150 fps | 893 | 59 | 0.74 | 100% |
| 35 ft | 8.33 ms, 6 dB | 1648 | 64 | 0.51 | 99% |
| 35 ft | 4 ms, 12.3 dB | 851 | 71 | 0.75 | 100% |
| 35 ft | 2 ms, 18.3 dB | 852 | 72 | 1.03 | 100% |
| 35 ft | 4 ms, 150 fps | 1297 | 66 | 0.75 | 100% |

Detector, flying frames with the shuttle among the blobs (`figures/E04/found_vs_junk.png`, `detector_hits_sample.jpg`, `detector_misses_sample.jpg`):

| range | setting | current detector | false blobs per frame | tuned, at most 0.3 false blobs per frame |
|---|---|---|---|---|
| 11 ft | 8.33 ms, 6 dB | 80% [63-93] | 2.7 | 78% [65-89] |
| 11 ft | 4 ms, 12.3 dB | 81% [61-96] | 2.3 | 80% [61-95] |
| 11 ft | 2 ms, 18.3 dB | 87% [72-95] | 2.1 | 86% [81-89] |
| 11 ft | 4 ms, 150 fps | 86% [71-96] | 2.2 | 86% [78-91] |
| 35 ft | 8.33 ms, 6 dB | 94% [91-96] | 1.7 | 85% [81-89] |
| 35 ft | 4 ms, 12.3 dB | 95% [92-98] | 1.9 | 89% [85-93] |
| 35 ft | 2 ms, 18.3 dB | 94% [89-98] | 1.8 | 92% [88-95] |
| 35 ft | 4 ms, 150 fps | 97% [93-100] | 1.8 | 89% [82-96] |

Requiring a matching blob in the right camera removes about 40% of the false blobs but loses 6 to 16 points of shuttle frames, and never wins at any number of false blobs: the false blobs are his hand, arm and racket, which both cameras see.

Flight fit (`figures/E04/flights_35ft.png`, `alpha_per_flight.png`):

| range | flights | points | alpha (per m) | terminal speed | alpha per flight, middle half | gravity off the image's down axis |
|---|---|---|---|---|---|---|
| 11 ft | 39 | 1661 | 0.231 | 6.52 m/s | 0.215 to 0.345 | 0.7 degrees |
| 35 ft | 40 | 3753 | 0.208 | 6.87 m/s | 0.200 to 0.221 | 3.4 degrees |

With the strength of gravity also left free the fit gives 9.56 m/s^2 at 35 ft and 9.42 at 11 ft, and alpha 0.186 at both, so measured distances are right to a few percent. The rig tipped about 3 degrees when it moved back.

3D scatter of flying points about their fitted flight, median, points over 15 px off left out (`figures/E04/scatter_3d.png`):

| range | setting | across the view | along the view (depth) | points over 15 px off |
|---|---|---|---|---|
| 11 ft | 8.33 ms, 6 dB | 9 mm | 20 mm | 30% |
| 11 ft | 4 ms, 12.3 dB | 9 mm | 21 mm | 20% |
| 11 ft | 2 ms, 18.3 dB | 9 mm | 19 mm | 18% |
| 11 ft | 4 ms, 150 fps | 12 mm | 30 mm | 31% |
| 35 ft | 8.33 ms, 6 dB | 11 mm | 29 mm | 4% |
| 35 ft | 4 ms, 12.3 dB | 10 mm | 29 mm | 4% |
| 35 ft | 2 ms, 18.3 dB | 10 mm | 25 mm | 6% |
| 35 ft | 4 ms, 150 fps | 15 mm | 36 mm | 3% |

The points are centred on the part of the shuttle that moved since both neighbouring frames; at 150 fps it moves less per frame, so that part is smaller and jumpier. The 150 fps rows measure that centring as much as the camera (E06).

Trackers, flying frames reported live (`figures/E04/detector_vs_tracker.png`, `tracker_tradeoff.png`):

| range | setting | current tracker | false/min | flights | 3D tracker | false/min | flights |
|---|---|---|---|---|---|---|---|
| 11 ft | 8.33 ms, 6 dB | 42% [22-64] | 4.1 | 10/12 | 21% [9-32] | 32.6 | 6/12 |
| 11 ft | 4 ms, 12.3 dB | 42% [24-61] | 0.0 | 8/10 | 28% [12-46] | 1.2 | 6/10 |
| 11 ft | 2 ms, 18.3 dB | 40% [22-56] | 15.7 | 8/12 | 37% [16-54] | 7.3 | 7/12 |
| 11 ft | 4 ms, 150 fps | 25% [13-37] | 4.4 | 7/11 | 14% [3-28] | 14.2 | 6/11 |
| 35 ft | 8.33 ms, 6 dB | 16% [12-20] | 0.9 | 15/16 | 59% [48-68] | 6.6 | 16/16 |
| 35 ft | 4 ms, 12.3 dB | 19% [15-21] | 0.0 | 8/8 | 70% [58-80] | 36.0 | 8/8 |
| 35 ft | 2 ms, 18.3 dB | 16% [13-20] | 0.0 | 8/8 | 72% [65-78] | 13.7 | 8/8 |
| 35 ft | 4 ms, 150 fps | 9% [7-11] | 0.0 | 8/8 | 40% [32-48] | 18.5 | 8/8 |

## Answer
In lab light every setting shows the flying shuttle far above the noise (median change 49 to 72 grey levels against a background of 0.5 to 1), and the motion detector holds it in 93 to 97% of flying frames at 35 ft whatever the setting. The settings only separate once a tracker has to keep the shuttle: at 35 ft, 4 ms and 2 ms kept 70 and 72% against 59% for 8.33 ms (8 flights each, intervals overlapping with the baseline's), and 150 fps did worst with the current centring. Alpha is 0.19 to 0.21 per m (terminal speed 6.9 to 7.3 m/s), next to the EKF's 0.215.

## Next
- Suggestion: 4 ms at 12.3 dB as the working setting for E05 and E06. It ties with 2 ms and has half its noise; the flicker it brings back never showed in the scores.
- The 35 ft ranking rests on one clip per setting outside the baseline; clips 6 to 8 (round 2) would double it.
- 150 fps gets a second look once E06 has a centring that does not depend on how far the shuttle moved between frames.
- The EKF needs gravity's direction in its own frame (the rig tilt moved it by 3 degrees here) and alpha from the 35 ft fit; the 11 ft tosses pin alpha down poorly.
- The tracker work (11 ft, held shuttles, hands) continues in E05; the ranking by range feeds the court routine (E07).

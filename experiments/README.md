# Experiments

Each experiment answers one question and gets one report here, written with the headings in `TEMPLATE.md` and filled in as it runs. Raw clips are too big for git and live in `~/Projects/baddy-bot/recordings/E<nn>_<name>/<clip>/`. The scripts that turn clips into results are in `../analysis/`.

The goals behind the series:

1. Camera settings that find the most shuttle frames with the fewest false reports.
2. The tracking algorithm that finds the shuttle in flight best, picks it up soonest after a hit, and keeps up live on two cameras.
3. The accuracy of each triangulated 3D point, and the processing that improves it.
4. A short routine to repeat at the court once a slot is booked.

Detection is scored in every frame where the shuttle is visible, flying and still separately. Flight start is scored per hit: found or not, and how many frames late.

| id | question | status | answer |
|----|----------|--------|--------|
| E00 | Tools: stereo recorder, labeller, scorer | built | `record_stereo`, `analysis/label.py`, `analysis/score_frames.py`; recorder tested at 1000 pairs, none dropped |
| E01 | Is the lab flicker 120 Hz or a multiple of it? | done | 120 Hz: 2.4% brightness swing at 1 ms, 0.09% at 8.33 ms; only whole multiples of 8.33 ms cancel it |
| E02 | How do noise and brightness change with gain, exposure and camera binning? | done (no binning) | Matched gains 12.3 dB at 4 ms, 18.3 dB at 2 ms; noise doubles per 6 dB but never trips the tracker, even at 24 dB |
| E03 | How far off is a still shuttle's 3D position at 3 to 13 m? | done (centre, 11 to 30 ft) | Within about 1 cm on average at every distance; camera noise moves one point's depth 1 mm at 11 ft to 13 mm at 30 ft per frame, about double with a hand-held stick. Found a camera turned 0.5 degrees in its mount, recalibrated |
| E04 | Which exposure, gain and frame rate track flights best at 11 and 35 ft? | done (no binning, 35 ft round 2 partial) | Every setting shows the shuttle far above the noise and the detector holds it in 93 to 97% of flying frames at 35 ft; through the 3D tracker 4 ms and 2 ms kept 70 and 72% against 59% for 8.33 ms (8 flights each). Alpha 0.19 to 0.21 per m |
| E05 | Which tracker variant scores best on the labelled E04 clips? | 3D tracker run offline | Current tracker finds 22% of flying frames live at 4 false reports per minute; the physics-gated 3D tracker finds 45% at 15 per minute, 59 to 72% at 35 ft against 9 to 19%, but is worse at 11 ft |
| E06 | Which way of centring the shuttle gives the most accurate 3D points? | planned | |
| E07 | What has to be rerun under court light? | after booking | |
| E08 | Does jtandjung's MOG2 detector beat `ra`, `prev` and `tri` on the E04 clips? | done (left camera) | No: colour MOG2 finds 87.2% at 0.3 false blobs per frame against 87.5% for `tri` and 89.5% for `prev`, and drops to 53.9% at 0.1. OpenCL on the 2070 runs it in 5 ms from Python |
| E09 | Does detecting only in a crop around the predicted shuttle cut false reports and processing without losing it? | done (`tri`) | Small crops (40 to 80 px circles, ellipses, 10 to 20 cm) give 43.6% found live at 10.1 false reports per minute against 45.7% at 14.2 with none; shape and size barely matter. Saves 10% of pixels on average, none on the worst frame. Costs come from a track on the raised hand holding the crop |
| E10 | How few points can a track confirm on, and does dropping quiet tracks help, judged by the predicted end point? | done | Neither helps: confirm 8 stays, dropping after 3 quiet frames cuts false reports 14.5 to 11.1 per minute only. 0.3 s after the hit the predicted end point is a median 25 cm off at 35 ft, mostly from the velocity estimate (0.43 m/s off) |

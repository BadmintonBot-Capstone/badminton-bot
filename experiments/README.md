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
| E02 | How do noise and brightness change with gain, exposure and camera binning? | planned | |
| E03 | How far off is a still shuttle's 3D position at 3 to 13 m? | planned | |
| E04 | Which exposure, gain, frame rate and binning track flights best at 3 to 13 m? | planned | |
| E05 | Which tracker variant scores best on the labelled E04 clips? | pilot done | Pilot on the old rally: precise (1 bad report in 880 frames) but finds only 39% of flying frames, losing the slow late part of each flight |
| E06 | Which way of centring the shuttle gives the most accurate 3D points? | planned | |
| E07 | What has to be rerun under court light? | after booking | |

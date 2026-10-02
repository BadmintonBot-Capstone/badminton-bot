# analysis scripts

Python for turning recorded clips into results. Every script takes a clip folder as its argument and reads both clip layouts: `record_stereo` (`left.raw`, `right.raw`) and the older `record_single` (`frames.raw`). Clips live outside git in `~/Projects/baddy-bot/recordings/`. Any Python with numpy, opencv and pyyaml works (`~/.venvs/cpsc330` does).

## Labelling and scoring

| script | what it does |
|--------|--------------|
| `label.py` | per-frame labelling, keyboard first: `label.py <clip> [--camera left\|right] [--proposals <track csv>]`. Keys are listed at the top of the file and on screen. Saves `<clip>/labels/<camera>.csv` |
| `score_frames.py` | scores a `track_single --out` csv against those labels: flying and still frames found, false reports, flights found and frames late, ms per frame |

A typical loop for one clip:

1. `vision/build/ninja/track_single --config vision/config --replay <clip> --camera left --out <clip>/track.csv --headless`
2. `python label.py <clip> --proposals <clip>/track.csv`. The tracker's reports give label.py a starting guess; check every one, they are wrong sometimes.
3. `python score_frames.py <clip> <clip>/track.csv`

Labels in `label.py`: `f` flying, `s` visible but still (hand, racket, floor), `n` not visible, `u` unsure. A state key labels the frame and moves on, a click sets the position first, space plays forward applying the current state to unlabelled frames, `i` fills short gaps inside a flight by fitting a curve through the labelled frames around them.

## Settings sweep, flight fit and 3D tracker (E04, E05)

Run in this order on labelled stereo clips; each later script reads what the earlier ones saved. All but `cands.py` take seconds to a minute.

| script | what it does |
|--------|--------------|
| `cands.py` | every motion blob in every frame for three background models (running average, last frame, both neighbouring frames) at eight thresholds, saved to `<clip>/sweep/<camera>.npz`. One read of the clip, about 8 minutes per 30 s clip; run 7 clips at a time, once per camera |
| `contrast.py` | how far the flying shuttle's change stands out from the typical background change, per setting, range and image speed |
| `sweep.py` | scores 7680 detector variants (model, threshold, blob size, yellowness, right camera check) with 5-fold cross validation by flight, tuned per setting; saves `sweep_<camera>.npz` next to the clips |
| `flightfit.py` | triangulates every labelled flight and fits drag plus gravity: alpha and gravity's direction per range, each flight's start, 3D scatter per setting. `tools/fit_alpha.py` is the older fit, alpha only, gravity straight down y |
| `track3d.py` | physics-gated 3D tracker run offline on the saved blobs of both cameras (EKF with the fitted drag and gravity, gates in metres), scored with the current tracker against the labels |
| `track3d_sweep.py` | runs `track3d.py` over a grid of settings and picks one by cross validation for a given number of false reports per minute; saves `track3d_grid.npz` |
| `figures_e04.py` | draws the E04 and E05 figures into `experiments/figures/E04/` from the saved results |

## Older tools (2026-09-27 and 09-30)

| script | what it does |
|--------|--------------|
| `rec.py` | shared loader: memory-mapped frames, 2x2 Bayer average to half resolution grey, debayer for viewing |
| `trail.py` | where yellow things moved over a stereo clip, both cameras side by side and coloured by time (`<clip>/<clip name>_trail.jpg`); the check after each E04 recording that the flights stayed in both views |
| `separation.py` | 3D distance between the two shuttles on the E03 stick in every frame, rectifying and triangulating the way `src/core/stereo.cpp` does, with check sheets of used and dropped frames |
| `noise.py` | brightness, noise, saturation and the tracker's false motion for each still clip, plus the gain at each exposure that matches a baseline clip's brightness (E02) |
| `flicker.py` | brightness swing and its strongest frequency for each still clip given, plus where each multiple of 120 Hz would show at the clip's fps (E01); needs scipy, so run it with `~/.venvs/cpsc330/bin/python` |
| `sheet.py` | contact sheet: `sheet.py <clip> <step> <start> <stop>` |
| `proto.py` | first tracker prototype, single track |
| `proto2.py` | the prototype that got ported to C++ (`src/shuttle_tracker.hpp`) |
| `segsheet.py` | one tile per tracked segment with the path drawn: `segsheet.py <clip> proto2` or `segsheet.py <clip> track.csv` |
| `colour.py` | colour of labelled shuttle points against false points; picked the yellow threshold |
| `score.py` | the first scoring scheme: labels only the points some tracker reported (`truth.csv`, `false.csv`, `skip.csv` in `throws2_e8333_g6` and `rally1_e8333_g6`), so it cannot count misses. Superseded by `label.py` and `score_frames.py` |

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

## Older tools (2026-09-27 and 09-30)

| script | what it does |
|--------|--------------|
| `rec.py` | shared loader: memory-mapped frames, 2x2 Bayer average to half resolution grey, debayer for viewing |
| `noise.py` | brightness, noise, saturation and the tracker's false motion for each still clip, plus the gain at each exposure that matches a baseline clip's brightness (E02) |
| `flicker.py` | brightness swing and its strongest frequency for each still clip given, plus where each multiple of 120 Hz would show at the clip's fps (E01); needs scipy, so run it with `~/.venvs/cpsc330/bin/python` |
| `sheet.py` | contact sheet: `sheet.py <clip> <step> <start> <stop>` |
| `proto.py` | first tracker prototype, single track |
| `proto2.py` | the prototype that got ported to C++ (`src/shuttle_tracker.hpp`) |
| `segsheet.py` | one tile per tracked segment with the path drawn: `segsheet.py <clip> proto2` or `segsheet.py <clip> track.csv` |
| `colour.py` | colour of labelled shuttle points against false points; picked the yellow threshold |
| `score.py` | the first scoring scheme: labels only the points some tracker reported (`truth.csv`, `false.csv`, `skip.csv` in `throws2_e8333_g6` and `rally1_e8333_g6`), so it cannot count misses. Superseded by `label.py` and `score_frames.py` |

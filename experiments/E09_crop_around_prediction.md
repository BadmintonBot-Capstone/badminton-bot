# E09 Crop around the predicted shuttle

Status: done 2026-10-13 (`tri` only)

## Question
While a track is confirmed, does detecting only inside a crop around the tracker's predicted shuttle position cut false reports and pixels processed without losing the shuttle, and which crop shape and size do best?

## Setup
The 3D physics tracker (`analysis/track3d.py`, E05) on the 13 labelled E04 clips, at the setting E05 picked most often (3 of 5 folds) for at most 15 false reports per minute: yellowness 0.35 to start a track and 0.3 to keep it, confirmed after 8 points, measurement noise 5, 5, 3 px, mean prediction error at most 2.5, reported speed at least 2.5 m/s. His rules, 2026-10-13:

- The crop is for both speed and fewer false reports.
- Full frames are processed only while no track is confirmed. Once a track confirms, both cameras are cropped around its prediction, and full frames come back when it dies.

How the detector behaves inside a crop depends on its memory:

| detector | memory | in a crop |
|---|---|---|
| `tri` | none: frames n-1, n and n+1 only | works anywhere the crop moves; its blobs inside the crop are the blobs it finds on the full frame, so the crop can be simulated from the saved E04 tables |
| `ra` | running average, alpha 0.05 (about 20 frames) | pixels outside the crop stop updating; needs a rerun from raw frames |
| MOG2, history 100 | about 100 frames | same as `ra`, over a longer memory |

Three ways to handle the memory outside the crop: keep updating it cheaply (every Nth frame, or at lower resolution), freeze it and accept a burst of false blobs when full frames return, or use `tri`, which has no memory.

## Protocol
1. `tri` from the saved blob tables (`cands.py`, both cameras). While a track is confirmed, the tracker sees only the stereo points whose left point lies in the left crop and whose right point lies in the right crop, and no new track starts. The right crop sits at the predicted left x minus the predicted disparity. A point counts as inside when its blob's centre is inside.
2. Crop shapes and sizes, in rectified full resolution px:
   - circle of fixed radius: 40, 80, 160, 320 px;
   - box of the same half widths;
   - ellipse from the tracker's predicted error at 3, 4, 5 and 6 standard deviations (it grows after missed frames and shrinks with distance);
   - circle of fixed radius in metres, converted to pixels at the predicted depth: 10, 20, 40 cm;
   - `rule`: the whole frame stays searched and only the no new tracks rule acts, to separate the rule from the crop.
3. If two tracks are confirmed at once, the crop is the union of their crops.
4. Score each variant with the E05 rules: flying frames found live, false reports per minute, flights found, and frames from flight start to first report. Also: flying frames searched in a crop with the label outside every crop, and the share of both full frames searched, each crop grown by 20 px for the blur and the blob's own size. The worst frame is always a full frame, whenever no track is confirmed.
5. Only if E08 shows MOG2 clearly ahead of `tri` on these clips: rerun `ra` and MOG2 from raw frames, both cameras, with the tracker in the loop, once with the outside memory frozen and once updated every Nth frame. (E08: it is not, so this step was not run.)

```
~/.venvs/cpsc330/bin/python analysis/track3d_crop.py ../recordings/E04_settings_sweep/ft* --workers 4
```

## Data
`~/Projects/baddy-bot/recordings/E04_settings_sweep/`: reports in each clip's `sweep/track3d_e09_<variant>.csv`, the crops searched in `sweep/track3d_e09_<variant>_crops.csv`, per flight scores in `track3d_crop.npz`. With the crop off, `track3d.py` reproduces the E05 output byte for byte (checked on `ft35_c1_e8333_g6`). Viewer: `e09_viewer/index.html` (two flights at 35 ft, any two variants side by side on the real frames).

## Results
Every variant, all 85 flights (found counts live reports; outside is flying frames searched in a crop with the label outside every crop):

| crop | found live | false/min | flights | frames to first report, median, p90 | outside | frames searched | frames in crop mode |
|---|---|---|---|---|---|---|---|
| none | 45.7% | 14.2 | 67/85 | 9, 43 | | 100% | 0% |
| rule | 44.6% | 13.0 | 66/85 | 9, 44 | | 100% | 0% |
| circle 40 px | 43.6% | 10.1 | 66/85 | 10, 56 | 4.3% | 90.4% | 9.6% |
| circle 80 px | 43.6% | 10.1 | 66/85 | 10, 56 | 3.9% | 90.5% | 9.6% |
| circle 160 px | 43.9% | 13.0 | 66/85 | 9, 49 | 2.4% | 90.8% | 9.6% |
| circle 320 px | 44.6% | 13.0 | 66/85 | 9, 44 | 0.8% | 91.9% | 9.7% |
| box 40 px | 43.6% | 10.1 | 66/85 | 10, 56 | 4.2% | 90.4% | 9.6% |
| box 80 px | 43.9% | 12.1 | 66/85 | 10, 52 | 3.5% | 90.5% | 9.6% |
| box 160 px | 44.6% | 13.0 | 66/85 | 9, 44 | 1.1% | 90.9% | 9.7% |
| box 320 px | 44.6% | 13.0 | 66/85 | 9, 44 | 0.2% | 92.3% | 9.7% |
| ellipse 3 sd | 43.7% | 10.1 | 66/85 | 10, 56 | 6.8% | 90.4% | 9.6% |
| ellipse 4 sd | 43.6% | 10.1 | 66/85 | 10, 56 | 4.8% | 90.4% | 9.6% |
| ellipse 5 sd | 43.6% | 10.1 | 66/85 | 10, 56 | 4.3% | 90.4% | 9.6% |
| ellipse 6 sd | 43.6% | 10.1 | 66/85 | 10, 56 | 4.2% | 90.4% | 9.6% |
| metres 0.1 | 43.6% | 10.1 | 66/85 | 10, 56 | 4.1% | 90.4% | 9.6% |
| metres 0.2 | 43.6% | 10.1 | 66/85 | 10, 56 | 4.0% | 90.6% | 9.6% |
| metres 0.4 | 43.9% | 13.0 | 66/85 | 10, 52 | 3.1% | 91.1% | 9.6% |

By range, the three that differ:

| crop | range | found live | false/min | flights | frames to first report, median, p90 | frames searched |
|---|---|---|---|---|---|---|
| none | 11 ft | 24.9% | 11.0 | 27/45 | 15, 56 | 100% |
| rule | 11 ft | 24.7% | 9.2 | 27/45 | 15, 56 | 100% |
| circle 40 px | 11 ft | 24.8% | 9.2 | 27/45 | 15, 56 | 95.1% |
| none | 35 ft | 57.8% | 19.7 | 40/40 | 8, 18 | 100% |
| rule | 35 ft | 56.1% | 19.7 | 39/40 | 8, 16 | 100% |
| circle 40 px | 35 ft | 54.5% | 11.7 | 39/40 | 8, 34 | 84.7% |

Circle 40 px against none, paired by flight (95% range from resampling flights): found live -4.3 to -0.3 points, false reports -9.7 to -0.4 per minute; at 35 ft, -6.8 to -0.4 points and -22.4 to 0.0 per minute.

Where a small crop lost the most (`ft35_c3_e2000_g18.3`, flight from frame 540): track 5 confirmed on the shuttle at the hit (reports at frames 537 and 538), then moved onto the hitter's raised free hand (checked on the frames), 117 to 425 px from the shuttle, for frames 545 to 584. The shuttle's own track 6 was still tentative; inside a small crop around track 5 it saw no points and died. The shuttle was picked up again as track 7 at frame 591, 49 frames late. Track 6 confirmed at frame 549 only with no crop, `rule`, the 320 px circle and the 160 and 320 px boxes; the other 12 crops lost it.

## Answer
Among the small crops the shape and size make no difference: the 40 and 80 px circles, the 40 px box, the ellipses at 3 to 6 standard deviations and the 10 and 20 cm circles all find 43.6 to 43.7% of flying frames live at 10.1 false reports per minute, against 45.7% at 14.2 with no crop. Searching the crop cuts false reports by 29% (40% at 35 ft) for about 2 points of found frames, but saves only 10% of the pixels on average and none on the worst frame, because a confirmed track exists in under 10% of these clips' frames.

## Next
- The cost is a confirmed track that leaves the shuttle (here onto the raised hand, the same false target E05 found) and holds the crop while the real shuttle goes unseen. A rule that drops the crop when its track stops being reportable, or keeps a cheap full frame search for new tracks, would target that.
- For speed, the crop cannot replace a detector that fits 10 ms on a full frame. These clips have long gaps between flights; a rally clip would spend more of its time in crop mode.
- The ellipse at 4 standard deviations or more always contains the projection of the tracker's own gate, so the confirmed track loses nothing to it; its effect comes from starving the other tracks.

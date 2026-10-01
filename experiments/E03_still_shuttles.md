# E03 Still shuttles

Status: planned

## Question
How far off is a still shuttle's triangulated 3D position, as a constant offset and as frame-to-frame scatter, at ranges from 3 to 13 m?

## Setup
Both cameras on the trigger, calibration `config/calibration.yaml` from commit bf409c1 (baseline 500.5 mm). Baseline settings: 8.33 ms, 6 dB, 100 fps.

Two yellow shuttles are taped to a rigid stick so their corks are 1.000 m apart, checked with a tape measure. The test compares the triangulated distance between the two shuttles with that 1.000 m. Distances from the camera are never used, because nobody knows exactly where the camera's optical centre sits inside the lens.

The aisle view is clear from 0 to 5 m; past 5 m only the upper half of the image sees over the large table, so the stick is held high there. The 13 m placements need the rig turned 90 degrees to face the corridor.

## Protocol
1. Measure and write down the cork to cork distance on the stick.
2. At each range (3, 5, 7 and 10 m in the aisle, 13 m in the corridor), place the stick at three spots across the view (left, centre, right). At each spot hold it still in two orientations: across the view, then pointing along the view (one shuttle nearer than the other).
3. Record 2 s per placement and note the range, spot and orientation in the clip notes.
4. In each clip, find each shuttle in both views (yellow blob inside a window around a hand-marked start point), triangulate every frame, and compute the shuttle to shuttle distance.

## Data
`~/Projects/baddy-bot/recordings/E03_still_shuttles/`

## Results
Per range: mean separation error (offset), its spread across placements, and the frame-to-frame scatter of each shuttle's 3D position.

## Answer

## Next
A range-dependent offset points back to the calibration. A spread that depends on orientation points to the two cameras centring on different parts of the shuttle, which E06 tries to fix.

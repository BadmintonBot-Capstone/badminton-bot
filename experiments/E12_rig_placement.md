# E12 Rig placement

Status: running (2026-10-15), side view only

## Question
Where should the stereo rig go (height, distance behind the baseline, tilt, which side of the court, baseline) so that it sees enough of each flight to predict the landing early, and also sees the robot's LEDs?

## Setup
His questions, 2026-10-15: the best camera height; tilting the cameras inwards or outwards; changing height and baseline; how far behind the baseline; the robot's side or the opponent's side. His guess: real clears and lifts have to go about twice as high as our lab hits to give the robot time to move.

- Lens and view from the rectified calibration (`config/calibration.yaml`, 2026-10-01 mounts): 13.75 degrees above the axis, 14.07 below, 43 degrees across.
- Shots flown with the E04 drag fit, alpha 0.208 per m (35 ft), gravity 9.81. Clear launch speed 43.1 m/s at 40 degrees: inside the 42 to 51 m/s measured for clears (Journal of Sports Science and Medicine, https://jssm.org/jssm-08-657.xml-Fulltext).
- Court: robot's baseline at 0, net at 6.7 m, opponent's baseline at 13.4 m. Shots go from the opponent toward the robot along the centre line.
- Robot: Year 1 spec 2.36 m/s^2 and 2.95 m/s, triangle profile. LED height 0.4 m (a guess), working zone 0.5 to 4 m from its baseline.
- Depth noise per frame from Z^2 x 0.2 px / (f x B), the camera noise E03 measured on a still shuttle.

## Protocol
1. From the E04 flight fits: each flight's highest point above the cameras, its distance, its image row, and the frame left above it in metres at that distance.
2. Fly a clear (hit at 2.6 m from 1 m inside the opponent's baseline, lands 11.4 m away) and a lift (hit at 0.8 m from 1 m behind the net, lands 6.5 m away).
3. For a rig 1 m behind either baseline, at heights 1 to 4 m and tilts of -10 (up), 0, 10 and 20 degrees down: the time each shot spends inside the vertical view, when the clear is first seen, its distance and depth noise there, where the floor comes into view, and where an LED at 0.4 m is seen in the working zone.

4. His question: rigs 0.3, 0.5 and 1 m high tilted up 10 to 40 degrees, with the time intervals each shot is in view, and LEDs at 0.4 m and on a 1.5 m mast.
5. Tosses from 4 m (the Dec 1 gate) at 40, 55 and 70 degrees, rig on the robot's side 1 to 3 m back, 1 or 1.5 m high, level or tilted up 5 or 10 degrees.

```
~/.venvs/cpsc330/bin/python analysis/rig_placement.py
```

The interactive side and top view is `figures/rig_view.html` (height 0.2 to 2.5 m, tilt 45 up to 30 down, side view to 8 m).

## Data
Inputs: `~/Projects/baddy-bot/recordings/E04_settings_sweep/flightfit.npy` and the calibration. Everything else is simulated.

## Results
E04 flights (median, 10th to 90th percentile):

| range | apex above the cameras | apex distance | frame left above the apex | flights off the top |
|---|---|---|---|---|
| 11 ft | 0.63 m (0.38 to 0.92) | 3.0 m | 0.13 m (98 px) | 6 of 39 |
| 35 ft | 1.23 m (0.88 to 1.44) | 6.4 m | 0.69 m (263 px) | 0 of 40 |

Simulated shots:

| shot | flight | apex | apex time | apex position |
|---|---|---|---|---|
| clear | 2.31 s | 7.36 m | 0.71 s | 4.9 m from the robot's baseline |
| lift | 1.70 s | 3.87 m | 0.66 s | 3.8 m |

Robot moves at the Year 1 spec: 0.5 m in 0.92 s, 1 m in 1.30 s, 2 m in 1.84 s, 3 m in 2.25 s.

Rig 1 m behind a baseline, selected rows (full table from the script):

| side | height | tilt down | floor from | clear seen | clear first seen | distance there | depth noise | lift seen | LED seen |
|---|---|---|---|---|---|---|---|---|---|
| robot | 1 m | 0 | 4.0 m | 0.22 s | at the hit | 13.4 m | 2.9 cm | 0.44 s | 1.4 to 4.0 m |
| robot | 1 m | 10 | 2.2 m | 0.16 s | 2.14 s | 2.2 m | 0.1 cm | 0.26 s | whole zone |
| robot | 4 m | -10 | 56 m | 0.79 s | 0.02 s | 12.8 m | 2.7 cm | 0.41 s | never |
| opponent | 1 m | -10 | 14 m | 1.05 s | 1.26 s | 11.6 m | 2.2 cm | 1.69 s | whole zone |
| opponent | 3 m | 0 | 12 m | 1.06 s | at the hit | 2.0 m | 0.1 cm | 1.66 s | whole zone |
| opponent | 4 m | -10 | 56 m | 1.78 s | 0.05 s | 3.4 m | 0.2 cm | 0.71 s | never |

Low rigs tilted up (his question, 2026-10-15), times in view out of the clear's 2.31 s and the lift's 1.70 s, selected rows (the script prints all):

| side | height | tilt up | clear in view | lift in view | LED at 0.4 m | LED at 1.5 m (mast) |
|---|---|---|---|---|---|---|
| robot | 0.5 m | 20 | 0.00-0.29 + 2.00-2.20 s | 0.05-0.61 + 1.21-1.57 s | never | whole zone |
| robot | 0.5 m | 30 | 0.06-0.52 + 1.87-2.14 s | 0.19-1.48 s | never | 0.5 to 2.5 m |
| robot | 1.0 m | 40 | 0.20-1.22 + 1.26-1.98 s | 0.48-1.23 s | never | never |
| opponent | 0.5 m | 10 | 1.35-2.31 s | whole flight | whole zone | whole zone |
| opponent | 0.5 m | 30 | 0.06-1.66 s | 0.27-0.84 s | never | never |

Tosses from 4 m, the Dec 1 gate in the 10/14 plan (robot 2 m in from its baseline, tosser 4 m further out, underhand from 1 m), rig on the robot's side, selected rows:

| toss | apex | flight | rig 1 m back, 1 m high, level | rig 3 m back, 1 m high, tilted up 5 |
|---|---|---|---|---|
| 40 degrees, 8.1 m/s | 1.94 m | 1.10 s | 96% in view, LED from 1.4 m | 97% in view, LED from 0.8 m |
| 55 degrees, 9.5 m/s | 2.80 m | 1.39 s | 46%, LED from 1.4 m | 98%, LED from 0.8 m |
| 70 degrees, 16.9 m/s | 5.21 m | 2.01 s | 19% | 35% |

## Answer

## Next
- Add the horizontal view (at 2 m the overlap is only 1.1 m wide), shots off the centre line, more shot types, LED depth noise at the robot's distance, and lens and baseline options.

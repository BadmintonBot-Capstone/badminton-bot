# E01 Flicker frequency

Status: planned

## Question
Do the Project Lab lights flicker at 120 Hz, or at a multiple of it such as 480 Hz?

## Setup
One camera is enough (left, 25376120). Gain 12 dB, exposure 1 ms so the flicker shows. Nothing moving in view: no people, nobody near the camera. Earlier clips (`flicker_e1000`, 2026-09-27) showed a brightness pattern repeating every 5 frames at 100 fps, about 4% peak, and none at 8.33 ms. That fits 120 Hz, and 480 Hz would fit too.

The frame rate decides how the flicker shows up. Flicker at f Hz filmed at r fps appears as a beat at the distance from f to the nearest multiple of r:

| flicker | at 100 fps | at 110 fps |
|---------|-----------|-----------|
| 120 Hz | 20 Hz (every 5 frames) | 10 Hz (every 11 frames) |
| 480 Hz | 20 Hz (every 5 frames) | 40 Hz (every 2.75 frames) |

## Protocol
1. Point the camera at a lit, static part of the room. Clear the view.
2. Record 2 s at 1 ms exposure, 100 fps.
3. Record 2 s at 1 ms exposure, 110 fps.
4. Record 2 s at 8.33 ms exposure, 110 fps, as the control.
5. For each clip, compute per-frame brightness as the median ratio of each pixel to its median over time (a person walking through barely moves a median), then find the strongest frequency.

## Data
`~/Projects/baddy-bot/recordings/E01_flicker/`

## Results

## Answer

## Next
The answer sets which exposures cancel flicker in the lab (any whole number of flicker periods) and is the first check in the court routine (E07).

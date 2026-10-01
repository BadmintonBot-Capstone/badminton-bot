# E01 Flicker frequency

Status: done 2026-10-01

## Question
Do the Project Lab lights flicker at 120 Hz, or at a multiple of it such as 480 Hz?

## Setup
Left camera only (25376120), gain 12 dB, aimed at a lit, still part of the lab with nobody in view.

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
5. For each clip, divide each pixel by its own median over time, then average over pixels leaving out the top and bottom 10% each frame, so a person walking through barely moves it. Pixels with a median under 10 or over 240 are skipped. The strongest frequency in that series is the beat.

```
./build/ninja/record_single --out ../recordings/E01_flicker/e1000_f100 --exposure 1000 --gain 12 --fps 100 --seconds 2 --delay 10
./build/ninja/record_single --out ../recordings/E01_flicker/e1000_f110 --exposure 1000 --gain 12 --fps 110 --seconds 2 --delay 5
./build/ninja/record_single --out ../recordings/E01_flicker/e8333_f110 --exposure 8333 --gain 12 --fps 110 --seconds 2 --delay 5
~/.venvs/cpsc330/bin/python analysis/flicker.py ../recordings/E01_flicker/*
```

## Data
`~/Projects/baddy-bot/recordings/E01_flicker/`: `e1000_f100`, `e1000_f110`, `e8333_f110`. No frames dropped in any clip.

## Results

| clip | exposure | fps | swing, peak to peak | swing, std | strongest frequency | peak over median of spectrum |
|------|----------|-----|---------------------|------------|---------------------|------------------------------|
| e1000_f100 | 1 ms | 100 | 2.36% | 0.85% | 20 Hz (every 5 frames) | 377x |
| e1000_f110 | 1 ms | 110 | 2.39% | 0.84% | 10 Hz (every 11 frames) | 335x |
| e8333_f110 | 8.33 ms | 110 | 0.09% | 0.02% | none above noise | 6x |

At 110 fps, flicker at each multiple of 120 Hz would show at: 120 Hz at 10 Hz, 240 at 20, 360 at 30, 480 at 40, 600 and 720 at 50. Only 120 Hz matches.

## Answer
The lab lights flicker at 120 Hz, twice the 60 Hz mains, with a 2.4% peak to peak brightness swing at 1 ms. Exposures of a whole number of 8.33 ms cycles cancel it (0.09% at 8.33 ms), and no exposure shorter than 8.33 ms does.

## Next
E04's 4 ms and 2 ms runs will carry the flicker, which is part of what they measure: a 20 m/s shuttle streaks 17 cm at 8.33 ms and 4 cm at 2 ms. The earlier "about 4%" from the 2026-09-27 clips came from a cruder method on clips with movement in view, and these numbers replace it. This test is the first check in the court routine (E07).

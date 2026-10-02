# E02 Noise and brightness

Status: done 2026-10-01 (camera binning not tested)

## Question
How do brightness and frame-to-frame noise change with gain, exposure and camera binning, and which gain matches the baseline brightness at each exposure E04 tests?

## Setup
Both cameras on the trigger, rig in the E04 wide aisle view (far end past the table in the upper half of the image), nothing moving. Recorded 17:48, windows dark, so lab lights only. Baseline is 8.33 ms, 6 dB, 100 fps, full resolution, BayerRG8.

## Protocol
1. Clear the view and keep people out of it for the whole run.
2. Record 2 s at each of these, in this order:
   - exposure 8.33 ms at gains 0, 6, 12, 18 and 24 dB
   - exposure 4 ms at gains 6, 12 and 18 dB
   - exposure 2 ms at gains 12, 18 and 24 dB
3. For every clip compute, on the half resolution grey image the tracker uses: mean brightness; noise (each pixel's standard deviation over time after dividing out the frame-wide flicker, median over pixels); share of raw pixels at 250 or more; and false motion, by running the tracker's own detection (running background, blur 1.5, threshold 12 grey levels, blobs of 4 px or more) on the still clip after a 40 frame warmup.
4. Read off the gain where each exposure's brightness crosses the baseline's, interpolating log brightness between the two gains around it.

Camera binning (step 2 of the original plan) was left out: `record_stereo` cannot set it yet.

```
./build/ninja/record_stereo --out ../recordings/E02_noise/e<us>_g<db> --exposure <us> --gain <db> --fps 100 --seconds 2 --delay 3 --note experiment=E02 --note view=aisle_wide --headless
~/.venvs/cpsc330/bin/python analysis/noise.py ../recordings/E02_noise/* --camera left --baseline ../recordings/E02_noise/e8333_g6
```

## Data
`~/Projects/baddy-bot/recordings/E02_noise/`, 11 stereo clips of 200 pairs, none dropped or mismatched.

## Results

Left camera (grey levels out of 255):

| exposure | gain | brightness | noise | saturated | false motion px | false blobs per frame |
|----------|------|-----------|-------|-----------|-----------------|-----------------------|
| 8.33 ms | 0 dB | 16.5 | 0.31 | 0.7% | 0 | 0 |
| 8.33 ms | 6 dB | 30.7 | 0.55 | 0.9% | 0 | 0 |
| 8.33 ms | 12 dB | 59.3 | 1.07 | 2.2% | 0 | 0 |
| 8.33 ms | 18 dB | 104.3 | 1.99 | 12.9% | 0 | 0 |
| 8.33 ms | 24 dB | 157.1 | 3.00 | 32.2% | 0 | 0 |
| 4 ms | 6 dB | 15.6 | 0.42 | 0.7% | 0 | 0 |
| 4 ms | 12 dB | 29.7 | 0.79 | 0.9% | 0 | 0 |
| 4 ms | 18 dB | 56.7 | 1.54 | 1.9% | 0 | 0 |
| 2 ms | 12 dB | 15.9 | 0.58 | 0.7% | 0 | 0 |
| 2 ms | 18 dB | 29.6 | 1.11 | 0.9% | 0 | 0 |
| 2 ms | 24 dB | 56.7 | 2.17 | 1.9% | 0 | 0 |

The right camera agrees within 2% on brightness and noise. Its only false motion was in the 8.33 ms clips (up to 0.6 blobs per frame), all from one small patch at its far right edge on the shelf, which the left camera does not see: a person moving slightly there, confirmed at the time.

Gain matching the baseline brightness (30.7):

| exposure | predicted from exposure ratio | measured |
|----------|-------------------------------|----------|
| 4 ms | 12.4 dB | 12.3 dB |
| 2 ms | 18.4 dB | 18.3 dB (right camera 18.4) |

## Answer
E04 should run 4 ms at 12.3 dB and 2 ms at 18.3 dB; the exposure ratio predicts both to 0.1 dB. Noise roughly doubles with every 6 dB, so at matched brightness 2 ms is twice as noisy as 8.33 ms (1.11 against 0.55 grey levels), but no setting up to 24 dB made the tracker flag a single pixel from noise alone.

## Next
- E04 uses the matched gains above. Its clips need the same light (evening, windows dark), or a fresh brightness check.
- Sensor noise sits far below the tracker's 12 grey level threshold, so a lower threshold is worth testing in E05 for the missed slow frames.
- The baseline image is dark (30.7 of 255). At 8.33 ms, 18 dB saturates 13% of the image, so whether a brighter image helps the shuttle more than the lost detail hurts is a question for E04.
- Camera binning once `record_stereo` can set it.

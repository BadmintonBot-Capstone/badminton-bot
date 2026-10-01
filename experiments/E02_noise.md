# E02 Noise and brightness

Status: planned

## Question
How do brightness and frame-to-frame noise change with gain, exposure and camera binning, and which gain matches the baseline brightness at each exposure E04 tests?

## Setup
Both cameras on the trigger. Static scene, nothing moving, the same view E04 will use. Baseline is 8.33 ms, 6 dB, 100 fps, full resolution, BayerRG8.

## Protocol
1. Clear the view and keep people out of it for the whole run.
2. Record 2 s at each of these, in this order:
   - exposure 8.33 ms at gains 0, 6, 12, 18 and 24 dB
   - exposure 4 ms at gains 6, 12 and 18 dB
   - exposure 2 ms at gains 12, 18 and 24 dB
   - camera binning 2x2 at 8.33 ms and 6 dB, once in Average mode and once in Sum mode
3. For every clip compute mean brightness, per-pixel noise over time (median of each pixel's standard deviation), and how much of the image is saturated.
4. Fit brightness against gain for each exposure and read off the gain that matches the baseline brightness at 4 ms and 2 ms.
5. For the binned clips, record the frame rate the camera reaches, the image size, and whether the colour still debayers correctly.

Exposures other than 8.33 ms show the lab flicker, so their brightness is averaged over whole 5-frame cycles.

## Data
`~/Projects/baddy-bot/recordings/E02_noise/`

## Results

## Answer

## Next
The matched gains feed straight into E04. The noise per gain decides whether the detector's threshold (12 grey levels) has to change with gain.

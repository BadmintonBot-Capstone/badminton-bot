# E11 LED appearance

Status: done 2026-10-20 (one camera, colour and saturation only; 3D jitter not measured)

## Question
At the birdie's camera setting, which LED colours keep a readable colour in the image, at what current, and how far does one still LED's 3D point move from frame to frame at the distances the robot works at?

## Setup
The robot is located by LEDs on it, seen by the stereo rig. His answers, 2026-10-15: the rig sits behind the robot during play, so the LEDs face backwards toward it; 5 to 10 of each colour arrived; the robot does not drive yet, so it is pushed by hand; the LEDs run from the lab bench supply. 2026-10-20: one camera only (25376121), his call that one is enough for colour and saturation.

| item | value |
|---|---|
| LEDs | one each: red Kingbright WP9294SECK/J3, blue VCC VAOL-5701SBY4, green VCC VAOL-5701DE4 (datasheets and the numbers in `baddy-bot/datasheets/README.md`) |
| wiring | the three in series on a breadboard on the bench supply in constant current mode, so all three carry the same current |
| camera setting | 4 ms, 12.3 dB, 100 fps, his pick 2026-10-15: the birdie's working setting from E04, since both are seen in the same frames |
| bench supply | constant current, voltage limit 9.2 V: the three datasheet maximum forward voltages at 20 mA (2.8 + 4.0 + 2.4), his number. Current steps are 10 mA, so 1, 2 and 5 mA could not be set |
| lens focus | about 4.6 m (2026-09-30) |

The camera records raw Bayer pixels, 0 to 255, so a red LED can saturate its red pixels while the others stay below 255. Saturation is checked per Bayer site on the raw frame.

## Protocol
1. Supply output off, voltage limit 9.2 V. Output on, check all three light.
2. LEDs about 1 cm apart at 3 m, facing the camera: a 3 s clip at 20 mA.
3. LEDs spread about 10 cm apart at 3 m: a 3 s clip at 10 mA.
4. LEDs on the floor at about 10 m (his estimate), turned at an angle to the camera: a 3 s clip at 10 mA.
5. Per clip, frame 150: each LED spot's highest raw value and the number of pixels at 255 per Bayer site, and a 4x crop to look at.

```
./build/ninja/record_single --out ../recordings/E11_led_appearance/<clip> --exposure 4000 --gain 12.3 --fps 100 --seconds 3
~/.venvs/cpsc330/bin/python analysis/led_spots.py <clip> <x0> <x1> <y0> <y1> <out.png>
```

## Data
`~/Projects/baddy-bot/recordings/E11_led_appearance/`: `3m_bunched_1cm_apart_20mA`, `3m_spread_10cm_apart_10mA`, `10m_floor_angled_10mA` (300 frames each, none dropped; renamed from `m3_20mA`, `m3_10mA`, `m10_floor_angled_10mA` on 2026-10-21), each with its zoomed crop. Copied to the T7 drive under `Badminton Bot/recordings/E11_led_appearance/` on 2026-10-21, every file checked byte for byte. Before the camera, a phone photo from about 15 cm showed the green LED far dimmer than the other two.

## Results
Pixels at 255 inside each spot (R, G, B sites):

| clip | red LED | blue LED | green LED |
|---|---|---|---|
| 3 m, 20 mA, 1 cm apart | 8, 7, 1 | 2, 7, 10 | not found (probably inside the others' glow) |
| 3 m, 10 mA, 10 cm apart | 13, 15, 6 | 0, 16, 19 | a faint dot just above the background, seen in the crop, not measured |
| ~10 m, 10 mA, floor, angled | 6, 5, 1 | 0, 3, 4 | not looked for |

At 10 m each spot is about 4 to 5 px across. Saturated centres read white; the ring around each still shows its colour.

## Answer
Red and blue are easy to find and tell apart from 3 to about 10 m at 10 mA, with a few saturated pixels at the centre. The green LED (100 mcd, a tenth of the other two) is too dim and is dropped. His call: red and blue at 10 mA is the working setting.

## Next
- An arrangement of red and blue LEDs on the robot to track its position, the way the birdie is tracked.
- 3D jitter of a still LED needs both cameras.

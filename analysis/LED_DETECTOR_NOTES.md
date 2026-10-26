# Red and blue LED detector (`leds.py`)

Notes as of 2026-10-26. The goal is to find the robot's red and blue marker LEDs separately in each frame, with a method that holds up under different room lighting.

```
python leds.py <clip> [<clip> ...] [--red r_min,b_max] [--blue b_min,r_max] [--size min,max]
```

Works on `record_single` clips (`frames.raw`, `frames.csv`, `meta.yaml`). Tested on two clips in `~/479_led_recordings/`. Both are still, 10 mA LEDs 10 cm apart, at 4000 µs exposure and 12.3 dB gain:

- `3m_spread_10cm_apart_10mA`
- `10m_floor_angled_10mA`

The bunched 1 cm clip was left out because it doesn't match the planned mount; at 3 m its two LEDs merge into one blob.

## The problem

The LED cores clip. At 10 mA and these camera settings, even the red LED's centre reads 255/255/255: the B pixels saturate too. A clipped cell has no colour, so a colour test on the brightest pixels fails, and HSV hue is undefined there. The colour survives in the ring of cells around the core. The method below reads colour from the ring and then fills in the core.

## How it works, step by step

Each frame is processed on its own.

### 1. Colour cells straight from the sensor pattern
The sensor's colour filter repeats every 2×2 pixels as R G / G B (BayerRG8). Each 2×2 block becomes one **cell**:

| Channel | Source |
|---|---|
| R | the block's R pixel |
| G | the average of its two G pixels |
| B | the block's B pixel |

That gives a 960×600 colour image in which every value is a real measurement. Normal demosaicing would instead guess each pixel's two missing colours from its neighbours. For an LED that's only 1–2 pixels across, those guesses invent colour: the green and magenta fringes in the 10 m clip.

### 2. Brightness and chromaticity per cell
- **Brightness** is the cell's **peak channel**, `max(R, G, B)`, on the raw 0–255 scale. It's the peak rather than an average because each LED is strong in mostly one channel. A blue ring cell at 20/60/150 has a peak of 150 but an average of only 77.
- **Chromaticity** is each channel's share of the total: `r = R/(R+G+B)`, `g = G/(R+G+B)` and `b = B/(R+G+B)`. It ignores how bright a cell is and keeps only its colour. White, including a clipped core, is r = g = b = ⅓.

### 3. Colour masks
A cell joins a colour's mask only if it passes all three tests:

| Test | Red | Blue |
|---|---|---|
| Brightness floor | peak > 50 | peak > 50 |
| Main channel | r ≥ 0.70 | b ≥ 0.42 |
| Opposite channel | b ≤ 0.10 | r ≤ 0.20 |

Clipped cores sit at the white point, so they fail both masks. What passes is the coloured ring.

### 4. Ring to region
1. **Closing:** a 3×3 morphological closing, which is a dilation followed by an erosion.
   - Dilation turns a cell on if *any* cell in its 3×3 neighbourhood is on, so shapes grow by one cell and gaps of up to 2 cells are covered from both sides.
   - Erosion keeps a cell on only if *all 9* cells in its neighbourhood are on. That takes the outside growth back off, but leaves the filled gaps filled, since every neighbour of a cell inside a bridged gap is on.

   The result is the original mask with **gaps of up to 2 cells bridged**:
   ```
   gap of 1:   .###.####...   ->   .#########..
   gap of 2:   .###..####..   ->   .##########.
   gap of 3:   .###...####.   ->   .###...####.   (not bridged)
   ```
2. **Outer contours** of each connected group are found (`RETR_EXTERNAL`, so the hole is ignored) and **drawn filled**. Closing alone leaves a white core wider than 2 cells as a hole; the fill takes in the core whatever its size, so no larger closing or erode-back-to-size step is needed.

Closing never removes a mask cell, so a one-cell speck stays one cell and the size check drops it. Two drawbacks:
- **Things less than 3 cells apart** (about 6 px) merge into one region, for example an LED and a reflection right beside it. The 10 cm LED pair was 42 px apart even at 10 m.
- **A ring with a gap of 3 or more cells isn't sealed.** The outer contour then runs into the core through the gap, the fill takes in only part of the core, and the centroid shifts.

### 5. Size check
Filled regions smaller than **3 cells** or larger than **40 cells** are dropped. One cell is 2×2 pixels.

### 6. Measurements per region

| Field | Meaning |
|---|---|
| `x_unw`, `y_unw` | Unweighted centroid of the filled region; every cell counts equally |
| `x_wtd`, `y_wtd` | Centroid weighted by each cell's R+G+B, so pulled towards the bright core |
| `area` | Cells in the filled region |
| `ring` | How many of those passed the colour mask |
| `r`, `g`, `b` | Chromaticity from the summed R, G and B of the ring cells only |
| `white` | Fraction of cells clipped in all three channels (≥ 250) |
| `core` | 1 if any cell has any channel ≥ 250 |

Coordinates are in full-resolution pixels. Cell *i* covers pixels 2*i* and 2*i*+1, so its centre is at 2*i* + 0.5.

### 7. Outputs, written to each clip folder

| File | Contents |
|---|---|
| `leds.csv` | Every region in every frame, with the fields above |
| `leds.jpg` | Frame 0 with each region seen in at least half the frames marked: a circle at the unweighted centroid, a cross at the weighted centroid, and its r and b |
| `leds_chroma.png` | An r–b plot of every bright cell in frame 0, with the threshold boxes and the marked regions |
| Printed summary | One line per region position, grouped across frames, with medians and centroid jitter. The header line shows the settings used. |

## How each value was chosen

Everything in these notes comes from all 300 frames of each clip. The threshold comparison below ran each threshold level through `leds.py`'s own functions, so with the 3–40 size check applied, side by side on every frame.

| Value | Setting | Why |
|---|---|---|
| Brightness measure | peak channel | Each LED is strong in one channel. An average dilutes it by about a third. |
| Brightness floor | 50 | A noise gate, not a detector. Below about 50, one count of sensor noise moves chromaticity by about 0.1 (a cell at 8/4/2 has meaningless colour). Most lit surfaces in the lab are above 50 too; colour and size do the detecting. The floor is an absolute level, so it depends on exposure, gain and LED current. |
| Red | r ≥ 0.70, b ≤ 0.10 | Raising red from 0.50/0.20 to 0.70/0.10 cut red false regions from about 155 to about 2 per frame at 3 m, and to about 1.3 at 10 m. Both red LEDs were still found in every frame. See the comparison table below. |
| Blue | b ≥ 0.42, r ≤ 0.20 | Kept loose because the 10 m blue LED's ring only reached about b 0.48: it sits in front of a lit, warm-coloured breadboard. At b ≥ 0.50 it wasn't found in any frame, because what passed was under the 3-cell minimum. Blue false regions were already low (3–5 per frame). |
| Closing | 3×3 | Bridges gaps of up to 2 cells in a ring. The fill step closes the core however big it is. |
| Size | 3–40 cells | LEDs measured 4–5 cells at 10 m and 18–31 at 3 m. Most false regions were 1–2 cells, and the largest red one near the LEDs was 64. The maximum only covers distances of about 3 m or more, because area grows as 1/distance². |
| Clip level | 250 | Used for `white` and `core`. Not used as a filter yet. |

### Threshold comparison

Each colour at three levels, all 300 frames, size check 3–40. "False" counts every other region of that colour per frame; "with core" counts only those containing a saturated cell. Jitter is the LED region's, unweighted / weighted, in full-res pixels.

**Red** (base r ≥ 0.50, b ≤ 0.20; mid 0.60/0.15; strict 0.70/0.10)

| Clip | Level | Found | Core | Area (ring) | r | b | Jitter | False/frame | With core |
|---|---|---|---|---|---|---|---|---|---|
| 3 m | base | 300/300 | 100% | 36 (27) | 0.66 | 0.08 | 0.12 / 0.03 | 153.4 | 25.4 |
| 3 m | mid | 300/300 | 100% | 23 (19) | 0.76 | 0.06 | 0.46 / 0.49 | 37.2 | 20.2 |
| 3 m | **strict** | 300/300 | 100% | 18 (16) | 0.79 | 0.05 | 1.00 / 0.65 | **2.0** | **0.0** |
| 10 m | base | 299/300 | 99% | 7 (7) | 0.72 | 0.04 | 1.71 / 1.49 | 158.5 | 25.7 |
| 10 m | mid | 300/300 | 100% | 7 (7) | 0.73 | 0.04 | 0.00 / 0.01 | 47.3 | 15.7 |
| 10 m | **strict** | 300/300 | 100% | 5 (5) | 0.83 | 0.03 | 0.00 / 0.02 | **1.3** | **0.1** |

**Blue** (base b ≥ 0.42, r ≤ 0.20; mid 0.50/0.15; strict 0.58/0.10)

| Clip | Level | Found | Core | Area (ring) | r | b | Jitter | False/frame | With core |
|---|---|---|---|---|---|---|---|---|---|
| 3 m | **base** | 300/300 | 100% | 31 (29) | 0.08 | 0.60 | 0.17 / 0.04 | **5.3** | **3.1** |
| 3 m | mid | 300/300 | 100% | 30 (24) | 0.07 | 0.64 | 0.13 / 0.05 | 2.0 | 0.8 |
| 3 m | strict | 300/300 | 100% | 22 (15) | 0.05 | 0.71 | 0.82 / 0.79 | 1.0 | 0.0 |
| 10 m | **base** | 300/300 | 100% | 4 (4) | 0.15 | 0.48 | 0.64 / 0.36 | **2.8** | **1.4** |
| 10 m | mid | 0/300 | – | – | – | – | – | 1.0 | 1.0 |
| 10 m | strict | 0/300 | – | – | – | – | – | 0.0 | 0.0 |

- **Red's false regions collapse at the strict level** while both LEDs stay found with a saturated core. At the base level, red false regions with a core outnumber blue ones 8 to 18 times, probably because the lab lighting is warm and lit surfaces sit on the warm side of white.
- **Blue is limited by the 10 m LED.** A stricter blue would help at 3 m, but the 10 m LED drops out entirely. A purer blue against a black robot mount may allow mid or strict later.
- **Stricter thresholds thin the ring.** The 3 m LEDs' jitter rises from about 0.1 to 0.7–1.0 px at the strict level, because a thinner ring's shape changes more from frame to frame.

## Results

Settings: red 0.70/0.10, blue 0.42/0.20, size 3–40. All 300 frames of each clip. Each LED's numbers come from its own region: in each frame, the largest region of its colour within 12 px of its position. Jitter is the spread (√(σx² + σy²)) of a centroid over the frames of a still clip, in full-res pixels. The printed summary's jitter can be higher where nearby regions get grouped in.

| | Found | Area (ring) | r | g | b | Saturated core | Unweighted centroid | Jitter | Weighted centroid | Jitter |
|---|---|---|---|---|---|---|---|---|---|---|
| 3 m red | 300/300 | 18 (16) | 0.79 | 0.16 | 0.05 | 100% | (1030.8, 928.2) | 1.00 px | (1030.7, 928.8) | 0.65 px |
| 3 m blue | 300/300 | 31 (29) | 0.08 | 0.31 | 0.60 | 100% | (1162.8, 925.7) | 0.17 px | (1163.1, 926.8) | 0.04 px |
| 10 m red | 300/300 | 5 (5) | 0.83 | 0.14 | 0.03 | 100% | (1436.1, 876.1) | 0.00 px | (1436.3, 875.7) | 0.02 px |
| 10 m blue | 300/300 | 4 (4) | 0.15 | 0.37 | 0.48 | 100% | (1479.5, 875.5) | 0.64 px | (1479.5, 875.5) | 0.36 px |

- **Reading colour from the ring is what makes it work.** Ordinary lit surfaces form a band in `leds_chroma.png` along r + b ≈ ⅔, between bluish-white and warm-white. The LEDs' rings sit well outside it.
- **The weighted centroid is steadier in 3 of the 4 cases.** The exception is the 10 m red LED, where the unweighted one doesn't move at all because the 5-cell shape never changes. The two centroids agree within about 1 px.
- **The 3 m red LED's ring is thinner at the strict red threshold.** Its shape varies from frame to frame, which is why both centroids jitter more (0.65–1.0 px) than the blue LED at the same distance.

### False regions seen in at least half the frames

| Clip | Where | Colour | Cells | Saturated core | What it is |
|---|---|---|---|---|---|
| 3 m | (1153, 948) | blue | 21 | no | Blue LED's reflection on the breadboard, 22 px below it |
| 3 m | (992, 882) | red | 24 | no | Unknown, near the red LED |
| 3 m | (873, 258), (858, 268) | blue | 13, 4 | yes, no | Fringe on the window |
| 3 m | (1375, 98), (1626, 202) | blue | 3, 3 | yes | Ends of ceiling tubes |
| 10 m | (873, 259) | blue | 16 | yes | Fringe on the window |
| 10 m | (1228, 649) | red | 7 | no | Red object near the bin area |

Before the size check there were 20–40 of these per clip, mostly 1–2 cell fringes on lights and windows, and a 64-cell red source beside the 3 m LEDs.

## Next steps

1. **Record the real configuration.**
   - LEDs on the black robot mount, at the final spacing.
   - Camera at the planned 8.33 ms and 6 dB. That gives about the same brightness as these clips (8.33 × 2.0 ≈ 4 × 4.1), so expect the same clipping.
   - Clips at the nearest and farthest robot distances, plus the robot moving and turning.
   - Ideally in the court environment.
2. **Choose the LED current and whether to diffuse** (see below), then re-measure:
   - size limits at the nearest and farthest distances
   - colour thresholds from `leds_chroma.png`
   - whether the 10 m ring cells stay above the floor of 50
3. **Add a minimum-peak check.** Every LED region here had a saturated core, while the reflection, the unknown red region and some window fringes didn't. Requiring a peak of at least 250 would remove them for now. With diffused LEDs that no longer clip, set it to the diffused LED's measured peak instead.
4. **Add an enclosure test.** An LED's ring surrounds its white core. A fringe on a light runs along one edge of a much larger white area. Measure what fraction of the saturated area touching a region lies inside its filled contour: close to 1 for an LED, low for a light.
5. **Pair check by spacing** (see below).
6. **Tracking:** search only near the robot's last position, which rejects fixed distractors and cuts the cost per frame.
7. **Fix the summary grouping.** Grouping by position merges nearby regions and inflates jitter. Tracking or the pair check would replace it.
8. **Port to C++** for the live pipeline once the method is settled.

## Implications

### Diffusing the LEDs
**Helps:**
- Light spreads over more pixels at a lower peak, so less clipping, and the core may keep its colour.
- Better colour at 10 m. LEDs that cover 1–2 cells take their colour from whichever sensor sites the light lands on.
- A steadier centroid and a wider viewing angle as the robot turns.
- A diffuser of known size (for example a 40 mm ball) has a predictable apparent size at each distance, which makes the size check stronger.

**What changes:**
- **If the LEDs stop clipping,** they become solid coloured discs. The pipeline still works, since the whole disc passes the mask and the fill does nothing extra. But the `core` flag and a peak ≥ 250 check stop meaning anything; use the diffused LED's measured peak. The enclosure test also becomes unnecessary.
- **Re-measure the size limits.**
- **The colour may be purer,** so the blue threshold may tighten.
- **Peak brightness drops,** so check the farthest LED against the floor of 50.

**Doesn't help:** other real red or blue lights in view, or reflections of the LEDs.

### Filtering by spacing
- **Rejects** single coloured lights and objects that have no partner of the other colour at the right distance.
- **Spacing in the image varies.** It shrinks with distance (about 1/distance), and with viewing angle as the robot turns. When the robot turns side-on, so the two LEDs line up with the camera, the spacing goes to zero and one LED can hide the other. A 2D check needs a range of allowed spacings, ideally tied to the robot's expected distance.
- **With the stereo pair, use 3D distance instead.** Triangulate each LED and require the red-to-blue distance to match the physical spacing (for example 10 cm). That doesn't depend on distance or viewing angle, so it's a much tighter test.
- **Reflections:** a reflection sits below the real LED (22 px in the 3 m clip). If both LEDs are at the same height on the robot, a pair check that also expects them at a similar height will prefer the real pair. Matte black around the LEDs reduces reflections at the source.
- **Spacing has a lower limit.** 1 cm merged into a single blob at 3 m. 10 cm separated cleanly at both 3 m and 10 m.

### LED current
Current trades clipping against visibility at range. 10 mA clipped at 3 m and at 10 m with these camera settings. A lower current, possibly with a diffuser, gives purer colour, but the farthest LED has to stay well above the floor. Choose it from clips at the farthest distance.

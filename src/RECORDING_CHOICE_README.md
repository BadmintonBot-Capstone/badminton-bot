# Which recorder to use for 100 fps experiment footage

This note complements `RECORDER_README.md`, which describes the disk-streaming recorder in detail.

## Recommendation

Use **`record_single`** (the RAM-based tool) as the default for experiment and analysis footage. Use **`preview` with `StereoRecorder`** (the disk-streaming recorder) only when a clip has to be longer than RAM allows, or when a live view is needed while recording.

## Why `record_single` for experiments

Analysis needs clips with every frame present and the camera settings known. `record_single` is the more reliable way to get that:

- **Nothing competes with the grab loop.** It only copies each frame into preallocated memory. There is no display, no disk write and no queue during capture, so a clip that fits in RAM should reach the full 100 fps regardless of disk speed.
- **Each clip documents itself.** `meta.yaml` records the exposure, gain and frame rate actually applied, the frame count, and how many frames were dropped or incomplete.
- **Settings are command-line flags.** `--exposure`, `--gain` and `--fps` make it easy to record a series of clips at different settings without editing config files.
- **A start delay is built in.** `--delay` gives time to get into position before recording starts.

Its limits:

- **Clip length is capped by available RAM.** Memory needed is seconds x fps x width x height bytes, which is about 230 MB per second at 1920 x 1200 and 100 fps. The Windows machine had about 3.4 GB available when checked, so roughly 10 seconds per clip with a safe margin.
- **No preview.** The camera has to be aimed and the exposure checked beforehand.
- **One camera only.**

## When to use the disk-streaming recorder instead

- A continuous recording longer than about 10 seconds is needed.
- A live view is needed during the recording.
- Two cameras are being recorded.

It can reach 100 fps too, but only if the build is Release and the disk sustains more than 230 MB per second. It reports dropped frames when it closes; check for that message after every recording.

## Suggested workflow

1. Run `preview` to aim the camera and check exposure and focus, then close it. Both programs cannot hold the camera at the same time.
2. Close memory-heavy programs and check available memory.
3. Record the clip:
   ```
   record_single --out <dir> --seconds 10 --delay 3 [--exposure us] [--gain db] [--fps n]
   ```
4. Check `meta.yaml` in the output folder: `dropped: 0` and `incomplete: 0` mean a clean clip. The program also prints the measured frame rate.
5. Use a new `--out` folder for every clip.

Several short clips are usually better for detection experiments than one long recording: each is clean, and each can use different settings.

## `record_single` output format

A recording is a folder containing:

| File | Contents |
|---|---|
| `frames.raw` | BayerRG8 frames back to back, no headers, width x height bytes each |
| `frames.csv` | Columns `index,frame_id,timestamp_ns`, one row per frame |
| `meta.yaml` | `serial`, `width`, `height`, `pixel_format`, `exposure_us`, `gain_db`, `fps`, `frames`, `dropped`, `incomplete` |

This differs from the disk-streaming recorder's format (`primary.raw`, `timestamps.csv`, `metadata.txt`) and from the older paired format that the existing Python scripts read.

## Tasks for Claude Code

1. **Make the Python scripts read `record_single` output.** The playback and conversion scripts currently expect `frames.raw` with each image stored twice, a `metadata.txt` of `key=value` lines, and CSV columns `ts_primary` and `id_primary`. For `record_single` clips they need to read `meta.yaml` for the size, use one image per record (reshape to `(n, h, w)`), and read `timestamp_ns` and `frame_id` from `frames.csv`. Ideally one loader detects which of the three formats a folder contains.
2. **Add a memory check to `record_single`.** Before allocating, compare the buffer size with the available physical memory and refuse or warn if it does not fit with a margin. Without this, an oversized request spills into the page file and drops frames.
3. **Add a batch script.** Record a numbered series of clips in sequence (for example `clip_01`, `clip_02`), with the settings and count as arguments. There is a gap between clips while each one is written to disk, which is expected.
4. **Confirm the frame rate on real hardware.** Record one 5-second clip and check that `dropped` and `incomplete` are 0 and the measured rate is 100 fps. If the rate is low, the camera settings or USB link are the limit, not the recorder; see the checklist in `RECORDER_README.md`.

`single_cam.hpp`, which `record_single` uses to configure the camera, was not reviewed when this note was written.

# Single-camera raw recorder: handoff notes

## Goal

Record raw sensor frames from one FLIR/Teledyne Blackfly S camera (BFS-U3-23S3C, 1920 x 1200, BayerRG8) to disk at 100 fps without dropping frames, with a live preview. The recordings are test footage for shuttlecock detection in the `479_Badminton_Bot` project.

Only one camera is in use for now. Stereo recording still works in the recorder class but is not the priority.

## Files in this handoff

| File | Goes to | What it is |
|---|---|---|
| `stereo_recorder.h` | `core/stereo_recorder.h` (overwrite the existing file) | Header-only recorder class |
| `preview.cpp` | Replace the existing `preview.cpp` | Viewer and recorder program for one camera |

Both replace existing files of the same name, so no build-file changes should be needed.

## Verification status

- **`stereo_recorder.h`** was tested against a stub of the Spinnaker image API, not the real SDK: single and stereo recordings, byte-for-byte content checks over 2000 frames, queue overflow, and rejected pushes, with address, undefined-behaviour and thread sanitizers clean.
- **`preview.cpp`** has never been compiled. It depends on `core/config.hpp`, Spinnaker and OpenCV, none of which were available.
- Nothing has been run with a real camera.

**First task:** build both against the real SDK, fix any compile errors, and make one short test recording.

## Output format

A recording is a folder containing:

| File | Contents |
|---|---|
| `primary.raw` | Raw sensor images back to back, no headers. Each is `bytes_per_image` bytes (width x height for BayerRG8). |
| `timestamps.csv` | One row per image. Row N describes image N in `primary.raw`. |
| `metadata.txt` | `key=value` lines: `cameras`, `width`, `height`, `pixel_format`, `pixel_format_id`, `bytes_per_image`. |
| `secondary.raw` | Stereo recordings only; same layout as `primary.raw`. |

CSV columns: `index,ts_primary,ts_secondary,id_primary,id_secondary,offset_drift_ns`

- `ts_primary` is the camera's own timestamp in nanoseconds.
- `id_primary` is the camera's frame counter. A jump greater than 1 means the camera or driver lost frames.
- In a single-camera recording, `ts_secondary`, `id_secondary` and `offset_drift_ns` are empty.

Check for a correct recording: the size of `primary.raw` equals the number of CSV data rows times `bytes_per_image`.

## Recorder (`StereoRecorder`)

```cpp
StereoRecorder rec(dir);            // optional second argument: max queued records (default 300)
rec.push(img, timestamp_ns);        // single camera
rec.push(imgP, imgS, tsP, tsS);     // stereo pair
// destructor writes everything still queued, then closes the files
```

Behaviour:

- `push()` copies the image and returns. A background thread does all disk writes.
- `push()` must be called before `img->Release()`, and from one thread only.
- The first push decides whether the recording is single or stereo. Pushes of the other kind, or with a different image size, are rejected and return `false`. This keeps the raw files and the CSV the same length.
- If more than `max_queued` records are waiting for the disk, new ones are dropped and `push()` returns `false`. At 1920 x 1200, 300 records is about 0.7 GB of RAM.
- Image buffers are recycled, so after the first few frames `push()` makes no heap allocations.
- On destruction it prints the dropped and rejected counts to stderr, if either is non-zero.

## Viewer (`preview.cpp`)

Usage: `preview [config_dir] [recording_dir]`

- Uses only the first camera found, free-running (trigger off), configured from the project's camera config.
- Records every complete frame through `StereoRecorder`.
- Displays only every fifth frame (`PREVIEW_EVERY`), with the fastest demosaic. Displaying every frame took longer than the 10 ms per-frame budget and caused dropped frames.
- Sets the driver's stream buffer count to 100 (`STREAM_BUFFERS`), so short stalls don't lose frames.
- Catches `GetNextImage` exceptions, so a timeout doesn't end the program and lose queued frames. It gives up after 20 errors.
- Prints the camera's `AcquisitionResultingFrameRate` at startup and a summary at exit: frames grabbed, measured loop fps, incomplete frames, frames not recorded.

The default recording folder is hard-coded as `C:/Users/justi/479_recording_single`. Pass a different folder as the second argument for each recording, because an existing recording in the same folder is overwritten.

## Getting to 100 fps

The data rate is 1920 x 1200 x 100 = 230.4 MB per second.

Requirements outside the code:

1. **Release build.** A Debug build on Windows makes the per-frame copy several times slower.
2. **SSD.** The disk must sustain more than 230 MB per second.
3. **Camera settings.** `device_link_throughput_limit` in the config must be above 230,400,000. Check whether the ADC bit depth setting limits the frame rate (the 12-bit mode is believed to; not confirmed). Exposure must be well under 10 ms.
4. **USB.** A USB 3 port and cable.

Diagnosing a shortfall:

| Symptom | Cause |
|---|---|
| "Camera resulting frame rate" below 100 at startup | Camera settings or the USB link |
| Loop fps below 100 at exit, or gaps in `id_primary` | The grab loop is too slow |
| "records dropped (disk too slow)" at exit | The disk |

A true 100 fps recording has no gaps in `id_primary`, `ts_primary` steps of about 10,000,000 ns, and no drop message.

## Known gaps and possible next steps

- **Separate grab thread.** The preview still runs in the grab loop, one frame in five. Moving grab and record to their own thread, with the main thread displaying the latest frame at its own pace, would remove display cost from the capture path entirely.
- **Python scripts.** The existing playback and conversion scripts expect the old format: `frames.raw` with each image stored twice, and `metadata.txt` in the same folder. For the new format they need to open `primary.raw`, use `record_size = bytes_per_image`, and reshape to `(n, h, w)`. The CSV columns `ts_primary` and `id_primary` are unchanged.
- **`StereoPlayer`.** The C++ player reads the old `frames.raw` pair format and cannot open new recordings.
- **`record_single`.** A separate tool in the project records one camera into RAM and writes to disk afterwards, with a different output format (`frames.raw`, `frames.csv`, `meta.yaml`). It avoids disk-speed drops but is limited by available RAM, which is about 3.4 GB on the Windows machine (roughly 10 seconds at full rate).
- **Recorded metadata.** Exposure, gain and frame rate are not written to `metadata.txt`. Adding them would make recordings self-describing.
- **Region of interest.** Cropping the image height on the camera reduces the data rate in proportion, if the disk or USB link is the limit.
- **Stereo sync.** `preview.cpp` runs free-running. Synchronised stereo pairs need the hardware-triggered path in the bot's `StereoCamera`.

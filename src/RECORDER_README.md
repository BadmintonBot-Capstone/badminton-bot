# Raw recorder (one camera or a stereo pair): handoff notes

## Goal

Record raw sensor frames from one FLIR/Teledyne Blackfly S camera (BFS-U3-23S3C, 1920 x 1200, BayerRG8), or from a hardware-triggered stereo pair, to disk at 100 fps without dropping frames, with a live preview. The recordings are test footage for shuttlecock detection in the `479_Badminton_Bot` project.

`preview` records one camera by default, and both cameras with `--stereo`.

## Files in this handoff

| File | Goes to | What it is |
|---|---|---|
| `stereo_recorder.h` | `core/stereo_recorder.h` (overwrite the existing file) | Header-only recorder class |
| `preview.cpp` | Replace the existing `preview.cpp` | Viewer and recorder program for one camera or a stereo pair |

Both replace existing files of the same name, so no build-file changes should be needed.

## Verification status

- **`stereo_recorder.h`** was tested against a stub of the Spinnaker image API, not the real SDK: single and stereo recordings, byte-for-byte content checks over 2000 frames, queue overflow, and rejected pushes, with address, undefined-behaviour and thread sanitizers clean. Both paths have since recorded real footage through `preview`.
- **`preview.cpp`** builds on Windows (Release, Visual Studio 2026). Single-camera mode has made real recordings, including a clean 60 s clip at 100 fps.
- **Stereo mode (`--stereo`)** has run on the real cameras (2026-10-21 to 10-26, 8.33 ms exposure, 6 dB gain). The trigger, pairing and recorder all work: every recorded pair has matching frame counters, the two clocks drift by under 50 µs, and both cameras produce every frame. What limits it is getting the data into the laptop.

### Stereo results with both cameras sharing one USB 3 link (a hub)

| frame rate | data rate | 10 s result |
|---|---|---|
| 40 fps | 184 MB/s | clean |
| 70 fps | 323 MB/s | clean |
| 80 fps | 369 MB/s | about 1 frame in 7 lost after about 2 s, also with the display off or the throughput limit raised; about half as many lost with `--no-record` |
| 100 fps | 461 MB/s | almost half lost, from about 0.6 s |

- On a shared link, **70 fps is the highest clean stereo rate.** The cut-off between 323 and 369 MB/s matches what one 5 Gbps USB 3 link carries in practice.
- The disk is not the limit: the recording drive (Micron 2300 NVMe) sustained about 880 MB/s in a write test.
- **100 fps stereo with each camera on its own laptop USB 3 port has not been tested yet.** That is the next test.
- A camera on a USB 2 link (for example behind a USB 2 hub such as `05E3:0610`, or with a badly seated cable) reports a maximum `DeviceLinkThroughputLimit` of 43,000,000, and `preview` stops with an out-of-range error. Windows Device Manager shows which hub each "USB3 Vision Camera" sits behind.

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

Usage: `preview [config_dir] [recording_dir] [seconds] [--stereo] [--no-display] [--no-record]`

- `config_dir` (default `config`) holds `camera.yaml`. Exposure, gain, frame rate and the USB throughput limit all come from there; there are no command-line flags for them.
- `seconds` stops after that much footage, measured by the (primary) camera's own timestamps. Omit it, or pass 0, to run until `q` is pressed in the preview window.
- `--stereo`, `--no-display` and `--no-record` can go anywhere among the arguments.
- `--no-display` opens no window and skips the conversion for it. It needs `seconds`, since there is no `q` to press.
- `--no-record` grabs (and pairs) without writing anything and creates no folder. Comparing a run with and without it separates disk or load limits from USB limits.

Example (PowerShell, from the repo root):

```powershell
.\build\Release\preview.exe config C:/Users/justi/479_stereo_test 10 --stereo
```

### Single camera (default)

- Uses only the first camera found, free-running (trigger off), configured from `camera.yaml`.
- Records every complete frame to `primary.raw`.

### Stereo (`--stereo`)

- Opens the two cameras named by `primary_serial` and `secondary_serial` in `camera.yaml`, or the first two found if those are empty. Stops with a message if they are not both connected.
- Applies the same settings to both, then the same trigger wiring as `StereoCamera`: the primary drives 3.3 V on Line2 with every exposure, and the secondary exposes on each rising edge on Line3. The secondary is started first, so it is waiting for the trigger.
- Records each complete pair to `primary.raw` and `secondary.raw`. The recorder queue is 150 pairs (about 0.7 GB).
- **Keeps the pairs in step.** If one camera loses a frame, its frame counter falls behind. The camera that is behind skips ahead, up to three tries, until both counters have moved by the same amount since the first pair. If they still differ, the pair is dropped rather than recording two different moments. The number of skipped frames is printed at exit; a non-zero count points to the trigger cable or the USB bandwidth.
- Shows both views side by side at half size.
- Timestamps are each camera's host-side `GetTimeStamp()`, not chunk-data hardware timestamps as in `record_stereo`. The two camera clocks differ by a constant; `offset_drift_ns` shows how far that difference has moved since the first pair.

### Both modes

- Displays only every fifth frame or pair (`PREVIEW_EVERY`), with the fastest demosaic. Displaying every frame took longer than the 10 ms per-frame budget and caused dropped frames.
- Sets each camera's driver stream buffer count to 100 (`STREAM_BUFFERS`), so short stalls don't lose frames.
- Catches `GetNextImage` exceptions, so a timeout doesn't end the program and lose queued frames. It gives up after 20 errors.
- Sets the USB throughput limit before the frame rate, because a low stored limit caps the maximum frame rate.
- Prints the camera's `AcquisitionResultingFrameRate` at startup and a summary at exit: frames (or pairs) grabbed, measured loop fps, incomplete, not recorded.
- Prints any startup error (for example a frame rate above what the USB link allows) instead of exiting silently.
- Prints how many frames (or pairs) are missing, from gaps in the (primary) camera's frame counter.

The default recording folder is hard-coded as `C:/Users/justi/479_recording_single`. Pass a different folder as the second argument for each recording, because an existing recording in the same folder is overwritten.

### Checking a stereo recording

- `primary.raw` and `secondary.raw` are the same size, equal to the number of CSV data rows times `bytes_per_image`.
- Neither `id_primary` nor `id_secondary` has gaps.
- `offset_drift_ns` stays small (well under one frame period, 10,000,000 ns at 100 fps).
- No "records dropped" message and no skipped-frame message at exit.

## Getting to 100 fps

The data rate is 1920 x 1200 x 100 = 230.4 MB per second per camera, so 460.8 MB per second in stereo.

Requirements outside the code:

1. **Release build.** A Debug build on Windows makes the per-frame copy several times slower.
2. **SSD.** The disk must sustain more than 230 MB per second for one camera, and more than 460 MB per second in stereo (in practice an NVMe drive).
3. **Camera settings.** `device_link_throughput_limit` in the config must be above 230,400,000. Check whether the ADC bit depth setting limits the frame rate (the 12-bit mode is believed to; not confirmed). Exposure must be well under 10 ms.
4. **USB.** A USB 3 port and cable for each camera, ideally on separate ports. A USB 2 link or hub caps the camera at about 18.7 fps at full resolution, and the frame rate setting then fails with an out-of-range error on `AcquisitionFrameRate`.

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
- **Stereo hardware timestamps.** `--stereo` records host-side timestamps. Switching to the chunk-data timestamps (as `StereoCamera` and `record_stereo` do) would need chunk mode enabled in `configure_preview`.
- **100 fps stereo.** Test with each camera on its own laptop USB 3 port. If frames are still lost there, try receiving each camera on its own thread, since the grab loop now waits on the two cameras one after the other.

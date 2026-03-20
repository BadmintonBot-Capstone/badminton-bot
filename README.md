# baddy-bot / vision

Stereo vision pipeline for a fully autonomous badminton-playing robot. Two synchronized cameras track a shuttlecock in 3D, run it through an Extended Kalman Filter, and predict where it will land on the court.

**Pipeline:** Capture → Detect → Triangulate → Track → Predict

---

## Directory structure

```
badminton-bot/
├── CMakeLists.txt
├── config/
│   ├── camera.yaml          # Camera hardware settings
│   ├── detection.yaml       # HSV colour thresholds for shuttlecock
│   └── ekf.yaml             # Kalman filter + physics parameters
├── src/
│   ├── replay_trajectory.cpp    # Executable: offline dataset testing
│   ├── realtime.cpp             # Executable: live stereo pipeline
│   ├── capture_trajectories.cpp # Executable: record new trajectory data
│   └── core/                    # Shared static library (baddy_core)
│       ├── types.hpp            # Shared structs and type aliases
│       ├── config.hpp/cpp       # YAML config loading
│       ├── camera.hpp/cpp       # Blackfly S camera interface (Spinnaker)
│       ├── detector.hpp/cpp     # HSV shuttlecock detection
│       ├── stereo.hpp/cpp       # Stereo rectification + 3D triangulation
│       ├── tracker.hpp/cpp      # SEARCHING → TRACKING → LOST state machine
│       ├── ekf.hpp/cpp          # Extended Kalman Filter (6D state)
│       ├── birdie_dynamics.hpp/cpp  # Physics model (drag + gravity ODEs)
│       ├── landing_predictor.hpp/cpp # RK4 forward-sim to predict landing
│       └── timing.hpp/cpp       # Pipeline profiling utilities
└── tools/                   # Python calibration scripts (run once, offline)
    ├── configure_cameras.py # Apply settings to cameras via PySpin
    ├── generate_board.py    # Generate ChArUco calibration board
    ├── capture_calibration.py  # Capture board images for calibration
    ├── stereo_calibrate.py  # Compute stereo rectification matrices
    ├── calibrate_court.py   # Compute camera-to-court rigid transform
    ├── fit_alpha.py         # Re-fit drag coefficient against ground truth
    └── trim_trajectory.py   # Segment recordings into trajectory CSVs
```

> **Not in repo (gitignored):**
> - `shuttlecock_trajectory_dataset/` — 247 trajectories at 120 FPS with Vicon ground truth (3.6 cm mean error). Each trajectory has stereo video, 2D annotations, 3D CSV, and camera configs. Ask a team member for access.
> - `build/` — CMake build artifacts

---

## Core library (`src/core/`)

All reusable logic is compiled into the `baddy_core` static library. The three executables just wire these components together.

| Component | What it does |
|-----------|-------------|
| `types.hpp` | Shared data types: `Detection`, `StereoDetection`, `CourtPosition`, `LandingPrediction`, `TrackerState` enum, Eigen type aliases |
| `config` | Loads `config/` YAML files into typed structs (`CameraConfig`, `DetectionConfig`, `EkfConfig`, `CalibrationData`) |
| `camera` | Drives two Blackfly S cameras in hardware-triggered sync at 100 FPS. Wraps Spinnaker SDK. Returns `(left_frame, right_frame, timestamp_ns)` pairs. |
| `detector` | Finds the shuttlecock in a single rectified frame. Converts to HSV, applies colour thresholds, finds contours, returns centroid + area. |
| `stereo` | Takes left/right pixel centroids → 3D point in camera frame via disparity. Also applies the pre-computed rigid transform to court coordinates. |
| `tracker` | State machine sitting on top of the EKF. Manages SEARCHING → TRACKING → LOST transitions, bootstraps initial velocity from the first two detections, and calls `landing_predictor` each frame. |
| `ekf` | Extended Kalman Filter over `[x, y, z, vx, vy, vz]`. Prediction step uses the drag ODE Jacobian; update step fuses position-only measurements. |
| `birdie_dynamics` | The physics model: drag force proportional to `α·v²` (α = 0.215, fitted to Shen et al. for now). Provides ODE derivatives, 2nd-order Taylor propagation, and the 6×6 state-transition Jacobian for the EKF. |
| `landing_predictor` | Integrates the drag ODE forward with RK4 until the shuttlecock hits `y = 0` (court floor). Returns predicted `(x, z)` landing position and ETA. |
| `timing` | RAII `ScopedTimer` + rolling-window `TimingLog`. Call `summary()` to get mean/p95 per pipeline stage. |

---

## Executables

### `replay_trajectory` — offline testing (always built)
Feeds a 3D trajectory CSV from the dataset through the full Tracker (skipping cameras/stereo) and reports prediction accuracy against ground truth. Use this for tuning EKF parameters without needing hardware.

```bash
./build/replay_trajectory \
  --config config/ \
  --trajectory shuttlecock_trajectory_dataset/vicon_dataset/001/3DModel.csv
```

### `realtime` — live pipeline (requires `WITH_SPINNAKER=ON`)
Full pipeline: captures stereo frames, detects, triangulates, tracks, and predicts in real time. Opens a GUI window by default.

```bash
./build/realtime \
  --config config/ \
  --calibration config/calibration.yaml \
  [--headless]
```

### `capture_trajectories` — data recording (requires `WITH_SPINNAKER=ON`)
Records synchronized stereo frames, runs detection + triangulation, and saves each trajectory to a CSV. Used to build training/validation datasets.

```bash
./build/capture_trajectories \
  --config config/ \
  --calibration config/calibration.yaml \
  --output data/ \
  [--duration N]
```

---

## Build

**Dependencies (convert below to use your Linux Package Manager):**
```bash
sudo pacman -S cmake opencv eigen yaml-cpp
```

**Without cameras (offline/replay only):**
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

**With Blackfly S cameras (full pipeline):**
```bash
# Requires Spinnaker SDK installed to /opt/spinnaker (or override SPINNAKER_DIR)
SPINNAKER_DIR=/opt/spinnaker cmake -B build -DCMAKE_BUILD_TYPE=Release -DWITH_SPINNAKER=ON
cmake --build build -j$(nproc)
```

`compile_commands.json` is written to `build/` automatically (for clangd / IDE tooling).
(Don't worry about this.)

---

## Configuration

| File | Key parameters |
|------|---------------|
| `camera.yaml` | 100 FPS, 1920×1200, 5 ms exposure, BayerRG8, hardware trigger serials |
| `detection.yaml` | HSV range `[140,80,80]`–`[179,255,255]` (orange feathers), min contour area 5 px |
| `ekf.yaml` | α = 0.215, σ_p = 0.05 m, σ_v = 2.0 m/s, σ_xy = 2 cm, σ_z = 5 cm, coast limit = 30 frames |

---

## Calibration (one-time setup)

Run these scripts in order when setting up new cameras or a new court:

```
configure_cameras.py   → set camera hardware parameters
capture_calibration.py → record board images
stereo_calibrate.py    → compute rectification matrices → calibration.yaml
calibrate_court.py     → compute camera-to-court transform
fit_alpha.py           → (optional) re-fit drag coefficient to local data
```

# baddy-bot / vision

Stereo vision pipeline for a fully autonomous badminton-playing robot. Two synchronized Blackfly S cameras track a shuttlecock in 3D, run it through an Extended Kalman Filter, and predict where it will land on the court.

**Pipeline:** Capture → Detect → Triangulate → Track → Predict

---

## Directory structure

```
vision/
├── CMakeLists.txt
├── vcpkg.json                   # C++ dependencies, installed by vcpkg at configure time
├── config/
│   ├── camera.yaml              # Camera hardware settings
│   ├── detection.yaml           # HSV colour thresholds for shuttlecock
│   ├── ekf.yaml                 # Kalman filter + physics parameters
│   └── calibration.yaml         # Stereo rectification from the last calibration
├── data/
│   └── calibration_images/      # Stereo pairs from capture_calibration
├── src/
│   ├── replay_trajectory.cpp    # Offline dataset testing (no cameras)
│   ├── preview.cpp              # Free-running camera viewer
│   ├── dump_nodes.cpp           # List every Spinnaker camera setting
│   ├── capture_calibration.cpp  # Save stereo pairs for calibration
│   ├── depth_verify.cpp         # Live HSV tuning + depth check
│   ├── realtime.cpp             # Live stereo pipeline
│   ├── capture_trajectories.cpp # Record new trajectory data
│   └── core/                    # Shared static library (baddy_core)
│       ├── types.hpp            # Shared structs and type aliases
│       ├── config.hpp/cpp       # YAML config loading
│       ├── camera.hpp/cpp       # Blackfly S camera interface (Spinnaker)
│       ├── detector.hpp/cpp     # HSV shuttlecock detection
│       ├── stereo.hpp/cpp       # Stereo rectification + 3D triangulation
│       ├── tracker.hpp/cpp      # SEARCHING → TRACKING → LOST state machine
│       ├── ekf.hpp/cpp          # Extended Kalman Filter (6D state)
│       ├── birdie_dynamics.hpp/cpp   # Physics model (drag + gravity ODEs)
│       ├── landing_predictor.hpp/cpp # RK4 forward-sim to predict landing
│       └── timing.hpp/cpp       # Pipeline profiling utilities
└── tools/                       # Python calibration scripts (run once, offline)
    ├── generate_board.py        # Generate ChArUco calibration board
    ├── stereo_calibrate.py      # Compute stereo rectification matrices
    ├── calibrate_court.py       # Compute camera-to-court rigid transform
    ├── fit_alpha.py             # Re-fit drag coefficient against ground truth
    └── trim_trajectory.py       # Segment recordings into trajectory CSVs
```

---

## Getting started

The build runs on Windows, Ubuntu and Arch. [vcpkg](https://github.com/microsoft/vcpkg) installs OpenCV, Eigen and yaml-cpp from `vcpkg.json` the first time you configure, so you only install a compiler, CMake, vcpkg and the Spinnaker SDK yourself. The first configure compiles OpenCV from source and takes a while. Later builds reuse it.

### 1. Install the tools

**Windows:** install Visual Studio with the "Desktop development with C++" workload, plus [Git](https://git-scm.com/download/win). If Visual Studio is already installed without that workload, open "Visual Studio Installer", click Modify and tick it there. Run the Windows commands in "Developer PowerShell".

**Ubuntu:**
```bash
sudo apt install build-essential cmake ninja-build git curl zip unzip pkg-config autoconf autoconf-archive automake libtool
```

**Arch:**
```bash
sudo pacman -Syu --needed base-devel cmake ninja git autoconf-archive
```

### 2. Install vcpkg

**Windows:** already there. In the Visual Studio installer, tick "vcpkg package manager" under the C++ workload. Open Developer PowerShell and run `echo $env:VCPKG_ROOT` to see where it landed.

**Arch:** `sudo pacman -Syu --needed vcpkg`. It sets `VCPKG_ROOT` for you.

**Ubuntu:** clone it and run its bootstrap script:

```bash
git clone https://github.com/microsoft/vcpkg ~/vcpkg
~/vcpkg/bootstrap-vcpkg.sh
echo 'export VCPKG_ROOT=$HOME/vcpkg' >> ~/.bashrc
```

The clone holds the build recipes but not the `vcpkg` program itself. Bootstrapping downloads that program into the folder, which is what the build in step 4 calls. Open a new terminal afterwards so `VCPKG_ROOT` is set.

### 3. Install the Spinnaker SDK

| OS | How | Where CMake looks |
|----|-----|-------------------|
| Windows | Installer from [Teledyne](https://www.teledynevisionsolutions.com/products/spinnaker-sdk/) | `C:\Program Files\Teledyne\Spinnaker` |
| Ubuntu | Teledyne's Ubuntu tarball, then run its `install_spinnaker.sh` | `/opt/spinnaker` |
| Arch | AUR package `spinnaker-sdk` | `/opt/spinnaker` |

If it lives somewhere else, add `-DSPINNAKER_DIR=<path>` to the configure command.

On Linux, raise the USB buffer limit or the cameras drop frames: add `usbcore.usbfs_memory_mb=1000` to your kernel command line and reboot. Spinnaker's Ubuntu installer offers to do this for you.

### 4. Build

```powershell
# windows
cmake -B build -DWITH_SPINNAKER=ON -DCMAKE_TOOLCHAIN_FILE="$env:VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
cmake --build build --config Release
```
```bash
# ubuntu / arch
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DWITH_SPINNAKER=ON -DCMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake
cmake --build build
```

Leave out `-DWITH_SPINNAKER=ON` to build only `replay_trajectory`.

Executables land in `build/` on Linux and `build\Release\` on Windows.

### 5. Check the cameras

Plug both cameras in and run `preview`. It streams both cameras without the hardware trigger, so it works before the trigger cable is wired.

```bash
./build/preview config                # linux
.\build\Release\preview.exe config    # windows
```

Then run the full pipeline, which needs the hardware trigger:

```bash
./build/realtime --config config/ --calibration config/calibration.yaml
```

The Windows build has not been tested on real hardware yet. If Windows reports a missing `Spinnaker_v140.dll`, add `C:\Program Files\Teledyne\Spinnaker\bin64\vs2015` to your `PATH`.

---

## Executables

| Executable | Needs cameras | What it does |
|------------|:---:|-------------|
| `replay_trajectory` | | Feeds a trajectory CSV through the tracker and reports prediction accuracy against ground truth. Use it to tune the EKF without hardware. |
| `preview` | yes | Live view of both cameras, free running (no trigger). |
| `dump_nodes` | yes | Prints every configurable Spinnaker node on the connected cameras. |
| `capture_calibration` | yes | Saves synchronized stereo pairs for calibration. `s` saves a pair, `q` quits. |
| `depth_verify` | yes | Live HSV tuning and stereo depth check: masks, epipolar lines, triangulated XYZ. |
| `realtime` | yes | Full pipeline: detect, triangulate, track, predict landing. Add `--headless` to skip the window. |
| `capture_trajectories` | yes | Records trajectories to CSV in the folder given by `--output`. |

Most take `--config config/`, and the ones that triangulate also take `--calibration config/calibration.yaml`. `preview` takes the config folder as a plain argument, `dump_nodes` takes nothing, and `replay_trajectory` takes `--trajectory <csv>`.

---

## Core library (`src/core/`)

Everything reusable compiles into the `baddy_core` static library, and the executables wire it together.

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

## Configuration

| File | Key parameters |
|------|---------------|
| `camera.yaml` | 100 FPS, 1920×1200, 5 ms exposure, BayerRG8, hardware trigger serials |
| `detection.yaml` | HSV range `[140,80,80]` to `[179,255,255]` (orange feathers), min contour area 5 px |
| `ekf.yaml` | α = 0.215, σ_p = 0.05 m, σ_v = 2.0 m/s, σ_xy = 2 cm, σ_z = 5 cm, coast limit = 30 frames |
| `calibration.yaml` | Stereo rectification from the last calibration run |

---

## Calibration

Run these in order when setting up new cameras or a new court. The Python scripts are in `tools/`.

```
generate_board.py      → print the ChArUco board
capture_calibration    → record board image pairs
stereo_calibrate.py    → compute rectification → config/calibration.yaml
calibrate_court.py     → compute camera-to-court transform
fit_alpha.py           → (optional) re-fit drag coefficient to local data
```

---

## Not in the repo

- `shuttlecock_trajectory_dataset/`: 247 trajectories at 120 FPS with Vicon ground truth (3.6 cm mean error). Each has stereo video, 2D annotations, a 3D CSV and camera configs. Ask a team member for access.
- `build/`: CMake output.

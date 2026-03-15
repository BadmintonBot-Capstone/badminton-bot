#!/usr/bin/env python3
"""Fit the drag coefficient alpha from trajectory CSVs using least-squares.

Usage:
    python fit_alpha.py <csv1> [csv2 ...] [--config config/ekf.yaml]

Reads one or more trimmed trajectory CSVs, runs scipy.optimize.least_squares
to find the alpha that minimises prediction error, and optionally writes the
result back to ekf.yaml.
"""

import numpy as np
import pandas as pd
import yaml
import sys
from scipy.integrate import odeint
from scipy.optimize import least_squares


G = 9.81


def birdie_dynamics(state, t, alpha):
    x, y, z, vx, vy, vz = state
    speed = np.sqrt(vx**2 + vy**2 + vz**2)
    return [vx, vy, vz,
            -alpha * speed * vx,
            -G - alpha * speed * vy,
            -alpha * speed * vz]


def forward_sim(state0, t, alpha):
    return odeint(birdie_dynamics, state0, t, args=(alpha,))


def residuals(params, trajectories):
    alpha = params[0]
    all_res = []
    for traj in trajectories:
        pos = traj["positions"]
        ts = traj["timestamps"]
        state0 = traj["state0"]
        sim = forward_sim(state0, ts, alpha)[:, :3]
        all_res.append((pos - sim).flatten())
    return np.concatenate(all_res)


def load_trajectory(path, remap=None):
    """Load a trajectory CSV and return positions, timestamps, and initial state."""
    df = pd.read_csv(path)

    if remap:
        x = df[remap["x_source"]].values + remap.get("x_offset", 0)
        y = df[remap["y_source"]].values + remap.get("y_offset", 0)
        z = df[remap["z_source"]].values + remap.get("z_offset", 0)
    else:
        x, y, z = df["X"].values, df["Y"].values, df["Z"].values

    positions = np.column_stack([x, y, z])
    timestamps = df["Timestamp"].values

    dt = timestamps[1] - timestamps[0]
    vel = (positions[1] - positions[0]) / dt if dt > 0 else np.zeros(3)
    state0 = np.concatenate([positions[0], vel])

    return {"positions": positions, "timestamps": timestamps, "state0": state0}


def main():
    csv_paths = []
    config_path = None

    i = 1
    while i < len(sys.argv):
        if sys.argv[i] == "--config" and i + 1 < len(sys.argv):
            config_path = sys.argv[i + 1]
            i += 2
        else:
            csv_paths.append(sys.argv[i])
            i += 1

    if not csv_paths:
        print("Usage: fit_alpha.py <csv1> [csv2 ...] [--config config/ekf.yaml]")
        sys.exit(1)

    remap = None
    if config_path:
        with open(config_path) as f:
            cfg = yaml.safe_load(f)
        if "dataset_remap" in cfg:
            remap = cfg["dataset_remap"]

    trajectories = []
    for p in csv_paths:
        traj = load_trajectory(p, remap)
        trajectories.append(traj)
        print(f"Loaded {p}: {len(traj['positions'])} points, "
              f"duration={traj['timestamps'][-1]:.3f}s")

    print(f"\nFitting alpha across {len(trajectories)} trajectories...")
    result = least_squares(residuals, x0=[0.2], args=(trajectories,),
                           bounds=(0.01, 1.0))

    alpha_fit = result.x[0]
    print(f"  Optimal alpha = {alpha_fit:.6f}")
    print(f"  Cost          = {result.cost:.4f}")
    print(f"  RMS residual  = {np.sqrt(result.cost / len(result.fun)):.4f} m")

    if config_path:
        with open(config_path) as f:
            cfg = yaml.safe_load(f)
        cfg["alpha"] = float(round(alpha_fit, 6))
        with open(config_path, "w") as f:
            yaml.dump(cfg, f, default_flow_style=False, sort_keys=False)
        print(f"\nWrote alpha={alpha_fit:.6f} to {config_path}")
    else:
        print("\nPass --config config/ekf.yaml to write the result automatically.")


if __name__ == "__main__":
    main()

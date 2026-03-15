#!/usr/bin/env python3
"""Interactive trajectory trimmer.

Opens a trajectory CSV, displays a 3D plot, and lets you enter start/end
frame numbers to trim.  Saves the trimmed result alongside the original.

Usage:
    python trim_trajectory.py <trajectory.csv>
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
import os


def main():
    if len(sys.argv) < 2:
        print("Usage: trim_trajectory.py <trajectory.csv>")
        sys.exit(1)

    path = sys.argv[1]
    df = pd.read_csv(path)

    print(f"Loaded {len(df)} rows.  Columns: {list(df.columns)}")
    print(f"Frame range: {df['Frame'].iloc[0]} – {df['Frame'].iloc[-1]}")

    # Determine coordinate columns.
    if "X" in df.columns and "Y" in df.columns and "Z" in df.columns:
        x_col, y_col, z_col = "X", "Y", "Z"
    else:
        print("Error: expected X, Y, Z columns.")
        sys.exit(1)

    visible = df[df.get("Visibility", pd.Series([1] * len(df))) == 1]

    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(visible[x_col], visible[z_col], visible[y_col], "b.-", markersize=3)
    ax.scatter(visible[x_col].iloc[0], visible[z_col].iloc[0],
               visible[y_col].iloc[0], c="green", s=80, label="Start")
    ax.scatter(visible[x_col].iloc[-1], visible[z_col].iloc[-1],
               visible[y_col].iloc[-1], c="red", s=80, label="End")
    ax.set_xlabel("X")
    ax.set_ylabel("Z (forward)")
    ax.set_zlabel("Y (up)")
    ax.legend()
    ax.set_title(os.path.basename(path))
    plt.show(block=False)

    print("\nEnter trim boundaries (or press Enter to keep as-is):")
    start_str = input(f"  Start frame [{df['Frame'].iloc[0]}]: ").strip()
    end_str = input(f"  End frame   [{df['Frame'].iloc[-1]}]: ").strip()

    start = int(start_str) if start_str else df["Frame"].iloc[0]
    end = int(end_str) if end_str else df["Frame"].iloc[-1]

    trimmed = df[(df["Frame"] >= start) & (df["Frame"] <= end)].copy()
    trimmed["Frame"] = range(len(trimmed))
    t0 = trimmed["Timestamp"].iloc[0]
    trimmed["Timestamp"] = trimmed["Timestamp"] - t0

    out_path = path.replace(".csv", "_trimmed.csv")
    trimmed.to_csv(out_path, index=False)
    print(f"Saved {len(trimmed)} rows to {out_path}")
    plt.close()


if __name__ == "__main__":
    main()

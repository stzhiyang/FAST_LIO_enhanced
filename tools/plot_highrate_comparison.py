#!/usr/bin/env python3
"""Compare recorded raw odometry: position difference, arrival rate, 3D paths.

CSV columns: stamp,x,y,z,arrival_monotonic. The two arrival clocks must share
the same monotonic time origin, as when recorded by a single subscriber process.
"""

import argparse
import csv
import os
from pathlib import Path
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load(path):
    with path.open(newline="") as stream:
        values = np.array([
            [float(row[key]) for key in ("stamp", "x", "y", "z", "arrival_monotonic")]
            for row in csv.DictReader(stream)
        ])
    if values.ndim != 2 or len(values) < 2 or values.shape[1] != 5:
        raise ValueError(f"Insufficient samples in {path}")
    if not np.isfinite(values).all() or np.any(np.diff(values[:, 0]) <= 0):
        raise ValueError(f"Invalid measurements or non-increasing stamps in {path}")
    if np.any(np.diff(values[:, -1]) < 0):
        raise ValueError(f"Non-increasing arrival clock in {path}")
    return values


def compare(original, highrate, tolerance):
    indices = np.searchsorted(highrate[:, 0], original[:, 0])
    before = np.clip(indices - 1, 0, len(highrate) - 1)
    after = np.clip(indices, 0, len(highrate) - 1)
    use_after = (np.abs(highrate[after, 0] - original[:, 0]) <
                 np.abs(highrate[before, 0] - original[:, 0]))
    nearest = np.where(use_after, after, before)
    matched = np.abs(highrate[nearest, 0] - original[:, 0]) <= tolerance
    difference = np.full(len(original), np.nan)
    difference[matched] = np.linalg.norm(
        highrate[nearest[matched], 1:4] - original[matched, 1:4], axis=1)
    return difference, matched


def arrival_rates(original, highrate, window):
    start = max(original[0, -1], highrate[0, -1])
    end = min(original[-1, -1], highrate[-1, -1])
    bins = int(np.floor((end - start) / window))
    if bins < 2:
        raise ValueError("Insufficient duration for arrival-rate comparison")
    edges = start + np.arange(bins + 1) * window
    centers = (edges[:-1] + edges[1:]) / 2 - start
    return centers, [np.histogram(values[:, -1], edges)[0] / window
                     for values in (original, highrate)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--original", type=Path, required=True)
    parser.add_argument("--highrate", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("assets/highrate_comparison.png"))
    parser.add_argument("--source", default="Recorded ROS odometry")
    parser.add_argument("--match-ms", type=float, default=3.)
    parser.add_argument("--rate-window", type=float, default=1.)
    parser.add_argument("--update-readme", action="store_true",
                        help="Insert the generated image into the repository README")
    args = parser.parse_args()
    if args.match_ms <= 0 or args.rate_window <= 0:
        parser.error("Matching tolerance and rate window must be positive")
    original, highrate = load(args.original), load(args.highrate)
    difference, matched = compare(original, highrate, args.match_ms / 1000)
    if not np.any(matched):
        parser.error("No timestamps matched within the requested tolerance")
    rate_time, rates = arrival_rates(original, highrate, args.rate_window)
    origin = original[0, 0]
    blue, orange = "#1976b9", "#e87722"
    plt.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": 12,
        "axes.titlesize": 13, "axes.labelsize": 12,
        "axes.edgecolor": "#333333", "axes.linewidth": .85,
        "grid.color": "#c4c4c4", "grid.alpha": .7,
        "grid.linewidth": .7, "savefig.facecolor": "white",
    })
    fig = plt.figure(figsize=(15, 7.2), facecolor="white")
    grid = fig.add_gridspec(2, 2, width_ratios=(1, 1.12), height_ratios=(1, 1),
                           left=.075, right=.975, bottom=.15, top=.83,
                           wspace=.24, hspace=.6)
    error_plot = fig.add_subplot(grid[0, 0])
    rate_plot = fig.add_subplot(grid[1, 0])
    path_plot = fig.add_subplot(grid[:, 1], projection="3d")

    error_plot.plot(original[:, 0] - origin, difference, color="#424242", linewidth=1)
    error_plot.set_title("(a) Position difference between the two outputs", loc="left")
    error_plot.set_ylabel("3D position difference (m)")
    error_plot.set_xlabel("Time from first LiDAR odometry (s)")
    error_plot.set_ylim(bottom=0)
    error_plot.grid(True)

    rate_plot.plot(rate_time, rates[1], color=orange, linewidth=1.5,
                   label=f"High-rate (mean {np.mean(rates[1]):.1f} Hz)")
    rate_plot.plot(rate_time, rates[0], color=blue, linewidth=1.5,
                   label=f"Original (mean {np.mean(rates[0]):.1f} Hz)")
    rate_plot.set_title("(b) Actual received messages per second", loc="left")
    rate_plot.set_ylabel("Output rate (Hz)")
    rate_plot.set_xlabel("Elapsed receiving time (s)")
    rate_plot.set_ylim(0, max(np.max(rates[0]), np.max(rates[1])) * 1.2)
    rate_plot.grid(True)
    rate_plot.legend(loc="center right", fontsize=10, framealpha=.95)

    # Plot every raw position. Dashed original lines remain visible when paths overlap.
    path_plot.plot(highrate[:, 1], highrate[:, 2], highrate[:, 3], color=orange,
                   linewidth=1.7, label="High-rate /Odometry_highrate")
    path_plot.plot(original[:, 1], original[:, 2], original[:, 3], color=blue,
                   linewidth=1.5, linestyle="--", label="Original /Odometry")
    path_plot.scatter(*original[0, 1:4], marker="o", color="#212121", s=28, label="Start")
    path_plot.scatter(*original[-1, 1:4], marker="x", color="#212121", s=45, label="End")
    path_plot.set_title("(c) 3D trajectories in camera_init", pad=22)
    path_plot.set_xlabel("X (m)", labelpad=10)
    path_plot.set_ylabel("Y (m)", labelpad=10)
    path_plot.set_zlabel("Z (m)", labelpad=8)
    path_plot.set_box_aspect((1, 1, .7))
    path_plot.view_init(elev=26, azim=-58)
    path_plot.legend(loc="upper right", fontsize=10, framealpha=.95)
    for axis in (path_plot.xaxis, path_plot.yaxis, path_plot.zaxis):
        axis.pane.fill = False

    fig.suptitle("FAST-LIO: original and IMU-rate odometry", y=.97, fontsize=20)
    fig.text(.5, .9, args.source + " | Raw published positions; no smoothing",
             ha="center", fontsize=12, color="#666666")
    fig.text(.075, .05,
             f"Position difference: nearest IMU timestamp within {args.match_ms:g} ms; "
             "unmatched samples are gaps. This is not ground-truth error.",
             fontsize=10, color="#666666")
    fig.text(.075, .02,
             f"Rate: actual subscriber arrivals in complete {args.rate_window:g} s windows; "
             "trajectories use the original XYZ coordinates.",
             fontsize=10, color="#666666")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180)
    plt.close(fig)
    if args.update_readme:
        readme = Path(__file__).resolve().parents[1] / "README.md"
        relative_image = Path(os.path.relpath(args.output.resolve(), readme.parent)).as_posix()
        start_marker = "<!-- highrate-comparison:start -->"
        end_marker = "<!-- highrate-comparison:end -->"
        pattern = re.escape(start_marker) + r".*?" + re.escape(end_marker)
        text = readme.read_text()
        replacement = (start_marker + "\n\n"
                       "![原始里程计与高频里程计的位置差、频率和三维轨迹对比]("
                       + relative_image + ")\n\n" + end_marker)
        updated, count = re.subn(pattern, lambda match: replacement, text, flags=re.DOTALL)
        if count != 1:
            parser.error(f"Image saved, but a unique comparison image block is missing in {readme}")
        readme.write_text(updated)
        print(f"Updated {readme}")
    print(f"Saved {args.output}")
    print(f"Original samples: {len(original)}; high-rate samples: {len(highrate)}; "
          f"time matches: {np.count_nonzero(matched)} / {len(original)}")
    print(f"Position difference median/max: {np.nanmedian(difference):.6f} / "
          f"{np.nanmax(difference):.6f} m")
    print(f"Mean arrival rates ({args.rate_window:g} s windows): "
          f"{np.mean(rates[0]):.2f} / {np.mean(rates[1]):.2f} Hz")


if __name__ == "__main__":
    main()

"""Plot the seed-1 time series written by sim/test_dual_eskf.exe.

Usage (from repo root):
    python sim/plot_dual_eskf.py [sim/dual_eskf_run]   -> sim/dual_eskf_run.png

Reads <prefix>_rotZ180.csv and <prefix>_rotX180.csv.
"""
import csv
import sys

import matplotlib.pyplot as plt

PREFIX = sys.argv[1] if len(sys.argv) > 1 else "sim/dual_eskf_run"
CASES = [("rotZ180", "IMU2 rotated 180 deg about Z (same Z)"),
         ("rotX180", "IMU2 rotated 180 deg about X (opposed Z)")]

# Column index -> label (must match FilterId order in test_dual_eskf.cpp).
# Categorical slots in fixed order (reference palette, light mode).
SERIES = [
    (3, "MEKF sign-fixed, 50/50 avg", "#2a78d6"),
    (5, "Dual ESKF, no ZARU", "#eb6834"),
    (8, "Dual ESKF, ZARU at stops", "#1baf7a"),
    (12, "Dual ESKF, ZARU + mag", "#eda100"),
]
INK, INK2, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    cols = {k: [float(r[k]) for r in rows] for k in rows[0]}
    return cols


def shade_stops(ax, t, z):
    start = None
    for ti, zi in zip(t, z):
        if zi and start is None:
            start = ti
        elif not zi and start is not None:
            ax.axvspan(start / 60, ti / 60, color=GRID, lw=0, zorder=0)
            start = None


def style(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, lw=0.6)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(INK2)
    ax.tick_params(colors=INK2)


fig, axes = plt.subplots(2, 2, figsize=(13, 7.5), sharex=True)
fig.patch.set_facecolor(SURFACE)

for col, (tag, title) in enumerate(CASES):
    d = load(f"{PREFIX}_{tag}.csv")
    t = d["t"]
    tm = [x / 60 for x in t]

    ax = axes[0][col]
    style(ax)
    shade_stops(ax, t, d["zaru"])
    for idx, label, color in SERIES:
        ax.plot(tm, d[f"yawerr{idx}"], color=color, lw=2, label=label)
    ax.set_yscale("symlog", linthresh=1.0)
    ax.set_title(title, color=INK, fontsize=11, loc="left")
    ax.set_ylabel("yaw error [deg] (symlog)", color=INK2)

    ax = axes[1][col]
    style(ax)
    shade_stops(ax, t, d["zaru"])
    ax.plot(tm, d["b1z_true"], color="#2a78d6", lw=2, label="b1z true")
    ax.plot(tm, d["b1z_est_d"], color="#2a78d6", lw=1.2, ls="--", label="b1z est (ZARU)")
    ax.plot(tm, d["b2z_true"], color="#eb6834", lw=2, label="b2z true")
    ax.plot(tm, d["b2z_est_d"], color="#eb6834", lw=1.2, ls="--", label="b2z est (ZARU)")
    ax.set_ylabel("body z gyro bias [deg/s]", color=INK2)
    ax.set_xlabel("time [min]   (grey = ZARU active)", color=INK2)

axes[0][0].legend(frameon=False, fontsize=9, labelcolor=INK)
axes[1][0].legend(frameon=False, fontsize=9, labelcolor=INK, ncol=2)
fig.tight_layout()
out = f"{PREFIX}.png"
fig.savefig(out, dpi=130, facecolor=SURFACE)
print("wrote", out)

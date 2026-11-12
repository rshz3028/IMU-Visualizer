"""
mount_demo.py - why the mounting rotation matters for bias cancellation (E4).

A bias component that is common to both dies and has the SAME SIGN IN EACH
SENSOR FRAME (e.g. a shared supply/temperature effect, b_s = (bx, by, bz)) appears
in the body frame as
    IMU1 : b_s
    IMU2 : R b_s
    avg  : (I + R)/2 b_s
  * 180 deg about Z: R = diag(-1,-1, 1) -> avg = (0, 0, bz)   z is NOT cancelled
  * 180 deg about X: R = diag( 1,-1,-1) -> avg = (bx, 0, 0)   z IS cancelled
True rotation (and Earth rate) is physical in the body frame and survives both.

This script simulates both mounts with a shared constant bias and a shared tempco
(no individual biases) and measures the average virtual gyro.

    python mount_demo.py [--minutes 30] [--shared-bias 30] [--tc 2]
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import imu_log  # noqa: E402
import make_synthetic as ms  # noqa: E402
import allan  # noqa: E402

RAD2DPH = 180.0 / np.pi * 3600.0


def run(minutes=30.0, fs=100.0, shared_bias=30.0, tc=3.0, seed=11):
    rows, adev = [], {}
    for mount in ("z180", "x180"):
        df, truth = ms.generate(hours=minutes / 60, fs=fs, mount=mount, seed=seed,
                                bias_std_dps=0.0, shared_bias_dph=shared_bias,
                                tc_shared=tc, tc_indep_std=0.0, tc2=0.0, tau_th=600.0,
                                rrw_k=0.0)  # RRW off: isolates the thermal/constant effect
        log = imu_log.from_dataframe(df, mount=mount)
        avg = (log.gyr1 + log.gyr2) / 2 * RAD2DPH
        Tm = (log.temp1 + log.temp2) / 2
        for j, ax in enumerate("xyz"):
            slope = np.polyfit(Tm, avg[:, j], 1)[0]
            rows.append(dict(mount=mount, axis=ax,
                             imu1_bias_dph=np.mean(log.gyr1[:, j]) * RAD2DPH,
                             imu2_body_bias_dph=np.mean(log.gyr2[:, j]) * RAD2DPH,
                             avg_bias_dph=np.mean(avg[:, j]),
                             avg_tempco_dph_per_C=slope,
                             expected_avg_bias=(1 + log.R_mount[j, j]) / 2
                             * (shared_bias + tc * (np.mean(Tm) - 25.0)),
                             expected_avg_tempco=(1 + log.R_mount[j, j]) / 2 * tc))
        res = allan.analyze(log, weight_tau="min")
        adev[mount] = res
    return pd.DataFrame(rows), adev


def plot(df, adev, out_prefix):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 3, figsize=(15, 4.4))
    x = np.arange(3)
    for k, (mount, col) in enumerate((("z180", "#2a78d6"), ("x180", "#eb6834"))):
        d = df[df.mount == mount]
        axs[0].bar(x + (k - 0.5) * 0.36, d.avg_bias_dph, width=0.34, color=col, label=f"mount {mount}")
        axs[1].bar(x + (k - 0.5) * 0.36, d.avg_tempco_dph_per_C, width=0.34, color=col, label=f"mount {mount}")
        cz = adev[mount]["curves"]["z"]
        axs[2].loglog(cz.tau, cz.adev1_u, color=col, lw=1, ls=":", label=f"{mount} IMU1 z")
        axs[2].loglog(cz.tau, cz.adev_avg_u, color=col, lw=2, label=f"{mount} average z")
    for a, t in zip(axs[:2], ("mean bias of 50/50 average [deg/h]", "tempco of 50/50 average [deg/h/C]")):
        a.set_xticks(x); a.set_xticklabels(["x", "y", "z"]); a.set_title(t, fontsize=10)
        a.axhline(0, color="#8a8984", lw=0.8)
    axs[2].set_title("z-axis Allan deviation, no temp comp [deg/h]", fontsize=10)
    axs[2].set_xlabel("tau [s]")
    for a in axs:
        a.legend(fontsize=8, frameon=False)
        a.grid(True, color="#e4e3df", lw=0.8)
        for s in ("top", "right"):
            a.spines[s].set_visible(False)
    fig.suptitle("Shared same-sign-in-sensor-frame bias: 180 deg about Z vs about X", fontsize=11)
    fig.tight_layout()
    fig.savefig(out_prefix + ".png", dpi=130, facecolor="#fcfcfb")
    plt.close(fig)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--minutes", type=float, default=30.0)
    p.add_argument("--shared-bias", type=float, default=30.0, help="deg/h, all axes, sensor frame")
    p.add_argument("--tc", type=float, default=3.0, help="shared tempco deg/h/C, all axes")
    p.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "out", "mount_demo"))
    a = p.parse_args(argv)
    df, adev = run(a.minutes, shared_bias=a.shared_bias, tc=a.tc)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    df.to_csv(a.out + ".csv", index=False)
    plot(df, adev, a.out)
    pd.set_option("display.width", 200)
    pd.set_option("display.float_format", lambda v: f"{v:.3f}")
    print(df.to_string(index=False))
    print(f"wrote {a.out}.csv, {a.out}.png")
    return df, adev


if __name__ == "__main__":
    main()

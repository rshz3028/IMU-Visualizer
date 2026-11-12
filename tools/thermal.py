"""
thermal.py - gyro / accel bias vs temperature from a static warm-up log (E2).

For each IMU and axis the signal is averaged into --bin second bins (removes the
white noise), then fitted against that IMU's own temperature:
    linear    : b(T) = p0 + p1 (T - Tref)
    quadratic : b(T) = q0 + q1 (T - Tref) + q2 (T - Tref)^2
Reported tempco: p1 (and q1 = slope of the quadratic at Tref, plus its slope at the
middle of the observed temperature range, which is much better conditioned when Tref
lies at the edge of the data, as in a warm-up ramp).
Caveat: in a single monotonic warm-up the slow stochastic bias (flicker / RRW) is
confounded with temperature; a chamber sweep up AND down (E2) separates them.
  gyro  : deg/h per degC (and deg/h per degC^2 for q2)
  accel : mg per degC
The board must be static. Gyro constant offsets (incl. Earth rate) go into p0 and do
not affect the slope; for accel, gravity sits in p0 as well.

The two IMUs' thermal behaviour is compared per axis by
  * the ratio and difference of tempcos, and
  * the Pearson correlation of the binned bias series (IMU1 vs IMU2), both raw and
    after removing each IMU's own linear-in-temperature fit (residual correlation).
Default frame for the comparison is the SENSOR frame (physical tempco of each die);
use --frame body to see the combination-relevant sign (z180 flips x and y of IMU2).

    python thermal.py raw_log.csv [--mount z180] [--bin 10] [--tref 25] [--frame sensor]
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import imu_log  # noqa: E402

RAD2DPH = 180.0 / np.pi * 3600.0
G0 = 9.80665
C1, C2 = "#2a78d6", "#eb6834"


def bin_average(t, x, width):
    k = np.floor((t - t[0]) / width).astype(int)
    cnt = np.bincount(k)
    ok = cnt > 0.5 * np.median(cnt)
    def avg(v):
        return (np.bincount(k, weights=v) / np.maximum(cnt, 1))[ok]
    if x.ndim == 1:
        return avg(x)
    return np.column_stack([avg(x[:, i]) for i in range(x.shape[1])])


def fit_poly(T, y, order, Tref):
    X = T - Tref
    A = np.column_stack([X ** k for k in range(order + 1)])
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    yhat = A @ coef
    ss_res = np.sum((y - yhat) ** 2)
    ss_tot = np.sum((y - y.mean()) ** 2)
    return coef, 1 - ss_res / ss_tot if ss_tot > 0 else np.nan, np.sqrt(ss_res / len(y)), yhat


def analyze(log: imu_log.IMULog, bin_s=10.0, tref=25.0, frame="sensor", skip_s=0.0):
    sel = log.t >= skip_s
    t = log.t[sel]
    T = [bin_average(t, log.temp1[sel], bin_s), bin_average(t, log.temp2[sel], bin_s)]
    tb = bin_average(t, t, bin_s)
    data = {
        "gyro": [log.gyr1[sel] * RAD2DPH,
                 (log.gyr2 if frame == "body" else log.gyr2_sensor)[sel] * RAD2DPH],
        "accel": [log.acc1[sel] / G0 * 1e3,
                  (log.acc2 if frame == "body" else log.acc2_sensor)[sel] / G0 * 1e3],
    }
    rows, comp, binned = [], [], {"t": tb, "T1": T[0], "T2": T[1]}
    for sensor, (s1, s2) in data.items():
        unit = "deg/h" if sensor == "gyro" else "mg"
        B = [bin_average(t, s1, bin_s), bin_average(t, s2, bin_s)]
        for i, ax in enumerate("xyz"):
            res_lin = []
            for imu in (0, 1):
                y = B[imu][:, i]
                binned[f"{sensor}_{ax}{imu+1}"] = y
                (p0, p1), r2l, rmsl, yl = fit_poly(T[imu], y, 1, tref)
                (q0, q1, q2), r2q, rmsq, _ = fit_poly(T[imu], y, 2, tref)
                res_lin.append(y - yl)
                Tmid = 0.5 * (T[imu].min() + T[imu].max())
                rows.append(dict(sensor=sensor, imu=imu + 1, axis=ax, unit=unit,
                                 offset_at_Tref=p0, tempco_lin=p1, R2_lin=r2l, rms_lin=rmsl,
                                 q0=q0, tempco_quad_at_Tref=q1, q2=q2,
                                 Tmid=Tmid, tempco_quad_at_Tmid=q1 + 2 * q2 * (Tmid - tref), R2_quad=r2q, rms_quad=rmsq,
                                 drift_over_range=p1 * (T[imu].max() - T[imu].min())))
            y1, y2 = B[0][:, i], B[1][:, i]
            tc1 = rows[-2]["tempco_lin"]; tc2 = rows[-1]["tempco_lin"]
            comp.append(dict(sensor=sensor, axis=ax, unit=f"{unit}/degC",
                             tempco1=tc1, tempco2=tc2, ratio_2_over_1=tc2 / tc1 if tc1 else np.nan,
                             tempco_avg50=(tc1 + tc2) / 2,
                             r_binned=np.corrcoef(y1, y2)[0, 1],
                             r_residual=np.corrcoef(res_lin[0], res_lin[1])[0, 1]))
    return dict(fits=pd.DataFrame(rows), comparison=pd.DataFrame(comp),
                binned=pd.DataFrame(binned), frame=frame, tref=tref)


def plot(res, out_prefix):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    b = res["binned"]; f = res["fits"]
    fig, axs = plt.subplots(2, 3, figsize=(15, 8))
    for r, sensor in enumerate(("gyro", "accel")):
        for c, ax in enumerate("xyz"):
            a = axs[r, c]
            for imu, col in ((1, C1), (2, C2)):
                T = b[f"T{imu}"]; y = b[f"{sensor}_{ax}{imu}"]
                yc = y - np.mean(y)
                a.plot(T, yc, ".", ms=3, color=col, alpha=0.5, label=f"IMU{imu} (mean removed)")
                row = f[(f.sensor == sensor) & (f.imu == imu) & (f.axis == ax)].iloc[0]
                Ts = np.linspace(T.min(), T.max(), 50)
                X = Ts - res["tref"]
                yq = row.q0 + row.tempco_quad_at_Tref * X + row.q2 * X * X
                a.plot(Ts, yq - np.mean(y), color=col, lw=2)
                a.text(0.02, 0.95 - 0.08 * (imu - 1), f"IMU{imu}: {row.tempco_lin:+.3g} {row.unit}/C",
                       transform=a.transAxes, fontsize=8, color="#0b0b0b", va="top")
            a.set_title(f"{sensor} {ax} ({res['frame']} frame)", fontsize=10)
            a.set_xlabel("temperature [degC]", fontsize=9, color="#52514e")
            a.set_ylabel(f"bias - mean [{'deg/h' if sensor=='gyro' else 'mg'}]", fontsize=9, color="#52514e")
            a.grid(True, color="#e4e3df", lw=0.8)
            for s in ("top", "right"):
                a.spines[s].set_visible(False)
    axs[0, 0].legend(fontsize=8, frameon=False, loc="lower right")
    fig.tight_layout()
    fig.savefig(out_prefix + "_bias_vs_T.png", dpi=120, facecolor="#fcfcfb")
    plt.close(fig)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("log")
    imu_log.add_mount_args(p)
    p.add_argument("--bin", type=float, default=10.0, help="bin width [s]")
    p.add_argument("--tref", type=float, default=25.0)
    p.add_argument("--frame", choices=["sensor", "body"], default="sensor")
    p.add_argument("--skip", type=float, default=0.0, help="ignore the first N seconds")
    p.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "out", "thermal"))
    a = p.parse_args(argv)
    log = imu_log.load(a.log, a.mount)
    res = analyze(log, a.bin, a.tref, a.frame, a.skip)
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    res["fits"].to_csv(a.out + "_fits.csv", index=False)
    res["comparison"].to_csv(a.out + "_comparison.csv", index=False)
    plot(res, a.out)
    pd.set_option("display.width", 220)
    pd.set_option("display.float_format", lambda v: f"{v:.4g}")
    print(res["fits"][["sensor", "imu", "axis", "unit", "tempco_lin", "R2_lin",
                       "tempco_quad_at_Tref", "tempco_quad_at_Tmid", "q2", "R2_quad", "drift_over_range"]].to_string(index=False))
    print("\nIMU1 vs IMU2 thermal comparison:")
    print(res["comparison"].to_string(index=False))
    print(f"\nwrote {a.out}_fits.csv, _comparison.csv, _bias_vs_T.png")
    return res


if __name__ == "__main__":
    main()

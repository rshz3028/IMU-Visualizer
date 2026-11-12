"""
six_position.py - accelerometer bias / scale from six static orientations (E3),
plus gyro bias per static segment.

Segments are either
  * auto-detected: windows where the accelerometer moving std is small and the
    gyro moving mean is close to the log's overall static level, lasting at least
    --min-dur seconds (edges trimmed by --trim), or
  * user-labelled: --segments file.csv with columns label,t_start,t_end
    (t in seconds from log start; label e.g. +Z,-Z,+X,-X,+Y,-Y = axis pointing UP).

Each IMU is calibrated in its OWN sensor frame; its six labels are derived from
the measured gravity direction in that frame (so IMU2's labels differ from IMU1's
under a rotated mount). Per axis (accelerometer reads +g when the axis points up):
    bias  = (a_up + a_down) / 2
    scale = (a_up - a_down) / (2 g) - 1          (scale-factor error)
A full 12-parameter least-squares model a_meas = M a_true + b is also fitted
(M = scale + misalignment) using the three-axis means of all six positions.

    python six_position.py six.csv [--segments segs.csv] [--g 9.80665] [--mount z180]
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
LABELS = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"]


def _movstat(x, w):
    """Centered moving mean and std along axis 0 (window w samples)."""
    c1 = np.cumsum(np.vstack([np.zeros((1, x.shape[1])), x]), axis=0)
    c2 = np.cumsum(np.vstack([np.zeros((1, x.shape[1])), x * x]), axis=0)
    n = len(x)
    i0 = np.clip(np.arange(n) - w // 2, 0, n)
    i1 = np.clip(i0 + w, 0, n)
    cnt = (i1 - i0)[:, None]
    m = (c1[i1] - c1[i0]) / cnt
    v = (c2[i1] - c2[i0]) / cnt - m * m
    return m, np.sqrt(np.maximum(v, 0))


def detect_static(log, win_s=1.0, acc_std_thr=0.05, gyr_thr=0.03, min_dur=10.0, trim=1.0):
    w = max(3, int(win_s * log.fs))
    static = np.ones(log.n, bool)
    for acc, gyr in ((log.acc1, log.gyr1), (log.acc2_sensor, log.gyr2_sensor)):
        _, sa = _movstat(acc, w)
        mg, _ = _movstat(gyr, w)
        g0 = np.median(gyr, axis=0)  # static bias level (most of the log is static)
        static &= (sa.max(axis=1) < acc_std_thr) & (np.linalg.norm(mg - g0, axis=1) < gyr_thr)
    # runs
    d = np.diff(np.concatenate([[0], static.astype(int), [0]]))
    starts, ends = np.nonzero(d == 1)[0], np.nonzero(d == -1)[0]
    segs = []
    for s, e in zip(starts, ends):
        t0, t1 = log.t[s] + trim, log.t[e - 1] - trim
        if t1 - t0 >= min_dur:
            segs.append(dict(label="", t_start=t0, t_end=t1))
    return pd.DataFrame(segs)


def classify(mean_acc):
    j = int(np.argmax(np.abs(mean_acc)))
    purity = abs(mean_acc[j]) / np.linalg.norm(mean_acc)
    return ("+" if mean_acc[j] > 0 else "-") + "XYZ"[j], purity


def segment_stats(log, segs):
    rows = []
    for k, s in segs.reset_index(drop=True).iterrows():
        sel = (log.t >= s.t_start) & (log.t <= s.t_end)
        for imu, acc, gyr, temp in ((1, log.acc1, log.gyr1, log.temp1),
                                    (2, log.acc2_sensor, log.gyr2_sensor, log.temp2)):
            ma = acc[sel].mean(axis=0)
            mg = gyr[sel].mean(axis=0)
            lab, pur = classify(ma)
            rows.append(dict(segment=k, user_label=s.get("label", ""), imu=imu, label=lab,
                             purity=pur, t_start=s.t_start, t_end=s.t_end, n=int(sel.sum()),
                             ax=ma[0], ay=ma[1], az=ma[2], a_norm=np.linalg.norm(ma),
                             gx_dph=mg[0] * RAD2DPH, gy_dph=mg[1] * RAD2DPH, gz_dph=mg[2] * RAD2DPH,
                             temp=temp[sel].mean()))
    return pd.DataFrame(rows)


def calibrate(stats, g=9.80665):
    out, lsq = [], {}
    for imu in (1, 2):
        st = stats[(stats.imu == imu) & (stats.purity > 0.9)]
        best = {}
        for lab in LABELS:
            c = st[st.label == lab]
            if len(c):
                best[lab] = c.loc[c.n.idxmax()]
        missing = [l for l in LABELS if l not in best]
        if missing:
            print(f"IMU{imu}: missing orientations {missing} - cannot calibrate those axes")
        for j, ax in enumerate("xyz"):
            up, dn = best.get("+" + ax.upper()), best.get("-" + ax.upper())
            if up is None or dn is None:
                continue
            col = "a" + ax
            b = (up[col] + dn[col]) / 2
            s = (up[col] - dn[col]) / (2 * g) - 1
            out.append(dict(imu=imu, axis=ax, bias_m_s2=b, bias_mg=b / 9.80665 * 1e3,
                            scale_err=s, scale_err_ppm=s * 1e6,
                            temp_up=up.temp, temp_down=dn.temp))
        if len(best) == 6:
            # a_meas = M a_true + b  -> [a_true^T 1] [M^T; b^T] = a_meas^T
            A, Y = [], []
            for lab, r in best.items():
                j = "XYZ".index(lab[1]); sgn = 1 if lab[0] == "+" else -1
                at = np.zeros(3); at[j] = sgn * g
                A.append(np.concatenate([at, [1.0]])); Y.append([r.ax, r.ay, r.az])
            X, *_ = np.linalg.lstsq(np.array(A), np.array(Y), rcond=None)
            lsq[imu] = dict(M=X[:3].T, b=X[3])
    return pd.DataFrame(out), lsq


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("log")
    imu_log.add_mount_args(p)
    p.add_argument("--segments", help="CSV with label,t_start,t_end (seconds)")
    p.add_argument("--g", type=float, default=9.80665, help="local gravity [m/s^2]")
    p.add_argument("--min-dur", type=float, default=10.0)
    p.add_argument("--acc-std", type=float, default=0.05, help="static threshold, accel moving std [m/s^2]")
    p.add_argument("--gyr-thr", type=float, default=0.03, help="static threshold, |gyro mean - median| [rad/s]")
    p.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "out", "six"))
    a = p.parse_args(argv)
    log = imu_log.load(a.log, a.mount)
    if a.segments:
        segs = pd.read_csv(a.segments)
    else:
        segs = detect_static(log, acc_std_thr=a.acc_std, gyr_thr=a.gyr_thr, min_dur=a.min_dur)
    print(f"{len(segs)} static segments")
    stats = segment_stats(log, segs)
    if a.segments:
        chk = stats[(stats.imu == 1) & (stats.user_label != stats.label)]
        for _, r in chk.iterrows():
            print(f"WARNING segment {r.segment}: user label {r.user_label} but IMU1 gravity says {r.label}")
    cal, lsq = calibrate(stats, a.g)
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    stats.to_csv(a.out + "_segments.csv", index=False)
    cal.to_csv(a.out + "_calibration.csv", index=False)
    pd.set_option("display.width", 220)
    pd.set_option("display.float_format", lambda v: f"{v:.5g}")
    print(stats[["segment", "imu", "label", "purity", "t_start", "t_end", "a_norm",
                 "gx_dph", "gy_dph", "gz_dph", "temp"]].to_string(index=False))
    print("\nAccelerometer calibration (sensor frames):")
    print(cal.to_string(index=False))
    gb = stats.groupby("imu")[["gx_dph", "gy_dph", "gz_dph"]].agg(["mean", "std"])
    print("\nGyro bias across segments [deg/h] (mean, std):\n", gb.to_string())
    for imu, r in lsq.items():
        print(f"\nIMU{imu} 12-parameter LSQ: M =\n{np.array2string(r['M'], precision=6)}\n b = {r['b']}")
    return stats, cal, lsq


if __name__ == "__main__":
    main()

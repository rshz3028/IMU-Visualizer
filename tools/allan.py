"""
allan.py - overlapping Allan deviation / Allan covariance for a dual-IMU log.

For each axis (body frame by default) it computes
  * AVAR of IMU1, IMU2,
  * AVAR of the 50/50 average virtual gyro  (x+y)/2,
  * Allan covariance between IMU1 and IMU2 (Vaccaro & Zaki, Sensors 2017):
        cov(tau) = (AVAR(x+y) - AVAR(x-y)) / 4,
    and the Allan correlation coefficient rho(tau) = cov / sqrt(AVAR(x) AVAR(y)),
  * the optimal combination weights c*(tau) = Q^-1 1 / (1^T Q^-1 1) with
    Q(tau) = [[AVAR1, cov], [cov, AVAR2]], the resulting per-tau optimal envelope
    1/(1^T Q^-1 1), and the AVAR of a *fixed-weight* virtual gyro using c* evaluated
    at --weight-tau (default: tau of the minimum of the averaged curve).

Noise-term extraction (IEEE Std 952 conventions):
  * ARW  : fit a line with fixed slope -1/2 to log(sigma) over --arw-range,
           read it at tau = 1 s.            gyro: deg/sqrt(h)   accel: m/s/sqrt(h)
  * BI   : min(sigma) / 0.664 (flicker floor), searched for tau <= T/20.  gyro: deg/h      accel: ug
  * RRW  : fixed slope +1/2 fit to the rising branch after the minimum, read at
           tau = 3 s.                       gyro: deg/h/sqrt(h)  accel: ug/sqrt(h)

Note on the identity: by linearity of the Allan second difference, AVAR(x+y) and
AVAR(x-y) are computed from the cluster differences d_x +/- d_y; this is exactly
the AVAR of the sum/difference signal. The fixed-weight virtual gyro AVAR equals
c^T Q c for the same reason.

Usage:
    python allan.py raw_log.csv [--mount z180] [--sensor gyro|accel] [--frame body|sensor]
                    [--temp-comp {0,1,2}] [--out out/allan] [--weight-tau min|<seconds>]
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

RAD2DEG = 180.0 / np.pi
G0 = 9.80665
AXES = "xyz"

# colour roles (reference categorical palette, fixed order)
C_IMU1, C_IMU2, C_AVG, C_W, C_OPT = "#2a78d6", "#eb6834", "#1baf7a", "#6250d6", "#52514e"


# ----------------------------------------------------------------------------
# core
# ----------------------------------------------------------------------------
def tau_grid(n: int, fs: float, per_decade: int = 12, max_frac: float = 0.25):
    m_max = max(1, int((n - 1) * max_frac) // 1)
    m_max = min(m_max, (n - 1) // 2)
    m = np.unique(np.round(np.logspace(0, np.log10(m_max), int(per_decade * np.log10(m_max)) + 1)).astype(int))
    return m, m / fs


def _theta(x: np.ndarray, fs: float) -> np.ndarray:
    """Integrated signal with leading zero; x: (N,) or (N,k)."""
    x = np.asarray(x, float)
    z = np.zeros((1,) + x.shape[1:])
    return np.concatenate([z, np.cumsum(x, axis=0) / fs], axis=0)


def _d2(theta: np.ndarray, m: int) -> np.ndarray:
    return theta[2 * m:] - 2.0 * theta[m:-m] + theta[:-2 * m]


def avar(x: np.ndarray, fs: float, m=None):
    """Overlapping AVAR of a single series (or columns). Returns tau, avar."""
    x = np.asarray(x, float)
    if m is None:
        m, _ = tau_grid(len(x), fs)
    th = _theta(x, fs)
    out = []
    for mi in m:
        tau = mi / fs
        out.append(np.mean(_d2(th, mi) ** 2, axis=0) / (2 * tau * tau))
    return m / fs, np.array(out)


def dual_avar(x: np.ndarray, y: np.ndarray, fs: float, m=None) -> pd.DataFrame:
    """Allan variance/covariance analysis for two co-sampled 1-D series."""
    n = len(x)
    if m is None:
        m, _ = tau_grid(n, fs)
    thx, thy = _theta(x, fs), _theta(y, fs)
    rows = []
    for mi in m:
        tau = mi / fs
        dx, dy = _d2(thx, mi), _d2(thy, mi)
        k = 1.0 / (2 * tau * tau)
        ax = k * np.mean(dx * dx)
        ay = k * np.mean(dy * dy)
        s = dx + dy
        d = dx - dy
        a_sum = k * np.mean(s * s)          # AVAR(x + y)
        a_dif = k * np.mean(d * d)          # AVAR(x - y)
        rows.append((mi, tau, n - 2 * mi, ax, ay, a_sum, a_dif))
    df = pd.DataFrame(rows, columns=["m", "tau", "nterms", "avar1", "avar2", "avar_sum", "avar_diff"])
    df["cov"] = (df.avar_sum - df.avar_diff) / 4.0
    df["rho"] = df["cov"] / np.sqrt(df.avar1 * df.avar2)
    df["avar_avg"] = df.avar_sum / 4.0
    # per-tau optimal weights and envelope
    w1, w2, env = [], [], []
    for a1, a2, c in zip(df.avar1, df.avar2, df["cov"]):
        Q = np.array([[a1, c], [c, a2]])
        try:
            qi1 = np.linalg.solve(Q, np.ones(2))
            den = qi1.sum()
            w1.append(qi1[0] / den); w2.append(qi1[1] / den); env.append(1.0 / den)
        except np.linalg.LinAlgError:
            w1.append(np.nan); w2.append(np.nan); env.append(np.nan)
    df["w1_opt"], df["w2_opt"], df["avar_opt_env"] = w1, w2, env
    # number of independent clusters (for confidence)
    df["nclusters"] = np.floor(n / df.m).astype(int)
    return df


def fixed_weight_avar(df: pd.DataFrame, c1: float, c2: float) -> np.ndarray:
    return c1 * c1 * df.avar1 + c2 * c2 * df.avar2 + 2 * c1 * c2 * df["cov"]


def adev_ci(adev, nclusters):
    """Approximate 1-sigma relative error of ADEV: 1/sqrt(2(K-1))."""
    rel = 1.0 / np.sqrt(np.maximum(2 * (nclusters - 1), 1))
    return adev * (1 - rel), adev * (1 + rel)


# ----------------------------------------------------------------------------
# noise-term extraction
# ----------------------------------------------------------------------------
def characterize(tau, adev, duration, arw_range=(0.1, 2.0), max_tau_frac=0.05):
    """Return dict with N (value at 1 s of -1/2 fit), B, tau_B, K (at 3 s of +1/2 fit).

    adev in the analysis unit (e.g. deg/h). N is returned in unit*sqrt(s)
    (i.e. adev at tau = 1 s), K in unit/sqrt(s) (adev at tau = 3 s).
    """
    tau = np.asarray(tau, float); adev = np.asarray(adev, float)
    good = np.isfinite(adev) & (adev > 0)
    lt, la = np.log10(tau[good]), np.log10(adev[good])
    tg = tau[good]
    res = dict(N=np.nan, N_slope=np.nan, B=np.nan, tau_B=np.nan, K=np.nan, K_npts=0)
    sel = (tg >= arw_range[0]) & (tg <= arw_range[1])
    if sel.sum() >= 2:
        res["N"] = 10 ** np.mean(la[sel] + 0.5 * lt[sel])
        res["N_slope"] = np.polyfit(lt[sel], la[sel], 1)[0]
    # BI: minimum over the trustworthy range (enough clusters). The minimum of a noisy
    # curve is biased low; on simulated flicker the bias is ~-7 % for tau <= T/20 and
    # ~-12 % for tau <= T/8 (2 h logs).
    usable = tg <= max_tau_frac * duration
    if usable.sum() > 0:
        i = np.argmin(np.where(usable, la, np.inf))
        res["B"] = 10 ** la[i] / 0.664
        res["tau_B"] = tg[i]
        # RRW: rising branch after the minimum, local slope ~ +1/2
        after = (tg > tg[i]) & (tg <= 0.25 * duration)
        if after.sum() >= 3:
            sl = np.gradient(la, lt)
            rr = after & (sl > 0.25) & (sl < 0.9)
            if rr.sum() >= 3:
                # anchored fit with slope +1/2, read at tau=3s
                res["K"] = 10 ** np.mean(la[rr] - 0.5 * lt[rr]) * np.sqrt(3.0)
                res["K_npts"] = int(rr.sum())
    return res


# ----------------------------------------------------------------------------
# preprocessing
# ----------------------------------------------------------------------------
def temp_compensate(x: np.ndarray, temp: np.ndarray, order: int) -> np.ndarray:
    """Remove a polynomial-in-temperature fit (per column), keeping the mean."""
    if order <= 0:
        return x
    T = temp - temp.mean()
    A = np.column_stack([T ** k for k in range(1, order + 1)] + [np.ones_like(T)])
    coef, *_ = np.linalg.lstsq(A, x, rcond=None)
    return x - A[:, :-1] @ coef[:-1]


def get_series(log: imu_log.IMULog, sensor: str, frame: str, temp_comp: int):
    if sensor == "gyro":
        s1, s2 = log.gyr1, (log.gyr2 if frame == "body" else log.gyr2_sensor)
    else:
        s1, s2 = log.acc1, (log.acc2 if frame == "body" else log.acc2_sensor)
    s1 = imu_log.uniform_series(log, s1)
    s2 = imu_log.uniform_series(log, s2)
    t1 = imu_log.uniform_series(log, log.temp1)
    t2 = imu_log.uniform_series(log, log.temp2)
    if temp_comp:
        s1 = temp_compensate(s1, t1, temp_comp)
        s2 = temp_compensate(s2, t2, temp_comp)
    return s1, s2


def unit_info(sensor):
    if sensor == "gyro":
        # rad/s -> deg/h
        return dict(scale=RAD2DEG * 3600.0, adev_unit="deg/h",
                    N_unit="deg/sqrt(h)", N_conv=1 / 60.0,   # (deg/h)*sqrt(s) -> deg/sqrt(h)
                    B_unit="deg/h", B_conv=1.0,
                    K_unit="deg/h/sqrt(h)", K_conv=60.0)     # (deg/h)/sqrt(s) -> deg/h/sqrt(h)
    # m/s^2 -> ug
    return dict(scale=1e6 / G0, adev_unit="ug",
                N_unit="m/s/sqrt(h)", N_conv=G0 * 1e-6 / 60.0,  # ug*sqrt(s) -> m/s/sqrt(h)
                B_unit="ug", B_conv=1.0,
                K_unit="ug/sqrt(h)", K_conv=60.0)


# ----------------------------------------------------------------------------
# full analysis
# ----------------------------------------------------------------------------
def analyze(log: imu_log.IMULog, sensor="gyro", frame="body", temp_comp=0,
            weight_tau="min", arw_range=(0.1, 2.0), per_decade=12, bi_max_frac=0.05):
    s1, s2 = get_series(log, sensor, frame, temp_comp)
    fs = log.fs
    n = len(s1)
    duration = n / fs
    m, _ = tau_grid(n, fs, per_decade)
    u = unit_info(sensor)
    curves, summary, weights = {}, [], {}
    for i, ax in enumerate(AXES):
        df = dual_avar(s1[:, i], s2[:, i], fs, m)
        # choose fixed weights
        if weight_tau == "min":
            usable = df.tau <= bi_max_frac * duration
            j = int(np.argmin(np.where(usable, df.avar_avg, np.inf)))
        else:
            j = int(np.argmin(np.abs(np.log(df.tau / float(weight_tau)))))
        c1, c2 = float(df.w1_opt.iloc[j]), float(df.w2_opt.iloc[j])
        weights[ax] = dict(c1=c1, c2=c2, tau=float(df.tau.iloc[j]))
        df["avar_w"] = fixed_weight_avar(df, c1, c2)
        sc2 = u["scale"] ** 2
        for col in ("avar1", "avar2", "avar_avg", "avar_w", "avar_opt_env", "avar_sum", "avar_diff"):
            df[col.replace("avar", "adev") + "_u"] = np.sqrt(np.abs(df[col]) * sc2)
        df["cov_u2"] = df["cov"] * sc2   # Allan covariance in (display unit)^2
        curves[ax] = df
        for name, col in (("IMU1", "adev1_u"), ("IMU2", "adev2_u"), ("avg50", "adev_avg_u"),
                          ("weighted", "adev_w_u"), ("opt_envelope", "adev_opt_env_u")):
            r = characterize(df.tau, df[col], duration, arw_range, bi_max_frac)
            summary.append(dict(sensor=sensor, axis=ax, series=name,
                                **{f"ARW[{u['N_unit']}]": r["N"] * u["N_conv"],
                                   "ARW_free_slope": r["N_slope"],
                                   f"BI[{u['B_unit']}]": r["B"] * u["B_conv"],
                                   "tau_BI[s]": r["tau_B"],
                                   f"RRW[{u['K_unit']}]": r["K"] * u["K_conv"],
                                   "RRW_npts": r["K_npts"]}))
    return dict(curves=curves, summary=pd.DataFrame(summary), weights=weights,
                units=u, duration=duration, fs=fs, sensor=sensor, frame=frame,
                temp_comp=temp_comp)


def rho_table(res, taus=(0.1, 1, 10, 100, 1000)):
    rows = []
    for ax, df in res["curves"].items():
        r = dict(axis=ax)
        for t in taus:
            if t <= df.tau.max():
                j = int(np.argmin(np.abs(np.log(df.tau / t))))
                r[f"rho@{t:g}s"] = df.rho.iloc[j]
        sel = df.tau <= 0.125 * res["duration"]  # median rho over the reliable range
        r["rho_median"] = float(np.median(df.rho[sel]))
        r["c1"], r["c2"], r["c_tau[s]"] = (res["weights"][ax]["c1"], res["weights"][ax]["c2"],
                                           res["weights"][ax]["tau"])
        rows.append(r)
    return pd.DataFrame(rows)


# ----------------------------------------------------------------------------
# plotting
# ----------------------------------------------------------------------------
def _style(ax):
    ax.grid(True, which="major", color="#e4e3df", lw=0.8)
    ax.grid(True, which="minor", color="#f0efec", lw=0.5)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color("#8a8984")
    ax.tick_params(colors="#52514e", labelsize=8)


def plot(res, out_prefix: str, title: str = ""):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    u = res["units"]
    fig, axs = plt.subplots(1, 3, figsize=(15, 4.8), sharey=True)
    for ax, (axn, df) in zip(axs, res["curves"].items()):
        df = df[df.nclusters >= 5]   # hide the last, unreliable points
        lo, hi = adev_ci(df.adev1_u, df.nclusters)
        ax.fill_between(df.tau, lo, hi, color=C_IMU1, alpha=0.12, lw=0)
        ax.loglog(df.tau, df.adev1_u, color=C_IMU1, lw=2, label="IMU1")
        ax.loglog(df.tau, df.adev2_u, color=C_IMU2, lw=2, label="IMU2 (body)")
        lo, hi = adev_ci(df.adev_avg_u, df.nclusters)
        ax.fill_between(df.tau, lo, hi, color=C_AVG, alpha=0.12, lw=0)
        ax.loglog(df.tau, df.adev_avg_u, color=C_AVG, lw=2, label="50/50 average")
        w = res["weights"][axn]
        ax.loglog(df.tau, df.adev_w_u, color=C_W, lw=2,
                  label=f"weighted c=({w['c1']:.2f},{w['c2']:.2f})")
        ax.loglog(df.tau, df.adev_opt_env_u, color=C_OPT, lw=1.2, ls="--", label="per-tau optimum")
        ax.set_title(f"{res['sensor']} {axn}", fontsize=10, color="#0b0b0b")
        ax.set_xlabel("tau [s]", fontsize=9, color="#52514e")
        _style(ax)
    axs[0].set_ylabel(f"Allan deviation [{u['adev_unit']}]", fontsize=9, color="#52514e")
    axs[0].legend(fontsize=8, frameon=False)
    fig.suptitle(title or f"Allan deviation ({res['frame']} frame, temp-comp={res['temp_comp']})",
                 fontsize=11, color="#0b0b0b")
    fig.tight_layout()
    fig.savefig(out_prefix + "_adev.png", dpi=130, facecolor="#fcfcfb")
    plt.close(fig)

    fig, axs = plt.subplots(1, 2, figsize=(12, 4.4))
    cols = {"x": C_IMU1, "y": C_IMU2, "z": C_AVG}
    for axn, df in res["curves"].items():
        sel = df.nclusters >= 5
        axs[0].semilogx(df.tau[sel], df.rho[sel], color=cols[axn], lw=2, label=f"{axn}")
        axs[1].semilogx(df.tau[sel], df.w1_opt[sel], color=cols[axn], lw=2, label=f"c1 {axn}")
    axs[0].axhline(0, color="#8a8984", lw=0.8)
    axs[0].set_ylim(-1.05, 1.05)
    axs[0].set_ylabel("Allan correlation rho(tau)", fontsize=9, color="#52514e")
    axs[1].axhline(0.5, color="#8a8984", lw=0.8, ls=":")
    axs[1].set_ylabel("optimal weight c1*(tau) (c2 = 1 - c1)", fontsize=9, color="#52514e")
    for a in axs:
        a.set_xlabel("tau [s]", fontsize=9, color="#52514e")
        a.legend(fontsize=8, frameon=False)
        _style(a)
    fig.suptitle(f"IMU1-IMU2 Allan covariance ({res['frame']} frame)", fontsize=11, color="#0b0b0b")
    fig.tight_layout()
    fig.savefig(out_prefix + "_rho.png", dpi=130, facecolor="#fcfcfb")
    plt.close(fig)


def save(res, out_prefix):
    Path(out_prefix).parent.mkdir(parents=True, exist_ok=True)
    res["summary"].to_csv(out_prefix + "_summary.csv", index=False)
    rho_table(res).to_csv(out_prefix + "_rho.csv", index=False)
    allc = []
    for ax, df in res["curves"].items():
        d = df.copy(); d.insert(0, "axis", ax); allc.append(d)
    pd.concat(allc).to_csv(out_prefix + "_curves.csv", index=False)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("log")
    imu_log.add_mount_args(p)
    p.add_argument("--sensor", choices=["gyro", "accel"], default="gyro")
    p.add_argument("--frame", choices=["body", "sensor"], default="body",
                   help="analyse IMU2 in body frame (default) or its own sensor frame")
    p.add_argument("--temp-comp", type=int, default=0, choices=[0, 1, 2],
                   help="remove a linear(1)/quadratic(2) fit vs own temperature first")
    p.add_argument("--weight-tau", default="min",
                   help="tau at which fixed weights c* are evaluated ('min' = min of average curve)")
    p.add_argument("--arw-range", type=float, nargs=2, default=(0.1, 2.0))
    p.add_argument("--bi-max-frac", type=float, default=0.05,
                   help="search the ADEV minimum only for tau <= frac * duration (default T/20)")
    p.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "out", "allan"))
    a = p.parse_args(argv)
    log = imu_log.load(a.log, a.mount)
    print(log.summary(), "\n")
    res = analyze(log, a.sensor, a.frame, a.temp_comp, a.weight_tau, tuple(a.arw_range),
                  bi_max_frac=a.bi_max_frac)
    save(res, a.out)
    plot(res, a.out)
    pd.set_option("display.width", 200)
    pd.set_option("display.float_format", lambda v: f"{v:.4g}")
    print(res["summary"].to_string(index=False))
    print("\nAllan correlation / weights:")
    print(rho_table(res).to_string(index=False))
    print(f"\nwrote {a.out}_summary.csv, _rho.csv, _curves.csv, _adev.png, _rho.png")
    return res


if __name__ == "__main__":
    main()

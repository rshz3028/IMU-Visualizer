"""
make_synthetic.py - synthetic dual-IMU raw logs with known error parameters.

Gyro model per IMU i, in its own SENSOR frame (rad/s):
    w_s,i = R_i^T w_body + b0_i + b_shared
            + n_white,i + b_BI,i + b_RRW,i
            + tc_i (T_i - Tref) + tc2 (T_i - Tref)^2
  * n_white : white noise, Allan sigma(1 s) = ARW       (default 0.30 deg/sqrt(h))
  * b_BI    : flicker (1/f, FFT-shaped; Allan floor 0.664 B) or first-order
              Gauss-Markov (peak ~0.664 B), B default 5 deg/h
  * b_RRW   : random walk, sigma(tau) = K sqrt(tau/3)     (default 2 deg/h/sqrt(h))
  * correlation between IMUs: component_2 = rho c_1 + sqrt(1-rho^2) c_indep, applied
    same-sign in the SENSOR frame (common-mode electrical / board effect), separately
    for white (--rho-white), BI (--rho-bi) and RRW (--rho-rrw)
  * tc_i = tc_shared + tc_indep,i : shared (same sign in sensor frame) + individual
    temperature coefficient; temperature is a slow warm-up ramp
        T_board = T0 + dT (1 - exp(-t / tau_th)),  T_i = T_board + offset_i + noise
    quantised to 1/256 degC like the ISM330DHCX.
Accel: a_s,i = (I + diag(s_i)) R_i^T f_body + b_i + tcA (T_i - Tref) + white + flicker.
R_1 = I, R_2 = mount rotation (v_body = R_2 v_sensor2).

Scenarios
  static : board at rest, +Z up (default)
  six    : six-position sequence (+Z,-Z,+X,-X,+Y,-Y up) with rotations in between

Outputs: CSV in the raw_log.csv format, optional binary dump (with optional
corruption to exercise resync), and <out>.truth.json with every injected value.

    python make_synthetic.py --out out/synth/raw_log.csv --hours 2 --fs 200
    python make_synthetic.py --scenario six --out out/synth/six.csv
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import imu_log  # noqa: E402

D2R = np.pi / 180.0
G0 = 9.80665
UG = G0 * 1e-6


# ----------------------------------------------------------------------------
# noise processes (unit-free: return arrays in the same unit as the parameter)
# ----------------------------------------------------------------------------
def white(n, k, N, fs, rng):
    """White noise whose Allan deviation at tau=1 s is N (N in unit*sqrt(s))."""
    return rng.standard_normal((n, k)) * N * np.sqrt(fs)


def flicker(n, k, B, fs, rng):
    """1/f noise, two-sided PSD S(f) = B^2/(2 pi f) (IEEE 952): Allan floor = 0.664 B.

    The FFT shaping uses the one-sided PSD 2 S(f) = B^2/(pi f).
    """
    L = 2 * n
    f = np.fft.rfftfreq(L, 1 / fs)
    H = np.zeros_like(f)
    H[1:] = np.sqrt(B * B / (np.pi * f[1:]) * fs / 2.0)
    out = np.empty((n, k))
    for j in range(k):
        w = np.fft.rfft(rng.standard_normal(L))
        out[:, j] = np.fft.irfft(w * H, L)[:n]
    return out - out.mean(axis=0)


def gauss_markov(n, k, B, fs, rng, Tc=300.0):
    """First-order GM tuned so that its Allan peak (~0.6185 sigma) = 0.664 B."""
    from scipy.signal import lfilter
    sig = 0.664 * B / 0.6185
    phi = np.exp(-1.0 / (fs * Tc))
    x0 = rng.standard_normal(k) * sig
    drive = rng.standard_normal((n, k)) * sig * np.sqrt(1 - phi * phi)
    out = np.empty((n, k))
    for j in range(k):
        out[:, j], _ = lfilter([1.0], [1.0, -phi], drive[:, j], zi=[phi * x0[j]])
    return out


def rrw(n, k, K, fs, rng):
    """Random walk, Allan sigma(tau) = K sqrt(tau/3) (K in unit/sqrt(s))."""
    return np.cumsum(rng.standard_normal((n, k)) * K / np.sqrt(fs), axis=0)


def correlated_pair(gen, rho):
    """Return (c1, c2) with corr(c1, c2) = rho given a generator of iid draws."""
    a, b = gen(), gen()
    return a, rho * a + np.sqrt(max(0.0, 1 - rho * rho)) * b


# ----------------------------------------------------------------------------
# kinematics
# ----------------------------------------------------------------------------
def _rodrigues(k, th):
    k = k / np.linalg.norm(k)
    Kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * Kx + (1 - np.cos(th)) * Kx @ Kx


SIX_ORDER = [("+Z", [0, 0, 1]), ("-Z", [0, 0, -1]), ("+X", [1, 0, 0]),
             ("-X", [-1, 0, 0]), ("+Y", [0, 1, 0]), ("-Y", [0, -1, 0])]


def six_position_trajectory(fs, dwell, transition):
    """Return (f_body N x3 [m/s^2], w_body N x3 [rad/s], labels list of (label,t0,t1))."""
    f_parts, w_parts, labels = [], [], []
    t = 0.0
    up_prev = None
    for lab, up in SIX_ORDER:
        up = np.array(up, float)
        if up_prev is not None:
            nT = int(transition * fs)
            ax = np.cross(up_prev, up)
            if np.linalg.norm(ax) < 1e-9:  # antiparallel: rotate about a perpendicular axis
                ax = np.cross(up_prev, [1, 0, 0] if abs(up_prev[0]) < 0.9 else [0, 1, 0])
            ax = ax / np.linalg.norm(ax)
            ang = np.arccos(np.clip(up_prev @ up, -1, 1))
            s = (1 - np.cos(np.pi * (np.arange(nT) + 0.5) / nT)) / 2  # smooth 0->1
            sdot = np.pi / (2 * transition) * np.sin(np.pi * (np.arange(nT) + 0.5) / nT)
            f_parts.append(np.array([_rodrigues(ax, ang * si) @ up_prev for si in s]) * G0)
            # body sees the gravity vector rotate by +theta about ax -> body rotates by -theta
            w_parts.append(-np.outer(ang * sdot, ax))
            t += nT / fs
        nD = int(dwell * fs)
        f_parts.append(np.tile(up * G0, (nD, 1)))
        w_parts.append(np.zeros((nD, 3)))
        labels.append((lab, t, t + nD / fs))
        t += nD / fs
        up_prev = up
    return np.vstack(f_parts), np.vstack(w_parts), labels


# ----------------------------------------------------------------------------
# generator
# ----------------------------------------------------------------------------
def _vec(s, n=3):
    if isinstance(s, (list, tuple, np.ndarray)):
        return np.asarray(s, float)
    v = np.array([float(x) for x in str(s).split(",")])
    return np.repeat(v, n) if v.size == 1 else v


def generate(hours=2.0, fs=200.0, mount="z180", scenario="static", seed=1,
             arw=0.30, bi=5.0, bi_model="flicker", gm_tc=300.0, rrw_k=2.0,
             rho_white=0.0, rho_bi=0.5, rho_rrw=0.5,
             bias_std_dps=0.2, shared_bias_dph=(0, 0, 0),
             tc_shared=(2.0, -1.5, 3.0), tc_indep_std=1.0, tc2=0.02,
             T0=25.0, dT=15.0, tau_th=1800.0, Tref=25.0,
             acc_white_ug=60.0, acc_bi_ug=30.0, acc_bias_mg_std=20.0,
             acc_scale_std=0.003, acc_tc_mg=(0.1, 0.1, -0.15),
             lat=None, drop_frac=0.0, t0_us=2 ** 32 - 30_000_000,
             six_dwell=60.0, six_transition=8.0):
    """Return (DataFrame in raw_log.csv format, truth dict)."""
    rng = np.random.default_rng(seed)
    R2 = imu_log.mount_matrix(mount)
    Rs = [np.eye(3), R2]

    if scenario == "six":
        f_body, w_body, labels = six_position_trajectory(fs, six_dwell, six_transition)
        n = len(f_body)
    else:
        n = int(round(hours * 3600 * fs))
        f_body = np.tile([0.0, 0.0, G0], (n, 1))
        w_body = np.zeros((n, 3))
        labels = [("+Z", 0.0, n / fs)]
    if lat is not None:  # Earth rate, body x=north, z=up (static only)
        we = 7.2921159e-5
        w_body = w_body + np.array([we * np.cos(lat * D2R), 0, we * np.sin(lat * D2R)])
    t = np.arange(n) / fs

    # temperatures
    Tb = T0 + dT * (1 - np.exp(-t / tau_th))
    slow = flicker(n, 1, 0.01, fs, rng)[:, 0]  # small shared board-temperature wander
    temps = []
    for off in (0.4, -0.3):
        Ti = Tb + off + slow + 0.02 * rng.standard_normal(n)
        temps.append(np.round(Ti * 256) / 256)

    # gyro stochastic terms in rad/s
    N = arw * D2R / 60.0
    B = bi * D2R / 3600.0
    K = rrw_k * D2R / 3600.0 / 60.0
    wh1, wh2 = correlated_pair(lambda: white(n, 3, N, fs, rng), rho_white)
    if bi_model == "gm":
        bg = lambda: gauss_markov(n, 3, B, fs, rng, gm_tc)  # noqa: E731
    else:
        bg = lambda: flicker(n, 3, B, fs, rng)  # noqa: E731
    bi1, bi2 = correlated_pair(bg, rho_bi)
    rw1, rw2 = correlated_pair(lambda: rrw(n, 3, K, fs, rng), rho_rrw)

    b0 = [rng.standard_normal(3) * bias_std_dps * D2R for _ in range(2)]
    shared_b = _vec(shared_bias_dph) * D2R / 3600.0
    tcs = _vec(tc_shared)
    tc = [tcs + rng.standard_normal(3) * tc_indep_std for _ in range(2)]   # deg/h/degC
    tc2v = _vec(tc2)

    # accel terms
    ab = [rng.standard_normal(3) * acc_bias_mg_std * 1e-3 * G0 for _ in range(2)]
    asc = [rng.standard_normal(3) * acc_scale_std for _ in range(2)]
    atc = _vec(acc_tc_mg) * 1e-3 * G0  # m/s^2 per degC, shared
    mag_body = np.array([20.0, 0.0, -40.0])
    hard_iron = [rng.standard_normal(3) * 5 for _ in range(2)]

    cols = {}
    stoch = [(wh1, bi1, rw1), (wh2, bi2, rw2)]
    for i in range(2):
        R = Rs[i]
        dTi = (temps[i] - Tref)[:, None]
        w_s = w_body @ R  # rows: R^T w
        g = (w_s + b0[i] + shared_b + sum(stoch[i])
             + (tc[i] * dTi + tc2v * dTi ** 2) * D2R / 3600.0)
        f_s = f_body @ R
        a = (f_s * (1 + asc[i]) + ab[i] + atc * dTi
             + white(n, 3, acc_white_ug * UG, fs, rng) + flicker(n, 3, acc_bi_ug * UG, fs, rng))
        m = np.tile(mag_body @ R, (n, 1)) + hard_iron[i] + 0.3 * rng.standard_normal((n, 3))
        k = i + 1
        for j, ax in enumerate("xyz"):
            cols[f"a{ax}{k}"] = a[:, j]
        for j, ax in enumerate("xyz"):
            cols[f"g{ax}{k}"] = g[:, j]
        for j, ax in enumerate("xyz"):
            cols[f"m{ax}{k}"] = m[:, j]
        cols[f"temp{k}"] = temps[i]

    seq = np.arange(n)
    t_us = (t0_us + np.round(t * 1e6) + rng.integers(-10, 11, n)).astype(np.int64)
    df = pd.DataFrame({"seq": seq % 65536, "t_us": t_us % 2 ** 32, **cols})[imu_log.CSV_COLUMNS]
    dropped = 0
    if drop_frac > 0:
        keep = rng.random(n) >= drop_frac
        keep[0] = keep[-1] = True
        dropped = int((~keep).sum())
        df = df[keep].reset_index(drop=True)

    truth = dict(
        scenario=scenario, hours=n / fs / 3600, fs=fs, mount=mount, R_mount=R2.tolist(), seed=seed,
        gyro=dict(arw_deg_sqrth=arw, bi_deg_h=bi, bi_model=bi_model, rrw_deg_h_sqrth=rrw_k,
                  rho_white=rho_white, rho_bi=rho_bi, rho_rrw=rho_rrw,
                  correlation_frame="sensor",
                  bias_deg_h=[list(b0[i] / D2R * 3600 + _vec(shared_bias_dph)) for i in range(2)],
                  shared_bias_deg_h=list(_vec(shared_bias_dph)),
                  tempco_deg_h_per_C=[list(tc[i]) for i in range(2)],
                  tempco2_deg_h_per_C2=list(tc2v), Tref=Tref),
        accel=dict(white_ug_sqrts=acc_white_ug, bi_ug=acc_bi_ug,
                   bias_m_s2=[list(ab[i]) for i in range(2)],
                   scale_err=[list(asc[i]) for i in range(2)],
                   tempco_m_s2_per_C=list(atc)),
        temperature=dict(T0=T0, dT=dT, tau_th_s=tau_th, offsets=[0.4, -0.3]),
        segments=[dict(label=l, t_start=a_, t_end=b_) for l, a_, b_ in labels],
        dropped_samples=dropped, lat=lat, frames="all sensor-frame values")
    return df, truth


def write_csv(df: pd.DataFrame, path):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(path, index=False, float_format="%.8g")


def write_binary(df: pd.DataFrame, path, corrupt=0, garbage=0, seed=7):
    rng = np.random.default_rng(seed)
    imu1 = df[imu_log.CSV_COLUMNS[2:12]].to_numpy(np.float32)
    imu2 = df[imu_log.CSV_COLUMNS[12:22]].to_numpy(np.float32)
    raw = bytearray(imu_log.encode_binary(df.seq.to_numpy(), df.t_us.to_numpy(), imu1, imu2))
    n_pk = len(df)
    # flip random bytes in random packets (each -> CRC failure -> packet lost)
    for p in rng.choice(n_pk, size=min(corrupt, n_pk), replace=False):
        off = int(p) * imu_log.PACKET_LEN + int(rng.integers(2, 90))
        raw[off] ^= 0xFF
    # insert garbage runs (incl. fake sync bytes) at random packet boundaries
    if garbage:
        pos = sorted(rng.choice(n_pk, size=min(garbage, n_pk), replace=False), reverse=True)
        for p in pos:
            junk = bytes(rng.integers(0, 256, int(rng.integers(1, 40))).astype(np.uint8)) + b"\xAA\x55\x00"
            raw[int(p) * imu_log.PACKET_LEN:int(p) * imu_log.PACKET_LEN] = junk
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_bytes(bytes(raw))


def expected_rho(tau, truth, axis, frame="body"):
    """Analytical Allan correlation for the injected stochastic model (no thermal)."""
    g = truth["gyro"]
    tau = np.asarray(tau, float)
    a_w = g["arw_deg_sqrth"] ** 2 * 3600 / tau            # (deg/h)^2
    a_b = (0.664 * g["bi_deg_h"]) ** 2 * np.ones_like(tau)
    a_k = g["rrw_deg_h_sqrth"] ** 2 / 3600 * tau / 3
    rho = (g["rho_white"] * a_w + g["rho_bi"] * a_b + g["rho_rrw"] * a_k) / (a_w + a_b + a_k)
    if frame == "body":
        j = "xyz".index(axis)
        rho = rho * np.array(truth["R_mount"])[j, j]
    return rho


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                 "out", "synth", "raw_log.csv"))
    p.add_argument("--bin", help="also write a binary serial dump here")
    p.add_argument("--corrupt", type=int, default=0, help="bytes to corrupt in the binary dump")
    p.add_argument("--garbage", type=int, default=0, help="garbage runs to insert in the binary dump")
    p.add_argument("--no-csv", action="store_true")
    p.add_argument("--scenario", choices=["static", "six"], default="static")
    p.add_argument("--hours", type=float, default=2.0)
    p.add_argument("--fs", type=float, default=200.0)
    imu_log.add_mount_args(p)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--arw", type=float, default=0.30, help="deg/sqrt(h)")
    p.add_argument("--bi", type=float, default=5.0, help="deg/h")
    p.add_argument("--bi-model", choices=["flicker", "gm"], default="flicker")
    p.add_argument("--gm-tc", type=float, default=300.0, help="GM correlation time [s]")
    p.add_argument("--rrw", type=float, default=2.0, help="deg/h/sqrt(h)")
    p.add_argument("--rho-white", type=float, default=0.0)
    p.add_argument("--rho-bi", type=float, default=0.5)
    p.add_argument("--rho-rrw", type=float, default=0.5)
    p.add_argument("--bias-std", type=float, default=0.2, help="turn-on bias std [deg/s]")
    p.add_argument("--shared-bias", default="0", help="deg/h, same sign in sensor frame (1 or 3 values)")
    p.add_argument("--tc-shared", default="2,-1.5,3", help="deg/h/degC shared tempco (1 or 3 values)")
    p.add_argument("--tc-indep", type=float, default=1.0, help="std of individual tempco [deg/h/degC]")
    p.add_argument("--tc2", default="0.02", help="quadratic tempco deg/h/degC^2")
    p.add_argument("--dT", type=float, default=15.0, help="warm-up amplitude [degC]")
    p.add_argument("--tau-th", type=float, default=1800.0, help="warm-up time constant [s]")
    p.add_argument("--lat", type=float, default=None, help="add Earth rate for this latitude [deg]")
    p.add_argument("--drop-frac", type=float, default=0.0, help="random packet loss fraction")
    p.add_argument("--six-dwell", type=float, default=60.0)
    a = p.parse_args(argv)
    df, truth = generate(hours=a.hours, fs=a.fs, mount=a.mount, scenario=a.scenario, seed=a.seed,
                         arw=a.arw, bi=a.bi, bi_model=a.bi_model, gm_tc=a.gm_tc, rrw_k=a.rrw,
                         rho_white=a.rho_white, rho_bi=a.rho_bi, rho_rrw=a.rho_rrw,
                         bias_std_dps=a.bias_std, shared_bias_dph=a.shared_bias,
                         tc_shared=a.tc_shared, tc_indep_std=a.tc_indep, tc2=a.tc2,
                         dT=a.dT, tau_th=a.tau_th, lat=a.lat, drop_frac=a.drop_frac,
                         six_dwell=a.six_dwell)
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    if not a.no_csv:
        write_csv(df, out)
        print(f"wrote {out} ({len(df)} rows)")
    if a.bin:
        write_binary(df, a.bin, a.corrupt, a.garbage)
        print(f"wrote {a.bin}")
    tpath = out.with_suffix(".truth.json")
    tpath.write_text(json.dumps(truth, indent=2))
    print(f"wrote {tpath}")
    return df, truth


if __name__ == "__main__":
    main()

"""
run_validation.py - end-to-end test of every tool on synthetic data with known truth.

Runs (outputs in tools/out/):
  A  2 h @ 200 Hz static log, default model (ARW 0.30, BI 5 flicker, RRW 2,
     rho_white 0, rho_bi = rho_rrw = 0.5, thermal ramp, 0.01 % packet loss,
     t_us wrap after 30 s) -> CSV + binary (corrupted) -> allan.py (raw and
     temperature-compensated), thermal.py, loader cross-check
  B  1 h @ 200 Hz, all stochastic terms correlated rho = 0.4 (incl. white),
     strong RRW (40 deg/h/sqrt h), no thermal, BINARY input only -> allan.py
  C  six-position sequence -> six_position.py (auto + user-labelled segments)
  D  mount_demo.py (z180 vs x180, shared same-sign bias)

    python run_validation.py [--quick]     (--quick: 0.5 h logs for a fast smoke test)
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import imu_log  # noqa: E402
import make_synthetic as ms  # noqa: E402

OUT = os.path.join(HERE, "out")
SYN = os.path.join(OUT, "synth")
PY = sys.executable
REPORT = []
CHECKS = []


def say(s=""):
    print(s, flush=True)
    REPORT.append(str(s))


def check(name, value, target, tol, rel=False):
    err = abs(value - target) / abs(target) if rel else abs(value - target)
    ok = bool(err <= tol)
    CHECKS.append((name, ok))
    say(f"  [{'PASS' if ok else 'FAIL'}] {name}: got {value:.4g}, expected {target:.4g} "
        f"(tol {'%g%%' % (100*tol) if rel else tol})")
    return ok


def run_cli(script, *args):
    cmd = [PY, os.path.join(HERE, script), *map(str, args)]
    say(f"$ {' '.join(os.path.basename(c) if i < 2 else c for i, c in enumerate(cmd))}")
    t0 = time.time()
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        say(r.stdout); say(r.stderr)
        raise SystemExit(f"{script} failed")
    say(f"  ({time.time()-t0:.1f} s)")
    return r.stdout


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--quick", action="store_true")
    a = p.parse_args()
    hA = 0.5 if a.quick else 2.0
    hB = 0.5 if a.quick else 1.0
    os.makedirs(SYN, exist_ok=True)

    # ------------------------------------------------------------------ A
    say("=" * 78); say(f"A) static {hA} h @ 200 Hz, CSV + binary"); say("=" * 78)
    csvA = os.path.join(SYN, "raw_log.csv"); binA = os.path.join(SYN, "raw_log.bin")
    if not (os.path.exists(csvA) and os.path.exists(csvA.replace(".csv", ".truth.json"))
            and json.load(open(csvA.replace(".csv", ".truth.json")))["hours"] > hA - 0.01
            and json.load(open(csvA.replace(".csv", ".truth.json")))["hours"] < hA + 0.01):
        run_cli("make_synthetic.py", "--out", csvA, "--hours", hA, "--drop-frac", 1e-4,
                "--bin", binA, "--corrupt", 200, "--garbage", 50)
    truthA = json.load(open(csvA.replace(".csv", ".truth.json")))

    # loader cross-check
    say("\nloader cross-check (CSV vs corrupted binary):")
    lc = imu_log.load(csvA); lb = imu_log.load(binA)
    say(lc.summary()); say("--- binary:"); say(lb.summary())
    common, ic, ib = np.intersect1d(lc.seq, lb.seq, return_indices=True)
    maxdiff = np.max(np.abs(lc.gyr2[ic] - lb.gyr2[ib]))
    check("binary packets lost to corruption ~= corrupt count (200)",
          lc.n - lb.n, 200, 30)
    check("CSV/binary gyro max |diff| on common samples [rad/s] (float32 rounding)", maxdiff, 0, 1e-6)
    check("CSV dropped samples detected", int(lc.gaps.missing.sum()), truthA["dropped_samples"], 0)
    check("t_us wrap unwrapped (duration)", lc.duration, truthA["hours"] * 3600, 0.05)
    del lb

    for tc in (0, 2):
        prefix = os.path.join(OUT, f"allan_A_tc{tc}")
        run_cli("allan.py", csvA, "--temp-comp", tc, "--out", prefix)
    sA0 = pd.read_csv(os.path.join(OUT, "allan_A_tc0_summary.csv"))
    sA2 = pd.read_csv(os.path.join(OUT, "allan_A_tc2_summary.csv"))
    cA2 = pd.read_csv(os.path.join(OUT, "allan_A_tc2_curves.csv"))
    cA0 = pd.read_csv(os.path.join(OUT, "allan_A_tc0_curves.csv"))
    ARW, BI, RRW = "ARW[deg/sqrt(h)]", "BI[deg/h]", "RRW[deg/h/sqrt(h)]"
    pd.set_option("display.width", 200); pd.set_option("display.float_format", lambda v: f"{v:.4g}")
    say("\nAllan summary, temp-comp=2 (quadratic in T removed):")
    say(sA2.to_string(index=False))
    say("\nAllan summary, no temperature compensation:")
    say(sA0[["axis", "series", ARW, BI, "tau_BI[s]"]].to_string(index=False))
    say("\nchecks (truth: ARW 0.30 single, 0.30/sqrt2 = 0.212 average; BI 5 deg/h):")
    single = sA2[sA2.series.isin(["IMU1", "IMU2"])]
    avg = sA2[sA2.series == "avg50"]
    for _, r in single.iterrows():
        check(f"ARW {r.series} {r.axis}", r[ARW], 0.30, 0.05, rel=True)
    for _, r in avg.iterrows():
        check(f"ARW avg50 {r.axis}", r[ARW], 0.30 / np.sqrt(2), 0.05, rel=True)
    check("mean ARW single (6 channels)", single[ARW].mean(), 0.30, 0.03, rel=True)
    check("mean ARW avg50 (3 axes)", avg[ARW].mean(), 0.2121, 0.03, rel=True)
    check("mean BI single, temp-comp (6 channels)", single[BI].mean(), 5.0, 0.25, rel=True)
    say(f"  BI single per channel (tc2): {np.round(single[BI].values, 2)}")
    say(f"  BI single per channel (raw): {np.round(sA0[sA0.series.isin(['IMU1','IMU2'])][BI].values, 2)}")
    say(f"  BI avg50 per axis (tc2): {np.round(avg[BI].values, 2)}   "
        f"weighted: {np.round(sA2[sA2.series=='weighted'][BI].values, 2)}")

    say("\nAllan correlation rho(tau), body frame, vs analytic expectation of injected model")
    say("(rho_bi = rho_rrw = 0.5 same-sign in sensor frame -> sign flips on x,y under z180):")
    rows = []
    for ax in "xyz":
        for lab, cur in (("tc2", cA2), ("raw", cA0)):
            c = cur[cur.axis == ax]
            for t in (1, 10, 100, 300, 900):
                j = int(np.argmin(np.abs(np.log(c.tau.values / t))))
                rows.append(dict(axis=ax, comp=lab, tau=c.tau.values[j], rho=c.rho.values[j],
                                 expected=float(ms.expected_rho(c.tau.values[j], truthA, ax))))
    rt = pd.DataFrame(rows)
    say(rt.pivot_table(index=["axis", "tau"], columns="comp", values=["rho", "expected"]).to_string())
    for ax in "xyz":
        r = rt[(rt.axis == ax) & (rt.comp == "tc2")]
        for t in (10, 100):
            rr = r.iloc[int(np.argmin(np.abs(np.log(r.tau.values / t))))]
            check(f"rho {ax} @ {rr.tau:.0f}s (tc2) vs expected", rr.rho, rr.expected, 0.15)
    # sensor-frame rho magnitude in BI region
    say("\nRegion where BI dominates: compute rho in SENSOR frame, tau 100..600 s:")
    run_cli("allan.py", csvA, "--temp-comp", 2, "--frame", "sensor", "--out", os.path.join(OUT, "allan_A_tc2_sensor"))
    cs = pd.read_csv(os.path.join(OUT, "allan_A_tc2_sensor_curves.csv"))
    for ax in "xyz":
        c = cs[(cs.axis == ax) & (cs.tau >= 100) & (cs.tau <= 600)]
        exp = np.mean(ms.expected_rho(c.tau.values, truthA, ax, frame="sensor"))
        check(f"mean rho sensor-frame {ax} (100-600 s)", c.rho.mean(), exp, 0.15)

    run_cli("thermal.py", csvA, "--out", os.path.join(OUT, "thermal_A"))
    fits = pd.read_csv(os.path.join(OUT, "thermal_A_fits.csv"))
    comp = pd.read_csv(os.path.join(OUT, "thermal_A_comparison.csv"))
    say("\nthermal.py vs injected tempco (sensor frame). The truth is tc*(T-25) + tc2*(T-25)^2;")
    say("the expected LINEAR-fit slope is the least-squares projection of that curve onto the")
    say("observed temperatures: tc + tc2*cov(dT,dT^2)/var(dT).")
    tcs = truthA["gyro"]["tempco_deg_h_per_C"]; tq = truthA["gyro"]["tempco2_deg_h_per_C2"]
    lA = imu_log.load(csvA)
    for imu in (1, 2):
        Tt = (lA.temp1 if imu == 1 else lA.temp2)[::200] - 25.0
        k = np.cov(Tt, Tt ** 2)[0, 1] / np.var(Tt, ddof=1)
        for j, ax in enumerate("xyz"):
            r = fits[(fits.sensor == "gyro") & (fits.imu == imu) & (fits.axis == ax)].iloc[0]
            exp_lin = tcs[imu - 1][j] + tq[j] * k
            say(f"  gyro IMU{imu} {ax}: lin {r.tempco_lin:+.3f} (expected {exp_lin:+.3f}) | "
                f"quad@25C {r.tempco_quad_at_Tref:+.3f} (truth {tcs[imu-1][j]:+.3f}) | "
                f"quad@{r.Tmid:.1f}C {r.tempco_quad_at_Tmid:+.3f} "
                f"(truth {tcs[imu-1][j] + 2*tq[j]*(r.Tmid-25):+.3f}) | q2 {r.q2:+.4f} (truth {tq[j]:+.4f})")
            check(f"gyro linear tempco IMU{imu} {ax} [deg/h/C]", r.tempco_lin, exp_lin, 0.35)
    del lA
    atc = np.array(truthA["accel"]["tempco_m_s2_per_C"]) / 9.80665 * 1e3
    for imu in (1, 2):
        for j, ax in enumerate("xyz"):
            r = fits[(fits.sensor == "accel") & (fits.imu == imu) & (fits.axis == ax)].iloc[0]
            check(f"accel tempco IMU{imu} {ax} [mg/C]", r.tempco_lin, atc[j], 0.01)
    say("\nIMU1 vs IMU2 thermal comparison (sensor frame):")
    say(comp.to_string(index=False))
    tc1 = np.array(tcs[0]); tc2v = np.array(tcs[1])
    g = comp[comp.sensor == "gyro"]
    say(f"  injected tempcos IMU1 {np.round(tc1,3)}  IMU2 {np.round(tc2v,3)}; "
        f"r(binned) per axis = {np.round(g.r_binned.values,3)}")

    # ------------------------------------------------------------------ B
    say("\n" + "=" * 78); say(f"B) {hB} h, all terms correlated rho=0.4, RRW 40, binary input"); say("=" * 78)
    binB = os.path.join(SYN, "corr04.bin")
    run_cli("make_synthetic.py", "--out", os.path.join(SYN, "corr04.csv"), "--no-csv", "--bin", binB,
            "--corrupt", 50, "--garbage", 20, "--hours", hB, "--rho-white", 0.4, "--rho-bi", 0.4,
            "--rho-rrw", 0.4, "--rrw", 40, "--tc-shared", 0, "--tc-indep", 0, "--tc2", 0, "--seed", 5)
    truthB = json.load(open(os.path.join(SYN, "corr04.truth.json")))
    run_cli("allan.py", binB, "--out", os.path.join(OUT, "allan_B"))
    run_cli("allan.py", binB, "--frame", "sensor", "--out", os.path.join(OUT, "allan_B_sensor"))
    sB = pd.read_csv(os.path.join(OUT, "allan_B_summary.csv"))
    rB = pd.read_csv(os.path.join(OUT, "allan_B_sensor_rho.csv"))
    rBb = pd.read_csv(os.path.join(OUT, "allan_B_rho.csv"))
    say(sB.to_string(index=False)); say("\nrho (sensor frame):"); say(rB.to_string(index=False))
    say("rho (body frame):"); say(rBb.to_string(index=False))
    for _, r in rB.iterrows():
        check(f"B rho median sensor {r.axis} (injected 0.4)", r.rho_median, 0.4, 0.05)
    for _, r in rBb.iterrows():
        exp = 0.4 * np.array(truthB["R_mount"])["xyz".index(r.axis)]["xyz".index(r.axis)]
        check(f"B rho median body {r.axis}", r.rho_median, exp, 0.05)
    sgl = sB[sB.series.isin(["IMU1", "IMU2"])]
    check("B mean RRW single (injected 40)", sgl[RRW].mean(), 40.0, 0.3, rel=True)
    for _, r in sB[sB.series == "avg50"].iterrows():
        jj = "xyz".index(r.axis)
        rho_b = 0.4 * truthB["R_mount"][jj][jj]
        check(f"B ARW avg50 {r.axis}: 0.30*sqrt((1+rho)/2) with rho_body={rho_b:+.1f}",
              r[ARW], 0.30 * np.sqrt((1 + rho_b) / 2), 0.05, rel=True)

    # ------------------------------------------------------------------ C
    say("\n" + "=" * 78); say("C) six-position calibration"); say("=" * 78)
    csvC = os.path.join(SYN, "six.csv")
    run_cli("make_synthetic.py", "--scenario", "six", "--out", csvC, "--seed", 3)
    truthC = json.load(open(csvC.replace(".csv", ".truth.json")))
    segf = os.path.join(SYN, "six_segments.csv")
    sg = pd.DataFrame(truthC["segments"]); sg["t_start"] += 2; sg["t_end"] -= 2
    sg.to_csv(segf, index=False)
    for mode, extra in (("auto", []), ("labelled", ["--segments", segf])):
        say(run_cli("six_position.py", csvC, "--out", os.path.join(OUT, f"six_{mode}"), *extra))
        cal = pd.read_csv(os.path.join(OUT, f"six_{mode}_calibration.csv"))
        seg = pd.read_csv(os.path.join(OUT, f"six_{mode}_segments.csv"))
        for _, r in cal.iterrows():
            j = "xyz".index(r.axis); i = r.imu - 1
            Tm = (r.temp_up + r.temp_down) / 2
            bt = truthC["accel"]["bias_m_s2"][i][j] + truthC["accel"]["tempco_m_s2_per_C"][j] * (Tm - 25)
            check(f"{mode} IMU{r.imu} {r.axis} accel bias [mg]", r.bias_mg, bt / 9.80665e-3, 0.3)
            check(f"{mode} IMU{r.imu} {r.axis} scale err [ppm]", r.scale_err_ppm,
                  truthC["accel"]["scale_err"][i][j] * 1e6, 300)
        for imu in (1, 2):
            gb = seg[seg.imu == imu][["gx_dph", "gy_dph", "gz_dph"]].mean().values
            say(f"  {mode} IMU{imu} gyro bias mean over segments [deg/h] {np.round(gb, 1)}; "
                f"truth at Tref {np.round(truthC['gyro']['bias_deg_h'][imu-1], 1)} (+ tempco drift)")

    # ------------------------------------------------------------------ D
    say("\n" + "=" * 78); say("D) mounting demonstration"); say("=" * 78)
    say(run_cli("mount_demo.py", "--out", os.path.join(OUT, "mount_demo")))
    md = pd.read_csv(os.path.join(OUT, "mount_demo.csv"))
    for _, r in md.iterrows():
        check(f"mount {r.mount} avg bias {r.axis} [deg/h]", r.avg_bias_dph, r.expected_avg_bias, 3.0)
        check(f"mount {r.mount} avg tempco {r.axis} [deg/h/C]", r.avg_tempco_dph_per_C, r.expected_avg_tempco, 0.6)

    say("\n" + "=" * 78)
    npass = sum(ok for _, ok in CHECKS)
    say(f"SUMMARY: {npass}/{len(CHECKS)} checks passed")
    for n, ok in CHECKS:
        if not ok:
            say(f"  FAILED: {n}")
    with open(os.path.join(OUT, "validation_report.txt"), "w") as f:
        f.write("\n".join(REPORT))
    print(f"report written to {os.path.join(OUT, 'validation_report.txt')}")


if __name__ == "__main__":
    main()

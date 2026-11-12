# Dual-IMU analysis tools

Python tools for the two-ISM330DHCX board (IMU2 currently mounted rotated 180° about Z
relative to IMU1). Requires only numpy, scipy, matplotlib and pandas (tested with
Python 3.13, numpy 2.4, scipy 1.16, matplotlib 3.10, pandas 2.3).

All scripts take `-h`. Outputs go to `tools/out/` by default (`--out <prefix>` overrides).

## Frames and conventions

* **Body frame = IMU1 sensor frame.** IMU2 is rotated into it with `v_body = R v_sensor2`.
* `--mount z180` (default): R = diag(-1,-1,+1). `--mount x180`: R = diag(+1,-1,-1).
  Also `y180`, `none`, or a chain such as `z90,x180`.
* Accelerometers read **+g on the axis pointing up** when the board is at rest.
* Units on input: accel m/s², gyro rad/s, mag µT, temp °C, `t_us` = ESP32 `micros()`.
* Reported units: gyro ARW °/√h, bias instability °/h, RRW °/h/√h, tempco °/h/°C;
  accel noise m/s/√h, BI µg, tempco mg/°C.

## Scripts

### `imu_log.py` — loaders
```
python imu_log.py raw_log.csv [--mount z180] [--export tidy.csv]
python imu_log.py dump.bin
```
* CSV `raw_log.csv` (header `seq,t_us,ax1,...,temp2`) or binary serial dump
  (90-byte packets: `AA 55`, u16 seq, u32 t_us, 2×10 float32, u16 CRC-16/CCITT-FALSE over
  bytes 2..87). Extension `.csv`/`.txt` → CSV, anything else → binary.
* Binary parser is vectorised; it resynchronises after bad CRC / garbage bytes and
  reports good packets, bad-CRC candidates and skipped bytes.
* Unwraps `t_us` (uint32) and `seq` (uint16), uses timestamps to disambiguate outages
  longer than 65536 samples, drops duplicates, and lists seq gaps (`log.gaps`).
* In Python: `log = imu_log.load(path, mount="z180")` → `IMULog` with `t, seq, fs,
  acc1, gyr1, mag1, temp1, acc2, gyr2, mag2, temp2` (N×3, body frame),
  `acc2_sensor, gyr2_sensor, mag2_sensor`, `R_mount`, `gaps`, `meta`, `to_dataframe()`.
* `imu_log.uniform_series(log, x)` fills dropped samples by linear interpolation
  (used by the Allan tool).

### `allan.py` — Allan deviation / covariance
```
python allan.py raw_log.csv [--sensor gyro|accel] [--frame body|sensor]
                [--temp-comp 0|1|2] [--weight-tau min|<s>] [--arw-range 0.1 2]
```
Per axis: overlapping ADEV of IMU1, IMU2, 50/50 average, fixed-weight optimal virtual
gyro, and the per-τ optimal envelope. Allan covariance (Vaccaro & Zaki 2017)
`cov = (AVAR(x+y) − AVAR(x−y))/4`, correlation `ρ(τ) = cov/√(AVAR1·AVAR2)`, optimal
weights `c* = Q⁻¹1 / (1ᵀQ⁻¹1)`. Fixed weights are evaluated at the minimum of the
averaged curve (default) or at `--weight-tau`.
Extraction: ARW = slope −½ fit read at τ = 1 s; BI = min/0.664 (searched up to T/8);
RRW = slope +½ fit on the rising branch read at τ = 3 s (NaN if no such branch).
`--temp-comp 2` removes a quadratic fit vs each IMU's own temperature first.
Outputs: `allan_summary.csv`, `allan_rho.csv` (ρ at 0.1/1/10/100/1000 s, median ρ,
weights), `allan_curves.csv` (all curves), `allan_adev.png`, `allan_rho.png`.

### `thermal.py` — bias vs temperature
```
python thermal.py warmup.csv [--bin 10] [--tref 25] [--frame sensor|body] [--skip 0]
```
Bins data (default 10 s), fits linear and quadratic bias(T) per IMU per axis against
the IMU's own temperature; reports tempco (linear; quadratic slope at Tref and at the
middle of the observed range), R², and per axis the IMU1/IMU2 tempco ratio, the
tempco of the 50/50 average, and the Pearson correlation of the binned bias series
(raw and after removing the linear thermal fit). Outputs `thermal_fits.csv`,
`thermal_comparison.csv`, `thermal_bias_vs_T.png`.

### `six_position.py` — accelerometer six-position calibration
```
python six_position.py six.csv                         # auto-detect static segments
python six_position.py six.csv --segments segs.csv     # label,t_start,t_end (s)
```
Each IMU is calibrated in its own sensor frame; orientation labels come from the
measured gravity direction. Per axis: `bias = (up+down)/2`,
`scale_err = (up−down)/(2g) − 1` (set `--g` to local gravity), plus a 12-parameter
LSQ model `a = M·a_true + b` (scale + misalignment). Gyro bias is reported for every
static segment. Outputs `six_segments.csv`, `six_calibration.csv`.

### `make_synthetic.py` — synthetic logs with known truth
```
python make_synthetic.py --out out/synth/raw_log.csv --hours 2 --fs 200 \
      [--mount z180|x180] [--arw 0.30] [--bi 5 --bi-model flicker|gm] [--rrw 2] \
      [--rho-white 0 --rho-bi 0.5 --rho-rrw 0.5] [--shared-bias 0] \
      [--tc-shared 2,-1.5,3 --tc-indep 1 --tc2 0.02] [--dT 15 --tau-th 1800] \
      [--drop-frac 1e-4] [--bin dump.bin --corrupt 200 --garbage 50] [--lat 52]
python make_synthetic.py --scenario six --out out/synth/six.csv
```
Writes the CSV, optional binary dump (with corrupted bytes / inserted garbage to test
resync), and `<name>.truth.json` with every injected value. Correlations and the shared
bias/tempco are applied with the **same sign in each sensor frame** (common-mode).
`t_us` starts 30 s before the uint32 wrap and `seq` is written as uint16, so the
unwrapping is always exercised.

### `mount_demo.py` — mounting effect (E4 in simulation)
Shows that a shared same-sign-in-sensor-frame bias survives the 50/50 average on z for
a 180°-about-Z mount and on x for a 180°-about-X mount, and is cancelled on the
flipped axes (`avg = (I+R)/2 · b`).

### `run_validation.py` — end-to-end test
```
python run_validation.py          # ~10 min, writes ~0.5 GB of synthetic data in out/synth
python run_validation.py --quick  # 0.5 h logs, ~2 min (looser statistics)
```
Generates synthetic data, runs every CLI, compares against the truth and writes
`out/validation_report.txt`.

## Recommended experiments

| | Experiment | Procedure | Tools |
|---|---|---|---|
| E1 | Static noise, 6–12 h | Board fixed on a vibration-free surface, constant room temperature (ideally in a closed box), log at the final ODR. Discard the first ~30 min warm-up or use `--temp-comp`. 6 h gives usable τ up to ~45 min; 12 h needed to see RRW. | `allan.py` (gyro and `--sensor accel`), also `--frame sensor` to see common-mode correlation |
| E2 | Thermal sweep | Static board in a chamber / insulated box: cold soak, then ramp up AND down (e.g. 10→50→10 °C, ≤0.5 °C/min), or at least a cold-start warm-up. Up+down separates temperature from slow stochastic drift and reveals hysteresis. | `thermal.py` (sensor frame for physics, `--frame body` for the fused output) |
| E3 | Six-position | Place the board on each face (±X, ±Y, ±Z up) against a square reference (machinist block), ≥60 s per face, still in between moves. Use local g. | `six_position.py` (auto or labelled segments) |
| E4 | Mounting comparison | Repeat E1 (and E2) with IMU2 rotated 180° about Z and then 180° about X. Compare per-axis ρ(τ) in the body frame and the bias/tempco of the 50/50 average; common-mode terms cancel only on the axes the rotation flips. | `allan.py --mount z180` / `--mount x180`, `thermal.py --frame body`, `mount_demo.py` for the expectation |

## Limitations / notes

* Real static logs contain Earth rate (15.04 °/h · components, depends on latitude and
  heading). It is a constant and does not affect ADEV, but it is part of any "bias"
  reported by `six_position.py`/`thermal.py` offsets (not the slopes). `--lat` adds it in synthesis.
* BI from `min/0.664` assumes a flicker floor; with a Gauss-Markov bias or a curve whose
  minimum is still dominated by ARW/RRW it is only an indicator. Long-τ ADEV of a 2 h log
  has large uncertainty (shaded bands are ±1σ approximate).
* `--temp-comp` regresses on temperature; during a monotonic warm-up it also absorbs part
  of the genuine slow stochastic drift, which lowers the long-τ ADEV.
* Allan correlation at long τ is computed from few clusters and is noisy; use median ρ
  over the reliable range, and E1 lengths ≥ 6 h for τ > 300 s.
* Packet loss is bridged by linear interpolation (fine for <1 %); large gaps should be
  split into separate analyses.
* The ISM330DHCX has no magnetometer; mag columns are rotated with the same mount matrix,
  which is only right if the magnetometer is on the same rotated carrier.
* Accelerometer six-position assumes the faces are orthogonal to gravity; non-square
  placement leaks into scale and misalignment.

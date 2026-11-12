"""
imu_log.py - loaders for the dual-ISM330DHCX raw logs.

Supported inputs
----------------
* CSV  (raw_log.csv) with header
  seq,t_us,ax1,ay1,az1,gx1,gy1,gz1,mx1,my1,mz1,temp1,ax2,ay2,az2,gx2,gy2,gz2,mx2,my2,mz2,temp2
  units: accel m/s^2, gyro rad/s, mag uT, temp degC, t_us = ESP32 micros() (uint32, wraps).
* Binary serial dump: 90-byte little-endian packets
  [0xAA 0x55][uint16 seq][uint32 t_us][IMU1 10xf32][IMU2 10xf32][uint16 CRC]
  CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, xorout 0)
  computed over bytes 2..87 inclusive. The parser resynchronises on bad CRC.

What the loader does
--------------------
1. unwraps t_us (uint32 micros) and seq (uint16 in the binary stream; also
   treated as possibly wrapping in CSV),
2. detects sequence gaps (dropped packets) and duplicates,
3. rotates IMU2 into the body frame (= IMU1 sensor frame) with a configurable
   mount rotation (default 180 deg about Z),
4. returns an `IMULog` object holding numpy arrays (N x 3) per sensor.

Frame convention:  v_body = R_mount @ v_sensor2.
For a 180 deg rotation about Z, R = diag(-1, -1, +1); about X, diag(+1, -1, -1).

Usage as a script prints a summary:
    python imu_log.py raw_log.csv [--mount z180]
"""
from __future__ import annotations

import argparse
import dataclasses
import warnings
from pathlib import Path

import numpy as np
import pandas as pd

CSV_COLUMNS = ["seq", "t_us",
               "ax1", "ay1", "az1", "gx1", "gy1", "gz1", "mx1", "my1", "mz1", "temp1",
               "ax2", "ay2", "az2", "gx2", "gy2", "gz2", "mx2", "my2", "mz2", "temp2"]

PACKET_LEN = 90
SYNC = b"\xAA\x55"
_PKT_DTYPE = np.dtype([("sync", "<u2"), ("seq", "<u2"), ("t_us", "<u4"),
                       ("imu1", "<f4", (10,)), ("imu2", "<f4", (10,)), ("crc", "<u2")])
assert _PKT_DTYPE.itemsize == PACKET_LEN

DEG = np.pi / 180.0


# ----------------------------------------------------------------------------
# rotations
# ----------------------------------------------------------------------------
def _rot(axis: str, deg: float) -> np.ndarray:
    a = deg * DEG
    c, s = np.cos(a), np.sin(a)
    if axis == "x":
        R = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    elif axis == "y":
        R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    elif axis == "z":
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    else:
        raise ValueError(axis)
    R[np.abs(R) < 1e-12] = 0.0
    return R


def mount_matrix(spec) -> np.ndarray:
    """Return R (3x3) with v_body = R @ v_sensor2.

    spec may be
      * a 3x3 array-like,
      * 'z180' (default board), 'x180', 'y180', 'none'/'identity',
      * a sequence of intrinsic rotations, e.g. 'z90,x180' (applied left to right,
        R = Rz(90) @ Rx(180)).
    """
    if spec is None:
        spec = "z180"
    if not isinstance(spec, str):
        R = np.asarray(spec, dtype=float)
        if R.shape != (3, 3):
            raise ValueError("mount matrix must be 3x3")
        return R
    s = spec.strip().lower()
    if s in ("none", "identity", "eye", "i"):
        return np.eye(3)
    R = np.eye(3)
    for part in s.split(","):
        part = part.strip().replace(":", "")
        if not part:
            continue
        axis, deg = part[0], float(part[1:])
        R = R @ _rot(axis, deg)
    return R


# ----------------------------------------------------------------------------
# data structure
# ----------------------------------------------------------------------------
@dataclasses.dataclass
class IMULog:
    t: np.ndarray            # s, from first sample, unwrapped
    seq: np.ndarray          # unwrapped sequence number (int64)
    fs: float                # nominal sample rate estimate [Hz]
    acc1: np.ndarray         # N x 3, m/s^2, IMU1 sensor = body frame
    gyr1: np.ndarray         # N x 3, rad/s
    mag1: np.ndarray         # N x 3, uT
    temp1: np.ndarray        # N, degC
    acc2: np.ndarray         # N x 3, IMU2 rotated into body frame
    gyr2: np.ndarray
    mag2: np.ndarray
    temp2: np.ndarray
    acc2_sensor: np.ndarray  # N x 3, IMU2 in its own sensor frame
    gyr2_sensor: np.ndarray
    mag2_sensor: np.ndarray
    R_mount: np.ndarray
    gaps: pd.DataFrame       # one row per gap: index, seq_before, missing, t_s
    meta: dict

    @property
    def n(self) -> int:
        return len(self.t)

    @property
    def duration(self) -> float:
        return float(self.t[-1] - self.t[0]) if self.n > 1 else 0.0

    def gyro(self, imu: int, frame: str = "body") -> np.ndarray:
        if imu == 1:
            return self.gyr1
        return self.gyr2 if frame == "body" else self.gyr2_sensor

    def accel(self, imu: int, frame: str = "body") -> np.ndarray:
        if imu == 1:
            return self.acc1
        return self.acc2 if frame == "body" else self.acc2_sensor

    def to_dataframe(self, frame: str = "body") -> pd.DataFrame:
        """Wide tidy DataFrame (one row per sample) in the requested frame."""
        a2 = self.acc2 if frame == "body" else self.acc2_sensor
        g2 = self.gyr2 if frame == "body" else self.gyr2_sensor
        m2 = self.mag2 if frame == "body" else self.mag2_sensor
        d = {"t": self.t, "seq": self.seq}
        for name, arr in (("a", self.acc1), ("g", self.gyr1), ("m", self.mag1)):
            for i, ax in enumerate("xyz"):
                d[f"{name}{ax}1"] = arr[:, i]
        d["temp1"] = self.temp1
        for name, arr in (("a", a2), ("g", g2), ("m", m2)):
            for i, ax in enumerate("xyz"):
                d[f"{name}{ax}2"] = arr[:, i]
        d["temp2"] = self.temp2
        return pd.DataFrame(d)

    def summary(self) -> str:
        nmiss = int(self.gaps["missing"].sum()) if len(self.gaps) else 0
        lines = [
            f"samples          : {self.n}",
            f"duration         : {self.duration:.1f} s ({self.duration/3600:.3f} h)",
            f"sample rate      : {self.fs:.3f} Hz (nominal estimate)",
            f"seq gaps         : {len(self.gaps)} gaps, {nmiss} missing samples "
            f"({100*nmiss/max(self.n+nmiss,1):.4f} %)",
            f"duplicates drop. : {self.meta.get('duplicates', 0)}",
            f"t_us wraps       : {self.meta.get('t_wraps', 0)}",
            f"mount R (IMU2->body):\n{np.array2string(self.R_mount, precision=3)}",
            f"temp1 range      : {self.temp1.min():.2f} .. {self.temp1.max():.2f} C",
            f"temp2 range      : {self.temp2.min():.2f} .. {self.temp2.max():.2f} C",
        ]
        if "bin_bad_crc" in self.meta:
            lines.append(f"binary: {self.meta['bin_packets']} good packets, "
                         f"{self.meta['bin_bad_crc']} bad-CRC candidates, "
                         f"{self.meta['bin_skipped_bytes']} bytes skipped")
        return "\n".join(lines)


# ----------------------------------------------------------------------------
# unwrapping
# ----------------------------------------------------------------------------
def unwrap_counter(x: np.ndarray, modulus: int) -> np.ndarray:
    """Unwrap a modular unsigned counter (e.g. uint32 micros, uint16 seq)."""
    x = np.asarray(x, dtype=np.int64)
    d = np.diff(x) % modulus
    # a "negative" step (modular diff > modulus/2) is a backwards jump/duplicate
    d = np.where(d > modulus // 2, d - modulus, d)
    return np.concatenate([[x[0]], x[0] + np.cumsum(d)])


def _process(seq_raw, t_us_raw, imu1, imu2, mount, seq_modulus, meta) -> IMULog:
    """imu1/imu2: N x 10 float arrays (ax,ay,az,gx,gy,gz,mx,my,mz,temp)."""
    t_us_raw = np.asarray(t_us_raw, dtype=np.int64)
    t_wraps = int(np.sum(np.diff(t_us_raw) < -(2 ** 31)))
    t_us = unwrap_counter(t_us_raw, 2 ** 32)

    dt_med = float(np.median(np.diff(t_us))) * 1e-6 if len(t_us) > 1 else 1.0
    fs = 1.0 / dt_med if dt_med > 0 else float("nan")

    # seq: if the stored seq never exceeds 65535 assume it may be a uint16 that wraps
    seq_raw = np.asarray(seq_raw, dtype=np.int64)
    if seq_modulus is None:
        seq_modulus = 2 ** 16 if seq_raw.max() < 2 ** 16 else 2 ** 32
    seq = unwrap_counter(seq_raw, seq_modulus)
    # disambiguate long outages (> modulus samples) using the timestamps
    dseq = np.diff(seq)
    dt_steps = np.rint(np.diff(t_us) * 1e-6 / dt_med).astype(np.int64)
    big = dt_steps > seq_modulus // 2
    if np.any(big):
        k = np.rint((dt_steps[big] - dseq[big]) / seq_modulus).astype(np.int64)
        dseq = dseq.copy()
        dseq[big] += k * seq_modulus
        seq = np.concatenate([[seq[0]], seq[0] + np.cumsum(dseq)])

    # drop duplicates / backward steps (keep first occurrence, monotone seq)
    keep = np.ones(len(seq), bool)
    run_max = np.maximum.accumulate(seq)
    keep[1:] = seq[1:] > run_max[:-1]
    n_dup = int((~keep).sum())
    seq, t_us = seq[keep], t_us[keep]
    imu1, imu2 = imu1[keep], imu2[keep]

    t = (t_us - t_us[0]) * 1e-6
    d = np.diff(seq)
    gi = np.nonzero(d > 1)[0]
    gaps = pd.DataFrame({"index": gi + 1, "seq_before": seq[gi],
                         "missing": d[gi] - 1, "t_s": t[gi]})

    R = mount_matrix(mount)
    a2s, g2s, m2s = imu2[:, 0:3], imu2[:, 3:6], imu2[:, 6:9]
    meta = dict(meta)
    meta.update(t_wraps=t_wraps, duplicates=n_dup, seq_modulus=seq_modulus,
                mount=mount if isinstance(mount, str) else "matrix")
    return IMULog(
        t=t, seq=seq, fs=fs,
        acc1=imu1[:, 0:3].astype(float), gyr1=imu1[:, 3:6].astype(float),
        mag1=imu1[:, 6:9].astype(float), temp1=imu1[:, 9].astype(float),
        acc2=(a2s @ R.T).astype(float), gyr2=(g2s @ R.T).astype(float),
        mag2=(m2s @ R.T).astype(float), temp2=imu2[:, 9].astype(float),
        acc2_sensor=a2s.astype(float), gyr2_sensor=g2s.astype(float),
        mag2_sensor=m2s.astype(float), R_mount=R, gaps=gaps, meta=meta)


# ----------------------------------------------------------------------------
# CSV
# ----------------------------------------------------------------------------
def from_dataframe(df: pd.DataFrame, mount="z180", seq_modulus=None, meta=None) -> IMULog:
    missing = [c for c in CSV_COLUMNS if c not in df.columns]
    if missing:
        raise ValueError(f"missing columns: {missing}")
    df = df.dropna(subset=CSV_COLUMNS)
    imu1 = df[CSV_COLUMNS[2:12]].to_numpy(float)
    imu2 = df[CSV_COLUMNS[12:22]].to_numpy(float)
    return _process(df["seq"].to_numpy(np.int64), df["t_us"].to_numpy(np.int64),
                    imu1, imu2, mount, seq_modulus, meta or {"source": "dataframe"})


def load_csv(path, mount="z180", seq_modulus=None) -> IMULog:
    df = pd.read_csv(path, comment="#", skipinitialspace=True, on_bad_lines="skip")
    df.columns = [c.strip() for c in df.columns]
    for c in CSV_COLUMNS:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce")
    return from_dataframe(df, mount, seq_modulus, {"source": str(path)})


# ----------------------------------------------------------------------------
# binary
# ----------------------------------------------------------------------------
def _crc_table() -> np.ndarray:
    tbl = np.zeros(256, dtype=np.uint16)
    for i in range(256):
        c = i << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) if (c & 0x8000) else (c << 1)
            c &= 0xFFFF
        tbl[i] = c
    return tbl


_CRC_TBL = _crc_table()


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ int(_CRC_TBL[((crc >> 8) ^ b) & 0xFF])
    return crc


def _crc_rows(rows: np.ndarray) -> np.ndarray:
    """Vectorised CRC-16/CCITT-FALSE over each row of a (K, L) uint8 array."""
    crc = np.full(rows.shape[0], 0xFFFF, dtype=np.uint32)
    tbl = _CRC_TBL.astype(np.uint32)
    for j in range(rows.shape[1]):
        crc = ((crc << 8) & 0xFFFF) ^ tbl[((crc >> 8) ^ rows[:, j]) & 0xFF]
    return crc.astype(np.uint16)


def parse_binary(buf: bytes):
    """Return (structured packet array, stats dict). Resyncs on bad CRC."""
    a = np.frombuffer(buf, dtype=np.uint8)
    n = len(a)
    if n < PACKET_LEN:
        return np.zeros(0, _PKT_DTYPE), dict(bin_packets=0, bin_bad_crc=0, bin_skipped_bytes=n)
    cand = np.nonzero((a[:-1] == 0xAA) & (a[1:] == 0x55))[0]
    cand = cand[cand + PACKET_LEN <= n]
    idx = cand[:, None] + np.arange(PACKET_LEN)[None, :]
    rows = a[idx]
    crc_calc = _crc_rows(rows[:, 2:88])
    crc_rx = rows[:, 88].astype(np.uint16) | (rows[:, 89].astype(np.uint16) << 8)
    ok = crc_calc == crc_rx
    good_pos = cand[ok]
    # greedy non-overlapping acceptance (a valid packet consumes 90 bytes)
    accepted = []
    nxt = 0
    for p in good_pos.tolist():
        if p >= nxt:
            accepted.append(p)
            nxt = p + PACKET_LEN
    accepted = np.asarray(accepted, dtype=np.int64)
    if len(accepted):
        pk = np.frombuffer(a[accepted[:, None] + np.arange(PACKET_LEN)].tobytes(), dtype=_PKT_DTYPE)
    else:
        pk = np.zeros(0, _PKT_DTYPE)
    stats = dict(bin_packets=len(accepted),
                 bin_bad_crc=int((~ok).sum()),
                 bin_skipped_bytes=int(n - PACKET_LEN * len(accepted)))
    return pk, stats


def load_binary(path, mount="z180") -> IMULog:
    buf = Path(path).read_bytes()
    pk, stats = parse_binary(buf)
    if len(pk) == 0:
        raise ValueError("no valid packets found")
    meta = {"source": str(path)}
    meta.update(stats)
    return _process(pk["seq"].astype(np.int64), pk["t_us"].astype(np.int64),
                    pk["imu1"].astype(float), pk["imu2"].astype(float),
                    mount, 2 ** 16, meta)


def encode_binary(seq, t_us, imu1, imu2) -> bytes:
    """Encode packets (used by the synthetic generator / tests)."""
    N = len(seq)
    pk = np.zeros(N, _PKT_DTYPE)
    pk["sync"] = 0x55AA  # little-endian -> bytes AA 55
    pk["seq"] = np.asarray(seq, np.int64) % 2 ** 16
    pk["t_us"] = np.asarray(t_us, np.int64) % 2 ** 32
    pk["imu1"] = imu1
    pk["imu2"] = imu2
    raw = np.frombuffer(pk.tobytes(), dtype=np.uint8).reshape(N, PACKET_LEN).copy()
    crc = _crc_rows(raw[:, 2:88])
    raw[:, 88] = crc & 0xFF
    raw[:, 89] = crc >> 8
    return raw.tobytes()


def load(path, mount="z180") -> IMULog:
    """Dispatch on extension: .csv/.txt -> CSV, anything else -> binary."""
    p = Path(path)
    if p.suffix.lower() in (".csv", ".txt"):
        return load_csv(p, mount)
    return load_binary(p, mount)


# ----------------------------------------------------------------------------
# helpers for analysis tools
# ----------------------------------------------------------------------------
def uniform_series(log: IMULog, x: np.ndarray, max_gap_frac: float = 0.01) -> np.ndarray:
    """Place samples on a uniform seq grid and linearly interpolate dropped samples.

    Allan variance assumes uniform sampling; small packet loss is bridged by linear
    interpolation (this slightly lowers short-tau white noise in the gap regions).
    """
    if len(log.gaps) == 0:
        return x
    full = np.arange(log.seq[0], log.seq[-1] + 1)
    frac = 1 - len(log.seq) / len(full)
    if frac > max_gap_frac:
        warnings.warn(f"{100*frac:.2f} % samples missing - Allan results near the "
                      "sample period may be biased")
    x = np.asarray(x, float)
    if x.ndim == 1:
        return np.interp(full, log.seq, x)
    return np.column_stack([np.interp(full, log.seq, x[:, i]) for i in range(x.shape[1])])


def add_mount_args(p: argparse.ArgumentParser):
    p.add_argument("--mount", default="z180",
                   help="IMU2 mount rotation (v_body = R v_sensor2): z180 (default), "
                        "x180, y180, none, or e.g. 'z90,x180'")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("log", help="raw_log.csv or binary dump")
    add_mount_args(p)
    p.add_argument("--export", help="write tidy body-frame CSV with t in seconds")
    a = p.parse_args()
    log = load(a.log, a.mount)
    print(log.summary())
    if len(log.gaps):
        print("\nfirst gaps:\n", log.gaps.head(10).to_string(index=False))
    if a.export:
        log.to_dataframe().to_csv(a.export, index=False)
        print("wrote", a.export)


if __name__ == "__main__":
    main()

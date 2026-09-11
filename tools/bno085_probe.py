"""BNO085 probe: achieved report rates, timestamp jitter, static bias/noise, the +9.81 on +Z check,
and a verdict against task §9.2 (>= 100 Hz gyro, jitter p99 < 5 ms).  Stop and report if it fails.

    .venv/bin/python tools/bno085_probe.py --seconds 30 [--gyro-hz 200 --accel-hz 100]
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from goldenfleece.clock import MonotonicClock                     # noqa: E402
from goldenfleece.config import load_config                        # noqa: E402
from goldenfleece.l02_imu_data_input.bno085_spi import Bno085SpiSource   # noqa: E402
from goldenfleece.types import ImuKind                             # noqa: E402


def pct(xs, p):
    s = sorted(xs)
    return s[min(len(s) - 1, int(p * len(s)))] if s else float("nan")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--gyro-hz", type=float, default=None)
    ap.add_argument("--accel-hz", type=float, default=None)
    ap.add_argument("--config", default=str(ROOT / "config"))
    ap.add_argument("--json", default=None)
    a = ap.parse_args(argv)
    cfg = load_config(a.config)
    imu_cfg = cfg.pipeline.imu
    d = imu_cfg.as_dict()
    if a.gyro_hz:
        d["gyro_rate_hz"] = a.gyro_hz
    if a.accel_hz:
        d["accel_rate_hz"] = a.accel_hz
    from goldenfleece.config import Section
    clk = MonotonicClock()
    src = Bno085SpiSource(Section(d, "pipeline.imu"), clk)
    src.start()
    t0 = clk.now()
    gyro, accel, events = [], [], []
    while clk.now() - t0 < a.seconds:
        for s in src.drain():
            (gyro if s.kind is ImuKind.GYRO else accel if s.kind is ImuKind.ACCEL else []).append(s)
        events += src.drain_events()
        clk.sleep(0.01)
    src.stop()
    dur = clk.now() - t0
    res = {"seconds": dur, "events": [(e.active, e.detail) for e in events], "errors": src.n_errors, "reinit": src.n_reinit}
    for name, xs in (("gyro", gyro), ("accel", accel)):
        if len(xs) < 3:
            res[name] = {"n": len(xs), "rate_hz": 0.0}
            continue
        dts = [b.t - a_.t for a_, b in zip(xs[:-1], xs[1:])]
        vals = list(zip(*[s.values for s in xs]))
        res[name] = {"n": len(xs), "rate_hz": len(xs) / dur, "dt_p50_ms": pct(dts, 0.5) * 1e3, "dt_p99_ms": pct(dts, 0.99) * 1e3,
                     "dt_max_ms": max(dts) * 1e3, "jitter_p99_ms": pct([abs(x - statistics.median(dts)) for x in dts], 0.99) * 1e3,
                     "mean": [statistics.fmean(v) for v in vals], "std": [statistics.pstdev(v) for v in vals]}
    g_ok = res["gyro"].get("rate_hz", 0) >= 100.0 and res["gyro"].get("jitter_p99_ms", 1e9) < 5.0
    a_ok = res["accel"].get("rate_hz", 0) >= 50.0
    res["verdict"] = "OK" if (g_ok and a_ok) else "STOP_AND_REPORT: library cannot sustain the rate/timing (task §9.2)"
    if "mean" in res["accel"]:
        m = res["accel"]["mean"]
        res["plus_g_on_up_axis_imu_frame"] = m
        res["note"] = "expect ~+9.81 on the IMU axis that points UP (then T_radar_imu maps it to radar +Z)"
    print(json.dumps(res, indent=1))
    if a.json:
        Path(a.json).write_text(json.dumps(res, indent=1))
    return 0 if res["verdict"] == "OK" else 2


if __name__ == "__main__":
    sys.exit(main())

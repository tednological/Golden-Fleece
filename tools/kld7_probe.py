"""K-LD7 probe (task §9.1).  Logs: firmware version, per-frame target count and cap-hit rate, frame-number
gaps, measured frame period per RSPI, achieved baud, GNFD->PDAT-header delay statistics (delta_sensor
estimate), header-timestamp jitter, RESP error counts, the early-poll experiment, and a magnitude-vs-range
log for alias-check calibration.  Writes a JSON summary.

    .venv/bin/python tools/kld7_probe.py --seconds 20 [--all-rspi] [--early-poll] [--fake]
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
from dataclasses import replace
from pathlib import Path
from typing import Dict, List

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from goldenfleece.clock import MonotonicClock, FakeClock            # noqa: E402
from goldenfleece.config import load_config                         # noqa: E402
from goldenfleece.l01_radar_data_input import protocol as K         # noqa: E402
from goldenfleece.l01_radar_data_input.driver import DirectKld7Source, open_pyserial   # noqa: E402
from goldenfleece.types import RadarConfigChanged                    # noqa: E402


def pct(xs, p):
    s = sorted(xs)
    return s[min(len(s) - 1, int(p * len(s)))] if s else float("nan")


def probe_rspi(cfg, clk, opener, rspi: int, seconds: float, early_poll: bool) -> Dict:
    out: Dict = {"rspi": rspi, "connected": False, "baud_attempts": []}
    drv = None
    rcfg = None
    for baud in [cfg.radar.baudrate] + [b for b in (460800, 115200) if b != cfg.radar.baudrate]:
        rcfg = replace(cfg.radar, params={**cfg.radar.params, "RSPI": rspi}, baudrate=baud)
        drv = DirectKld7Source(rcfg, clk, open_serial=opener)
        ok = drv._connect()
        out["baud_attempts"].append((baud, ok, [str(e) for e in drv.drain_events() if not isinstance(e, RadarConfigChanged)][:3]))
        if ok:
            break
        drv = None
    if drv is None:
        return out
    out["connected"] = True
    out["firmware"] = drv.firmware_version
    out["baud"] = rcfg.baudrate            # achieved baud
    t0 = clk.now()
    headers: List[float] = []
    delays: List[float] = []
    counts: List[int] = []
    mags: List[List[int]] = []
    fns: List[int] = []
    busy = 0
    while clk.now() - t0 < seconds:
        t_send = clk.now()
        drv._poll_cycle()
        f = drv.get(0.0)
        if f is None:
            continue
        headers.append(f.t_header)
        delays.append(f.t_header - t_send)
        counts.append(len(f.targets))
        fns.append(f.frame_number)
        mags.append([[t.distance_cm, t.magnitude_raw, t.speed_raw, t.angle_raw] for t in f.targets])
        if early_poll:
            # experiment: send the next GNFD immediately (before this frame's data would normally be requested)
            try:
                drv.ser.write(K.cmd_gnfd())
                code = drv._wait_resp(0.05)
                if code == K.RespCode.SENSOR_BUSY:
                    busy += 1
            except Exception:  # noqa: BLE001
                pass
    periods = [b - a for a, b in zip(headers[:-1], headers[1:])]
    per_frame = [(hb - ha) / max(fb - fa, 1) for ha, hb, fa, fb in zip(headers[:-1], headers[1:], fns[:-1], fns[1:]) if fb > fa]
    out.update({
        "frames": len(headers), "gaps": drv.stats.gaps, "gap_frames_missed": drv.stats.gap_frames_missed,
        "frame_period_ms": {"p50": pct(per_frame, 0.5) * 1e3, "p10": pct(per_frame, 0.1) * 1e3, "p90": pct(per_frame, 0.9) * 1e3},
        "poll_to_header_ms": {"min": min(delays) * 1e3 if delays else None, "mean": statistics.fmean(delays) * 1e3 if delays else None,
                              "p99": pct(delays, 0.99) * 1e3},
        "delta_sensor_estimate_ms": {"if_free_running(min)": min(delays) * 1e3 if delays else None,
                                     "if_poll_triggered(mean - T)": (statistics.fmean(delays) - cfg.radar.frame_duration_s[rspi]) * 1e3 if delays else None},
        "header_jitter_ms_p99": pct([abs(p - statistics.median(per_frame)) for p in per_frame], 0.99) * 1e3 if per_frame else None,
        "targets_per_frame": {"mean": statistics.fmean(counts) if counts else 0, "max": max(counts) if counts else 0,
                              "cap_hit_rate": (sum(1 for c in counts if c >= 12) / len(counts)) if counts else 0.0},
        "resp_codes": drv.stats.resp_codes, "resync_bytes": drv.stats.resync_bytes, "poll_timeouts": drv.stats.poll_timeouts,
        "early_poll_busy": busy if early_poll else None,
        "magnitude_vs_range_sample": mags[:50],
    })
    drv._disconnect(send_gbye=True)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--all-rspi", action="store_true", help="measure the frame period for RSPI 0..3")
    ap.add_argument("--early-poll", action="store_true")
    ap.add_argument("--fake", action="store_true", help="dry run against tools/fake_kld7.py")
    ap.add_argument("--config", default=str(ROOT / "config"))
    ap.add_argument("--json", default=str(ROOT / "docs" / "probe_kld7.json"))
    a = ap.parse_args(argv)
    cfg = load_config(a.config)
    if a.fake:
        from tools.fake_kld7 import FakeKld7Serial
        from goldenfleece.types import RawRadarTarget
        clk = FakeClock(0.0)
        fake = FakeKld7Serial(clk, lambda fn, t: [RawRadarTarget(1500 + (fn % 7) * 30, 900 + (fn % 3) * 78, (fn % 21 - 10) * 100, 4000 - (fn % 5) * 100)],
                              sensor_baud=921600, sensor_delay_s=0.011)

        def opener(port, baud):
            fake.baudrate = baud
            return fake
    else:
        clk = MonotonicClock()
        opener = open_pyserial
    results = {"port": cfg.radar.port, "configured_baud": cfg.radar.baudrate, "runs": []}
    rspis = [0, 1, 2, 3] if a.all_rspi else [cfg.radar.params["RSPI"]]
    for r in rspis:
        print(f"probing RSPI={r} for {a.seconds:.0f} s ...", flush=True)
        res = probe_rspi(cfg, clk, opener, r, a.seconds, a.early_poll)
        results["runs"].append(res)
        print(json.dumps({k: v for k, v in res.items() if k != "magnitude_vs_range_sample"}, indent=1))
    Path(a.json).write_text(json.dumps(results, indent=1))
    print("summary written to", a.json)
    return 0


if __name__ == "__main__":
    sys.exit(main())

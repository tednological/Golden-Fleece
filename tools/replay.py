"""Replay a recording through l03-l09 and check determinism.

    .venv/bin/python tools/replay.py recordings/session.jsonl            # replay, compare with recorded decisions
    .venv/bin/python tools/replay.py recordings/session.jsonl --twice    # also assert two replays are bit-identical
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from goldenfleece.config import load_config                                       # noqa: E402
from goldenfleece.orchestrator.pipeline import Pipeline                           # noqa: E402
from goldenfleece.orchestrator.recording import imu_from_rec, radar_from_rec, read_recording   # noqa: E402
from goldenfleece.types import HealthBits                                         # noqa: E402


def cmd_key(cmd) -> Dict[str, Any]:
    return {"lvl": int(cmd.level), "side": cmd.side.value, "bkt": int(cmd.t_arrival_bucket), "hs": cmd.health_state.name, "bits": int(cmd.health_bits)}


def replay(path: str, cfg) -> Dict[str, Any]:
    pipe = Pipeline(cfg)
    pipe.set_health(HealthBits.PIPELINE_RESTARTING, True, 0.0, "orchestrator", "replay start")
    outputs: List[Dict[str, Any]] = []
    recorded: Dict[int, Dict[str, Any]] = {}
    n_imu = n_rf = 0
    for r in read_recording(path):
        k = r["k"]
        if k == "imu":
            pipe.ingest_imu(imu_from_rec(r))
            n_imu += 1
        elif k == "rf":
            raw = radar_from_rec(r)
            t_now = float(r.get("tn", raw.t_header + 0.003))
            res = pipe.process_frame(raw, t_now)
            n_rf += 1
            outputs.append({"fn": raw.frame_number, "t": t_now, **cmd_key(res.command), "ntr": len(res.tracks.tracks), "ndet": len(res.radar.detections)})
        elif k == "cmd":
            recorded[int(r["seq"])] = r
    return {"outputs": outputs, "recorded": recorded, "n_imu": n_imu, "n_rf": n_rf, "counters": {
        "tracker": dict(pipe.tracker.total), "coasts": dict(pipe.tracker.coast_counts), "resolutions": dict(pipe.tracker.resolution_counts),
        "flicker": pipe.policy.flicker_count, "escalations": pipe.policy.n_escalations}}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("recording")
    ap.add_argument("--twice", action="store_true")
    ap.add_argument("--config", default=str(ROOT / "config"))
    ap.add_argument("--dump", default=None, help="write per-frame outputs as JSON")
    a = ap.parse_args(argv)
    cfg = load_config(a.config)
    r1 = replay(a.recording, cfg)
    print(f"replayed {r1['n_rf']} radar frames, {r1['n_imu']} IMU samples; counters={json.dumps(r1['counters'])}")
    levels = [o["lvl"] for o in r1["outputs"]]
    print(f"levels: max={max(levels) if levels else 0}, frames at >=WARNING: {sum(1 for l in levels if l >= 2)}")
    # compare with recorded decisions (recorded cmds are only written on change/refresh, so compare per matching t)
    rec_by_t = {round(v["t"], 6): v for v in r1["recorded"].values()}
    mism = 0
    checked = 0
    for o in r1["outputs"]:
        rv = rec_by_t.get(round(o["t"], 6))
        if rv is None:
            continue
        checked += 1
        if rv["lvl"] != o["lvl"] or rv["side"] != o["side"] or rv["hs"] != o["hs"]:
            mism += 1
    print(f"vs recorded decisions: {checked} compared, {mism} mismatches")
    if a.twice:
        r2 = replay(a.recording, cfg)
        identical = r1["outputs"] == r2["outputs"]
        print("second replay bit-identical:", identical)
        if not identical:
            return 2
    if a.dump:
        with open(a.dump, "w") as f:
            json.dump(r1["outputs"], f)
    return 1 if mism else 0


if __name__ == "__main__":
    sys.exit(main())

"""Run scenarios through the pure pipeline and report metrics.

    .venv/bin/python -m tools.sim.run                # all scenarios
    .venv/bin/python -m tools.sim.run overtake_left_10mps --verbose
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List, Optional

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from goldenfleece.clock import FakeClock, MonotonicClock          # noqa: E402
from goldenfleece.config import load_config                        # noqa: E402
from goldenfleece.orchestrator.pipeline import Pipeline            # noqa: E402
from goldenfleece.types import HealthBits                          # noqa: E402
from tools.sim.imu_model import ImuModel                           # noqa: E402
from tools.sim.kinematics import RiderKinematics                   # noqa: E402
from tools.sim.metrics import MetricsCollector, ScenarioMetrics    # noqa: E402
from tools.sim.radar_model import RadarModel                       # noqa: E402
from tools.sim.scenarios import BY_NAME, Scenario, all_scenarios   # noqa: E402

PIPELINE_COMPUTE_S = 0.003     # nominal Pi-side compute assumed between header arrival and decision (sim only)


def run_scenario(sc: Scenario, cfg, verbose: bool = False, on_frame=None) -> ScenarioMetrics:
    rider = RiderKinematics(sc.world.road, sc.rider)
    radar = RadarModel(sc.world, rider, sc.radar)
    imu = ImuModel(rider, sc.imu)
    clock = FakeClock(0.0)
    pipe = Pipeline(cfg, perf_clock=MonotonicClock())
    pipe.set_health(HealthBits.PIPELINE_RESTARTING, True, 0.0, "orchestrator", "start")
    mc = MetricsCollector(sc.name, sc.notes, sc.expect_warning)
    T = radar.T
    k = 0
    t_last_tick = 0.0
    while True:
        t_mid = 0.5 * T + k * T
        if t_mid > sc.duration_s:
            break
        raw, truth = radar.frame(t_mid)
        if raw is None:
            # no frame: advance time with health-only ticks so RADAR_SILENT can be detected
            t_now = t_mid + T / 2 + sc.radar.sensor_delay_s
            for s in imu.samples_until(t_now):
                pipe.ingest_imu(s)
            clock.set(t_now)
            if t_now - t_last_tick >= cfg.pipeline.orchestrator.tick_timeout_s:
                cmd, ev = pipe.tick(t_now)
                t_last_tick = t_now
                for e in ev:
                    if e.active:
                        mc.health_event(e.bit.name, e.t)
                mc.frame(truth, cmd, False, 0.0, None, False, bool(pipe.health.bits & HealthBits.RADAR_POSSIBLY_BLOCKED), {}, None)
            k += 1
            continue
        t_now = raw.t_header + PIPELINE_COMPUTE_S
        for s in imu.samples_until(raw.t_header):
            pipe.ingest_imu(s)
        clock.set(t_now)
        res = pipe.process_frame(raw, t_now)
        t_last_tick = t_now
        for e in res.health_events:
            if e.active:
                mc.health_event(e.bit.name, e.t)
        e2e = (t_now - res.radar.t_mid)
        mc.frame(truth, res.command, res.ego.valid, res.ego.speed, res.ego.psi_travel, raw.cap_hit,
                 bool(pipe.health.bits & HealthBits.RADAR_POSSIBLY_BLOCKED), res.stage_s, e2e,
                 ego_reason=res.ego.source.name if res.ego.valid else res.ego.invalid_reason.name)
        if on_frame is not None:
            on_frame(truth, res)
        if verbose and k % 10 == 0:
            v = truth.vehicles[0] if truth.vehicles else None
            print(f"t={t_mid:6.2f} lvl={int(res.command.level)} side={res.command.side.value:6s} tracks={len(res.tracks.tracks)} "
                  f"dets={len(res.radar.detections)} ego={'V' if res.ego.valid else '-'}{res.ego.speed:4.1f} "
                  f"truth={'r=%.1f az=%.0f ta=%s' % (v.r, __import__('math').degrees(v.az), f'{v.t_arrival:.1f}' if v.t_arrival else '-') if v else '-'} "
                  f"health={res.command.health_state.name}")
        k += 1
    return mc.finish(sc.fault_onsets, sc.radar.blockage_t_start)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("names", nargs="*")
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--json", type=str, default=None)
    ap.add_argument("--config", type=str, default=str(ROOT / "config"))
    a = ap.parse_args(argv)
    cfg = load_config(a.config)
    scs = [BY_NAME[n]() for n in a.names] if a.names else all_scenarios()
    results: List[ScenarioMetrics] = []
    for sc in scs:
        m = run_scenario(sc, cfg, verbose=a.verbose)
        results.append(m)
        print(m.summary_line())
        if m.health_onset_latency_s:
            print(f"{'':32s} health onset->detect: " + ", ".join(f"{k}={v:.2f}s" if v is not None else f"{k}=NOT DETECTED" for k, v in m.health_onset_latency_s.items()))
        if m.ego_reasons:
            print(f"{'':32s} ego: " + ", ".join(f"{k}={v}" for k, v in sorted(m.ego_reasons.items(), key=lambda kv: -kv[1])))
        if m.psi_travel_n:
            print(f"{'':32s} psi_travel diag: mean err {m.psi_travel_err_mean_deg:+.1f} deg, std {m.psi_travel_err_std_deg:.1f} deg over {m.psi_travel_n} frames")
        if m.blockage_false_alarm_frames or m.blockage_detect_latency_s is not None:
            print(f"{'':32s} blockage: false-alarm frames={m.blockage_false_alarm_frames} detect latency={m.blockage_detect_latency_s}")
    # aggregate Python-side stage compute cost (measured on this machine with a real clock)
    agg: Dict[str, List[float]] = {}
    for m in results:
        for k, v in m.stage_p99_ms.items():
            agg.setdefault(k, []).append(v)
    print("stage compute p99 (ms), worst scenario: " + ", ".join(f"{k}={max(v):.2f}" for k, v in agg.items()))
    if a.json:
        with open(a.json, "w") as f:
            json.dump([m.__dict__ | {"vehicles": [v.__dict__ for v in m.vehicles]} for m in results], f, indent=1, default=str)
    return 0


if __name__ == "__main__":
    sys.exit(main())

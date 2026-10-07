"""Golden Fleece pipeline entry point (composition root).

    .venv/bin/python tools/run_pipeline.py --config config [--no-imu] [--no-haptics] [--deploy]
    .venv/bin/python tools/run_pipeline.py --radar sim:overtake_left_10mps --imu sim     # real runner + real haptics on the simulator
"""
from __future__ import annotations

import argparse
import gc
import logging
import os
import signal
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from goldenfleece.clock import MonotonicClock                              # noqa: E402
from goldenfleece.config import load_config, log_config_warnings            # noqa: E402
from goldenfleece.l01_radar_data_input.driver import DirectKld7Source       # noqa: E402
from goldenfleece.l01_radar_data_input.source import QueueRadarSource       # noqa: E402
from goldenfleece.l02_imu_data_input.bno085_spi import Bno085SpiSource      # noqa: E402
from goldenfleece.l02_imu_data_input.source import QueueImuSource           # noqa: E402
from goldenfleece.l10_haptics.motors import BlinkaPwmMotor, NullMotor    # noqa: E402
from goldenfleece.l10_haptics.output import HapticOutput                  # noqa: E402
from goldenfleece.l10_haptics.patterns import haptics_config              # noqa: E402
from goldenfleece.orchestrator.power_monitor import PowerMonitor            # noqa: E402
from goldenfleece.orchestrator.recording import RecordingWriter             # noqa: E402
from goldenfleece.orchestrator.runner import Runner                         # noqa: E402
from goldenfleece.orchestrator.sdnotify import SdNotifier                   # noqa: E402
from goldenfleece.types import HealthBits, HealthEvent                      # noqa: E402

log = logging.getLogger("goldenfleece.main")


class RealTimeSimSources:
    """Paces the simulator against the real clock so the real driver loop and haptics see realistic timing."""

    def __init__(self, scenario_name: str, clock: MonotonicClock, use_imu: bool) -> None:
        from tools.sim.imu_model import ImuModel
        from tools.sim.kinematics import RiderKinematics
        from tools.sim.radar_model import RadarModel
        from tools.sim.scenarios import BY_NAME
        sc = BY_NAME[scenario_name]()
        rider = RiderKinematics(sc.world.road, sc.rider)
        self.radar = RadarModel(sc.world, rider, sc.radar)
        self.imu = ImuModel(rider, sc.imu) if use_imu else None
        self.clock = clock
        self.t0 = clock.now()
        self.k = 0
        self.duration = sc.duration_s
        self.radar_src = QueueRadarSource()
        self.imu_src = QueueImuSource()
        self.done = False

    def pump(self) -> None:
        """Called from the main loop between steps: emit frames whose header time has passed."""
        T = self.radar.T
        while True:
            t_mid = 0.5 * T + self.k * T
            if t_mid > self.duration:
                self.done = True
                return
            t_header = t_mid + T / 2 + self.radar.p.sensor_delay_s
            now_rel = self.clock.now() - self.t0
            if t_header > now_rel:
                return
            raw, truth = self.radar.frame(t_mid)
            self.k += 1
            if self.imu is not None:
                for s in self.imu.samples_until(t_header):
                    self.imu_src.push(type(s)(t=s.t + self.t0, kind=s.kind, values=s.values, seq=s.seq))
            if raw is not None:
                self.radar_src.push(type(raw)(**{**raw.__dict__, "t_header": raw.t_header + self.t0}))


def apply_deploy_settings() -> None:
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(30))
        log.info("SCHED_FIFO 30 applied to the main thread")
    except (PermissionError, OSError) as e:
        log.warning("SCHED_FIFO not applied (%s); continuing without it", e)
    gc.collect()
    gc.freeze()
    gc.disable()
    log.info("gc frozen and disabled on the hot path; collected explicitly on idle ticks")


def git_rev() -> str:
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True, cwd=ROOT, timeout=2).stdout.strip()
    except Exception:  # noqa: BLE001
        return ""


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=str(ROOT / "config"))
    ap.add_argument("--radar", default="kld7", help="kld7 | sim:<scenario>")
    ap.add_argument("--imu", default="bno085", help="bno085 | sim | none")
    ap.add_argument("--no-imu", action="store_true")
    ap.add_argument("--no-haptics", action="store_true", help="drive no GPIO: only log (HAPTICS lines) and record what the motors would do")
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--deploy", action="store_true")
    ap.add_argument("--log-level", default="INFO")
    a = ap.parse_args(argv)
    logging.basicConfig(level=getattr(logging, a.log_level.upper()), format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    cfg = load_config(a.config)
    log_config_warnings(cfg)
    clock = MonotonicClock()

    sim = None
    if a.radar.startswith("sim:"):
        sim = RealTimeSimSources(a.radar.split(":", 1)[1], clock, use_imu=(a.imu == "sim"))
        radar_src = sim.radar_src
        imu_src = sim.imu_src if a.imu == "sim" else QueueImuSource()
    else:
        radar_src = DirectKld7Source(cfg.radar, clock)
        if a.no_imu or a.imu == "none":
            imu_src = QueueImuSource()
        elif a.imu == "sim":
            log.error("--imu sim requires --radar sim:<scenario>")
            return 2
        else:
            imu_src = Bno085SpiSource(cfg.pipeline.imu, clock)
    if a.no_imu or a.imu == "none":
        imu_src.push_event(HealthEvent(clock.now(), HealthBits.IMU_FAULT, True, "main", "IMU disabled by flag"))

    hcfg = haptics_config(cfg.pipeline.haptics)
    if a.no_haptics:
        motor_factory = lambda m: NullMotor(m.name)  # noqa: E731
        log.warning("--no-haptics: no motor is driven; HAPTICS log lines show what they would do")
    else:
        motor_factory = lambda m: BlinkaPwmMotor(m.pin, hcfg.frequency_hz, hcfg.duty_u16)  # noqa: E731
    haptics = HapticOutput(hcfg, clock, motor_factory)
    recorder = None
    if not a.no_record and bool(cfg.pipeline.recording.enabled):
        from goldenfleece.clock import wall_clock_iso
        path = Path(str(cfg.pipeline.recording.dir)) / f"session_{wall_clock_iso().replace(':', '').replace('+', 'p')}.jsonl"
        recorder = RecordingWriter(path, clock, int(cfg.pipeline.recording.queue_depth), float(cfg.pipeline.recording.flush_period_s))
        recorder.start()
        log.info("recording to %s", path)
    power = PowerMonitor(clock, float(cfg.pipeline.power.poll_period_s))
    notifier = SdNotifier()
    runner = Runner(cfg, clock, radar_src, imu_src, haptics, recorder=recorder, power=power, notifier=notifier, git_rev=git_rev(),
                    firmware_version=getattr(radar_src, "firmware_version", ""))

    stop = {"flag": False}

    def _sig(signum, frame):
        log.info("signal %d: stopping", signum)
        stop["flag"] = True
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    power.start()
    radar_src.start()
    imu_src.start()
    haptics.start()              # its watchdog's grace starts now: the loop's heartbeats follow at once
    if haptics.up:
        log.info("haptics: %s at %.0f %% duty", ", ".join(f"{m.name} on {m.pin}" for m in hcfg.motors), hcfg.duty_cycle * 100)
    if a.deploy:
        apply_deploy_settings()
    try:
        runner.announce_restart()
        idle_ticks = 0
        while not stop["flag"]:
            if sim is not None:
                sim.pump()
                if sim.done:
                    break
            processed = runner.step()
            if a.deploy and not processed:
                idle_ticks += 1
                if idle_ticks % 20 == 0:
                    gc.collect(0)
    finally:
        haptics.stop()               # first: every motor off, whatever else fails below
        radar_src.stop()
        imu_src.stop()
        power.stop()
        if recorder is not None:
            recorder.close()
        st = haptics.stats
        log.info("haptics stats: writes=%d write_errors=%d open_failures=%d fallbacks=%d changes=%d", st.writes,
                 st.write_errors, st.open_failures, st.fallbacks, st.changes)
    return 0


if __name__ == "__main__":
    sys.exit(main())

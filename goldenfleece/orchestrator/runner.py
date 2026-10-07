"""Composition root: the single-threaded hot loop.

Per radar frame: decode -> IMU state at t_mid -> ego -> clutter -> tracking -> threat -> policy -> haptics.
Heartbeats and WATCHDOG=1 are emitted FROM THIS LOOP on progress.  On start, PIPELINE_RESTARTING
is handed to the haptics before anything else.  Recording, latency and telemetry are side outputs.
No threads here; adapters own theirs.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

from ..clock import Clock
from ..config import Config
from ..health import HealthTracker
from ..l01_radar_data_input.source import RadarFrameSource
from ..l02_imu_data_input.source import ImuSource
from ..l10_haptics.output import HapticOutput
from ..types import HealthBits, HealthEvent, RadarConfigChanged, WarningCommand
from . import recording as R
from .latency import LatencyStats
from .pipeline import Pipeline
from .power_monitor import THROTTLED_NOW, UNDERVOLTAGE_NOW, FREQ_CAPPED_NOW
from .sdnotify import SdNotifier

log = logging.getLogger("goldenfleece.runner")


@dataclass
class Telemetry:
    """Read-only, non-blocking snapshot for future UIs.  Nothing depends on a consumer existing."""
    t: float = 0.0
    loop_iter: int = 0
    frames_processed: int = 0
    last_frame_number: int = -1
    level: int = 0
    side: str = "NONE"
    health_state: str = "OFFLINE"
    health_bits: List[str] = field(default_factory=list)
    n_tracks: int = 0
    n_detections: int = 0
    ego_valid: bool = False
    ego_speed: float = 0.0
    latency: Dict[str, Dict[str, float]] = field(default_factory=dict)
    haptics: Dict[str, Any] = field(default_factory=dict)


class Runner:
    def __init__(self, cfg: Config, clock: Clock, radar: RadarFrameSource, imu: ImuSource, haptics: HapticOutput,
                 recorder: Optional[R.RecordingWriter] = None, power: Optional[Any] = None, notifier: Optional[SdNotifier] = None,
                 perf_clock: Optional[Clock] = None, firmware_version: str = "", git_rev: str = "") -> None:
        self.cfg = cfg
        self.clock = clock
        self.radar = radar
        self.imu = imu
        self.haptics = haptics
        self.recorder = recorder
        self.power = power
        self.notifier = notifier or SdNotifier(addr="")
        self.perf = perf_clock or clock
        self.health = HealthTracker()
        self.pipe = Pipeline(cfg, self.health, perf_clock=self.perf)
        self.latency = LatencyStats()
        self.telemetry = Telemetry()
        self.firmware_version = firmware_version
        self.git_rev = git_rev
        o = cfg.pipeline.orchestrator
        self.tick_timeout = float(o.tick_timeout_s)
        self.wd_min_period = float(o.watchdog_notify_min_period_s)
        self.lat_period = float(o.latency_report_period_s)
        self.cmd_refresh = float(cfg.pipeline.recording.cmd_refresh_s)
        self.hap_refresh = float(cfg.pipeline.recording.haptics_refresh_s)
        self._last_hap_rec_t = -1e9
        self._last_cmd_rec_t = -1e9
        self._last_cmd_rec: Optional[WarningCommand] = None
        self._last_wd_t = -1e9
        self._last_lat_t = -1e9
        self._last_progress_iter = -1
        self.frames_seen = 0
        self.on_frame_hook: Optional[Callable[[Any], None]] = None

    # -- helpers -------------------------------------------------------------------------------------
    def _record(self, rec: Dict[str, Any]) -> None:
        if self.recorder is not None:
            self.recorder.put(rec)

    def _emit_health_events(self, evs: List[HealthEvent], t_now: float) -> None:
        for e in evs:
            self._record(R.rec_health(e))
            log.info("HEALTH %s %s (%s) %s", e.bit.name, "ON" if e.active else "off", e.source, e.detail)
        if evs:
            self.haptics.send_health(self.health.state)

    def _send_heartbeat(self) -> None:
        """Every iteration: the haptics' watchdog falls back to "warnings offline" when this counter stops."""
        self.haptics.send_heartbeat(self.pipe.loop_iter)

    def _send_warning(self, cmd: WarningCommand, t_now: float) -> None:
        self.haptics.send_warning(cmd)
        last = self._last_cmd_rec
        changed = (last is None or cmd.level != last.level or cmd.side != last.side
                   or cmd.health_state != last.health_state or cmd.health_bits != last.health_bits
                   or cmd.assert_alert != last.assert_alert or cmd.t_arrival_bucket != last.t_arrival_bucket)
        if changed or (t_now - self._last_cmd_rec_t) >= self.cmd_refresh:
            self._last_cmd_rec = cmd
            self._last_cmd_rec_t = t_now
            self._record(R.rec_cmd(cmd))

    def _record_haptics(self, t_now: float) -> None:
        """Every change in what the motors render, and the current state again when there was none for a while
        (so a viewer joining a long session's tail still learns it)."""
        for c in self.haptics.drain_changes():
            self._record(R.rec_haptic(c))
            self._last_hap_rec_t = t_now
            log.info("HAPTICS %s%s%s", c.text, f" [{c.cause}]" if c.cause else "",
                     f" ({c.latency_s * 1e3:.1f} ms after the decision)" if c.latency_s is not None else "")
        if t_now - self._last_hap_rec_t >= self.hap_refresh:
            c = self.haptics.current()
            if c is not None:
                self._record(R.rec_haptic(c, refresh=True))
                self._last_hap_rec_t = t_now

    def _watchdog(self, t_now: float) -> None:
        """WATCHDOG=1 only when the loop made progress since the last notification."""
        if self.pipe.loop_iter != self._last_progress_iter and (t_now - self._last_wd_t) >= self.wd_min_period:
            self.notifier.watchdog()
            self._last_wd_t = t_now
            self._last_progress_iter = self.pipe.loop_iter

    def _ingest_sources(self, t_now: float) -> None:
        for s in self.imu.drain():
            self.pipe.ingest_imu(s)
            self._record(R.rec_imu(s))
        for e in self.imu.drain_events():
            ev = self.health.set(e.bit, e.active, t_now, e.source, e.detail)
            if ev:
                self._emit_health_events([ev], t_now)
        for e in self.radar.drain_events():
            if isinstance(e, RadarConfigChanged):
                self._record(R.rec_rcfg(e))
            else:
                ev = self.health.set(e.bit, e.active, t_now, e.source, e.detail)
                if ev:
                    self._emit_health_events([ev], t_now)
        if self.power is not None:
            for t, flags in self.power.drain():
                self._record(R.rec_power(t, flags))
                evs = []
                e1 = self.health.set(HealthBits.UNDERVOLTAGE, bool(flags & UNDERVOLTAGE_NOW), t_now, "power", f"0x{flags:X}")
                e2 = self.health.set(HealthBits.THROTTLED, bool(flags & (THROTTLED_NOW | FREQ_CAPPED_NOW)), t_now, "power", f"0x{flags:X}")
                evs = [e for e in (e1, e2) if e]
                if evs:
                    self._emit_health_events(evs, t_now)
        for e in self.haptics.drain_events():
            ev = self.health.set(e.bit, e.active, t_now, e.source, e.detail)
            if ev:
                self._emit_health_events([ev], t_now)

    def _update_telemetry(self, t_now: float, res=None, cmd: Optional[WarningCommand] = None) -> None:
        tm = self.telemetry
        tm.t = t_now
        tm.loop_iter = self.pipe.loop_iter
        tm.frames_processed = self.pipe.frames_processed
        tm.last_frame_number = self.pipe.last_frame_number
        c = cmd or (res.command if res is not None else None)
        if c is not None:
            tm.level = int(c.level)
            tm.side = c.side.value
            tm.health_state = c.health_state.name
            tm.health_bits = [b.name for b in HealthBits if b != HealthBits.NONE and c.health_bits & b]
        if res is not None:
            tm.n_tracks = len(res.tracks.tracks)
            tm.n_detections = len(res.radar.detections)
            tm.ego_valid = res.ego.valid
            tm.ego_speed = res.ego.speed
        hs = self.haptics.stats
        tm.haptics = {"up": self.haptics.up, "mode": hs.mode, "render": hs.render, "writes": hs.writes,
                      "write_errors": hs.write_errors, "fallbacks": hs.fallbacks}

    # -- main loop ----------------------------------------------------------------------------------------------
    def announce_restart(self) -> None:
        t_now = self.clock.now()
        ev = self.health.set(HealthBits.PIPELINE_RESTARTING, True, t_now, "orchestrator", "process start")
        # announce BEFORE any decision reaches the haptics
        self.haptics.send_health(self.health.state)
        self._send_heartbeat()
        if ev:
            self._record(R.rec_health(ev))
        self._record(R.rec_header(t_now, self._config_summary(), self.firmware_version, self.git_rev))
        self.notifier.ready()

    def _config_summary(self) -> Dict[str, Any]:
        r = self.cfg.radar
        return {"rrai": r.params["RRAI"], "rspi": r.params["RSPI"], "baud": r.baudrate, "sensor_delay_s": r.sensor_delay_s,
                "sensor_delay_measured": r.sensor_delay_measured, "allow_unmeasured": self.cfg.allow_unmeasured,
                "blind_band_mps": float(self.cfg.pipeline.clutter.doppler_blind_band_mps), "topology": r.topology.value}

    def step(self) -> bool:
        """One loop iteration.  Returns True if a radar frame was processed."""
        frame = self.radar.get(self.tick_timeout)
        t_now = self.clock.now()
        self._ingest_sources(t_now)
        if frame is not None:
            self.frames_seen += 1
            t_p0 = self.perf.now()
            res = self.pipe.process_frame(frame, t_now)
            self._record(R.rec_radar(frame) | {"tn": t_now})
            self._emit_health_events(res.health_events, t_now)
            self._send_warning(res.command, t_now)
            self._send_heartbeat()
            t_handed = self.perf.now()
            self.latency.loop.add(t_handed - t_p0)
            for k, v in res.stage_s.items():
                self.latency.add_stage(k, v)
            self.latency.e2e_pi.add(t_now - res.radar.t_mid + (t_handed - t_p0))
            self.latency.header_to_decision.add(t_now - frame.t_header + (t_handed - t_p0))
            self._update_telemetry(t_now, res=res)
            if self.on_frame_hook is not None:
                self.on_frame_hook(res)
            processed = True
        else:
            cmd, evs = self.pipe.tick(t_now)
            self._emit_health_events(evs, t_now)
            self._send_warning(cmd, t_now)
            self._send_heartbeat()
            self._update_telemetry(t_now, cmd=cmd)
            processed = False
        self._watchdog(t_now)
        if (t_now - self._last_lat_t) >= self.lat_period:
            self._last_lat_t = t_now
            self.telemetry.latency = self.latency.report()
            self._record({"k": "lat", "t": t_now, **{k: v for k, v in self.latency.report().items()}})
            log.info("LATENCY %s", self.latency.log_line())
        self._record_haptics(t_now)
        return processed

    def run(self, should_stop: Callable[[], bool]) -> None:
        self.announce_restart()
        try:
            while not should_stop():
                self.step()
        finally:
            self.notifier.stopping()
            log.info("FINAL LATENCY %s", self.latency.log_line())

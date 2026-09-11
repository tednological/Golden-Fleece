"""Pure composition of l03 -> l09 for one radar frame.

    +X -> behind the rider      +Y -> rider's RIGHT      -Y -> rider's LEFT      +Z -> up

Deterministic: same inputs (raw frames, raw IMU samples, timestamps) -> same
outputs.  Nothing here reads a clock or touches hardware; the runner, the
simulator and the replay tool all drive this object.  Per-stage compute timing
is measured with an optional injected Clock for diagnostics only.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np

from .. import frames as fr
from ..clock import Clock
from ..config import Config
from ..health import HealthTracker
from ..l03_radar_decode.decode import decode
from ..l04_imu_decode.imu_decoder import ImuDecoder
from ..l05_ego_motion.ego import estimate as ego_estimate
from ..l06_clutter_rejection.clutter import SaturationMonitor, StaticRangePhantomFilter, classify
from ..l07_tracking.tracker import Tracker
from ..l08_threat.threat import ThreatAssessor
from ..l09_warning_policy.policy import BlockageDetector, WarningPolicy
from ..types import (ClutterOutput, EgoMotion, HealthBits, HealthEvent, ImuHealth, ImuState, RadarFrame, RawImuSample,
                     RawRadarFrame, ThreatAssessment, TrackerOutput, WarningCommand)

STAGES = ("l03_decode", "l04_imu_state", "l05_ego", "l06_clutter", "l07_tracking", "l08_threat", "l09_policy")


@dataclass
class FrameResult:
    raw: RawRadarFrame
    radar: RadarFrame
    imu: ImuState
    ego: EgoMotion
    clutter: ClutterOutput
    tracks: TrackerOutput
    assessments: List[ThreatAssessment]
    command: WarningCommand
    health_events: List[HealthEvent]
    stage_s: Dict[str, float] = field(default_factory=dict)
    delta_yaw: float = 0.0


class GapMonitor:
    def __init__(self, window_s: float, fraction: float) -> None:
        self.window_s = float(window_s)
        self.fraction = float(fraction)
        self._hist: Deque[Tuple[float, int]] = deque()

    def update(self, t: float, gap: int) -> bool:
        self._hist.append((t, gap))
        while self._hist and self._hist[0][0] < t - self.window_s:
            self._hist.popleft()
        if len(self._hist) < 5:
            return False
        received = len(self._hist)
        missed = sum(g for _, g in self._hist)
        return missed / (received + missed) >= self.fraction


class Pipeline:
    def __init__(self, cfg: Config, health: Optional[HealthTracker] = None, perf_clock: Optional[Clock] = None) -> None:
        self.cfg = cfg
        p = cfg.pipeline
        self.health = health if health is not None else HealthTracker()
        self.perf = perf_clock
        self.imu = ImuDecoder(p.imu, cfg.frames.T_radar_imu)
        self.blind = float(p.clutter.doppler_blind_band_mps)
        self.saturation = SaturationMonitor(p.clutter.saturation_window_s, p.clutter.saturation_fraction)
        pf = p.clutter.phantom_filter
        self.phantom = StaticRangePhantomFilter(pf.window_frames, pf.min_hits, pf.min_rdot_mps, pf.cell_r_m, pf.cell_rdot_mps) if bool(pf.enabled) else None
        self.gaps = GapMonitor(p.radar_health.gap_window_s, p.radar_health.gap_fraction_degraded)
        self.tracker = Tracker(p.tracker, self.blind, cfg.radar.az_clip_rad)
        self.threat = ThreatAssessor(p.threat)
        self.policy = WarningPolicy(p.policy, self.threat.level_from_t)
        self.blockage = BlockageDetector(p.blockage)
        self.prev_ego: Optional[EgoMotion] = None
        self.last_frame_t_mid: Optional[float] = None
        self.last_frame_t_now: Optional[float] = None
        self.frames_processed = 0
        self.loop_iter = 0
        self.last_frame_number: int = -1
        self.silent_after_s = cfg.radar.silent_after_frames * cfg.radar.frame_duration_s[cfg.radar.params["RSPI"]]
        self.last_command: Optional[WarningCommand] = None

    # -- inputs --------------------------------------------------------------------------------------
    def ingest_imu(self, s: RawImuSample) -> None:
        self.imu.ingest(s, self.prev_ego)        # explicit one-step-delayed ego velocity

    def _t(self) -> float:
        return self.perf.now() if self.perf is not None else 0.0

    # -- per frame -----------------------------------------------------------------------------------
    def process_frame(self, raw: RawRadarFrame, t_now: float) -> FrameResult:
        self.loop_iter += 1
        h = self.health
        ev: List[HealthEvent] = []
        timing: Dict[str, float] = {}

        t0 = self._t()
        radar = decode(raw, self.cfg.radar)
        timing["l03_decode"] = self._t() - t0

        t0 = self._t()
        imu = self.imu.state_at(radar.t_mid, t_now=t_now)
        delta_yaw = 0.0
        R_level = None
        if imu.health in (ImuHealth.OK, ImuHealth.ACCEL_GATED_LONG, ImuHealth.STALE):
            if self.last_frame_t_mid is not None:
                delta_yaw = self.imu.delta_yaw(self.last_frame_t_mid, radar.t_mid)
            R_level = fr.q_to_matrix(imu.q_level_radar)
        timing["l04_imu_state"] = self._t() - t0

        t0 = self._t()
        ego = ego_estimate(radar, imu, self.prev_ego, self.cfg.pipeline.ego)
        timing["l05_ego"] = self._t() - t0

        t0 = self._t()
        clutter = classify(radar, ego, self.cfg.pipeline.clutter, self.phantom)
        timing["l06_clutter"] = self._t() - t0

        t0 = self._t()
        tracks = self.tracker.step(clutter, delta_yaw, R_level)
        timing["l07_tracking"] = self._t() - t0

        t0 = self._t()
        assessments = self.threat.assess(tracks.tracks)
        timing["l08_threat"] = self._t() - t0

        # health bits owned by the pure pipeline
        t0 = self._t()
        e = h.set(HealthBits.PDAT_SATURATED, self.saturation.update(radar.t_mid, radar.cap_hit), t_now, "l06",
                  f"cap_hit_rate={self.saturation.cap_hit_rate:.2f}")
        if e: ev.append(e)
        e = h.set(HealthBits.RADAR_GAPS, self.gaps.update(radar.t_mid, radar.gap), t_now, "l01", f"gap={radar.gap}")
        if e: ev.append(e)
        imu_fault = imu.health in (ImuHealth.NO_DATA, ImuHealth.INIT_FAILED, ImuHealth.RESET)
        e = h.set(HealthBits.IMU_FAULT, imu_fault, t_now, "l04", imu.health.name)
        if e: ev.append(e)
        e = h.set(HealthBits.EGO_INVALID, not ego.valid, t_now, "l05", ego.invalid_reason.name)
        if e: ev.append(e)
        blocked = self.blockage.update(radar.t_mid, imu.in_motion and not imu_fault, radar.n_raw, ego.n_inliers)
        e = h.set(HealthBits.RADAR_POSSIBLY_BLOCKED, blocked, t_now, "l09", "")
        if e: ev.append(e)
        e = h.set(HealthBits.RADAR_SILENT, False, t_now, "orchestrator", "frame received")
        if e: ev.append(e)
        e = h.set(HealthBits.PIPELINE_RESTARTING, False, t_now, "orchestrator", "first frame processed")
        if e: ev.append(e)
        cmd = self.policy.step(t_now, assessments, h.state, h.bits)
        timing["l09_policy"] = self._t() - t0

        self.prev_ego = ego
        self.last_frame_t_mid = radar.t_mid
        self.last_frame_t_now = t_now
        self.last_frame_number = radar.frame_number
        self.frames_processed += 1
        self.last_command = cmd
        return FrameResult(raw=raw, radar=radar, imu=imu, ego=ego, clutter=clutter, tracks=tracks, assessments=assessments,
                           command=cmd, health_events=ev, stage_s=timing, delta_yaw=delta_yaw)

    # -- health-only tick (no radar frame) -------------------------------------------------------------
    def tick(self, t_now: float) -> Tuple[WarningCommand, List[HealthEvent]]:
        self.loop_iter += 1
        h = self.health
        ev: List[HealthEvent] = []
        ref = self.last_frame_t_now
        silent = ref is None or (t_now - ref) > self.silent_after_s
        if ref is None and self.health.active_since(HealthBits.PIPELINE_RESTARTING) is not None:
            silent = (t_now - self.health.active_since(HealthBits.PIPELINE_RESTARTING)) > self.silent_after_s
        e = h.set(HealthBits.RADAR_SILENT, silent, t_now, "orchestrator", f"no frame for {self.silent_after_s * 1e3:.0f} ms")
        if e: ev.append(e)
        imu = self.imu.state_at(t_now, t_now=t_now)
        imu_fault = imu.health in (ImuHealth.NO_DATA, ImuHealth.INIT_FAILED, ImuHealth.RESET)
        e = h.set(HealthBits.IMU_FAULT, imu_fault, t_now, "l04", imu.health.name)
        if e: ev.append(e)
        # threat state is kept (no de-escalation because frames stopped); assessments from the last tracks persist
        assessments = self.threat.assess(self.tracker.live_tracks())
        cmd = self.policy.step(t_now, assessments, h.state, h.bits)
        self.last_command = cmd
        return cmd, ev

    def set_health(self, bit: HealthBits, active: bool, t_now: float, source: str, detail: str = "") -> Optional[HealthEvent]:
        return self.health.set(bit, active, t_now, source, detail)

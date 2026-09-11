"""Scenario metrics (task §13.3)."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from goldenfleece.types import HealthBits, ThreatLevel, WarningCommand
from .radar_model import FrameTruth


@dataclass
class VehicleOutcome:
    name: str
    passed: bool
    warned: bool
    lead_time_s: Optional[float]         # truth t_arrival when WARNING+ was first issued
    first_warn_t: Optional[float]
    max_level: int
    side_truth: str
    side_reported: Optional[str]


@dataclass
class ScenarioMetrics:
    name: str
    duration_s: float
    frames: int
    vehicles: List[VehicleOutcome] = field(default_factory=list)
    false_alarm_episodes: int = 0
    false_alarm_frames: int = 0
    false_alarms_per_hour: float = 0.0
    flicker_max: int = 0
    level_changes: int = 0
    cap_hit_rate: float = 0.0
    ego_valid_fraction: float = 0.0
    ego_speed_rmse: Optional[float] = None
    ego_reasons: Dict[str, int] = field(default_factory=dict)
    psi_travel_err_mean_deg: Optional[float] = None
    psi_travel_err_std_deg: Optional[float] = None
    psi_travel_n: int = 0
    blockage_false_alarm_frames: int = 0
    blockage_detect_latency_s: Optional[float] = None
    health_onset_latency_s: Dict[str, Optional[float]] = field(default_factory=dict)
    stage_p50_ms: Dict[str, float] = field(default_factory=dict)
    stage_p99_ms: Dict[str, float] = field(default_factory=dict)
    e2e_p50_ms: float = 0.0
    e2e_p99_ms: float = 0.0
    e2e_max_ms: float = 0.0
    notes: str = ""
    warnings_expected: bool = True

    def missed(self) -> int:
        return sum(1 for v in self.vehicles if v.passed and not v.warned)

    def summary_line(self) -> str:
        leads = [f"{v.name}:{v.lead_time_s:.1f}s" if v.lead_time_s is not None else f"{v.name}:MISSED" for v in self.vehicles if v.passed]
        return (f"{self.name:32s} lead=[{', '.join(leads) or '-'}] missed={self.missed()} FA/h={self.false_alarms_per_hour:6.1f} "
                f"flicker={self.flicker_max} cap={self.cap_hit_rate:.2f} ego_valid={self.ego_valid_fraction:.2f} "
                f"e2e_p99={self.e2e_p99_ms:.1f}ms")


class MetricsCollector:
    def __init__(self, name: str, notes: str = "", expect_warning: bool = True) -> None:
        self.m = ScenarioMetrics(name=name, duration_s=0.0, frames=0, notes=notes, warnings_expected=expect_warning)
        self._veh: Dict[str, dict] = {}
        self._fa_in_episode = False
        self._levels: List[int] = []
        self._ego_valid = 0
        self._ego_err2: List[float] = []
        self._psi_err: List[float] = []
        self._cap = 0
        self._block_fa = 0
        self._block_detect_t: Optional[float] = None
        self._health_seen: Dict[str, float] = {}
        self._stage: Dict[str, List[float]] = {}
        self._e2e: List[float] = []
        self._last_level = 0

    def frame(self, truth: FrameTruth, cmd: Optional[WarningCommand], ego_valid: bool, ego_speed: float, psi_travel: Optional[float],
              cap_hit: bool, blocked_flag: bool, stage_s: Dict[str, float], e2e_s: Optional[float], ego_reason: str = "") -> None:
        self.m.frames += 1
        if ego_reason:
            self.m.ego_reasons[ego_reason] = self.m.ego_reasons.get(ego_reason, 0) + 1
        self.m.duration_s = truth.t_mid
        level = int(cmd.level) if cmd is not None else 0
        self._levels.append(level)
        if level != self._last_level:
            self.m.level_changes += 1
            self._last_level = level
        if cmd is not None:
            self.m.flicker_max = max(self.m.flicker_max, cmd.flicker_count)
        closing_any = False
        recently_passed = any(0.0 <= v.s_rel <= 3.0 * max(v.closing_speed, 0.5) for v in truth.vehicles)   # grace after a pass
        for v in truth.vehicles:
            rec = self._veh.setdefault(v.name, {"passed": False, "warned": False, "lead": None, "first": None, "max": 0,
                                                "side_truth": v.side, "side_rep": None})
            rec["side_truth"] = v.side if v.s_rel < 0 else rec["side_truth"]
            if v.s_rel >= 0:
                rec["passed"] = True
            closing_here = v.t_arrival is not None and v.t_arrival < 8.0 and v.r < 45.0
            closing_any |= closing_here
            if closing_here and level >= int(ThreatLevel.WARNING):
                rec["max"] = max(rec["max"], level)
                if not rec["warned"]:
                    rec["warned"] = True
                    rec["lead"] = v.t_arrival
                    rec["first"] = truth.t_mid
                    rec["side_rep"] = cmd.side.value if cmd is not None else None
        if level >= int(ThreatLevel.WARNING) and not closing_any and not recently_passed:
            self.m.false_alarm_frames += 1
            if not self._fa_in_episode:
                self.m.false_alarm_episodes += 1
                self._fa_in_episode = True
        else:
            self._fa_in_episode = False
        if ego_valid:
            self._ego_valid += 1
            self._ego_err2.append((ego_speed - truth.rider.speed) ** 2)
        if psi_travel is not None and ego_valid:
            err = psi_travel - (-truth.rider.torso_yaw)
            self._psi_err.append(math.degrees(math.atan2(math.sin(err), math.cos(err))))
        self._cap += int(cap_hit)
        if blocked_flag and not truth.blocked:
            self._block_fa += 1
        if blocked_flag and truth.blocked and self._block_detect_t is None:
            self._block_detect_t = truth.t_mid
        for k, v in stage_s.items():
            self._stage.setdefault(k, []).append(v)
        if e2e_s is not None:
            self._e2e.append(e2e_s)

    def health_event(self, bit_name: str, t: float) -> None:
        self._health_seen.setdefault(bit_name, t)

    def finish(self, fault_onsets: Dict[str, float], blockage_t: Optional[float]) -> ScenarioMetrics:
        m = self.m
        for name, rec in self._veh.items():
            m.vehicles.append(VehicleOutcome(name, rec["passed"], rec["warned"], rec["lead"], rec["first"], rec["max"], rec["side_truth"], rec["side_rep"]))
        hours = max(m.duration_s, 1e-6) / 3600.0
        m.false_alarms_per_hour = m.false_alarm_episodes / hours
        m.cap_hit_rate = self._cap / max(m.frames, 1)
        m.ego_valid_fraction = self._ego_valid / max(m.frames, 1)
        if self._ego_err2:
            m.ego_speed_rmse = math.sqrt(sum(self._ego_err2) / len(self._ego_err2))
        if self._psi_err:
            n = len(self._psi_err)
            mean = sum(self._psi_err) / n
            m.psi_travel_err_mean_deg = mean
            m.psi_travel_err_std_deg = math.sqrt(sum((e - mean) ** 2 for e in self._psi_err) / n)
            m.psi_travel_n = n
        m.blockage_false_alarm_frames = self._block_fa
        if blockage_t is not None and self._block_detect_t is not None:
            m.blockage_detect_latency_s = self._block_detect_t - blockage_t
        for bit, onset in fault_onsets.items():
            seen = self._health_seen.get(bit)
            m.health_onset_latency_s[bit] = (seen - onset) if seen is not None else None

        def pct(xs, p):
            if not xs:
                return 0.0
            s = sorted(xs)
            return s[min(len(s) - 1, int(p * len(s)))]
        for k, xs in self._stage.items():
            m.stage_p50_ms[k] = pct(xs, 0.5) * 1e3
            m.stage_p99_ms[k] = pct(xs, 0.99) * 1e3
        m.e2e_p50_ms = pct(self._e2e, 0.5) * 1e3
        m.e2e_p99_ms = pct(self._e2e, 0.99) * 1e3
        m.e2e_max_ms = (max(self._e2e) * 1e3) if self._e2e else 0.0
        return m

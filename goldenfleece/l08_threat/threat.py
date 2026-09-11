"""l08: per-track threat assessment.

    +X -> behind the rider      +Y -> rider's RIGHT      -Y -> rider's LEFT      +Z -> up

Honest semantics: t_arrival = range / closing speed is the time to arrival at the
rider's LONGITUDINAL position.  It is not a time to collision; lateral offset is
not observable to anything better than "roughly left or right".
Side is coarse (LEFT | CENTER | RIGHT) with a CENTER band that widens with range
to reflect the +/-15 deg torso-yaw floor.  Confidence decreases with range.
"""
from __future__ import annotations

import math
from typing import Dict, List, Sequence

from ..config import Section
from ..types import CoastReason, Side, StageCounters, ThreatAssessment, ThreatLevel, Track, TrackStatus

STAGE = "l08_threat"


class ThreatAssessor:
    def __init__(self, tcfg: Section) -> None:
        self.k = float(tcfg.k_sigma)
        self.t_alert = float(tcfg.t_alert_s)
        self.t_warning = float(tcfg.t_warning_s)
        self.t_advisory = float(tcfg.t_advisory_s)
        self.r_close = float(tcfg.r_close_m)
        self.prox_level = ThreatLevel(int(tcfg.proximity_level))
        self.c0 = float(tcfg.side_center_halfwidth_m)
        self.yaw_sigma = float(tcfg.side_yaw_sigma_rad)
        self.r_conf = float(tcfg.confidence_r_max_m)
        self.coast_conf = float(tcfg.coasting_confidence_factor)
        self.hold_unexplained = bool(tcfg.unexplained_coast_holds_level)
        self._last_level: Dict[int, ThreatLevel] = {}
        self.counters = StageCounters(STAGE, 0, 0, {})

    def level_from_t(self, t_low: float, factor: float = 1.0) -> ThreatLevel:
        if t_low < self.t_alert * factor:
            return ThreatLevel.ALERT
        if t_low < self.t_warning * factor:
            return ThreatLevel.WARNING
        if t_low < self.t_advisory * factor:
            return ThreatLevel.ADVISORY
        return ThreatLevel.NONE

    def side_of(self, r: float, az: float) -> Side:
        y = r * math.sin(az)
        half = self.c0 + r * math.tan(self.yaw_sigma)
        if y < -half:
            return Side.LEFT
        if y > half:
            return Side.RIGHT
        return Side.CENTER

    def assess(self, tracks: Sequence[Track]) -> List[ThreatAssessment]:
        out: List[ThreatAssessment] = []
        rej = {"not_confirmed": 0, "not_closing": 0, "kinematically_inconsistent": 0}
        live_ids = set()
        for tk in tracks:
            if tk.status not in (TrackStatus.CONFIRMED, TrackStatus.COASTING):
                rej["not_confirmed"] += 1
                continue
            live_ids.add(tk.id)
            if not tk.kinematic_consistent:
                rej["kinematically_inconsistent"] += 1
                continue
            v_close = tk.v_closing
            if not tk.closing or v_close <= 0.0:
                rej["not_closing"] += 1
                self._last_level[tk.id] = ThreatLevel.NONE
                continue
            r = max(tk.r, 0.0)
            t_arr = r / v_close if v_close > 1e-3 else float("inf")
            num = max(r - self.k * tk.sigma_r, 0.0)
            den = v_close + self.k * tk.sigma_r_dot
            t_low = num / den if den > 1e-3 else float("inf")
            level = self.level_from_t(t_low)
            prox = False
            if r < self.r_close:
                prox = True
                level = max(level, self.prox_level)
            coasting = tk.status is TrackStatus.COASTING
            if coasting and tk.coast_reason is CoastReason.UNEXPLAINED and self.hold_unexplained:
                held = self._last_level.get(tk.id, level)
                level = min(level, held) if held is not ThreatLevel.NONE else level
            if not coasting:
                self._last_level[tk.id] = level
            conf = max(0.3, 1.0 - 0.7 * r / self.r_conf)
            if coasting:
                conf *= self.coast_conf
            out.append(ThreatAssessment(track_id=tk.id, level=level, side=self.side_of(r, tk.az), t_arrival=t_arr,
                                        t_arrival_low=t_low, r=r, v_closing=v_close, confidence=conf, coasting=coasting,
                                        coast_reason=tk.coast_reason, proximity_triggered=prox))
        for tid in list(self._last_level):
            if tid not in live_ids:
                self._last_level.pop(tid, None)
        self.counters = StageCounters(STAGE, n_in=len(tracks), n_out=len(out), rejected=rej)
        return out

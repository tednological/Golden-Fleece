"""l09: arbitration, hysteresis, health channel -> WarningCommand.

Escalate immediately.  De-escalate only after a dwell plus hysteresis, and never
because a track vanished: assessments end only when a coast resolves (l07).
Health is a separate channel from threat.  The alert channel is asserted for the
highest threat level only and never for a fault.
"""
from __future__ import annotations

from collections import deque
from typing import Deque, List, Optional, Sequence

from ..config import Section
from ..types import HealthBits, HealthState, Side, TArrivalBucket, ThreatAssessment, ThreatLevel, WarningCommand


def bucket_of(t_low: float) -> TArrivalBucket:
    if t_low == float("inf"):
        return TArrivalBucket.NONE
    if t_low < 1.5:
        return TArrivalBucket.LT_1P5
    if t_low < 3.0:
        return TArrivalBucket.S1P5_3
    if t_low < 6.0:
        return TArrivalBucket.S3_6
    return TArrivalBucket.GT_6S


class WarningPolicy:
    def __init__(self, pcfg: Section, level_from_t) -> None:
        self.dwell = float(pcfg.dwell_s)
        self.hyst = float(pcfg.hysteresis_factor)
        self.min_on = float(pcfg.min_on_time_s)
        self.min_on_level = ThreatLevel(int(pcfg.min_on_level))
        self.alert_level = ThreatLevel(int(pcfg.alert_level))
        self._level_from_t = level_from_t         # l08's threshold function, so hysteresis uses the same thresholds
        self.level = ThreatLevel.NONE
        self.side = Side.NONE
        self.level_since = 0.0
        self.pending_lower: Optional[ThreatLevel] = None
        self.pending_since = 0.0
        self.seq = 0
        self._changes: Deque[float] = deque()
        self.flicker_window_s = 60.0
        self._last_deesc_t: Optional[float] = None
        self.flicker_reescalate_s = 3.0
        self.flicker_count = 0            # re-escalations within flicker_reescalate_s of a de-escalation
        self.n_escalations = 0
        self.n_deescalations = 0
        self.n_held_by_dwell = 0

    def _flicker(self, t: float) -> int:
        while self._changes and self._changes[0] < t - self.flicker_window_s:
            self._changes.popleft()
        return len(self._changes)

    def step(self, t: float, assessments: Sequence[ThreatAssessment], health_state: HealthState, health_bits: HealthBits) -> WarningCommand:
        target = ThreatLevel.NONE
        dominant: Optional[ThreatAssessment] = None
        for a in assessments:
            if a.level > target or (a.level == target and dominant is not None and a.t_arrival_low < dominant.t_arrival_low):
                target = a.level
                dominant = a
        side = Side.NONE
        if dominant is not None and target > ThreatLevel.NONE:
            side = dominant.side
            sides = {a.side for a in assessments if a.level == target}
            if Side.LEFT in sides and Side.RIGHT in sides:
                side = Side.BOTH
        if target > self.level:
            self.level = target
            self.level_since = t
            self.pending_lower = None
            self.n_escalations += 1
            self._changes.append(t)
            if self._last_deesc_t is not None and (t - self._last_deesc_t) < self.flicker_reescalate_s:
                self.flicker_count += 1
        elif target < self.level:
            if self.pending_lower is None:
                self.pending_since = t          # dwell starts when the target first drops below the current level
            self.pending_lower = target
            hold = False
            if self.level >= self.min_on_level and (t - self.level_since) < self.min_on:
                hold = True
            if (t - self.pending_since) < self.dwell:
                hold = True
            if dominant is not None and self._level_from_t(dominant.t_arrival_low, self.hyst) >= self.level:
                hold = True
            if hold:
                self.n_held_by_dwell += 1
            else:
                self.level = target
                self.level_since = t
                self.pending_lower = None
                self.n_deescalations += 1
                self._changes.append(t)
                self._last_deesc_t = t
        else:
            self.pending_lower = None
        if self.level > ThreatLevel.NONE:
            if side is not Side.NONE:
                self.side = side
        else:
            self.side = Side.NONE
        t_low = dominant.t_arrival_low if (dominant is not None and self.level > ThreatLevel.NONE) else float("inf")
        self.seq += 1
        return WarningCommand(seq=self.seq, t_decided=t, level=self.level, side=self.side, t_arrival_bucket=bucket_of(t_low),
                              health_state=health_state, health_bits=health_bits,
                              assert_alert=(self.level == self.alert_level),
                              dominant_track_id=dominant.track_id if dominant is not None else None,
                              flicker_count=self.flicker_count)


class BlockageDetector:
    """RADAR_POSSIBLY_BLOCKED: over a window of min_silent_s the rider was in motion throughout,
    there were no ego-motion inliers at all, and raw targets were (almost) absent -- a small mean
    is tolerated because a covered sensor still produces occasional noise false alarms.  A covered
    radar keeps delivering valid, empty frames, so the heartbeat stays honest while the system is
    blind (task §10.2).  Advisory severity: open roads with little clutter will trigger it."""

    def __init__(self, bcfg: Section) -> None:
        self.enabled = bool(bcfg.enabled)
        self.min_silent = float(bcfg.min_silent_s)
        self.max_mean_targets = float(bcfg.max_mean_targets_per_frame)
        self._hist: Deque[tuple] = deque()
        self.n_raised = 0

    def update(self, t: float, in_motion: bool, n_raw_targets: int, n_ego_inliers: int) -> bool:
        if not self.enabled:
            return False
        self._hist.append((t, in_motion, n_raw_targets, n_ego_inliers))
        while self._hist and self._hist[0][0] < t - self.min_silent:
            self._hist.popleft()
        if len(self._hist) < 10 or (t - self._hist[0][0]) < self.min_silent * 0.95:
            return False
        if not all(h[1] for h in self._hist):
            return False
        if any(h[3] > 0 for h in self._hist):
            return False
        mean_targets = sum(h[2] for h in self._hist) / len(self._hist)
        if mean_targets <= self.max_mean_targets:
            self.n_raised += 1
            return True
        return False

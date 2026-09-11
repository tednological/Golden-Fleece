"""l06: classify detections; drop stationary clutter.

    +X -> behind the rider      +Y -> rider's RIGHT      -Y -> rider's LEFT      +Z -> up

Physical fact: with a rear-facing radar and a rider moving forward (or stopped),
stationary objects can only recede.  An approaching detection is always a moving
object.  Invariant: approaching detections pass regardless of EgoMotion.valid.
Ego-motion only classifies receding detections.
"""
from __future__ import annotations

import math
from collections import deque
from typing import Deque, List, Optional, Tuple

from ..config import Section
from ..types import ClassifiedDetection, ClutterOutput, DetectionClass, EgoMotion, RadarFrame, StageCounters

STAGE = "l06_clutter_rejection"


class SaturationMonitor:
    """Fraction of frames hitting the 12-target cap over a sliding window (pure, deterministic)."""

    def __init__(self, window_s: float, fraction: float) -> None:
        self.window_s = float(window_s)
        self.fraction = float(fraction)
        self._hist: Deque[Tuple[float, bool]] = deque()
        self.n_cap_frames = 0
        self.n_frames = 0

    def update(self, t: float, cap_hit: bool) -> bool:
        self._hist.append((t, cap_hit))
        self.n_frames += 1
        self.n_cap_frames += int(cap_hit)
        while self._hist and self._hist[0][0] < t - self.window_s:
            self._hist.popleft()
        if len(self._hist) < 5:
            return False
        frac = sum(1 for _, c in self._hist if c) / len(self._hist)
        return frac >= self.fraction

    @property
    def cap_hit_rate(self) -> float:
        return self.n_cap_frames / self.n_frames if self.n_frames else 0.0


def alias_suspect(r: float, magnitude_db: float, c_alias_db: float) -> bool:
    """Range-aliased returns (targets beyond the RRAI range wrapping to a short range) are far too weak
    for their apparent range.  Reject if magnitude < c_alias - 40 log10(r).  Only bites at short range.
    c_alias depends on the sensor's magnitude scale: UNVALIDATED until the probe records magnitude vs range."""
    if r <= 0.0:
        return False
    return magnitude_db < c_alias_db - 40.0 * math.log10(r)


def classify(frame: RadarFrame, ego: Optional[EgoMotion], ccfg: Section) -> ClutterOutput:
    blind = float(ccfg.doppler_blind_band_mps)
    resid_thresh = float(ccfg.stationary_resid_mps)
    pass_receding = bool(ccfg.pass_receding_movers)
    alias_on = bool(ccfg.alias_check.enabled)
    c_alias = float(ccfg.alias_check.c_alias_db)
    ego_valid = ego is not None and ego.valid
    kept: List[ClassifiedDetection] = []
    rej = {"stationary": 0, "receding_mover_dropped": 0, "receding_unclassified_dropped": 0, "alias_suspect": 0}
    n_app = n_rm = n_ru = 0
    for d in frame.detections:
        if alias_on and alias_suspect(d.r, d.magnitude_db, c_alias):
            rej["alias_suspect"] += 1
            continue
        if d.v_closing > 0.0:
            cls = DetectionClass.APPROACHING          # always kept, ego-motion or not
            n_app += 1
            kept.append(ClassifiedDetection(d, cls, ego_valid))
            continue
        if ego_valid:
            pred = -(ego.v_s[0] * math.cos(d.az) + ego.v_s[1] * math.sin(d.az))
            if abs(d.v_radial - pred) < max(resid_thresh, blind):
                rej["stationary"] += 1
                continue
            cls = DetectionClass.RECEDING_MOVER
            n_rm += 1
        else:
            cls = DetectionClass.RECEDING_UNCLASSIFIED
            n_ru += 1
        if pass_receding:
            kept.append(ClassifiedDetection(d, cls, ego_valid))
        elif cls is DetectionClass.RECEDING_MOVER:
            rej["receding_mover_dropped"] += 1
        else:
            rej["receding_unclassified_dropped"] += 1
    counters = StageCounters(STAGE, n_in=len(frame.detections), n_out=len(kept),
                             rejected={**rej, "approaching_kept": n_app, "receding_mover": n_rm, "receding_unclassified": n_ru})
    return ClutterOutput(t_mid=frame.t_mid, frame_number=frame.frame_number, kept=tuple(kept), cap_hit=frame.cap_hit,
                         counters=counters)

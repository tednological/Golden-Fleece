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


class StaticRangePhantomFilter:
    """A return that claims |r_dot| > min_rdot but keeps appearing in the same range cell frame after
    frame is not a moving object (wheel micro-Doppler, spinning parts): a genuine 3 m/s closer leaves a
    1 m cell within 0.35 s.  Pure and deterministic; keyed by (range cell, r_dot cell)."""

    def __init__(self, window_frames: int, min_hits: int, min_rdot_mps: float, cell_r_m: float, cell_rdot_mps: float) -> None:
        self.window = int(window_frames)
        self.min_hits = int(min_hits)
        self.min_rdot = float(min_rdot_mps)
        self.cell_r = float(cell_r_m)
        self.cell_rd = float(cell_rdot_mps)
        # two range grids offset by half a cell: a jittering return sits inside one cell of at least one grid,
        # while a genuine |r_dot| >= min_rdot target leaves any cell of either grid within the window
        self._hist: List[dict] = [{}, {}]      # key -> deque of frame indices
        self._frame = 0
        self.n_dropped = 0

    def _keys(self, r: float, rd: float) -> Tuple[Tuple[int, int], Tuple[int, int]]:
        kd = int(math.floor(rd / self.cell_rd))
        return ((int(math.floor(r / self.cell_r)), kd), (int(math.floor((r + 0.5 * self.cell_r) / self.cell_r)), kd))

    def step(self, dets) -> List[bool]:
        """Returns a mask: True = phantom (drop).  Call once per frame with all detections."""
        self._frame += 1
        f = self._frame
        keys = [self._keys(d.r, d.v_radial) if abs(d.v_radial) >= self.min_rdot else None for d in dets]
        mask = []
        for kk in keys:
            if kk is None:
                mask.append(False)
                continue
            best = 0
            for g, k in enumerate(kk):
                dq = self._hist[g].get(k)
                if dq:
                    best = max(best, sum(1 for fi in dq if fi > f - self.window))
            mask.append(best >= self.min_hits)
        for g in (0, 1):
            for k in set(kk[g] for kk in keys if kk is not None):
                dq = self._hist[g].setdefault(k, deque(maxlen=self.window))
                dq.append(f)
        if f % 200 == 0:   # prune stale keys
            for g in (0, 1):
                for k in [k for k, dq in self._hist[g].items() if not dq or dq[-1] <= f - self.window]:
                    del self._hist[g][k]
        self.n_dropped += sum(mask)
        return mask


def alias_suspect(r: float, magnitude_db: float, c_alias_db: float) -> bool:
    """Range-aliased returns (targets beyond the RRAI range wrapping to a short range) are far too weak
    for their apparent range.  Reject if magnitude < c_alias - 40 log10(r).  Only bites at short range.
    c_alias depends on the sensor's magnitude scale: UNVALIDATED until the probe records magnitude vs range."""
    if r <= 0.0:
        return False
    return magnitude_db < c_alias_db - 40.0 * math.log10(r)


def classify(frame: RadarFrame, ego: Optional[EgoMotion], ccfg: Section,
             phantom: Optional[StaticRangePhantomFilter] = None) -> ClutterOutput:
    blind = float(ccfg.doppler_blind_band_mps)
    resid_thresh = float(ccfg.stationary_resid_mps)
    pass_receding = bool(ccfg.pass_receding_movers)
    alias_on = bool(ccfg.alias_check.enabled)
    c_alias = float(ccfg.alias_check.c_alias_db)
    ego_valid = ego is not None and ego.valid
    kept: List[ClassifiedDetection] = []
    rej = {"stationary": 0, "receding_mover_dropped": 0, "receding_unclassified_dropped": 0, "alias_suspect": 0, "static_range_phantom": 0}
    n_app = n_rm = n_ru = 0
    phantom_mask = phantom.step(frame.detections) if phantom is not None else [False] * len(frame.detections)
    for d, is_phantom in zip(frame.detections, phantom_mask):
        if is_phantom:
            rej["static_range_phantom"] += 1
            continue
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

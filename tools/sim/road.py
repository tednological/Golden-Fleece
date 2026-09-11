"""Road geometry for the synthetic world.

The simulator's internal coordinates (``sim_world``: x forward along the initial
road direction, y to the LEFT, z up) never leave tools/sim.  The pipeline only
sees wire-level radar frames and raw IMU samples.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class RoadSegment:
    length_m: float
    curvature: float          # 1/m, positive = left turn


class Road:
    """Piecewise-constant-curvature road.  pose(s) -> (x, y, heading)."""

    def __init__(self, segments: Sequence[RoadSegment] | None = None) -> None:
        self.segments: List[RoadSegment] = list(segments) if segments else [RoadSegment(1e6, 0.0)]
        self._s0: List[float] = []
        self._pose0: List[Tuple[float, float, float]] = []
        s, x, y, th = 0.0, 0.0, 0.0, 0.0
        for seg in self.segments:
            self._s0.append(s)
            self._pose0.append((x, y, th))
            x, y, th = self._advance(x, y, th, seg.curvature, seg.length_m)
            s += seg.length_m
        self.total_length = s

    @staticmethod
    def _advance(x, y, th, k, ds):
        if abs(k) < 1e-9:
            return x + ds * math.cos(th), y + ds * math.sin(th), th
        th2 = th + k * ds
        x2 = x + (math.sin(th2) - math.sin(th)) / k
        y2 = y - (math.cos(th2) - math.cos(th)) / k
        return x2, y2, th2

    def pose(self, s: float) -> Tuple[float, float, float]:
        if s < 0:
            x, y, th = self._pose0[0]
            return x + s * math.cos(th), y + s * math.sin(th), th
        i = 0
        for j, s0 in enumerate(self._s0):
            if s >= s0:
                i = j
        seg = self.segments[i]
        x, y, th = self._pose0[i]
        ds = min(s - self._s0[i], seg.length_m) if i < len(self.segments) - 1 else s - self._s0[i]
        return self._advance(x, y, th, seg.curvature, ds)

    def curvature(self, s: float) -> float:
        if s < 0:
            return 0.0
        i = 0
        for j, s0 in enumerate(self._s0):
            if s >= s0:
                i = j
        return self.segments[i].curvature

    def world_point(self, s: float, d: float, z: float = 0.0) -> np.ndarray:
        """Point at arc length s, lateral offset d (positive = LEFT of travel), height z."""
        x, y, th = self.pose(s)
        return np.array([x - d * math.sin(th), y + d * math.cos(th), z])

    def heading(self, s: float) -> float:
        return self.pose(s)[2]

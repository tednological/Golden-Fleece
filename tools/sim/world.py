"""Vehicles and roadside scatterers."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .road import Road


@dataclass(frozen=True)
class ScatterPoint:
    """A point reflector attached to a body.  Offsets in the body frame (x fwd, y left, z up)."""
    dx: float
    dy: float
    dz: float
    rcs_m2: float
    velocity_scale: float = 1.0     # 0 = wheel contact patch (stationary), 2 = wheel top


CAR_POINTS: Tuple[ScatterPoint, ...] = (
    ScatterPoint(0.0, 0.0, 0.5, 10.0),          # rear bumper / plate
    ScatterPoint(0.5, 0.8, 0.35, 1.5),          # rear-left wheel arch
    ScatterPoint(0.5, -0.8, 0.35, 1.5),         # rear-right wheel arch
    ScatterPoint(1.0, 0.0, 1.4, 3.0),           # roof rear edge
    ScatterPoint(0.5, 0.8, 0.6, 0.3, 2.0),      # wheel top micro-Doppler
    ScatterPoint(0.5, -0.8, 0.6, 0.3, 2.0),
    ScatterPoint(0.5, 0.8, 0.05, 0.2, 0.0),     # contact patches (stationary)
    ScatterPoint(0.5, -0.8, 0.05, 0.2, 0.0),
)

BIKE_POINTS: Tuple[ScatterPoint, ...] = (ScatterPoint(0.0, 0.0, 0.9, 0.5),)


@dataclass
class Vehicle:
    name: str
    s0: float                                   # arc length at t=0 (negative = behind the rider start)
    speed_mps: float
    lateral_m: float                            # positive = rider's LEFT (road-left)
    points: Tuple[ScatterPoint, ...] = CAR_POINTS
    speed_keypoints: Optional[Sequence[Tuple[float, float]]] = None    # optional (t, v) profile
    t_appear: float = -1e9
    t_disappear: float = 1e9

    def speed(self, t: float) -> float:
        if not self.speed_keypoints:
            return self.speed_mps
        kp = list(self.speed_keypoints)
        if t <= kp[0][0]:
            return kp[0][1]
        for (t0, v0), (t1, v1) in zip(kp[:-1], kp[1:]):
            if t0 <= t <= t1:
                u = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
                return v0 + u * (v1 - v0)
        return kp[-1][1]

    def arc_length(self, t: float) -> float:
        if not self.speed_keypoints:
            return self.s0 + self.speed_mps * t
        # integrate piecewise-linear speed exactly
        s = self.s0
        kp = list(self.speed_keypoints)
        tt = 0.0
        # from 0 to t
        segs = [(kp[0][0], kp[0][1])] + list(kp) + [(1e9, kp[-1][1])]
        prev_t, prev_v = 0.0, self.speed(0.0)
        for (t1, v1) in segs:
            if t1 <= 0:
                continue
            te = min(t1, t)
            if te > prev_t:
                ve = self.speed(te)
                s += 0.5 * (prev_v + ve) * (te - prev_t)
                prev_t, prev_v = te, ve
            if t1 >= t:
                break
        return s

    def active(self, t: float) -> bool:
        return self.t_appear <= t <= self.t_disappear


@dataclass(frozen=True)
class Scatterer:
    s: float
    lateral_m: float
    z: float
    rcs_m2: float


@dataclass
class World:
    road: Road
    vehicles: List[Vehicle] = field(default_factory=list)
    scatterers: List[Scatterer] = field(default_factory=list)

    @staticmethod
    def roadside_clutter(rng: np.random.Generator, s_min: float, s_max: float, density_per_m: float,
                         lateral_range=(2.5, 8.0), rcs_range=(0.3, 12.0)) -> List[Scatterer]:
        n = rng.poisson(max(0.0, density_per_m * (s_max - s_min)))
        out = []
        for _ in range(n):
            side = 1.0 if rng.random() < 0.5 else -1.0
            out.append(Scatterer(
                s=float(rng.uniform(s_min, s_max)),
                lateral_m=float(side * rng.uniform(*lateral_range)),
                z=float(rng.uniform(0.2, 1.6)),
                rcs_m2=float(10 ** rng.uniform(math.log10(rcs_range[0]), math.log10(rcs_range[1]))),
            ))
        return out

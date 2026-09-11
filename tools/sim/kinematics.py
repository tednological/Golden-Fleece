"""Rider kinematics: speed profile, torso yaw, lean, and the radar-frame pose.

One truth drives both sensors: the radar model and the IMU model both call
``RiderKinematics.state(t)``.

sim_world axes: x forward (initial road direction), y LEFT, z up.
Radar frame axes (task §5.2):  +X rearward, +Y rider's RIGHT, +Z up.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

import numpy as np

from .road import Road

G = 9.80665
RADAR_HEIGHT_M = 1.2


@dataclass(frozen=True)
class ShoulderCheck:
    t_start: float
    duration_s: float
    amplitude_rad: float        # positive = rider looks over the LEFT shoulder (torso yaws left)


@dataclass
class RiderProfile:
    speed_keypoints: Sequence[Tuple[float, float]] = ((0.0, 5.0),)   # (t, m/s), piecewise linear
    torso_yaw_amp_rad: float = 0.0
    torso_yaw_period_s: float = 4.0
    torso_yaw_phase: float = 0.0
    torso_yaw_bias_rad: float = 0.0       # static vest-to-travel yaw offset (unobservable to the pipeline)
    shoulder_checks: List[ShoulderCheck] = field(default_factory=list)
    lean_from_curvature: bool = True
    vibration_rms_mps2: float = 0.3       # road buzz when moving

    def speed(self, t: float) -> float:
        kp = list(self.speed_keypoints)
        if t <= kp[0][0]:
            return kp[0][1]
        for (t0, v0), (t1, v1) in zip(kp[:-1], kp[1:]):
            if t0 <= t <= t1:
                u = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
                return v0 + u * (v1 - v0)
        return kp[-1][1]

    def torso_yaw(self, t: float) -> float:
        psi = self.torso_yaw_bias_rad
        if self.torso_yaw_amp_rad:
            psi += self.torso_yaw_amp_rad * math.sin(2 * math.pi * t / self.torso_yaw_period_s + self.torso_yaw_phase)
        for sc in self.shoulder_checks:
            if sc.t_start <= t <= sc.t_start + sc.duration_s:
                u = (t - sc.t_start) / sc.duration_s
                psi += sc.amplitude_rad * 0.5 * (1 - math.cos(2 * math.pi * u))   # raised cosine pulse
        return psi


@dataclass(frozen=True)
class RiderState:
    t: float
    s: float                    # arc length along the road
    speed: float
    accel_along: float
    heading: float              # road heading at s
    torso_yaw: float
    lean: float                 # positive = leaning LEFT
    p_w: np.ndarray             # radar origin in sim_world
    v_w: np.ndarray             # rider velocity in sim_world
    a_w: np.ndarray             # rider acceleration in sim_world (incl. centripetal)
    R_w_r: np.ndarray           # radar axes expressed in sim_world (columns)

    @property
    def R_r_w(self) -> np.ndarray:
        return self.R_w_r.T


class RiderKinematics:
    def __init__(self, road: Road, profile: RiderProfile, s0: float = 0.0, dt_integrate: float = 0.001) -> None:
        self.road = road
        self.profile = profile
        self.s0 = s0
        self._dt = dt_integrate
        self._s_cache: List[float] = [s0]
        self._t_cache_max = 0.0

    def arc_length(self, t: float) -> float:
        # integrate speed with a fine fixed step, cached forward
        if t <= 0:
            return self.s0 + self.profile.speed(0.0) * t
        n_needed = int(math.ceil(t / self._dt))
        while len(self._s_cache) <= n_needed:
            k = len(self._s_cache)
            tk = (k - 1) * self._dt
            v = 0.5 * (self.profile.speed(tk) + self.profile.speed(tk + self._dt))
            self._s_cache.append(self._s_cache[-1] + v * self._dt)
        k = int(t / self._dt)
        frac = t / self._dt - k
        if k + 1 < len(self._s_cache):
            return self._s_cache[k] + frac * (self._s_cache[k + 1] - self._s_cache[k])
        return self._s_cache[-1]

    def _lean(self, t: float, v: float, kappa: float) -> float:
        if not self.profile.lean_from_curvature:
            return 0.0
        return math.atan2(v * v * kappa, G)

    def state(self, t: float) -> RiderState:
        s = self.arc_length(t)
        v = self.profile.speed(t)
        h = 1e-3
        a_along = (self.profile.speed(t + h) - self.profile.speed(max(0.0, t - h))) / (2 * h if t > h else h)
        x, y, th = self.road.pose(s)
        kappa = self.road.curvature(s)
        lean = self._lean(t, v, kappa)
        psi = self.profile.torso_yaw(t)
        f = np.array([math.cos(th), math.sin(th), 0.0])
        l = np.array([-math.sin(th), math.cos(th), 0.0])
        p = np.array([x, y, RADAR_HEIGHT_M])
        v_w = v * f
        a_w = a_along * f + v * v * kappa * l
        vest_h = th + psi
        fv = np.array([math.cos(vest_h), math.sin(vest_h), 0.0])
        lv = np.array([-math.sin(vest_h), math.cos(vest_h), 0.0])
        u = np.array([0.0, 0.0, 1.0])
        # lean LEFT by `lean`: rotate the body about the vest-forward axis so the top moves toward +l
        c, sn = math.cos(lean), math.sin(lean)
        # rotation of (lv, u) about fv: for a left lean the up axis tilts toward +lv
        u2 = c * u + sn * lv
        lv2 = c * lv - sn * u
        X = -fv                 # rearward
        Y = -lv2                # rider's RIGHT (after lean)
        Z = u2
        R_w_r = np.column_stack([X, Y, Z])
        return RiderState(t=t, s=s, speed=v, accel_along=a_along, heading=th, torso_yaw=psi, lean=lean,
                          p_w=p, v_w=v_w, a_w=a_w, R_w_r=R_w_r)

    def body_rates(self, t: float, h: float = 2e-3) -> np.ndarray:
        """Angular velocity of the radar frame, expressed in the radar frame (rad/s), by finite difference."""
        R0 = self.state(max(0.0, t - h)).R_w_r
        R1 = self.state(t + h).R_w_r
        dR = R0.T @ R1                      # rotation from frame(t-h) to frame(t+h), in body coords
        ang = math.acos(max(-1.0, min(1.0, (np.trace(dR) - 1) / 2)))
        if ang < 1e-12:
            return np.zeros(3)
        axis = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]) / (2 * math.sin(ang))
        return axis * ang / (2 * h if t > h else t + h)

"""l04: raw IMU samples -> ImuState (SI, radar frame, ESKF attitude + gyro bias).

    +X -> behind the rider      +Y -> rider's RIGHT      -Y -> rider's LEFT      +Z -> up

Yaw is unobservable: no magnetometer, no absolute-yaw estimator.  Downstream
consumes only relative yaw increments, delta_yaw(t1, t2).
Centripetal compensation (optional) uses the PREVIOUS cycle's ego velocity,
handed in explicitly by the orchestrator: no same-cycle loop between l04 and l05.
"""
from __future__ import annotations

import bisect
import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

import numpy as np

from .. import frames as fr
from ..config import Section
from ..types import EgoMotion, ImuHealth, ImuKind, ImuState, RawImuSample, StageCounters, Transform
from .eskf import Eskf, EskfParams

STAGE = "l04_imu_decode"

# The mandated library (adafruit_bno08x) delivers gyro in rad/s and acceleration in m/s^2.
# This is the explicit (identity) SI conversion site; the +9.81 m/s^2 on +Z test pins it.
LIBRARY_GYRO_UNIT = "rad/s"
LIBRARY_ACCEL_UNIT = "m/s^2"


def to_si_gyro(values) -> np.ndarray:
    return np.asarray(values, dtype=float).reshape(3) * 1.0


def to_si_accel(values) -> np.ndarray:
    return np.asarray(values, dtype=float).reshape(3) * 1.0


@dataclass
class _Node:
    t: float
    q: np.ndarray          # q_ref_radar
    omega: np.ndarray      # bias-corrected, radar frame
    bias: np.ndarray
    cov_diag: np.ndarray
    yaw_sigma: float


class ImuDecoder:
    def __init__(self, imu_cfg: Section, T_radar_imu: Transform) -> None:
        e = imu_cfg.eskf
        self.eskf = Eskf(EskfParams(
            gyro_noise_rad_s=float(e.gyro_noise_rad_s), gyro_bias_rw_rad_s2=float(e.gyro_bias_rw_rad_s2),
            accel_dir_noise_rad=float(e.accel_dir_noise_rad), accel_norm_gate_mps2=float(e.accel_norm_gate_mps2),
            gyro_rate_gate_rad_s=float(e.gyro_rate_gate_rad_s), init_sigma_att_rad=float(e.init_sigma_att_rad),
            init_sigma_bias_rad_s=float(e.init_sigma_bias_rad_s)))
        self.centripetal = bool(e.centripetal_compensation)
        self.gated_long_s = float(e.gated_long_s)
        self.buffer_s = float(e.buffer_s)
        self.no_data_timeout_s = float(imu_cfg.no_data_timeout_s)
        self.stale_after_s = float(imu_cfg.stale_after_s)
        R = fr.rotation(T_radar_imu)
        fr.validate_rotation(R)
        self.R_radar_imu = R
        m = imu_cfg.motion
        self.motion_window_s = float(m.window_s)
        self.accel_std_thresh = float(m.accel_std_thresh_mps2)
        self.gyro_std_thresh = float(m.gyro_std_thresh_rad_s)
        self._nodes: Deque[_Node] = deque()
        self._times: List[float] = []
        self._last_gyro_t: Optional[float] = None
        self._last_any_t: Optional[float] = None
        self._last_accel_applied_t: Optional[float] = None
        self._accel_norms: Deque[Tuple[float, float]] = deque()
        self._gyro_norms: Deque[Tuple[float, float]] = deque()
        self._last_omega = np.zeros(3)
        self.n_gyro = 0
        self.n_accel = 0
        self.n_accel_applied = 0
        self.n_dropped_out_of_order = 0
        self.reset_count = 0
        self._health_override: Optional[ImuHealth] = None

    # -- ingestion ---------------------------------------------------------------------------------
    def notify_reset(self) -> None:
        self.reset_count += 1
        self._health_override = ImuHealth.RESET

    def ingest(self, s: RawImuSample, prev_ego: Optional[EgoMotion] = None) -> None:
        if self._last_any_t is not None and s.t < self._last_any_t - 0.05:
            self.n_dropped_out_of_order += 1
            return
        self._last_any_t = max(s.t, self._last_any_t or s.t)
        if s.kind is ImuKind.GYRO:
            w_imu = to_si_gyro(s.values)
            w = self.R_radar_imu @ w_imu
            if self._last_gyro_t is not None:
                self.eskf.propagate(w, s.t - self._last_gyro_t)
            self._last_gyro_t = s.t
            self._last_omega = w - self.eskf.b
            self.n_gyro += 1
            self._gyro_norms.append((s.t, float(np.linalg.norm(self._last_omega))))
            self._push_node(s.t)
        elif s.kind is ImuKind.ACCEL:
            f_imu = to_si_accel(s.values)
            f = self.R_radar_imu @ f_imu
            self.n_accel += 1
            self._accel_norms.append((s.t, float(np.linalg.norm(f))))
            if self.centripetal and prev_ego is not None and prev_ego.valid:
                v_ego = np.array([prev_ego.v_s[0], prev_ego.v_s[1], 0.0])
                f = f - np.cross(self._last_omega, v_ego)
            if self.eskf.update_gravity(f, self._last_omega):
                self.n_accel_applied += 1
                self._last_accel_applied_t = s.t
                self._health_override = None
            if self._last_gyro_t is None:
                self._push_node(s.t)
        else:
            return   # GAME_RV: diagnostic only, never fused
        self._trim(s.t)

    def _push_node(self, t: float) -> None:
        e = self.eskf
        node = _Node(t=t, q=e.q.copy(), omega=self._last_omega.copy(), bias=e.b.copy(),
                     cov_diag=np.diag(e.P).copy(), yaw_sigma=e.yaw_sigma())
        if self._times and t <= self._times[-1]:
            t = self._times[-1] + 1e-6
            node.t = t
        self._nodes.append(node)
        self._times.append(t)

    def _trim(self, t_now: float) -> None:
        cutoff = t_now - self.buffer_s
        while self._nodes and self._nodes[0].t < cutoff:
            self._nodes.popleft()
            self._times.pop(0)
        for dq in (self._accel_norms, self._gyro_norms):
            while dq and dq[0][0] < t_now - self.motion_window_s:
                dq.popleft()

    # -- motion indicator ----------------------------------------------------------------------------
    def motion(self) -> Tuple[bool, float]:
        a = np.array([v for _, v in self._accel_norms]) if len(self._accel_norms) >= 5 else None
        g = np.array([v for _, v in self._gyro_norms]) if len(self._gyro_norms) >= 5 else None
        energy = 0.0
        moving = False
        if a is not None:
            sa = float(np.std(a))
            energy += sa / max(self.accel_std_thresh, 1e-9)
            moving |= sa > self.accel_std_thresh
        if g is not None:
            sg = float(np.std(g))
            energy += sg / max(self.gyro_std_thresh, 1e-9)
            moving |= sg > self.gyro_std_thresh
        return moving, energy

    # -- queries ---------------------------------------------------------------------------------------
    def _interp(self, t: float) -> Tuple[_Node, ImuHealth]:
        n = len(self._times)
        if n == 0:
            raise LookupError("no IMU data")
        i = bisect.bisect_left(self._times, t)
        if i >= n:
            last = self._nodes[-1]
            health = ImuHealth.OK if (t - last.t) <= self.stale_after_s else ImuHealth.STALE
            return last, health
        if i == 0:
            first = self._nodes[0]
            return first, (ImuHealth.OK if (first.t - t) <= 0.05 else ImuHealth.STALE)   # before the buffer
        a, b = self._nodes[i - 1], self._nodes[i]
        u = (t - a.t) / (b.t - a.t) if b.t > a.t else 0.0
        q = fr.q_slerp(a.q, b.q, u)
        node = _Node(t=t, q=q, omega=a.omega + u * (b.omega - a.omega), bias=b.bias, cov_diag=b.cov_diag, yaw_sigma=b.yaw_sigma)
        return node, ImuHealth.OK

    def state_at(self, t: float, t_now: Optional[float] = None) -> ImuState:
        """ImuState at time t.  With no data, an identity-attitude state flagged NO_DATA is returned so the
        warning path keeps working on approaching tracks (invariant 1)."""
        moving, energy = self.motion()
        try:
            node, health = self._interp(t)
        except LookupError:
            return ImuState(t=t, q_level_radar=(1, 0, 0, 0), q_ref_radar=(1, 0, 0, 0), omega_radar=(0, 0, 0),
                            gyro_bias=(0, 0, 0), cov_diag=tuple([float("inf")] * 6), yaw_sigma=float("inf"),
                            roll=0.0, pitch=0.0, in_motion=False, motion_energy=0.0, health=ImuHealth.NO_DATA,
                            n_gyro=self.n_gyro, n_accel_applied=self.n_accel_applied,
                            n_accel_gated=self.eskf.n_gated_norm + self.eskf.n_gated_rate)
        if t_now is not None and self._last_any_t is not None and (t_now - self._last_any_t) > self.no_data_timeout_s:
            health = ImuHealth.NO_DATA
        if self._health_override is not None and health is ImuHealth.OK:
            health = self._health_override
        if (health is ImuHealth.OK and self._last_accel_applied_t is not None
                and (t - self._last_accel_applied_t) > self.gated_long_s):
            health = ImuHealth.ACCEL_GATED_LONG
        ql = fr.level_from_attitude(node.q)
        _, pitch, roll = fr.euler_zyx_for_display(node.q)
        return ImuState(t=t, q_level_radar=tuple(float(v) for v in ql), q_ref_radar=tuple(float(v) for v in node.q),
                        omega_radar=tuple(float(v) for v in node.omega), gyro_bias=tuple(float(v) for v in node.bias),
                        cov_diag=tuple(float(v) for v in node.cov_diag), yaw_sigma=float(node.yaw_sigma),
                        roll=float(roll), pitch=float(pitch), in_motion=moving, motion_energy=float(energy),
                        health=health, n_gyro=self.n_gyro, n_accel_applied=self.n_accel_applied,
                        n_accel_gated=self.eskf.n_gated_norm + self.eskf.n_gated_rate)

    def delta_yaw(self, t1: float, t2: float) -> float:
        """Yaw increment of the radar frame about level +Z between t1 and t2 (rad, right-handed).
        Relative only; the reference yaw is arbitrary."""
        if not self._times:
            return 0.0
        a, _ = self._interp(t1)
        b, _ = self._interp(t2)
        y1, _, _ = fr.euler_zyx_for_display(a.q)
        y2, _, _ = fr.euler_zyx_for_display(b.q)
        d = y2 - y1
        return math.atan2(math.sin(d), math.cos(d))

    def counters(self) -> StageCounters:
        return StageCounters(STAGE, n_in=self.n_gyro + self.n_accel, n_out=len(self._nodes),
                             rejected={"accel_gated_norm": self.eskf.n_gated_norm, "accel_gated_rate": self.eskf.n_gated_rate,
                                       "out_of_order": self.n_dropped_out_of_order})

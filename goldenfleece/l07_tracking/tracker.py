"""l07: multi-target tracker in the level frame.

    +X -> behind the rider      +Y -> rider's RIGHT      -Y -> rider's LEFT      +Z -> up

State per track (decoupled, see Stage 0 plan §4.7):
  * range / range-rate: linear 2-state KF, constant range-rate.  Doppler IS the
    range rate, range is direct: both strong measurements.
  * azimuth: 1-state KF in the level frame, rotated by -delta_yaw between frames
    (gyro compensation).  Azimuth is weak (1 deg quantisation, +/-15 deg yaw floor).
Assignment: Hungarian on a Mahalanobis cost with gating.
Lifecycle: TENTATIVE -> CONFIRMED (M of N) -> COASTING (with a reason) -> DELETED,
and deletion happens only through a coast resolution.  A track never dies because
measurements stopped: "radar lost it" never means "threat gone".
Nothing here assumes the radar is attached to anything but the plate.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment

from ..config import Section
from ..types import (AZ_CLIP, ClassifiedDetection, ClutterOutput, CoastReason, CoastResolution, DetectionClass, StageCounters,
                     Track, TrackStatus, TrackerOutput)

STAGE = "l07_tracking"


@dataclass
class Measurement:
    r: float
    r_dot: float
    az: float                    # level frame
    var_r: float
    var_r_dot: float
    var_az: float
    cls: DetectionClass
    n_members: int
    magnitude_db: float


@dataclass
class _TrackState:
    id: int
    status: TrackStatus
    x: np.ndarray                # [r, r_dot]
    P: np.ndarray                # 2x2
    az: float
    var_az: float
    t: float
    hits: int = 0
    misses: int = 0
    age: int = 0
    frames_since_update: int = 0
    coast_reason: CoastReason = CoastReason.NONE
    coast_resolution: CoastResolution = CoastResolution.NONE
    coast_started_t: Optional[float] = None
    last_class: DetectionClass = DetectionClass.APPROACHING
    n_merged: int = 0
    assoc: Deque[bool] = field(default_factory=lambda: deque(maxlen=6))
    fov_pass_t: Optional[float] = None
    last_az_obs: float = 0.0
    last_r_dot_obs: float = 0.0
    closing: bool = False
    r_hist: Deque[Tuple[float, float]] = field(default_factory=lambda: deque(maxlen=12))
    inconsistent_checks: int = 0
    consistent: bool = True
    consistency_checked: bool = False

    def to_track(self, blind: float) -> Track:
        r = float(self.x[0])
        rd = float(self.x[1])
        return Track(id=self.id, status=self.status, coast_reason=self.coast_reason, coast_resolution=self.coast_resolution,
                     t=self.t, r=r, r_dot=rd, sigma_r=float(math.sqrt(max(self.P[0, 0], 0))), sigma_r_dot=float(math.sqrt(max(self.P[1, 1], 0))),
                     az=self.az, sigma_az=float(math.sqrt(max(self.var_az, 0))), x=r * math.cos(self.az), y=r * math.sin(self.az),
                     v_closing=-rd, hits=self.hits, misses=self.misses, age=self.age, frames_since_update=self.frames_since_update,
                     coast_started_t=self.coast_started_t, last_class=self.last_class, n_merged_measurements=self.n_merged,
                     closing=self.closing,
                     kinematic_consistent=(self.consistent and self.consistency_checked and self.inconsistent_checks == 0))


def cluster_detections(kept: Sequence[ClassifiedDetection], dr: float, drdot: float, daz: float,
                       R_level_radar: Optional[np.ndarray], sig_r: float, sig_rdot: float, sig_az: float) -> List[Measurement]:
    """Single-linkage clustering of a car's point cloud into one measurement."""
    n = len(kept)
    if n == 0:
        return []
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i in range(n):
        a = kept[i].det
        for j in range(i + 1, n):
            b = kept[j].det
            if abs(a.r - b.r) < dr and abs(a.v_radial - b.v_radial) < drdot and abs(a.az - b.az) < daz:
                ra, rb = find(i), find(j)
                if ra != rb:
                    parent[ra] = rb
    groups: Dict[int, List[ClassifiedDetection]] = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(kept[i])
    out: List[Measurement] = []
    for members in groups.values():
        w = np.array([10 ** (m.det.magnitude_db / 20) for m in members])
        w = w / w.sum()
        r = float(np.dot(w, [m.det.r for m in members]))
        rd = float(np.dot(w, [m.det.v_radial for m in members]))
        az = float(np.dot(w, [m.det.az for m in members]))
        var_r = sig_r ** 2 + (float(np.dot(w, [(m.det.r - r) ** 2 for m in members])) if len(members) > 1 else 0.0)
        var_rd = sig_rdot ** 2 + (float(np.dot(w, [(m.det.v_radial - rd) ** 2 for m in members])) if len(members) > 1 else 0.0)
        var_az = sig_az ** 2 + (float(np.dot(w, [(m.det.az - az) ** 2 for m in members])) if len(members) > 1 else 0.0)
        if R_level_radar is not None:
            d = R_level_radar @ np.array([math.cos(az), math.sin(az), 0.0])
            az_level = math.atan2(d[1], d[0])
        else:
            az_level = az
        n_app = sum(1 for m in members if m.cls is DetectionClass.APPROACHING)
        cls = DetectionClass.APPROACHING if n_app * 2 >= len(members) else members[0].cls
        out.append(Measurement(r=r, r_dot=rd, az=az_level, var_r=var_r, var_r_dot=var_rd, var_az=var_az, cls=cls,
                               n_members=len(members), magnitude_db=max(m.det.magnitude_db for m in members)))
    return out


class Tracker:
    def __init__(self, tcfg: Section, blind_band_mps: float, az_clip_rad: float = AZ_CLIP) -> None:
        c = tcfg
        self.cluster_dr = float(c.cluster_dr_m)
        self.cluster_drdot = float(c.cluster_drdot_mps)
        self.cluster_daz = float(c.cluster_daz_rad)
        self.gate = float(c.gate_chi2)
        self.sig_r = float(c.sigma_r_meas_m)
        self.sig_rdot = float(c.sigma_rdot_meas_mps)
        self.sig_az = float(c.sigma_az_meas_rad)
        self.q_accel = float(c.q_accel_mps2)
        self.q_az_rate = float(c.q_az_rate_rad_s)
        self.min_var_az = float(c.sigma_az_meas_rad) ** 2 * float(c.az_var_floor_factor)
        self.init_sig_rdot = float(c.init_sigma_rdot_mps)
        self.init_sig_az = float(c.init_sigma_az_rad)
        self.confirm_app = tuple(int(v) for v in c.confirm_approaching)
        self.confirm_other = tuple(int(v) for v in c.confirm_other)
        self.tentative_max_misses = int(c.tentative_max_misses)
        self.fov_margin = float(c.fov_exit_margin_rad)
        self.blind_margin = float(c.doppler_blind_margin_mps)
        self.sigma_r_bound = float(c.coast_sigma_r_bound_m)
        self.q_unexplained = float(c.coast_unexplained_q_accel_mps2)
        self.fov_pass_hold = float(c.fov_pass_hold_s)
        self.max_tracks = int(c.max_tracks)
        self.cons_min_samples = int(c.consistency_min_samples)
        self.cons_tol = float(c.consistency_tol_mps)
        self.cons_frac = float(c.consistency_tol_frac)
        self.cons_strikes = int(c.consistency_strikes)
        self.reacq_dr = float(c.reacquire_dr_m)
        self.reacq_drdot = float(c.reacquire_drdot_mps)
        self.blind = float(blind_band_mps)
        self.az_clip = float(az_clip_rad)
        self._tracks: List[_TrackState] = []
        self._next_id = 1
        self._last_t: Optional[float] = None
        self.total = {"spawned": 0, "confirmed": 0, "deleted": 0}
        self.coast_counts: Dict[str, int] = {r.name: 0 for r in CoastReason if r is not CoastReason.NONE}
        self.resolution_counts: Dict[str, int] = {r.name: 0 for r in CoastResolution if r is not CoastResolution.NONE}

    # -- prediction ----------------------------------------------------------------------------------
    def _predict(self, tr: _TrackState, dt: float, delta_yaw: float) -> None:
        if tr.status is TrackStatus.COASTING and tr.coast_reason is CoastReason.DOPPLER_BLIND:
            # The invisibility condition bounds the velocity: |r_dot| <= blind band, and sigma_r grows
            # linearly at most at that rate:  P_rr = P_rr0 + (blind * tau)^2.
            rd = max(-self.blind, min(self.blind, float(tr.x[1])))
            tau = tr.t + dt - (tr.coast_started_t if tr.coast_started_t is not None else tr.t)
            tau_prev = max(tau - dt, 0.0)
            # Range cannot go below zero: an unseen slow object predicted to reach the rider is held there, not
            # carried through to the far side.  It still survives until its sigma bound (a pacer must outlive its
            # relative-speed zero crossing), and it is not closing: the coast model puts |r_dot| inside the band.
            tr.x = np.array([max(tr.x[0] + rd * dt, 0.0), rd])
            tr.closing = False
            tr.P = tr.P + np.diag([(self.blind * tau) ** 2 - (self.blind * tau_prev) ** 2, 0.0])
        else:
            F = np.array([[1.0, dt], [0.0, 1.0]])
            q = self.q_unexplained if (tr.status is TrackStatus.COASTING and tr.coast_reason is CoastReason.UNEXPLAINED) else self.q_accel
            # continuous white-noise acceleration, spectral density q^2: P_rr grows as q^2 tau^3 / 3 while coasting
            Q = q ** 2 * np.array([[dt ** 3 / 3, dt ** 2 / 2], [dt ** 2 / 2, dt]])
            tr.x = F @ tr.x
            tr.P = F @ tr.P @ F.T + Q
        tr.az = math.atan2(math.sin(tr.az - delta_yaw), math.cos(tr.az - delta_yaw))
        tr.var_az = max(tr.var_az + (self.q_az_rate ** 2) * dt, self.min_var_az)
        tr.t += dt
        tr.age += 1

    # -- update ----------------------------------------------------------------------------------------
    def _update(self, tr: _TrackState, m: Measurement) -> None:
        z = np.array([m.r, m.r_dot])
        R = np.diag([m.var_r, m.var_r_dot])
        S = tr.P + R
        K = tr.P @ np.linalg.inv(S)
        tr.x = tr.x + K @ (z - tr.x)
        tr.P = (np.eye(2) - K) @ tr.P
        k_az = tr.var_az / (tr.var_az + m.var_az)
        d = math.atan2(math.sin(m.az - tr.az), math.cos(m.az - tr.az))
        tr.az = math.atan2(math.sin(tr.az + k_az * d), math.cos(tr.az + k_az * d))
        tr.var_az = max((1 - k_az) * tr.var_az, self.min_var_az)
        tr.hits += 1
        tr.misses = 0
        tr.frames_since_update = 0
        tr.last_class = m.cls
        tr.n_merged += m.n_members
        tr.last_az_obs = m.az
        tr.last_r_dot_obs = m.r_dot
        tr.closing = float(tr.x[1]) < -self.blind
        tr.assoc.append(True)
        self._check_consistency(tr, m)

    def _check_consistency(self, tr: _TrackState, m: Measurement) -> None:
        """Range trend (LS slope of measured range) must agree with the estimated range-rate.
        Wheel micro-Doppler returns close at ~2x the car speed while their range follows the car."""
        tr.r_hist.append((tr.t, m.r))
        if len(tr.r_hist) < self.cons_min_samples:
            return
        ts = np.array([h[0] for h in tr.r_hist])
        rs = np.array([h[1] for h in tr.r_hist])
        span = ts[-1] - ts[0]
        if span < 0.08:
            return
        tm = ts.mean()
        slope = float(((ts - tm) * (rs - rs.mean())).sum() / max(((ts - tm) ** 2).sum(), 1e-9))
        rd = float(tr.x[1])
        tol = max(self.cons_tol, self.cons_frac * abs(rd))
        tr.consistency_checked = True
        if abs(slope - rd) > tol:
            tr.inconsistent_checks += 1
            if tr.inconsistent_checks >= self.cons_strikes:
                tr.consistent = False
        else:
            tr.inconsistent_checks = 0
            tr.consistent = True

    def _cost(self, tr: _TrackState, m: Measurement) -> float:
        if tr.status is TrackStatus.COASTING:
            # a coasting track's inflated covariance must not turn it into a magnet for false alarms
            if abs(m.r - tr.x[0]) > max(self.reacq_dr, 2.0 * math.sqrt(max(tr.P[0, 0], 0.0))) or abs(m.r_dot - tr.x[1]) > self.reacq_drdot:
                return float("inf")
        S_r = tr.P[0, 0] + m.var_r
        S_rd = tr.P[1, 1] + m.var_r_dot
        S_az = tr.var_az + m.var_az
        daz = math.atan2(math.sin(m.az - tr.az), math.cos(m.az - tr.az))
        return (m.r - tr.x[0]) ** 2 / S_r + (m.r_dot - tr.x[1]) ** 2 / S_rd + daz ** 2 / S_az

    # -- coasting ----------------------------------------------------------------------------------------
    def _choose_coast_reason(self, tr: _TrackState, delta_yaw: float, az_before_rotation: float) -> CoastReason:
        rd = float(tr.x[1])
        if abs(rd) < self.blind + self.blind_margin:
            return CoastReason.DOPPLER_BLIND
        if abs(az_before_rotation) <= self.az_clip and abs(tr.az) > self.az_clip and abs(delta_yaw) > math.radians(1.0):
            return CoastReason.YAW_EXIT
        if abs(tr.last_az_obs) > self.az_clip - self.fov_margin and rd < 0:
            return CoastReason.FOV_EXIT
        return CoastReason.UNEXPLAINED

    def _start_coast(self, tr: _TrackState, reason: CoastReason) -> None:
        tr.status = TrackStatus.COASTING
        tr.coast_reason = reason
        tr.coast_started_t = tr.t
        self.coast_counts[reason.name] += 1
        if reason is CoastReason.DOPPLER_BLIND:
            tr.closing = False       # from this frame on: |r_dot| is inside the blind band, which is not closing
        if reason is CoastReason.FOV_EXIT:
            v_close = max(-float(tr.x[1]), 0.5)
            x_long = max(float(tr.x[0]) * math.cos(tr.az), 0.0)
            tr.fov_pass_t = tr.t + x_long / v_close + self.fov_pass_hold

    def _resolve(self, tr: _TrackState, resolution: CoastResolution) -> None:
        tr.status = TrackStatus.DELETED
        tr.coast_resolution = resolution
        self.resolution_counts[resolution.name] += 1
        self.total["deleted"] += 1

    # -- main step ---------------------------------------------------------------------------------------------
    def step(self, co: ClutterOutput, delta_yaw: float, R_level_radar: Optional[np.ndarray] = None) -> TrackerOutput:
        t = co.t_mid
        dt = 0.0 if self._last_t is None else max(0.0, t - self._last_t)
        self._last_t = t
        # drop tracks deleted in the previous step
        self._tracks = [tr for tr in self._tracks if tr.status is not TrackStatus.DELETED]
        az_before = {tr.id: tr.az for tr in self._tracks}
        for tr in self._tracks:
            self._predict(tr, dt, delta_yaw)

        meas = cluster_detections(co.kept, self.cluster_dr, self.cluster_drdot, self.cluster_daz, R_level_radar,
                                  self.sig_r, self.sig_rdot, self.sig_az)
        n_tr, n_m = len(self._tracks), len(meas)
        assigned_tr: Dict[int, int] = {}
        gated_out = 0
        if n_tr and n_m:
            C = np.full((n_tr, n_m), self.gate * 10.0)
            for i, tr in enumerate(self._tracks):
                for j, m in enumerate(meas):
                    c = self._cost(tr, m)
                    if c < self.gate and c != float("inf"):
                        C[i, j] = c
                    else:
                        gated_out += 1
            rows, cols = linear_sum_assignment(C)
            for i, j in zip(rows, cols):
                if C[i, j] < self.gate:
                    assigned_tr[i] = j
        for i, tr in enumerate(self._tracks):
            if i in assigned_tr:
                self._update(tr, meas[assigned_tr[i]])
                if tr.status is TrackStatus.TENTATIVE:
                    M, N = self.confirm_app if tr.last_class is DetectionClass.APPROACHING else self.confirm_other
                    recent = list(tr.assoc)[-N:]
                    if sum(recent) >= M and tr.consistent:
                        tr.status = TrackStatus.CONFIRMED
                        self.total["confirmed"] += 1
                elif tr.status is TrackStatus.COASTING:
                    tr.status = TrackStatus.CONFIRMED
                    tr.coast_resolution = CoastResolution.REACQUIRED
                    self.resolution_counts["REACQUIRED"] += 1
                    tr.coast_reason = CoastReason.NONE
                    tr.coast_started_t = None
                    tr.fov_pass_t = None
                    tr.r_hist.clear()                    # consistency must be re-established before any threat
                    tr.consistency_checked = False
                    tr.inconsistent_checks = 0
            else:
                tr.misses += 1
                tr.frames_since_update += 1
                tr.assoc.append(False)
                if tr.status is TrackStatus.TENTATIVE:
                    if tr.misses > self.tentative_max_misses:
                        self._resolve(tr, CoastResolution.NEVER_CONFIRMED)
                elif tr.status is TrackStatus.CONFIRMED:
                    self._start_coast(tr, self._choose_coast_reason(tr, delta_yaw, az_before.get(tr.id, tr.az)))
                if tr.status is TrackStatus.COASTING:
                    if tr.coast_reason is CoastReason.FOV_EXIT and tr.fov_pass_t is not None and tr.t >= tr.fov_pass_t:
                        self._resolve(tr, CoastResolution.PASS_THROUGH_COMPLETE)
                    elif math.sqrt(max(tr.P[0, 0], 0.0)) > self.sigma_r_bound:
                        self._resolve(tr, CoastResolution.SIGMA_BOUND_EXCEEDED)
                    elif float(tr.x[0]) <= 0.0 and tr.coast_reason is not CoastReason.DOPPLER_BLIND:
                        self._resolve(tr, CoastResolution.PASS_THROUGH_COMPLETE)
        # spawn
        spawned = 0
        used = set(assigned_tr.values())
        for j, m in enumerate(meas):
            if j in used:
                continue
            if len(self._tracks) >= self.max_tracks:
                break
            tr = _TrackState(id=self._next_id, status=TrackStatus.TENTATIVE, x=np.array([m.r, m.r_dot]),
                             P=np.diag([m.var_r, max(m.var_r_dot, self.init_sig_rdot ** 2)]), az=m.az,
                             var_az=max(m.var_az, self.init_sig_az ** 2), t=t, last_class=m.cls, last_az_obs=m.az,
                             last_r_dot_obs=m.r_dot, closing=m.r_dot < -self.blind)
            tr.hits = 1
            tr.assoc.append(True)
            tr.n_merged = m.n_members
            self._next_id += 1
            self._tracks.append(tr)
            spawned += 1
        self.total["spawned"] += spawned
        tracks = tuple(tr.to_track(self.blind) for tr in self._tracks)
        counters = StageCounters(STAGE, n_in=len(co.kept), n_out=sum(1 for tr in self._tracks if tr.status in (TrackStatus.CONFIRMED, TrackStatus.COASTING)),
                                 rejected={"gated_out_pairs": gated_out, "clustered_away": len(co.kept) - n_m, "spawned": spawned,
                                           "inconsistent_tracks": sum(1 for tr in self._tracks if not tr.consistent),
                                           **{f"coast_{k}": v for k, v in self.coast_counts.items()},
                                           **{f"resolved_{k}": v for k, v in self.resolution_counts.items()}})
        return TrackerOutput(t_mid=t, frame_number=co.frame_number, tracks=tracks, counters=counters)

    def live_tracks(self) -> List[Track]:
        return [tr.to_track(self.blind) for tr in self._tracks if tr.status is not TrackStatus.DELETED]

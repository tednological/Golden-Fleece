"""l05: rider (sensor) velocity from stationary radar returns.

    +X -> behind the rider      +Y -> rider's RIGHT      -Y -> rider's LEFT      +Z -> up

Stationary scatterers satisfy  v_radial_i = -(v_s . u_i),  u_i = (cos az_i, sin az_i).
Rider forward at 5 m/s => v_s = (-5, 0) (forward is -X); a pole directly behind
recedes at +5.0 m/s, a pole at az = 30 deg at +4.330 m/s.

Datasheet caveat: raw targets are Doppler bins, and scatterers at +az and -az share
a bin; the merged bin reports a phase-mixed angle.  A symmetric scene therefore
biases a naive fit low.  The top-of-band cross-check (max receding v_radial ~ rider
speed) flags that case as MERGED_BINS_SUSPECTED.

psi_travel is a DIAGNOSTIC only.  It is computed and logged, never consumed.
"""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np

from ..config import Section
from ..types import (EgoInvalidReason, EgoMotion, EgoSource, ImuHealth, ImuState, RadarFrame, StageCounters)

STAGE = "l05_ego_motion"


def _invalid(t: float, reason: EgoInvalidReason, n_in: int, n_cand: int, rej: dict, mask: Tuple[bool, ...] = ()) -> EgoMotion:
    return EgoMotion(t=t, v_s=(0.0, 0.0), cov=((1e6, 0.0), (0.0, 1e6)), speed=0.0, inlier_mask=mask,
                     n_candidates=n_cand, n_inliers=0, az_spread_rad=0.0, valid=False, invalid_reason=reason,
                     source=EgoSource.NONE, psi_travel=None, psi_travel_sigma=None,
                     counters=StageCounters(STAGE, n_in=n_in, n_out=0, rejected=rej))


def _solve_ls(u: np.ndarray, vr: np.ndarray) -> Tuple[np.ndarray, float]:
    # u @ v_s = -vr
    sol, *_ = np.linalg.lstsq(u, -vr, rcond=None)
    resid = vr + u @ sol
    rms = float(np.sqrt(np.mean(resid ** 2))) if len(vr) else 0.0
    return sol, rms


def estimate(frame: RadarFrame, imu: Optional[ImuState], prev: Optional[EgoMotion], ecfg: Section,
             rng_seed: int = 0) -> EgoMotion:
    t = frame.t_mid
    dets = frame.detections
    n_in = len(dets)
    rej = {"approaching": 0, "too_slow": 0, "outlier": 0}
    cand_idx: List[int] = []
    for i, d in enumerate(dets):
        if d.v_radial <= 0:
            rej["approaching"] += 1
            continue
        if d.v_radial < float(ecfg.candidate_min_receding_mps):
            rej["too_slow"] += 1
            continue
        cand_idx.append(i)
    n_cand = len(cand_idx)
    imu_stopped = imu is not None and imu.health in (ImuHealth.OK, ImuHealth.ACCEL_GATED_LONG) and not imu.in_motion

    min_inliers = int(ecfg.min_inliers)
    if n_cand < min_inliers:
        if imu_stopped:
            return _stopped(t, n_in, n_cand, rej)
        return _invalid(t, EgoInvalidReason.NO_CANDIDATES if n_cand == 0 else EgoInvalidReason.TOO_FEW_INLIERS, n_in, n_cand, rej)

    az = np.array([dets[i].az for i in cand_idx])
    vr = np.array([dets[i].v_radial for i in cand_idx])
    u = np.column_stack([np.cos(az), np.sin(az)])
    thresh = float(ecfg.inlier_thresh_mps)
    prior_gate = float(ecfg.prior_speed_gate_mps)
    prior_speed = prev.speed if (prev is not None and prev.valid) else None

    # RANSAC over 2-point minimal sets (deterministic: seeded by frame number)
    rng = np.random.default_rng(rng_seed + frame.frame_number)
    best_mask = None
    best_score = -1.0
    n = n_cand
    pairs = [(i, j) for i in range(n) for j in range(i + 1, n) if abs(az[i] - az[j]) > math.radians(5)]
    if not pairs:
        # all at the same azimuth: 1-D fit along that direction only
        pairs = [(0, min(1, n - 1))]
    iters = min(int(ecfg.ransac_iterations), len(pairs))
    order = rng.permutation(len(pairs))[:iters] if len(pairs) > iters else range(len(pairs))
    for k in order:
        i, j = pairs[k]
        A = u[[i, j]]
        if abs(np.linalg.det(A)) < 1e-3:
            sol = np.array([-(vr[i] + vr[j]) / 2 / max(math.cos(az[i]), 1e-3), 0.0])
        else:
            sol = np.linalg.solve(A, -vr[[i, j]])
        resid = np.abs(vr + u @ sol)
        mask = resid < thresh
        score = float(mask.sum())
        if prior_speed is not None and abs(np.linalg.norm(sol) - prior_speed) > prior_gate:
            score -= 0.5 * n       # hypotheses far from the previous speed rank below consistent ones
        if score > best_score:
            best_score = score
            best_mask = mask
    if best_mask is None or best_mask.sum() < min_inliers:
        rej["outlier"] += int(n_cand - (0 if best_mask is None else best_mask.sum()))
        if imu_stopped:
            return _stopped(t, n_in, n_cand, rej)
        return _invalid(t, EgoInvalidReason.TOO_FEW_INLIERS, n_in, n_cand, rej)

    sol, rms = _solve_ls(u[best_mask], vr[best_mask])
    # refine once with the refit
    resid = np.abs(vr + u @ sol)
    mask = resid < thresh
    if mask.sum() >= min_inliers:
        sol, rms = _solve_ls(u[mask], vr[mask])
    else:
        mask = best_mask
    n_inl = int(mask.sum())
    rej["outlier"] += int(n_cand - n_inl)
    speed = float(np.linalg.norm(sol))
    spread = float(az[mask].max() - az[mask].min()) if n_inl else 0.0
    full_mask = [False] * n_in
    for k, idx in enumerate(cand_idx):
        full_mask[idx] = bool(mask[k])
    full_mask_t = tuple(full_mask)

    sigma2 = max(rms ** 2, (0.11) ** 2)          # never below half a speed bin
    try:
        cov = sigma2 * np.linalg.inv(u[mask].T @ u[mask])
    except np.linalg.LinAlgError:
        cov = np.diag([sigma2 * 10, sigma2 * 100])

    reason = EgoInvalidReason.NONE
    if spread < float(ecfg.min_az_spread_rad):
        reason = EgoInvalidReason.LOW_AZ_SPREAD
    elif rms > float(ecfg.max_rms_mps):
        reason = EgoInvalidReason.POOR_FIT
    else:
        v_top = float(vr.max())
        if speed < v_top * (1.0 - float(ecfg.top_of_band_tolerance)):
            reason = EgoInvalidReason.MERGED_BINS_SUSPECTED
    valid = reason is EgoInvalidReason.NONE
    if not valid and imu_stopped and speed < float(ecfg.stopped_speed_mps):
        return _stopped(t, n_in, n_cand, rej)

    psi = math.atan2(-sol[1], -sol[0]) if speed > 0.2 else None
    psi_sigma = None
    if psi is not None:
        # sigma of the perpendicular component divided by speed
        perp = np.array([-math.sin(psi), math.cos(psi)])
        psi_sigma = float(math.sqrt(max(perp @ cov @ perp, 0.0)) / speed)
    return EgoMotion(t=t, v_s=(float(sol[0]), float(sol[1])),
                     cov=((float(cov[0, 0]), float(cov[0, 1])), (float(cov[1, 0]), float(cov[1, 1]))),
                     speed=speed, inlier_mask=full_mask_t, n_candidates=n_cand, n_inliers=n_inl, az_spread_rad=spread,
                     valid=valid, invalid_reason=reason, source=EgoSource.RADAR_RANSAC if valid else EgoSource.NONE,
                     psi_travel=psi, psi_travel_sigma=psi_sigma,
                     counters=StageCounters(STAGE, n_in=n_in, n_out=n_inl, rejected=rej))


def _stopped(t: float, n_in: int, n_cand: int, rej: dict) -> EgoMotion:
    return EgoMotion(t=t, v_s=(0.0, 0.0), cov=((0.05, 0.0), (0.0, 0.05)), speed=0.0, inlier_mask=(),
                     n_candidates=n_cand, n_inliers=0, az_spread_rad=0.0, valid=True, invalid_reason=EgoInvalidReason.NONE,
                     source=EgoSource.IMU_STOPPED, psi_travel=None, psi_travel_sigma=None,
                     counters=StageCounters(STAGE, n_in=n_in, n_out=0, rejected=rej))

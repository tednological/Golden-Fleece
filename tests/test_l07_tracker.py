import math

import numpy as np
import pytest

from goldenfleece.l07_tracking.tracker import Tracker, cluster_detections
from goldenfleece.types import (AZ_CLIP, ClassifiedDetection, ClutterOutput, CoastReason, CoastResolution, DetectionClass, RadarDetection,
                                StageCounters, TrackStatus)

DT = 0.029


def _det(r, az, v_radial, mag=50.0, i=0):
    return RadarDetection(r=r, az=az, v_radial=v_radial, v_closing=-v_radial, x=r * math.cos(az), y=r * math.sin(az), magnitude_db=mag, raw_index=i)


def _co(t, dets, fn=0):
    kept = tuple(ClassifiedDetection(d, DetectionClass.APPROACHING if d.v_closing > 0 else DetectionClass.RECEDING_MOVER, True) for d in dets)
    return ClutterOutput(t_mid=t, frame_number=fn, kept=kept, cap_hit=False, counters=StageCounters("l06", len(dets), len(dets)))


def _tracker(cfg):
    return Tracker(cfg.pipeline.tracker, cfg.pipeline.clutter.doppler_blind_band_mps)


def _run_car(tr, r0, v_closing, az, n, t0=0.0, fn0=0, extra=None, delta_yaw=0.0):
    outs = []
    for k in range(n):
        t = t0 + k * DT
        r = r0 - v_closing * k * DT
        dets = [_det(r, az, -v_closing, i=0), _det(r + 0.6, az + 0.02, -v_closing + 0.15, mag=40, i=1)]
        if extra:
            dets += extra(k)
        outs.append(tr.step(_co(t, dets, fn0 + k), delta_yaw))
    return outs


def test_clustering_merges_point_cloud():
    dets = [_det(20.0, 0.1, -10.0, 50, 0), _det(20.5, 0.11, -9.8, 40, 1), _det(21.0, 0.09, -10.3, 35, 2), _det(10.0, -0.3, 5.0, 45, 3)]
    kept = [ClassifiedDetection(d, DetectionClass.APPROACHING, True) for d in dets]
    ms = cluster_detections(kept, 2.0, 1.5, 0.14, None, 0.25, 0.25, 0.035)
    assert len(ms) == 2
    big = max(ms, key=lambda m: m.n_members)
    assert big.n_members == 3 and abs(big.r - 20.2) < 0.4


def test_track_confirms_and_estimates_range_rate(cfg):
    tr = _tracker(cfg)
    outs = _run_car(tr, 30.0, 10.0, 0.1, 12)
    live = [t for t in outs[-1].tracks if t.status is TrackStatus.CONFIRMED]
    assert len(live) == 1
    tk = live[0]
    assert abs(tk.v_closing - 10.0) < 0.5 and abs(tk.r - (30 - 10 * 11 * DT)) < 0.6
    assert tk.closing and tk.sigma_r < 0.5


def test_no_threat_resolves_by_disappearance_unexplained_coast(cfg):
    """Mid-beam dropout at high closing speed -> UNEXPLAINED coast; the track survives with growing sigma
    and only dies through SIGMA_BOUND_EXCEEDED."""
    tr = _tracker(cfg)
    _run_car(tr, 30.0, 12.0, 0.05, 10)
    statuses = []
    for k in range(60):
        out = tr.step(_co(10 * DT + k * DT, [], 10 + k), 0.0)
        statuses.append([(t.status, t.coast_reason, t.coast_resolution, t.sigma_r) for t in out.tracks])
    first = statuses[0][0]
    assert first[0] is TrackStatus.COASTING and first[1] is CoastReason.UNEXPLAINED
    coasting_frames = sum(1 for s in statuses if s and s[0][0] is TrackStatus.COASTING)
    assert coasting_frames >= 5
    deleted = [s[0] for s in statuses if s and s[0][0] is TrackStatus.DELETED]
    assert deleted and deleted[0][2] is CoastResolution.SIGMA_BOUND_EXCEEDED
    assert tr.resolution_counts["SIGMA_BOUND_EXCEEDED"] == 1


def test_doppler_blind_coast_bounds_velocity(cfg):
    tr = _tracker(cfg)
    # pacer closing slowly at 0.6 m/s then vanishing into the blind band
    for k in range(10):
        tr.step(_co(k * DT, [_det(8.0 - 0.6 * k * DT, 0.0, -0.6, i=0)], k), 0.0)
    out = None
    for k in range(10, 40):
        out = tr.step(_co(k * DT, [], k), 0.0)
    tk = out.tracks[0]
    assert tk.status is TrackStatus.COASTING and tk.coast_reason is CoastReason.DOPPLER_BLIND
    assert abs(tk.r_dot) <= cfg.pipeline.clutter.doppler_blind_band_mps + 1e-9
    assert tk.sigma_r < 1.0                               # grows at most at the blind rate
    # re-acquire when it re-emerges closing
    out = tr.step(_co(40 * DT, [_det(tk.r - 0.1, 0.0, -1.0, i=0)], 40), 0.0)
    assert out.tracks[0].status is TrackStatus.CONFIRMED and out.tracks[0].coast_resolution is CoastResolution.REACQUIRED


def test_fov_exit_pass_through(cfg):
    tr = _tracker(cfg)
    az = AZ_CLIP - 0.03
    for k in range(8):
        tr.step(_co(k * DT, [_det(3.0 - 8.0 * k * DT, az, -8.0, i=0)], k), 0.0)
    seen_coast = False
    resolved = None
    for k in range(8, 80):
        out = tr.step(_co(k * DT, [], k), 0.0)
        for t in out.tracks:
            if t.status is TrackStatus.COASTING:
                seen_coast = True
                assert t.coast_reason is CoastReason.FOV_EXIT
            if t.status is TrackStatus.DELETED:
                resolved = t.coast_resolution
    assert seen_coast and resolved is CoastResolution.PASS_THROUGH_COMPLETE


def test_yaw_exit_and_gyro_compensation(cfg):
    """A shoulder check rotates a target out of the beam; the tracker predicts with the gyro and re-acquires."""
    tr = _tracker(cfg)
    az0 = 0.55
    for k in range(8):
        tr.step(_co(k * DT, [_det(25.0 - 8 * k * DT, az0, -8.0, i=0)], k), 0.0)
    # rider yaws +Delta each frame; target appears at az - Delta ... move it OUT of the beam: need az to grow, so yaw negative
    dpsi = -math.radians(4.0)
    total = 0.0
    out = None
    for k in range(8, 14):
        total += dpsi
        az_now = az0 - total
        dets = [_det(25.0 - 8 * k * DT, az_now, -8.0, i=0)] if abs(az_now) <= AZ_CLIP else []
        out = tr.step(_co(k * DT, dets, k), dpsi)
    tk = out.tracks[0]
    assert tk.status is TrackStatus.COASTING and tk.coast_reason is CoastReason.YAW_EXIT
    assert abs(tk.az - (az0 - total)) < 0.05           # predicted with the gyro
    # yaw back: target returns into the beam and is re-acquired
    for k in range(14, 22):
        total -= dpsi
        az_now = az0 - total
        dets = [_det(25.0 - 8 * k * DT, az_now, -8.0, i=0)] if abs(az_now) <= AZ_CLIP else []
        out = tr.step(_co(k * DT, dets, k), -dpsi)
    assert out.tracks[0].status is TrackStatus.CONFIRMED and out.tracks[0].coast_resolution is CoastResolution.REACQUIRED


def test_two_crossing_targets_keep_identity(cfg):
    tr = _tracker(cfg)
    ids_a, ids_b = set(), set()
    for k in range(40):
        t = k * DT
        a = _det(30.0 - 12.0 * t, -0.3 + 0.6 * (t / (39 * DT)), -12.0, i=0)     # sweeps left -> right
        b = _det(20.0 - 6.0 * t, 0.3 - 0.6 * (t / (39 * DT)), -6.0, i=1)        # sweeps right -> left
        out = tr.step(_co(t, [a, b], k), 0.0)
        for tk in out.tracks:
            if tk.status is TrackStatus.CONFIRMED:
                (ids_a if abs(tk.v_closing - 12.0) < 2 else ids_b).add(tk.id)
    assert len(ids_a) == 1 and len(ids_b) == 1 and ids_a != ids_b


def test_tentative_tracks_die_quietly(cfg):
    tr = _tracker(cfg)
    tr.step(_co(0.0, [_det(15.0, 0.2, -5.0)], 0), 0.0)
    out = None
    for k in range(1, 6):
        out = tr.step(_co(k * DT, [], k), 0.0)
    assert all(t.status is TrackStatus.DELETED for t in out.tracks) or not out.tracks
    assert tr.resolution_counts["NEVER_CONFIRMED"] == 1

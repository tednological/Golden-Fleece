import math

import numpy as np
import pytest

from goldenfleece.l05_ego_motion.ego import estimate
from goldenfleece.types import (EgoInvalidReason, EgoSource, ImuHealth, ImuState, RadarDetection, RadarFrame, StageCounters)


def _det(r, az, v_radial, mag=50.0, i=0):
    return RadarDetection(r=r, az=az, v_radial=v_radial, v_closing=-v_radial, x=r * math.cos(az), y=r * math.sin(az),
                          magnitude_db=mag, raw_index=i)


def _frame(dets, t=1.0, fn=1):
    return RadarFrame(t_mid=t, t_header=t, frame_number=fn, gap=0, rspi=3, rrai=2, detections=tuple(dets), n_raw=len(dets),
                      cap_hit=False, counters=StageCounters("l03", len(dets), len(dets)))


def _imu(in_motion=True, health=ImuHealth.OK):
    return ImuState(t=1.0, q_level_radar=(1, 0, 0, 0), q_ref_radar=(1, 0, 0, 0), omega_radar=(0, 0, 0), gyro_bias=(0, 0, 0),
                    cov_diag=(0,) * 6, yaw_sigma=0.1, roll=0, pitch=0, in_motion=in_motion, motion_energy=1.0, health=health,
                    n_gyro=1, n_accel_applied=1, n_accel_gated=0)


def test_golden_vectors_forward_5mps():
    v_s = np.array([-5.0, 0.0])
    pole_behind = -(v_s @ np.array([1, 0]))
    pole_30 = -(v_s @ np.array([math.cos(math.radians(30)), math.sin(math.radians(30))]))
    assert pole_behind == pytest.approx(5.0)
    assert pole_30 == pytest.approx(4.330, abs=1e-3)


def test_ransac_recovers_forward_velocity_with_outliers(cfg):
    v = np.array([-5.0, 0.0])
    dets = []
    for i, azd in enumerate([-30, -15, 0, 12, 25, 35]):
        az = math.radians(azd)
        vr = -(v @ np.array([math.cos(az), math.sin(az)]))
        dets.append(_det(10 + i, az, vr + 0.05 * ((-1) ** i), i=i))
    dets.append(_det(20, math.radians(5), 2.0, i=6))        # slower same-direction vehicle (receding at 2 m/s)
    dets.append(_det(15, math.radians(-10), -8.0, i=7))     # approaching car
    ego = estimate(_frame(dets), _imu(), None, cfg.pipeline.ego)
    assert ego.valid and ego.source is EgoSource.RADAR_RANSAC
    assert abs(ego.v_s[0] + 5.0) < 0.15 and abs(ego.v_s[1]) < 0.15
    assert ego.n_inliers == 6 and not ego.inlier_mask[6] and not ego.inlier_mask[7]
    assert ego.counters.rejected["approaching"] == 1
    assert abs(ego.psi_travel) < 0.05 and ego.psi_travel_sigma < 0.1


def test_heading_diagnostic_sees_vest_yaw(cfg):
    psi = math.radians(12)                          # vest yawed 12 deg relative to travel
    v = 6.0 * np.array([-math.cos(psi), -math.sin(psi)])
    dets = []
    for i, azd in enumerate([-35, -20, -5, 10, 25, 38]):
        az = math.radians(azd)
        dets.append(_det(8 + i, az, -(v @ np.array([math.cos(az), math.sin(az)])), i=i))
    ego = estimate(_frame(dets), _imu(), None, cfg.pipeline.ego)
    assert ego.valid
    assert abs(ego.psi_travel - psi) < 0.03


def test_open_road_is_invalid_but_stopped_rider_is_valid_zero(cfg):
    ego = estimate(_frame([]), _imu(in_motion=True), None, cfg.pipeline.ego)
    assert not ego.valid and ego.invalid_reason is EgoInvalidReason.NO_CANDIDATES
    ego2 = estimate(_frame([]), _imu(in_motion=False), None, cfg.pipeline.ego)
    assert ego2.valid and ego2.source is EgoSource.IMU_STOPPED and ego2.speed == 0.0
    ego3 = estimate(_frame([]), None, None, cfg.pipeline.ego)
    assert not ego3.valid


def test_low_azimuth_spread_invalid(cfg):
    dets = [_det(5 + i, math.radians(2 + i), 5.0, i=i) for i in range(4)]
    ego = estimate(_frame(dets), _imu(), None, cfg.pipeline.ego)
    assert not ego.valid and ego.invalid_reason is EgoInvalidReason.LOW_AZ_SPREAD


def test_merged_bins_suspected(cfg):
    """Symmetric scene: every bin reports az ~ 0 but v_radial = v cos(az) < v.  The fit is biased low
    and must be flagged rather than trusted."""
    v = 8.0
    dets = []
    for i, azd in enumerate([10, 20, 30, 38]):
        vr = v * math.cos(math.radians(azd))
        dets.append(_det(10 + i, math.radians(0.5 * (-1) ** i), vr, i=i))
    dets.append(_det(9, math.radians(-30), v * math.cos(math.radians(30)), i=4))    # one honest one
    ego = estimate(_frame(dets), _imu(), None, cfg.pipeline.ego)
    assert not ego.valid or ego.speed > 6.0


def test_prior_gate_prefers_continuity(cfg):
    """A dominant slower vehicle offers a bigger consensus than the clutter; the prior keeps the clutter."""
    v = np.array([-8.0, 0.0])
    clutter = []
    for i, azd in enumerate([-25, 0, 30]):
        az = math.radians(azd)
        clutter.append(_det(10 + i, az, -(v @ np.array([math.cos(az), math.sin(az)])), i=i))
    truck = [_det(12 + k, math.radians(-5 + 4 * k), 1.5 + 0.02 * k, i=3 + k) for k in range(4)]   # 4 points receding at ~1.5 m/s
    prev = estimate(_frame(clutter), _imu(), None, cfg.pipeline.ego)
    assert prev.valid and abs(prev.speed - 8.0) < 0.2
    ego = estimate(_frame(clutter + truck, fn=2), _imu(), prev, cfg.pipeline.ego)
    assert abs(ego.speed - 8.0) < 0.3

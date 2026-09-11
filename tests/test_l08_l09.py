import math

from goldenfleece.l08_threat.threat import ThreatAssessor
from goldenfleece.l09_warning_policy.policy import BlockageDetector, WarningPolicy, bucket_of
from goldenfleece.types import (CoastReason, CoastResolution, DetectionClass, HealthBits, HealthState, Side, TArrivalBucket, ThreatLevel, Track,
                                TrackStatus)


def _track(tid, r, v_closing, az=0.0, status=TrackStatus.CONFIRMED, coast=CoastReason.NONE, sr=0.3, sv=0.3):
    return Track(id=tid, status=status, coast_reason=coast, coast_resolution=CoastResolution.NONE, t=0.0, r=r, r_dot=-v_closing,
                 sigma_r=sr, sigma_r_dot=sv, az=az, sigma_az=0.05, x=r * math.cos(az), y=r * math.sin(az), v_closing=v_closing,
                 hits=5, misses=0, age=5, frames_since_update=0, coast_started_t=None, last_class=DetectionClass.APPROACHING,
                 n_merged_measurements=5, closing=v_closing > 0.5, kinematic_consistent=True)


def test_t_arrival_semantics_and_levels(cfg):
    ta = ThreatAssessor(cfg.pipeline.threat)
    a = ta.assess([_track(1, 30.0, 10.0)])[0]
    assert abs(a.t_arrival - 3.0) < 1e-9
    assert a.t_arrival_low < a.t_arrival                    # conservative bound
    assert a.level is ThreatLevel.WARNING                    # 2.7 s < 3 s
    b = ta.assess([_track(1, 10.0, 10.0)])[0]
    assert b.level is ThreatLevel.ALERT
    c = ta.assess([_track(1, 30.0, 4.0)])[0]
    assert c.level is ThreatLevel.NONE or c.level is ThreatLevel.ADVISORY
    d = ta.assess([_track(2, 3.0, 0.8)])[0]                 # proximity rule
    assert d.proximity_triggered and d.level >= ThreatLevel.WARNING


def test_side_band_widens_with_range(cfg):
    ta = ThreatAssessor(cfg.pipeline.threat)
    az = math.atan2(5.0, 8.0)                               # 5 m lateral at 8 m (band half-width 1 + 8 tan 15 = 3.1 m)
    near = ta.assess([_track(1, math.hypot(5, 8), 10.0, az)])[0]
    far = ta.assess([_track(1, 30.0, 10.0, math.atan2(5.0, 30.0))])[0]
    assert near.side is Side.RIGHT                           # +y = rider's RIGHT
    assert far.side is Side.CENTER                           # 5 m at 30 m is inside the yaw floor (9 m band)
    assert near.confidence > far.confidence
    left = ta.assess([_track(1, math.hypot(5, 8), 10.0, -az)])[0]
    assert left.side is Side.LEFT


def test_receding_and_tentative_ignored(cfg):
    ta = ThreatAssessor(cfg.pipeline.threat)
    out = ta.assess([_track(1, 10.0, -3.0), _track(2, 10.0, 10.0, status=TrackStatus.TENTATIVE)])
    assert out == []
    assert ta.counters.rejected["not_closing"] == 1 and ta.counters.rejected["not_confirmed"] == 1


def test_coasting_keeps_contributing(cfg):
    ta = ThreatAssessor(cfg.pipeline.threat)
    ta.assess([_track(1, 12.0, 10.0)])
    a = ta.assess([_track(1, 8.0, 10.0, status=TrackStatus.COASTING, coast=CoastReason.FOV_EXIT, sr=1.5)])[0]
    assert a.coasting and a.level >= ThreatLevel.WARNING and a.confidence < 0.6


def test_policy_escalates_immediately_and_deescalates_after_dwell(cfg):
    ta = ThreatAssessor(cfg.pipeline.threat)
    pol = WarningPolicy(cfg.pipeline.policy, ta.level_from_t)
    cmd = pol.step(0.0, ta.assess([_track(1, 10.0, 10.0)]), HealthState.OK, HealthBits.NONE)
    assert cmd.level is ThreatLevel.ALERT and cmd.assert_alert and cmd.t_arrival_bucket is TArrivalBucket.LT_1P5
    # threat gone (track resolved): must hold through min on-time + dwell
    cmd = pol.step(0.2, [], HealthState.OK, HealthBits.NONE)
    assert cmd.level is ThreatLevel.ALERT
    cmd = pol.step(0.9, [], HealthState.OK, HealthBits.NONE)
    assert cmd.level is ThreatLevel.ALERT                     # dwell 1.0 s not yet elapsed
    cmd = pol.step(1.3, [], HealthState.OK, HealthBits.NONE)
    assert cmd.level is ThreatLevel.NONE and not cmd.assert_alert
    assert pol.n_deescalations == 1


def test_policy_hysteresis_holds_near_threshold(cfg):
    ta = ThreatAssessor(cfg.pipeline.threat)
    pol = WarningPolicy(cfg.pipeline.policy, ta.level_from_t)
    pol.step(0.0, ta.assess([_track(1, 12.0, 10.0)]), HealthState.OK, HealthBits.NONE)      # ALERT (1.1 s)
    for k in range(1, 60):
        cmd = pol.step(k * 0.05, ta.assess([_track(1, 17.0, 10.0)]), HealthState.OK, HealthBits.NONE)   # t_low ~1.6 s: WARNING nominally
    assert cmd.level is ThreatLevel.ALERT                     # 1.6 < 1.5 * 1.3 -> hysteresis holds
    for k in range(60, 120):
        cmd = pol.step(k * 0.05, ta.assess([_track(1, 28.0, 10.0)]), HealthState.OK, HealthBits.NONE)
    assert cmd.level is ThreatLevel.WARNING


def test_side_both_and_dominant(cfg):
    ta = ThreatAssessor(cfg.pipeline.threat)
    pol = WarningPolicy(cfg.pipeline.policy, ta.level_from_t)
    az = math.atan2(5.0, 8.0)
    r = math.hypot(5, 8)
    cmd = pol.step(0.0, ta.assess([_track(1, r, 10.0, az), _track(2, r + 0.5, 10.0, -az)]), HealthState.OK, HealthBits.NONE)
    assert cmd.side is Side.BOTH
    cmd = pol.step(0.1, ta.assess([_track(1, r, 10.0, az), _track(2, 25.0, 5.0, -az)]), HealthState.OK, HealthBits.NONE)
    assert cmd.side is Side.RIGHT and cmd.dominant_track_id == 1


def test_alert_never_asserted_for_fault(cfg):
    ta = ThreatAssessor(cfg.pipeline.threat)
    pol = WarningPolicy(cfg.pipeline.policy, ta.level_from_t)
    cmd = pol.step(0.0, [], HealthState.OFFLINE, HealthBits.RADAR_SILENT)
    assert cmd.level is ThreatLevel.NONE and not cmd.assert_alert and cmd.health_state is HealthState.OFFLINE
    cmd = pol.step(0.1, ta.assess([_track(1, 30.0, 10.0)]), HealthState.DEGRADED, HealthBits.IMU_FAULT)
    assert cmd.level is ThreatLevel.WARNING and not cmd.assert_alert          # warnings continue while degraded


def test_flicker_metric(cfg):
    ta = ThreatAssessor(cfg.pipeline.threat)
    pol = WarningPolicy(cfg.pipeline.policy, ta.level_from_t)
    t = 0.0
    for k in range(6):
        pol.step(t, ta.assess([_track(1, 10.0, 10.0)]), HealthState.OK, HealthBits.NONE)
        t += 1.5
        pol.step(t, [], HealthState.OK, HealthBits.NONE)        # threat resolved: dwell starts
        t += 1.2
        cmd = pol.step(t, [], HealthState.OK, HealthBits.NONE)  # dwell elapsed: de-escalates
        t += 5.0                                                 # well clear of the 3 s flicker window
    assert cmd.flicker_count == 0            # clean on/off cycles 3 s apart are not flicker
    pol2 = WarningPolicy(cfg.pipeline.policy, ta.level_from_t)
    pol2.step(0.0, ta.assess([_track(1, 10.0, 10.0)]), HealthState.OK, HealthBits.NONE)
    pol2.step(2.0, [], HealthState.OK, HealthBits.NONE)
    pol2.step(3.2, [], HealthState.OK, HealthBits.NONE)                                        # de-escalated
    cmd = pol2.step(3.5, ta.assess([_track(1, 10.0, 10.0)]), HealthState.OK, HealthBits.NONE)  # back within 3 s: flicker
    assert cmd.flicker_count == 1


def test_blockage_detector(cfg):
    bd = BlockageDetector(cfg.pipeline.blockage)
    raised = False
    for k in range(300):
        raised = bd.update(k * 0.029, in_motion=True, n_raw_targets=0, n_ego_inliers=0)
    assert raised                                             # 8.7 s of silence while moving
    assert not bd.update(9.0, in_motion=True, n_raw_targets=0, n_ego_inliers=1)   # one inlier clears it
    for k in range(300):
        raised = bd.update(10 + k * 0.029, in_motion=False, n_raw_targets=0, n_ego_inliers=0)
    assert not raised                                         # stopped rider: expected emptiness


def test_bucket():
    assert bucket_of(0.5) is TArrivalBucket.LT_1P5 and bucket_of(2) is TArrivalBucket.S1P5_3
    assert bucket_of(4) is TArrivalBucket.S3_6 and bucket_of(10) is TArrivalBucket.GT_6S and bucket_of(float("inf")) is TArrivalBucket.NONE

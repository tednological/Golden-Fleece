import math

from goldenfleece.l06_clutter_rejection.clutter import SaturationMonitor, classify
from goldenfleece.types import (DetectionClass, EgoInvalidReason, EgoMotion, EgoSource, RadarDetection, RadarFrame, StageCounters)


def _det(r, az, v_radial, i=0):
    return RadarDetection(r=r, az=az, v_radial=v_radial, v_closing=-v_radial, x=r * math.cos(az), y=r * math.sin(az), magnitude_db=50, raw_index=i)


def _frame(dets, cap=False):
    return RadarFrame(t_mid=1.0, t_header=1.0, frame_number=1, gap=0, rspi=3, rrai=2, detections=tuple(dets), n_raw=len(dets), cap_hit=cap,
                      counters=StageCounters("l03", len(dets), len(dets)))


def _ego(valid, v=(-5.0, 0.0)):
    return EgoMotion(t=1.0, v_s=v, cov=((0.01, 0), (0, 0.01)), speed=math.hypot(*v), inlier_mask=(), n_candidates=0, n_inliers=0,
                     az_spread_rad=0.5, valid=valid, invalid_reason=EgoInvalidReason.NONE if valid else EgoInvalidReason.NO_CANDIDATES,
                     source=EgoSource.RADAR_RANSAC if valid else EgoSource.NONE, psi_travel=None, psi_travel_sigma=None,
                     counters=StageCounters("l05", 0, 0))


def test_stationary_dropped_and_movers_kept(cfg):
    dets = [_det(10, 0.0, 5.0, 0), _det(12, math.radians(30), 4.33, 1), _det(20, 0.0, 2.0, 2), _det(15, 0.1, -8.0, 3)]
    out = classify(_frame(dets), _ego(True), cfg.pipeline.clutter)
    classes = [c.cls for c in out.kept]
    assert out.counters.rejected["stationary"] == 2
    assert DetectionClass.RECEDING_MOVER in classes and DetectionClass.APPROACHING in classes
    assert out.counters.n_in == 4 and out.counters.n_out == 2


def test_approaching_passes_with_ego_invalid(cfg):
    """Invariant 1: the warning path works with ego-motion invalid."""
    dets = [_det(10, 0.0, 5.0, 0), _det(15, 0.1, -8.0, 1)]
    out = classify(_frame(dets), _ego(False), cfg.pipeline.clutter)
    app = [c for c in out.kept if c.cls is DetectionClass.APPROACHING]
    assert len(app) == 1 and not app[0].ego_valid
    out2 = classify(_frame(dets), None, cfg.pipeline.clutter)
    assert any(c.cls is DetectionClass.APPROACHING for c in out2.kept)
    assert any(c.cls is DetectionClass.RECEDING_UNCLASSIFIED for c in out2.kept)


def test_saturation_monitor(cfg):
    m = SaturationMonitor(1.0, 0.5)
    sat = False
    for k in range(40):
        sat = m.update(k * 0.029, cap_hit=(k % 3 != 0))    # 2/3 of frames at cap
    assert sat and m.cap_hit_rate > 0.6
    m2 = SaturationMonitor(1.0, 0.5)
    for k in range(40):
        sat = m2.update(k * 0.029, cap_hit=(k % 5 == 0))
    assert not sat

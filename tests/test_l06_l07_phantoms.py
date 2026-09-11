"""Sensor artefacts with real-world signatures: range aliasing and wheel micro-Doppler."""
import math

from goldenfleece.l06_clutter_rejection.clutter import alias_suspect, classify
from goldenfleece.l07_tracking.tracker import Tracker
from goldenfleece.l08_threat.threat import ThreatAssessor
from goldenfleece.types import ClassifiedDetection, ClutterOutput, DetectionClass, RadarDetection, RadarFrame, StageCounters, TrackStatus

DT = 0.029


def _det(r, az, v_radial, mag=50.0, i=0):
    return RadarDetection(r=r, az=az, v_radial=v_radial, v_closing=-v_radial, x=r * math.cos(az), y=r * math.sin(az), magnitude_db=mag, raw_index=i)


def test_alias_check_rejects_weak_short_range_only(cfg):
    c = cfg.pipeline.clutter.alias_check.c_alias_db
    assert alias_suspect(1.5, 30.0, c)            # a 31.5 m car wrapped to 1.5 m: ~30 dB, far too weak
    assert alias_suspect(5.0, 28.0, c)
    assert not alias_suspect(5.0, 52.0, c)        # a pedestrian at 5 m
    assert not alias_suspect(12.0, 30.0, c)       # weak but far: plausible
    frame = RadarFrame(t_mid=0, t_header=0, frame_number=0, gap=0, rspi=3, rrai=2, detections=(_det(1.5, 0, -5.0, 30.0), _det(12.0, 0, -5.0, 45.0, 1)),
                       n_raw=2, cap_hit=False, counters=StageCounters("l03", 2, 2))
    out = classify(frame, None, cfg.pipeline.clutter)
    assert out.counters.rejected["alias_suspect"] == 1 and len(out.kept) == 1


def test_wheel_micro_doppler_phantom_is_kinematically_inconsistent(cfg):
    """A return closing at 2x the car speed whose range follows the car must never become a threat."""
    tr = Tracker(cfg.pipeline.tracker, cfg.pipeline.clutter.doppler_blind_band_mps)
    ta = ThreatAssessor(cfg.pipeline.threat)
    levels = []
    for k in range(30):
        t = k * DT
        r = 25.0 - 8.0 * t                           # car closes at 8 m/s
        body = _det(r, 0.05, -8.0, 55.0, 0)
        wheel = _det(r + 0.5, 0.06, -16.0, 40.0, 1)  # wheel top: Doppler 16 m/s, range trend 8 m/s
        kept = tuple(ClassifiedDetection(d, DetectionClass.APPROACHING, True) for d in (body, wheel))
        out = tr.step(ClutterOutput(t, k, kept, False, StageCounters("l06", 2, 2)), 0.0)
        levels.append(ta.assess(out.tracks))
    bad = [tk for tk in out.tracks if abs(tk.v_closing - 16.0) < 2.0]
    good = [tk for tk in out.tracks if abs(tk.v_closing - 8.0) < 2.0 and tk.status is TrackStatus.CONFIRMED]
    assert good and good[0].kinematic_consistent
    assert all(not tk.kinematic_consistent or tk.status is TrackStatus.TENTATIVE for tk in bad)
    assert all(a.track_id == good[0].id for a in levels[-1])
    assert ta.counters.rejected["kinematically_inconsistent"] >= 1 or not bad


def test_static_range_phantom_filter(cfg):
    from goldenfleece.l06_clutter_rejection.clutter import StaticRangePhantomFilter
    pf = cfg.pipeline.clutter.phantom_filter
    f = StaticRangePhantomFilter(pf.window_frames, pf.min_hits, pf.min_rdot_mps, pf.cell_r_m, pf.cell_rdot_mps)
    dropped_phantom = dropped_real = 0
    for k in range(30):
        phantom = _det(12.0 + 0.15 * ((-1) ** k), 0.0, -6.5, 35.0, 0)     # constant range, "closing" 6.5 m/s
        real = _det(30.0 - 6.5 * k * DT, 0.1, -6.5, 50.0, 1)              # genuinely closing
        m = f.step([phantom, real])
        dropped_phantom += m[0]
        dropped_real += m[1]
    assert dropped_phantom >= 15 and dropped_real == 0

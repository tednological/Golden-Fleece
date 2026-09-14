"""Golden vectors from task §6.3, verbatim."""
import math

import pytest

from goldenfleece.l03_radar_decode.decode import decode
from goldenfleece.types import RawRadarFrame, RawRadarTarget


def _frame(*targets, t_header=10.0, rspi=3):
    return RawRadarFrame(t_header=t_header, frame_number=7, gap=0, rspi=rspi, rrai=2,
                         targets=tuple(RawRadarTarget(*t) for t in targets), cap_hit=False, source_seq=1)


def test_car_20m_behind_3m_left(cfg):
    f = decode(_frame((2000, 0, 863, 5000)), cfg.radar)
    d = f.detections[0]
    assert d.az == pytest.approx(-0.150622, abs=1e-6)
    assert d.x == pytest.approx(19.7736, abs=1e-4)
    assert d.y == pytest.approx(-3.0011, abs=1e-4)     # LEFT => negative y


def test_car_30m_behind_right(cfg):
    f = decode(_frame((3000, 0, -2000, 5000)), cfg.radar)
    d = f.detections[0]
    assert d.az == pytest.approx(0.349066, abs=1e-6)
    assert d.x == pytest.approx(28.1908, abs=1e-4)
    assert d.y == pytest.approx(10.2606, abs=1e-4)


def test_speed_signs(cfg):
    f = decode(_frame((1000, -5400, 0, 5000), (1000, 5400, 0, 5000)), cfg.radar)
    a, b = f.detections
    assert a.v_radial == pytest.approx(-15.0) and a.v_closing == pytest.approx(15.0)
    assert b.v_closing == pytest.approx(-15.0)


def test_beam_edge_and_out_of_beam(cfg):
    f = decode(_frame((1000, 0, 4000, 5000), (1000, 0, 4001, 5000), (1000, 0, 5500, 5000), (1000, 0, -4001, 5000)), cfg.radar)
    assert len(f.detections) == 1
    assert f.counters.rejected["az_clip"] == 3
    assert f.counters.n_in == 4 and f.counters.n_out == 1


def test_midpoint_timestamp_and_passthrough(cfg):
    f = decode(_frame(t_header=10.0, rspi=3), cfg.radar)
    assert f.t_mid == pytest.approx(10.0 - cfg.radar.sensor_delay_s - cfg.radar.frame_duration_s[3] / 2)
    assert f.frame_number == 7 and f.rspi == 3 and f.gap == 0
    f0 = decode(_frame(t_header=10.0, rspi=0), cfg.radar)
    assert f0.t_mid == pytest.approx(10.0 - cfg.radar.sensor_delay_s - 0.229 / 2)


def test_no_z_on_detections(cfg):
    f = decode(_frame((1000, 0, 0, 5000)), cfg.radar)
    d = f.detections[0]
    assert not hasattr(d, "z") and d.elev_min < 0 < d.elev_max

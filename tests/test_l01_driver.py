"""Driver state machine against the simulated sensor: baud recovery, parameter write/verify, framing,
timestamps, gaps, silence/reconnect, resync on garbage."""
import pytest

from goldenfleece.clock import FakeClock
from goldenfleece.l01_radar_data_input import protocol as K
from goldenfleece.l01_radar_data_input.driver import DirectKld7Source, DriverError
from goldenfleece.types import HealthBits, RadarConfigChanged, RawRadarTarget
from tools.fake_kld7 import FakeKld7Serial


def _gen(fn, t):
    return [RawRadarTarget(2000 - (fn % 500), -5400, 863, 5000), RawRadarTarget(1000, 500, -200, 4000)]


def _make(cfg, **fake_kw):
    clk = FakeClock(10.0)
    fake = FakeKld7Serial(clk, _gen, **fake_kw)

    def opener(port, baud):
        fake.baudrate = baud
        fake.closed = False
        return fake
    drv = DirectKld7Source(cfg.radar, clk, open_serial=opener)
    return drv, fake, clk


def test_connect_recovers_stale_high_baud_and_verifies_params(cfg):
    drv, fake, clk = _make(cfg, sensor_baud=921600)        # previous process died without GBYE
    assert drv._connect()
    assert fake.sensor_baud == cfg.radar.baudrate
    assert "GBYE" in fake.log and fake.log.index("INIT") < fake.log.index("GRPS") < fake.log.index("SRPS")
    assert fake.params.max_range == 2 and fake.params.max_speed == 3
    assert drv.firmware_version == "K-LD7_APP-RFB-0104"
    ev = drv.drain_events()
    rc = [e for e in ev if isinstance(e, RadarConfigChanged)]
    assert rc and rc[0].firmware_version == "K-LD7_APP-RFB-0104" and rc[0].rspi == 3 and rc[0].rrai == 2
    assert any(isinstance(e, HealthBits.__class__) is False and getattr(e, "bit", None) is HealthBits.RADAR_CONFIG_MISMATCH and not e.active for e in ev)


def test_frames_flow_with_timestamps_and_no_gaps(cfg):
    drv, fake, clk = _make(cfg)
    assert drv._connect()
    frames = []
    for _ in range(5):
        drv._poll_cycle()
        f = drv.get(0.0)
        if f is not None:
            frames.append(f)
    assert len(frames) == 5
    fns = [f.frame_number for f in frames]
    assert fns == list(range(fns[0], fns[0] + 5)) and all(f.gap == 0 for f in frames)
    assert frames[0].targets[0].angle_raw == 863 and frames[0].rspi == 3 and frames[0].rrai == 2
    # t_header ~ when the sensor started transmitting (frame end + delay), before the payload finished arriving
    expected = frames[1].frame_number * fake.frame_period_s + fake.sensor_delay_s
    assert abs(frames[1].t_header - expected) < 0.003
    assert drv.stats.frames == 5 and drv.stats.gaps == 0


def test_gap_detection(cfg):
    drv, fake, clk = _make(cfg)
    assert drv._connect()
    first = None
    for _ in range(3):
        drv._poll_cycle()
        f = drv.get(0.0)
        if f is not None and first is None:
            first = f.frame_number
    fake.skip_frames.update({first + 4, first + 5})
    gaps = []
    for _ in range(8):
        drv._poll_cycle()
        f = drv.get(0.0)
        if f is not None:
            gaps.append(f.gap)
    assert max(gaps) >= 2 and drv.stats.gap_frames_missed >= 2


def test_parameter_rejection_refuses_to_stream(cfg):
    drv, fake, clk = _make(cfg, reject_srps_code=K.RespCode.INVALID_RPST_VERSION)
    assert not drv._connect()
    ev = [e for e in drv.drain_events() if hasattr(e, "bit")]
    assert any(e.bit is HealthBits.RADAR_CONFIG_MISMATCH and e.active for e in ev)
    assert drv.ser is None


def test_silence_raises_radar_silent_then_reconnects(cfg):
    drv, fake, clk = _make(cfg)
    assert drv._connect()
    for _ in range(3):
        drv._poll_cycle()
    fake.silent_from = clk.now()
    fake.silent_until = clk.now() + 2.0
    with pytest.raises(DriverError):
        for _ in range(5):
            drv._poll_cycle()
    ev = [e for e in drv.drain_events() if hasattr(e, "bit")]
    assert any(e.bit is HealthBits.RADAR_SILENT and e.active for e in ev)
    assert drv.stats.poll_timeouts >= 3
    # sensor comes back: reconnect succeeds and frames resume
    clk.advance(2.5)
    drv._disconnect(send_gbye=False)
    assert drv._connect()
    drv._poll_cycle()
    assert drv.get(0.0) is not None
    ev = [e for e in drv.drain_events() if hasattr(e, "bit")]
    assert any(e.bit is HealthBits.RADAR_SILENT and not e.active for e in ev)


def test_resync_on_garbage(cfg):
    drv, fake, clk = _make(cfg, corrupt_every_n=2)
    assert drv._connect()
    got = 0
    for _ in range(6):
        drv._poll_cycle()
        if drv.get(0.0) is not None:
            got += 1
    assert got == 6 and drv.stats.resync_bytes >= 4


def test_threaded_smoke(cfg):
    from goldenfleece.clock import MonotonicClock
    clk = MonotonicClock()
    fake = FakeKld7Serial(clk, _gen)

    def opener(port, baud):
        fake.baudrate = baud
        return fake
    drv = DirectKld7Source(cfg.radar, clk, open_serial=opener)
    drv.start()
    f = None
    for _ in range(50):
        f = drv.get(0.1)
        if f is not None:
            break
    drv.stop()
    assert f is not None and f.frame_number >= 0

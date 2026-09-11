"""Framing round trip, corruption rejection, heartbeat honesty / stall detection, alert staleness,
restart ordering, outage log."""
import pytest

from goldenfleece.clock import FakeClock
from goldenfleece.l10_mcu_link import protocol as P
from goldenfleece.l10_mcu_link.emulator import McuEmulator, McuMode, OutageCause, Render
from goldenfleece.types import HealthBits, HealthState, Side, TArrivalBucket, ThreatLevel, WarningCommand


def _cmd(level=ThreatLevel.NONE, side=Side.NONE, hs=HealthState.OK, bits=HealthBits.NONE, seq=1, t=1.0):
    return WarningCommand(seq=seq, t_decided=t, level=level, side=side, t_arrival_bucket=TArrivalBucket.NONE, health_state=hs,
                          health_bits=bits, assert_alert=(level == ThreatLevel.ALERT), dominant_track_id=None, flicker_count=0)


def test_crc_reference_vector():
    assert P.crc16_ccitt_false(b"123456789") == 0x29B1        # CRC-16/CCITT-FALSE check value


def test_round_trip_and_corruption():
    line = P.build_warn(7, _cmd(ThreatLevel.WARNING, Side.LEFT, HealthState.DEGRADED, HealthBits.IMU_FAULT))
    assert line.startswith(b"$GF1,WARN,7,2,L,0,DEGRADED,10,") and line.endswith(b"\n") and len(line) <= P.MAX_LINE
    m = P.decode_line(line)
    v = P.parse_warn(m)
    assert v.level is ThreatLevel.WARNING and v.side is Side.LEFT and v.health_bits == HealthBits.IMU_FAULT
    bad = bytearray(line)
    bad[10] ^= 0x01
    with pytest.raises(P.FrameError):
        P.decode_line(bytes(bad))
    with pytest.raises(P.FrameError):
        P.decode_line(line[:-6] + b"0000\n")
    sp = P.LineSplitter()
    lines = sp.feed(b"garbage\xff\x00" + line[:5])
    assert lines == [] or lines == [b"garbage\xff\x00\n"] and sp.n_garbage_bytes >= 0
    lines = sp.feed(line[5:] + b"xx" + line)
    assert len(lines) == 2 and lines[1] == line
    assert sp.n_garbage_bytes >= 9


def test_all_builders_parse():
    for line in (P.build_alert(1, True), P.build_hb(2, 10, 5, 99, HealthState.OK, HealthBits.NONE),
                 P.build_rcfg(3, 2, 3, 921600, 50, 1500, 3000), P.build_hlth(4, HealthState.OFFLINE, HealthBits.RADAR_SILENT, 1234),
                 P.build_stat(5, "NORMAL", 100, 3, 20, 0), P.build_outg(6, 100, 250, "HB_STALL")):
        m = P.decode_line(line)
        assert m.seq in range(1, 7) and len(line) <= P.MAX_LINE


def _hb(seq, loop, frames, hs=HealthState.OK, bits=HealthBits.NONE):
    return P.build_hb(seq, loop, frames, frames, hs, bits)


def test_progress_stall_triggers_fallback_even_with_bytes_arriving():
    """Heartbeat honesty: a beating heartbeat whose progress counter does not advance is a stall."""
    clk = FakeClock(0.0)
    mcu = McuEmulator(clk)
    for k in range(10):
        clk.advance(0.05)
        mcu.feed(_hb(k, loop=k, frames=k))
    assert mcu.mode is McuMode.NORMAL and mcu.rendered.render is Render.NONE
    # process alive but pipeline deadlocked: same loop counter forever
    t_stall = clk.now()
    fell = None
    for k in range(10, 30):
        clk.advance(0.05)
        mcu.feed(_hb(k, loop=9, frames=9))
        if mcu.mode is McuMode.FALLBACK and fell is None:
            fell = clk.now()
    assert fell is not None and (fell - t_stall) <= 0.31
    assert mcu.rendered.render is Render.FALLBACK_OFFLINE
    assert mcu.outages and mcu.outages[-1].cause is OutageCause.HB_STALL
    # recovery
    for k in range(30, 40):
        clk.advance(0.05)
        mcu.feed(_hb(k, loop=k, frames=k))
    assert mcu.mode is McuMode.NORMAL and mcu.outages[-1].end_s is not None


def test_absent_heartbeat_and_worst_case_time():
    clk = FakeClock(0.0)
    mcu = McuEmulator(clk)
    for k in range(5):
        clk.advance(0.05)
        mcu.feed(_hb(k, k, k))
    t_hang = clk.now()
    while mcu.mode is McuMode.NORMAL:
        clk.advance(0.01)
        mcu.poll()
    assert clk.now() - t_hang <= 0.26
    assert mcu.outages[-1].cause is OutageCause.HB_ABSENT


def test_alert_is_a_threat_only_while_fresh_and_heartbeat_alive():
    clk = FakeClock(0.0)
    mcu = McuEmulator(clk)
    for k in range(5):
        clk.advance(0.05)
        mcu.feed(_hb(k, k, k))
    mcu.feed(P.build_alert(1, True) + P.build_warn(2, _cmd(ThreatLevel.ALERT, Side.RIGHT)))
    assert mcu.rendered.alert and mcu.rendered.render is Render.THREAT and mcu.rendered.level is ThreatLevel.ALERT
    # refreshed alert stays asserted
    for k in range(5, 10):
        clk.advance(0.05)
        mcu.feed(_hb(k, k, k) + P.build_alert(k, True))
    assert mcu.rendered.alert
    # stuck: no refresh for > alert_stale_s while heartbeat alive -> fault, not threat
    for k in range(10, 20):
        clk.advance(0.05)
        mcu.feed(_hb(k, k, k))
    assert not mcu.rendered.alert and mcu.n_alert_stale_events == 1
    assert any(o.cause is OutageCause.ALERT_STALE for o in mcu.outages)
    # asserted but heartbeat dies -> fallback, alert dropped
    mcu.feed(P.build_alert(21, True))
    clk.advance(0.4)
    mcu.poll()
    assert mcu.mode is McuMode.FALLBACK and not mcu.rendered.alert


def test_fault_never_rendered_as_threat_and_degraded_keeps_threat():
    clk = FakeClock(0.0)
    mcu = McuEmulator(clk)
    clk.advance(0.05)
    mcu.feed(_hb(1, 1, 1, HealthState.OFFLINE, HealthBits.RADAR_SILENT))
    assert mcu.rendered.render is Render.OFFLINE and mcu.rendered.level is ThreatLevel.NONE and not mcu.rendered.alert
    assert mcu.outages[-1].cause is OutageCause.PI_OFFLINE
    clk.advance(0.05)
    mcu.feed(_hb(2, 2, 2, HealthState.DEGRADED, HealthBits.IMU_FAULT) + P.build_warn(3, _cmd(ThreatLevel.WARNING, Side.LEFT, HealthState.DEGRADED, HealthBits.IMU_FAULT)))
    assert mcu.rendered.render is Render.DEGRADED and mcu.rendered.level is ThreatLevel.WARNING   # warning continues while degraded


def test_restart_announced_before_any_threat():
    clk = FakeClock(0.0)
    mcu = McuEmulator(clk)
    clk.advance(0.01)
    mcu.feed(P.build_hlth(1, HealthState.OFFLINE, HealthBits.PIPELINE_RESTARTING, 0) + _hb(2, 1, 0, HealthState.OFFLINE, HealthBits.PIPELINE_RESTARTING))
    assert mcu.rendered.render is Render.OFFLINE
    clk.advance(0.05)
    mcu.feed(_hb(3, 2, 1) + P.build_warn(4, _cmd(ThreatLevel.WARNING, Side.LEFT)))
    assert mcu.restart_announced_before_threat is True
    assert mcu.rendered.render is Render.THREAT


def test_outage_lines_and_status():
    clk = FakeClock(0.0)
    mcu = McuEmulator(clk)
    clk.advance(0.05)
    mcu.feed(_hb(1, 1, 1))
    clk.advance(1.0)
    mcu.poll()
    clk.advance(0.05)
    mcu.feed(_hb(2, 2, 2))
    st = P.parse_stat(P.decode_line(mcu.status_line()))
    assert st.mode == "NORMAL" and st.outages == 1
    outg = mcu.outage_lines()
    assert len(outg) == 1 and b"HB_ABSENT" in outg[0]

"""l02 BNO085 SPI adapter against a fake sensor that behaves like the part measured on 2026-09-14:
startup by riding writes on the sensor's own transfers, every report a sample stamped at INT, recovery from
sensor resets and silence, and the fake reproducing the two hardware findings the adapter is built around."""
import pytest

pytest.importorskip("adafruit_bno08x")

from goldenfleece.clock import MonotonicClock                                          # noqa: E402
from goldenfleece.l02_imu_data_input.bno085_spi import Bno085SpiSource, ShtpSpi, shtp_packet   # noqa: E402
from goldenfleece.types import HealthBits, ImuKind                                    # noqa: E402
from tools.fake_bno085 import FakeBno085                                              # noqa: E402

GYRO, ACCEL = 0x02, 0x01


def _make(cfg, clock):
    fake = FakeBno085(clock)
    return Bno085SpiSource(cfg.pipeline.imu, clock, device_factory=lambda: fake), fake


def _poll_for(src, clock, seconds):
    end = clock.now() + seconds
    while clock.now() < end:
        src._poll_once()


def _step_for(src, clock, seconds):
    end = clock.now() + seconds
    while clock.now() < end:
        src._step()


def _boot(fake, clock):
    fake.set_reset(False)
    fake.set_reset(True)


def test_fake_ignores_writes_while_int_is_deasserted(clock):
    """Bench phase A: with PS0/WAKE tied high the sensor drops a host write unless H_INTN is asserted."""
    fake = FakeBno085(clock)
    _boot(fake, clock)
    clock.advance(1.0)
    shtp = ShtpSpi(fake)
    while fake.int_asserted():
        shtp.transfer()
    shtp.send(2, bytes([0xF9, 0]))
    clock.advance(0.5)
    assert fake.ignored and not fake.accepted and not fake.int_asserted()


def test_fake_reproduces_the_stock_driver_failure(clock):
    """A plain 6-byte write while the 20-byte boot packet is pending (what adafruit_bno08x's SPI class does)
    leaves an 18-byte continuation: the 0x12 0x80 header seen on hardware."""
    fake = FakeBno085(clock)
    _boot(fake, clock)
    clock.advance(0.1)
    ShtpSpi(fake).transfer()                               # the advertisement, read in full
    clock.advance(0.1)
    with fake.transaction() as exchange:
        exchange(shtp_packet(2, 0, bytes([0xF9, 0])))      # write only; the input is discarded
    hdr = fake.queue[0][1][:4]
    assert fake.continuations == 1 and hdr[0] == 0x12 and hdr[1] & 0x80 and hdr[2] == 2


def test_startup_rides_every_write_on_a_sensor_transfer(cfg, clock):
    src, fake = _make(cfg, clock)
    assert src._init_device()
    assert set(fake.features) == {GYRO, ACCEL}
    assert fake.features[GYRO][0] == pytest.approx(0.005) and fake.features[ACCEL][0] == pytest.approx(0.010)
    assert not fake.ignored and fake.continuations == 0
    ev = src.drain_events()
    assert ev[-1].bit is HealthBits.IMU_FAULT and not ev[-1].active


def test_every_report_is_a_sample_stamped_at_int(cfg, clock):
    src, fake = _make(cfg, clock)
    assert src._init_device()
    src.drain()
    _poll_for(src, clock, 1.0)
    s = src.drain()
    g = [x for x in s if x.kind is ImuKind.GYRO]
    a = [x for x in s if x.kind is ImuKind.ACCEL]
    assert 199 <= len(g) <= 201 and 99 <= len(a) <= 101
    dts = [b.t - a_.t for a_, b in zip(g[:-1], g[1:])]
    assert max(abs(d - 0.005) for d in dts) < 0.001
    assert a[-1].values == pytest.approx((0.0, 0.0, 9.81), abs=0.01)     # the library's parser, SI units
    seqs = [x.seq for x in s]
    assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs)
    assert fake.continuations == 0


def test_sensor_reset_mid_stream_is_reported_and_recovered(cfg, clock):
    src, fake = _make(cfg, clock)
    _step_for(src, clock, 0.3)
    src.drain()
    src.drain_events()
    fake.spontaneous_reset()
    _step_for(src, clock, 1.0)
    ev = src.drain_events()
    assert any(e.active and "reset" in e.detail for e in ev) and not ev[-1].active
    assert set(fake.features) == {GYRO, ACCEL} and src.n_reinit == 1
    assert any(x.kind is ImuKind.GYRO for x in src.drain())


def test_unplugged_sensor_raises_fault_and_recovers_when_back(cfg, clock):
    src, fake = _make(cfg, clock)
    _step_for(src, clock, 0.3)
    fake.dead = True
    _step_for(src, clock, 3.0)
    ev = src.drain_events()
    assert any(e.active and "silent" in e.detail for e in ev)
    assert any(e.active and e.detail.startswith("init failed") for e in ev)
    src.drain()
    fake.dead = False
    _step_for(src, clock, 6.0)
    assert src.drain_events()[-1].active is False
    assert any(x.kind is ImuKind.GYRO for x in src.drain())


def test_threaded_smoke(cfg):
    clk = MonotonicClock()
    fake = FakeBno085(clk)
    src = Bno085SpiSource(cfg.pipeline.imu, clk, device_factory=lambda: fake)
    src.start()
    got = []
    end = clk.now() + 2.0
    while clk.now() < end and not any(s.kind is ImuKind.ACCEL for s in got):
        got += src.drain()
        clk.sleep(0.02)
    src.stop()
    assert any(s.kind is ImuKind.GYRO for s in got) and any(s.kind is ImuKind.ACCEL for s in got)
    assert fake.closed

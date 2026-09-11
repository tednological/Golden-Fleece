import json

from goldenfleece.clock import FakeClock, MonotonicClock
from goldenfleece.orchestrator import recording as R
from goldenfleece.types import HealthBits, HealthEvent, ImuKind, RawImuSample, RawRadarFrame, RawRadarTarget
from tools.outage_report import analyse, format_report
from goldenfleece.types import HealthState


def test_radar_and_imu_round_trip():
    f = RawRadarFrame(t_header=12.5, frame_number=42, gap=1, rspi=3, rrai=2, targets=(RawRadarTarget(2000, -5400, 863, 5000),), cap_hit=False, source_seq=9)
    assert R.radar_from_rec(json.loads(json.dumps(R.rec_radar(f)))) == f
    s = RawImuSample(t=1.25, kind=ImuKind.GYRO, values=(0.1, -0.2, 0.3), seq=5)
    assert R.imu_from_rec(json.loads(json.dumps(R.rec_imu(s)))) == s


def test_writer_thread_and_outage_report(tmp_path):
    clk = MonotonicClock()
    path = tmp_path / "session.jsonl"
    w = R.RecordingWriter(path, clk, queue_depth=64, flush_period_s=0.05)
    w.start()
    w.put(R.rec_header(100.0, {"rspi": 3}, "K-LD7_APP-RFB-0104", "abc"))
    w.put(R.rec_health(HealthEvent(101.0, HealthBits.RADAR_GAPS, True, "l01")))
    w.put(R.rec_health(HealthEvent(103.5, HealthBits.RADAR_GAPS, False, "l01")))
    w.put(R.rec_health(HealthEvent(110.0, HealthBits.RADAR_SILENT, True, "l01")))
    w.put(R.rec_health(HealthEvent(110.2, HealthBits.IMU_FAULT, True, "l04")))
    w.put(R.rec_health(HealthEvent(111.0, HealthBits.RADAR_SILENT, False, "l01")))
    w.put(R.rec_health(HealthEvent(112.0, HealthBits.IMU_FAULT, False, "l04")))
    for k in range(10):
        w.put(R.rec_radar(RawRadarFrame(120.0 + k * 0.029, k, 0, 3, 2, (), False, k)))
    w.close()
    rep = analyse(path)
    assert rep.frames == 10 and rep.drops == 0
    assert rep.count(HealthState.DEGRADED) == 2 and rep.count(HealthState.OFFLINE) == 1
    assert abs(rep.total_time(HealthState.OFFLINE) - 1.0) < 1e-6
    lg = rep.longest()
    assert lg.state is HealthState.DEGRADED and abs(lg.duration - 2.5) < 1e-6
    assert "IMU_FAULT" in [c for e in rep.episodes for c in e.causes]
    assert rep.riding_time_s > 20
    txt = format_report(rep)
    assert "OFFLINE" in txt and "longest" in txt


def test_writer_drops_are_counted_not_blocking(tmp_path):
    clk = FakeClock(0.0)
    w = R.RecordingWriter(tmp_path / "x.jsonl", clk, queue_depth=4)
    # not started: queue fills, put never blocks
    for k in range(10):
        w.put({"k": "imu", "t": k})
    assert w.drops == 6

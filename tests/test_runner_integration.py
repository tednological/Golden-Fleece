"""Runner + link + MCU emulator on the simulator (deterministic FakeClock):
pipeline stall -> emulator fallback within its timeout and 'warnings offline' under PI_POLLS;
crash/restart -> PIPELINE_RESTARTING reaches the emulator before any threat; link loss -> MCU_LINK_DOWN;
full pipeline warns on an overtake; recording replays bit-identically."""
import json
from pathlib import Path

import pytest

from goldenfleece.clock import FakeClock
from goldenfleece.l02_imu_data_input.source import QueueImuSource
from goldenfleece.l10_mcu_link.emulator import McuEmulator, McuMode, Render
from goldenfleece.l10_mcu_link.link import LoopbackTransport, McuLink
from goldenfleece.orchestrator.recording import RecordingWriter
from goldenfleece.orchestrator.runner import Runner
from goldenfleece.types import HealthBits, HealthState, ThreatLevel
from tools.sim.imu_model import ImuModel
from tools.sim.kinematics import RiderKinematics
from tools.sim.radar_model import RadarModel
from tools.sim.scenarios import BY_NAME
from tools.sim.sources import SimRadarSource


def _build(cfg, name, duration=None, recorder=None):
    sc = BY_NAME[name]()
    clock = FakeClock(0.0)
    rider = RiderKinematics(sc.world.road, sc.rider)
    radar = RadarModel(sc.world, rider, sc.radar)
    imu = ImuModel(rider, sc.imu)
    imu_src = QueueImuSource()
    radar_src = SimRadarSource(radar, imu, imu_src, clock, duration or sc.duration_s)
    mcu = McuEmulator(clock, stall_timeout_s=cfg.pipeline.link.mcu_stall_timeout_s, absent_timeout_s=cfg.pipeline.link.mcu_absent_timeout_s,
                      alert_stale_s=cfg.pipeline.link.alert_stale_s)
    transport = LoopbackTransport(mcu.feed)
    link_events = []
    link = McuLink(cfg.pipeline.link, clock, lambda: transport, on_link_state=lambda up, t, why: link_events.append((up, t, why)), synchronous=True)
    link.start()
    runner = Runner(cfg, clock, radar_src, imu_src, link, recorder=recorder)
    return runner, mcu, transport, radar_src, clock, link_events


def test_full_pipeline_warns_and_restart_is_announced_first(cfg):
    runner, mcu, transport, src, clock, _ = _build(cfg, "overtake_left_10mps")
    runner.announce_restart()
    assert mcu.rendered.render is Render.OFFLINE and (mcu.health_bits & HealthBits.PIPELINE_RESTARTING)
    max_level = 0
    while not src.finished:
        runner.step()
        mcu.poll()
        max_level = max(max_level, int(mcu.rendered.level))
        assert mcu.mode is McuMode.NORMAL, "a healthy pipeline must never trip the MCU watchdog"
    assert max_level >= int(ThreatLevel.WARNING)
    assert mcu.restart_announced_before_threat is True
    assert mcu.n_crc_errors == 0
    assert runner.latency.e2e_pi.n > 100
    assert runner.pipe.frames_processed == runner.frames_seen


def test_pipeline_stall_trips_fallback_within_timeout(cfg):
    runner, mcu, transport, src, clock, _ = _build(cfg, "overtake_left_10mps", duration=2.0)
    runner.announce_restart()
    for _ in range(40):
        runner.step()
        mcu.poll()
    assert mcu.mode is McuMode.NORMAL
    # Pi hang proxy: the loop stops; the emulator's clock keeps running
    t_hang = clock.now()
    fell = None
    for _ in range(20):
        clock.advance(0.02)
        mcu.poll()
        if mcu.mode is McuMode.FALLBACK:
            fell = clock.now()
            break
    assert fell is not None and (fell - t_hang) <= cfg.pipeline.link.mcu_absent_timeout_s + 0.021
    assert mcu.rendered.render is Render.FALLBACK_OFFLINE        # "warnings offline" under PI_POLLS
    assert not mcu.rendered.alert
    # recovery when the loop resumes
    for _ in range(10):
        runner.step()
        mcu.poll()
    assert mcu.mode is McuMode.NORMAL and mcu.outages[-1].end_s is not None


def test_stalled_progress_with_bytes_flowing_is_detected(cfg):
    """Heartbeat honesty at the runner level: if the loop stopped processing but something kept sending the
    same heartbeat, the MCU still falls back."""
    runner, mcu, transport, src, clock, _ = _build(cfg, "overtake_left_10mps", duration=2.0)
    runner.announce_restart()
    for _ in range(20):
        runner.step()
        mcu.poll()
    frozen_iter = runner.pipe.loop_iter
    for _ in range(20):
        clock.advance(0.03)
        runner.link.send_heartbeat(frozen_iter, runner.pipe.frames_processed, runner.pipe.last_frame_number, HealthState.OK, HealthBits.NONE)
        mcu.poll()
    assert mcu.mode is McuMode.FALLBACK and mcu.outages[-1].cause.value == "HB_STALL"


def test_link_loss_is_reported_and_recovers(cfg):
    runner, mcu, transport, src, clock, events = _build(cfg, "overtake_left_10mps", duration=3.0)
    runner.announce_restart()
    for _ in range(10):
        runner.step()
    transport.fail_writes = True
    for _ in range(5):
        runner.step()
    assert any(up is False for up, _, _ in events)
    assert runner.link.stats.write_errors >= 1
    transport.fail_writes = False
    for _ in range(10):
        runner.step()
        mcu.poll()
    assert runner.link.up


def test_alert_channel_precedes_warning_and_is_never_set_for_faults(cfg):
    runner, mcu, transport, src, clock, _ = _build(cfg, "overtake_left_20mps")
    order = []
    orig = mcu.feed

    def feed(data):
        for line in data.split(b"\n"):
            if line.startswith(b"$GF1,ALERT") or line.startswith(b"$GF1,WARN,"):
                order.append(line.split(b",")[1])
        return orig(data)
    transport._sink = feed
    runner.announce_restart()
    while not src.finished:
        runner.step()
        mcu.poll()
        if mcu.rendered.alert:
            assert mcu.rendered.level is ThreatLevel.ALERT and mcu.health is not HealthState.OFFLINE
    first_alert = order.index(b"ALERT") if b"ALERT" in order else None
    assert first_alert is not None
    assert order[first_alert + 1] == b"WARN"       # alert asserted before the WARN of the same decision


def test_recording_replays_bit_identically(cfg, tmp_path):
    path = tmp_path / "s.jsonl"
    rec = RecordingWriter(path, FakeClock(0.0), queue_depth=200000, flush_period_s=0.1)
    rec.start()
    runner, mcu, transport, src, clock, _ = _build(cfg, "overtake_left_10mps", duration=4.0, recorder=rec)
    runner.announce_restart()
    live = []
    while not src.finished:
        if runner.step():
            live.append((runner.pipe.last_frame_number, int(runner.pipe.last_command.level), runner.pipe.last_command.side.value))
    rec.close()
    assert rec.drops == 0
    from tools.replay import replay
    r1 = replay(str(path), cfg)
    r2 = replay(str(path), cfg)
    assert r1["outputs"] == r2["outputs"]
    rep = [(o["fn"], o["lvl"], o["side"]) for o in r1["outputs"]]
    assert rep == live


def test_undervoltage_flag_reaches_mcu(cfg):
    """§13.2 undervoltage: the power monitor's flag becomes a DEGRADED health bit in the MCU's heartbeat within ~1 s."""
    from goldenfleece.orchestrator.power_monitor import PowerMonitor
    runner, mcu, transport, src, clock, _ = _build(cfg, "overtake_left_10mps", duration=3.0)
    flags = {"v": "throttled=0x0"}
    pm = PowerMonitor(clock, period_s=1.0, reader=lambda: flags["v"])
    runner.power = pm
    runner.announce_restart()
    for _ in range(10):
        runner.step()
        mcu.poll()
    assert not (mcu.health_bits & HealthBits.UNDERVOLTAGE)
    flags["v"] = "throttled=0x50001"          # under-voltage now + occurred bits
    pm.sample_once()                            # what the adapter thread does once per period
    t0 = clock.now()
    while not (mcu.health_bits & HealthBits.UNDERVOLTAGE):
        runner.step()
        mcu.poll()
        assert clock.now() - t0 < 1.1
    assert mcu.health is HealthState.DEGRADED
    flags["v"] = "throttled=0x50000"           # only the sticky "occurred" bits remain
    pm.sample_once()
    for _ in range(5):
        runner.step()
        mcu.poll()
    assert not (mcu.health_bits & HealthBits.UNDERVOLTAGE)

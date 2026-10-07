"""Runner + haptics on the simulator (deterministic FakeClock):
pipeline stall -> the render thread's watchdog falls back to 'warnings offline' within its timeout;
crash/restart -> PIPELINE_RESTARTING is rendered before any threat; a lost motor -> HAPTICS_FAULT (OFFLINE) and
recovery; full pipeline warns on an overtake, on the correct side's motor; recording replays bit-identically."""
import json

from goldenfleece.clock import FakeClock
from goldenfleece.l02_imu_data_input.source import QueueImuSource
from goldenfleece.l10_haptics.motors import MotorError, NullMotor
from goldenfleece.l10_haptics.output import HapticOutput
from goldenfleece.l10_haptics.patterns import HapticMode, Render, haptics_config
from goldenfleece.orchestrator.recording import RecordingWriter
from goldenfleece.orchestrator.runner import Runner
from goldenfleece.types import HealthBits, HealthState, Side, ThreatLevel
from tools.sim.imu_model import ImuModel
from tools.sim.kinematics import RiderKinematics
from tools.sim.radar_model import RadarModel
from tools.sim.scenarios import BY_NAME
from tools.sim.sources import SimRadarSource


class FlakyMotor(NullMotor):
    def __init__(self, name, broken):
        super().__init__(name)
        self.broken = broken

    def set(self, on):
        if self.broken["now"]:
            raise MotorError(f"{self.name}: write failed")
        super().set(on)


def _build(cfg, name, duration=None, recorder=None):
    sc = BY_NAME[name]()
    clock = FakeClock(0.0)
    rider = RiderKinematics(sc.world.road, sc.rider)
    radar = RadarModel(sc.world, rider, sc.radar)
    imu = ImuModel(rider, sc.imu)
    imu_src = QueueImuSource()
    radar_src = SimRadarSource(radar, imu, imu_src, clock, duration or sc.duration_s)
    motors, broken = {}, {"now": False}

    def factory(m):
        motors[m.name] = FlakyMotor(m.name, broken)
        return motors[m.name]
    haptics = HapticOutput(haptics_config(cfg.pipeline.haptics), clock, factory, synchronous=True)
    haptics.start()
    runner = Runner(cfg, clock, radar_src, imu_src, haptics, recorder=recorder)
    return runner, haptics, motors, broken, radar_src, clock


def test_full_pipeline_warns_on_the_left_motor_and_restart_is_rendered_first(cfg):
    runner, haptics, motors, _, src, clock = _build(cfg, "overtake_left_10mps")
    runner.announce_restart()
    assert haptics.state.render is Render.OFFLINE and haptics.motor_states() == (True, True)   # before any threat
    renders, left_alone = [haptics.state], False
    while not src.finished:
        runner.step()
        assert haptics.mode is HapticMode.NORMAL, "a healthy pipeline must never trip the haptics watchdog"
        renders.append(haptics.state)
        assert not (motors["right"].on and not motors["left"].on), "a car on the rider's left never buzzes only the right"
        if haptics.state.side is Side.LEFT:
            assert not motors["right"].on
            left_alone |= bool(motors["left"].on)
    assert max(int(r.level) for r in renders) >= int(ThreatLevel.WARNING)
    assert left_alone                             # the pass itself is felt on the left motor only (CENTER uses both)
    assert runner.latency.e2e_pi.n > 100
    assert runner.pipe.frames_processed == runner.frames_seen


def test_pipeline_stall_trips_fallback_within_timeout(cfg):
    runner, haptics, motors, _, src, clock = _build(cfg, "overtake_left_10mps", duration=2.0)
    runner.announce_restart()
    for _ in range(40):
        runner.step()
    assert haptics.mode is HapticMode.NORMAL
    # Pi-hang proxy: the loop stops; the render thread's clock keeps running
    t_hang = clock.now()
    fell = None
    for _ in range(20):
        clock.advance(0.02)
        haptics.poll()
        if haptics.mode is HapticMode.FALLBACK:
            fell = clock.now()
            break
    assert fell is not None and (fell - t_hang) <= cfg.pipeline.haptics.stall_timeout_s + 0.021
    assert haptics.state.render is Render.FALLBACK_OFFLINE and not haptics.state.alert       # "warnings offline"
    assert haptics.motor_states() == (True, True)
    assert haptics.drain_changes()[-1].cause == "HB_ABSENT"
    # recovery when the loop resumes
    runner.step()
    assert haptics.mode is HapticMode.NORMAL and haptics.stats.fallbacks == 1


def test_stalled_progress_with_heartbeats_flowing_is_detected(cfg):
    """Heartbeat honesty at the runner level: if the loop stopped processing but something kept handing over the
    same counter, the haptics still fall back."""
    runner, haptics, motors, _, src, clock = _build(cfg, "overtake_left_10mps", duration=2.0)
    runner.announce_restart()
    for _ in range(20):
        runner.step()
    frozen_iter = runner.pipe.loop_iter
    for _ in range(20):
        clock.advance(0.03)
        runner.haptics.send_heartbeat(frozen_iter)
    assert haptics.mode is HapticMode.FALLBACK
    assert [c.cause for c in haptics.drain_changes() if c.cause] == ["HB_STALL"]


def test_lost_motor_raises_haptics_fault_and_recovers(cfg):
    runner, haptics, motors, broken, src, clock = _build(cfg, "overtake_left_10mps", duration=4.0)
    runner.announce_restart()
    for _ in range(10):
        runner.step()
    broken["now"] = True
    t0 = clock.now()
    while not runner.health.bits & HealthBits.HAPTICS_FAULT:
        runner.step()
        assert clock.now() - t0 < 2.0
    assert runner.health.state is HealthState.OFFLINE and runner.pipe.last_command.health_state is HealthState.OFFLINE
    assert haptics.stats.write_errors >= 1 and not haptics.up
    broken["now"] = False
    t0 = clock.now()
    while runner.health.bits & HealthBits.HAPTICS_FAULT:
        runner.step()
        assert clock.now() - t0 < 2.0
    assert haptics.up and runner.telemetry.haptics["up"]


def test_alert_pattern_only_for_level_3_and_never_for_faults(cfg):
    runner, haptics, motors, _, src, clock = _build(cfg, "overtake_left_20mps")
    runner.announce_restart()
    alerts = 0
    while not src.finished:
        runner.step()
        rs = haptics.state
        if rs.alert:
            alerts += 1
            assert rs.level is ThreatLevel.ALERT and rs.render in (Render.THREAT, Render.DEGRADED)
        if rs.render in (Render.OFFLINE, Render.FALLBACK_OFFLINE):
            assert not rs.alert and rs.level is ThreatLevel.NONE
    assert alerts > 0


def test_recording_replays_bit_identically(cfg, tmp_path):
    path = tmp_path / "s.jsonl"
    rec = RecordingWriter(path, FakeClock(0.0), queue_depth=200000, flush_period_s=0.1)
    rec.start()
    runner, haptics, motors, _, src, clock = _build(cfg, "overtake_left_10mps", duration=12.0, recorder=rec)
    runner.announce_restart()
    live = []
    while not src.finished:
        if runner.step():
            live.append((runner.pipe.last_frame_number, int(runner.pipe.last_command.level), runner.pipe.last_command.side.value))
    rec.close()
    assert rec.drops == 0
    recs = [json.loads(line) for line in path.read_text().splitlines()]
    assert recs[0]["k"] == "hev" and recs[1]["k"] == "hdr"
    hap = [r for r in recs if r["k"] == "hap"]
    changes = [r for r in hap if not r.get("rf")]
    assert changes[0]["r"] == "OFFLINE" and changes[0]["hs"] == "OFFLINE"                  # restart rendered first, and recorded
    assert len(changes) == haptics.stats.changes and any(r["lvl"] >= 2 and r["side"] == "LEFT" for r in changes)
    assert any(r.get("rf") for r in hap)                                                    # quiet stretches are re-recorded
    assert all(isinstance(r["lat"], float) for r in changes[1:] if "lat" in r)
    from tools.replay import replay
    r1 = replay(str(path), cfg)
    r2 = replay(str(path), cfg)
    assert r1["outputs"] == r2["outputs"]
    rep = [(o["fn"], o["lvl"], o["side"]) for o in r1["outputs"]]
    assert rep == live


def test_undervoltage_flag_reaches_the_haptics(cfg):
    """§13.2 undervoltage: the power monitor's flag becomes a DEGRADED health bit the haptics render within ~1 s."""
    from goldenfleece.orchestrator.power_monitor import PowerMonitor
    runner, haptics, motors, _, src, clock = _build(cfg, "overtake_left_10mps", duration=3.0)
    flags = {"v": "throttled=0x0"}
    pm = PowerMonitor(clock, period_s=1.0, reader=lambda: flags["v"])
    runner.power = pm
    runner.announce_restart()
    for _ in range(10):
        runner.step()
    assert haptics.state.health is not HealthState.DEGRADED
    flags["v"] = "throttled=0x50001"          # under-voltage now + occurred bits
    pm.sample_once()                            # what the adapter thread does once per period
    t0 = clock.now()
    while haptics.state.health is not HealthState.DEGRADED:
        runner.step()
        assert clock.now() - t0 < 1.1
    assert runner.health.bits & HealthBits.UNDERVOLTAGE and haptics.state.render is Render.DEGRADED
    flags["v"] = "throttled=0x50000"           # only the sticky "occurred" bits remain
    pm.sample_once()
    for _ in range(5):
        runner.step()
    assert not (runner.health.bits & HealthBits.UNDERVOLTAGE)

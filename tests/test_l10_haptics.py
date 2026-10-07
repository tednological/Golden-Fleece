"""l10 haptics: pattern timing, the configured patterns meet the rendering requirements, rendering priority (a fault
is never a threat, DEGRADED keeps the threat), side -> motor mapping, the render thread's watchdog (stalled or absent
heartbeat -> FALLBACK within its timeout), restart ordering, motor faults and recovery, change records."""
import copy

import pytest

from goldenfleece.clock import FakeClock, MonotonicClock
from goldenfleece.config import ConfigError, Section
from goldenfleece.l10_haptics.motors import MotorError, NullMotor
from goldenfleece.l10_haptics.output import HapticOutput
from goldenfleece.l10_haptics.patterns import (HapticMode, Motor, Pattern, Render, Renderer, describe, haptics_config,
                                               render_state, side_mask)
from goldenfleece.types import HealthBits, HealthState, Side, TArrivalBucket, ThreatLevel, WarningCommand

L, R = Motor("left", "D12", Side.LEFT), Motor("right", "D13", Side.RIGHT)


def _cmd(level=ThreatLevel.NONE, side=Side.NONE, hs=HealthState.OK, seq=1, t=0.0):
    return WarningCommand(seq=seq, t_decided=t, level=level, side=side, t_arrival_bucket=TArrivalBucket.NONE, health_state=hs,
                          health_bits=HealthBits.NONE, assert_alert=(level == ThreatLevel.ALERT), dominant_track_id=None,
                          flicker_count=0)


@pytest.fixture
def hcfg(cfg):
    return haptics_config(cfg.pipeline.haptics)


def _section(cfg, **patch):
    d = copy.deepcopy(cfg.pipeline.haptics.as_dict())
    for path, v in patch.items():
        node = d
        keys = path.split("__")
        for k in keys[:-1]:
            node = node[int(k)] if isinstance(node, list) else node[k]
        node[keys[-1]] = v
    return Section(d, "haptics")


# --- patterns ------------------------------------------------------------------------------------------------
def test_pattern_timing():
    p = Pattern(pulse_s=0.15, gap_s=0.1, count=2, period_s=1.0)
    on = [0.0, 0.149, 0.25, 0.399, 1.0, 1.26]
    off = [-0.01, 0.15, 0.24, 0.4, 0.99, 1.41]
    assert all(p.is_on(t) for t in on) and not any(p.is_on(t) for t in off)
    assert p.on_fraction == pytest.approx(0.3)
    assert Pattern(0.25, 0.0, 1, 0.25).text().startswith("continuous")


def test_repo_config_drives_motors_at_full_duty_and_meets_the_rendering_requirements(hcfg):
    assert hcfg.duty_cycle == 1.0 and hcfg.duty_u16 == 0xFFFF            # team decision: 100 % duty
    assert {m.side for m in hcfg.motors} == {Side.LEFT, Side.RIGHT}
    p = hcfg.patterns
    assert max(p[n].pulse_s for n in ("degraded", "offline")) < min(p[n].pulse_s for n in ("advisory", "warning", "alert"))
    assert p["advisory"].on_fraction < p["warning"].on_fraction < p["alert"].on_fraction
    assert all(p[n].on_fraction < p["alert"].on_fraction for n in p if n != "alert")


@pytest.mark.parametrize("patch, msg", [
    ({"patterns__offline__pulse_s": 0.3, "patterns__offline__period_s": 5.0}, "never feel like a threat"),
    ({"patterns__offline__gap_s": 0.0, "patterns__offline__count": 1, "patterns__offline__period_s": 0.05}, "only maximum-intensity"),
    ({"patterns__advisory__count": 3, "patterns__advisory__period_s": 1.4}, "levels must be distinct"),
    ({"patterns__warning__period_s": 0.2}, "shorter than its pulses"),
    ({"pwm_duty_cycle": 0.0}, "pwm_duty_cycle"),
    ({"motors__1__side": "CENTER"}, "side must be LEFT or RIGHT"),
    ({"motors__1__pin": "D12"}, "duplicate pin"),
    ({"stall_timeout_s": 0.001}, "stall_timeout_s"),
])
def test_config_refuses_unsafe_or_broken_settings(cfg, patch, msg):
    with pytest.raises(ConfigError, match=msg):
        haptics_config(_section(cfg, **patch))


# --- rendering ------------------------------------------------------------------------------------------------
def test_rendering_priority_fault_is_never_a_threat():
    N, F = HapticMode.NORMAL, HapticMode.FALLBACK
    rs = render_state(F, ThreatLevel.ALERT, Side.LEFT, True, HealthState.OK)
    assert rs.render is Render.FALLBACK_OFFLINE and rs.level == ThreatLevel.NONE and not rs.alert
    rs = render_state(N, ThreatLevel.ALERT, Side.LEFT, True, HealthState.OFFLINE)
    assert rs.render is Render.OFFLINE and rs.level == ThreatLevel.NONE and not rs.alert
    rs = render_state(N, ThreatLevel.WARNING, Side.RIGHT, False, HealthState.DEGRADED)
    assert rs.render is Render.DEGRADED and rs.level == ThreatLevel.WARNING        # the warning continues while degraded
    assert describe(rs) == "warning right + degraded marker"
    assert render_state(N, ThreatLevel.ADVISORY, Side.LEFT, False, HealthState.OK).render is Render.THREAT
    rs = render_state(N, ThreatLevel.NONE, Side.LEFT, False, HealthState.OK)
    assert rs.render is Render.NONE and rs.side is Side.NONE and describe(rs) == "nothing"


def test_side_mask():
    assert side_mask(Side.LEFT, (L, R)) == (True, False)
    assert side_mask(Side.RIGHT, (L, R)) == (False, True)
    assert side_mask(Side.BOTH, (L, R)) == side_mask(Side.CENTER, (L, R)) == (True, True)
    assert side_mask(Side.RIGHT, (L,)) == (True,)                  # a one-motor vest still warns, without the side


def test_renderer_phases(hcfg):
    rd = Renderer((L, R), hcfg.patterns)
    warn_l = render_state(HapticMode.NORMAL, ThreatLevel.WARNING, Side.LEFT, False, HealthState.OK)
    assert rd.outputs(warn_l, 10.0) == (True, False)               # a new threat starts with a pulse, on its side
    assert rd.outputs(warn_l, 10.2) == (False, False)              # in the gap
    warn_b = render_state(HapticMode.NORMAL, ThreatLevel.WARNING, Side.BOTH, False, HealthState.OK)
    assert rd.outputs(warn_b, 10.2) == (True, True)                # a side change starts the pattern again
    off = render_state(HapticMode.NORMAL, ThreatLevel.NONE, Side.NONE, False, HealthState.OFFLINE)
    assert rd.outputs(off, 11.0) == (True, True)                   # "warnings offline": every motor, at once
    fb = render_state(HapticMode.FALLBACK, ThreatLevel.NONE, Side.NONE, False, HealthState.OFFLINE)
    assert rd.outputs(fb, 11.1) == (False, False)                  # same pattern, same phase: between two ticks
    assert rd.outputs(fb, 11.21) == (True, True)
    deg = render_state(HapticMode.NORMAL, ThreatLevel.ADVISORY, Side.RIGHT, False, HealthState.DEGRADED)
    assert rd.outputs(deg, 20.0) == (True, True)                   # marker on every motor, over the advisory pulse
    assert rd.outputs(deg, 20.1) == (False, True)                  # the advisory pulse continues on its side


# --- the render thread (synchronous) ----------------------------------------------------------------------------
class LogMotor(NullMotor):
    def __init__(self, name, clock, log, fail):
        super().__init__(name)
        self.clock, self.log, self.fail = clock, log, fail

    def set(self, on):
        if self.fail.get("write"):
            raise MotorError(f"{self.name}: lost")
        super().set(on)
        self.log.append((self.clock.now(), self.name, on))


def _output(hcfg, clock, fail=None):
    log, fail = [], fail if fail is not None else {}
    motors = {}

    def factory(m):
        if fail.get("open"):
            raise MotorError(f"{m.pin}: GPIO busy")
        motors[m.name] = LogMotor(m.name, clock, log, fail)
        return motors[m.name]
    out = HapticOutput(hcfg, clock, factory, synchronous=True)
    out.start()
    return out, motors, log


def _beat(out, clock, n, dt=0.03, start=0, frozen=None):
    for k in range(n):
        clock.advance(dt)
        out.send_heartbeat(frozen if frozen is not None else start + k)
        out.poll()


def test_restart_is_rendered_offline_before_any_threat_and_writes_only_changes(hcfg):
    clock = FakeClock(0.0)
    out, motors, log = _output(hcfg, clock)
    out.send_health(HealthState.OFFLINE)
    out.send_heartbeat(1)
    assert out.state.render is Render.OFFLINE and out.motor_states() == (True, True)
    n = len(log)
    out.poll()
    assert len(log) == n                                           # nothing changed, nothing written
    clock.advance(0.03)
    out.send_heartbeat(2)
    out.send_warning(_cmd(ThreatLevel.WARNING, Side.LEFT, seq=5, t=clock.now()))
    assert out.state.render is Render.THREAT and out.motor_states() == (True, False)
    ch = out.drain_changes()
    assert [c.state.render for c in ch] == [Render.OFFLINE, Render.THREAT]
    assert ch[1].latency_s == 0.0 and ch[1].text == "warning left" and out.drain_changes() == []
    out.stop()
    assert all(m.closed and m.on is False for m in motors.values())


def test_stalled_counter_falls_back_within_the_timeout_and_recovers(hcfg):
    clock = FakeClock(0.0)
    out, motors, _ = _output(hcfg, clock)
    _beat(out, clock, 10)
    out.send_warning(_cmd(ThreatLevel.ALERT, Side.RIGHT, t=clock.now()))
    assert out.state.alert and out.mode is HapticMode.NORMAL
    out.drain_changes()
    t_stall = clock.now()
    fell = None
    for _ in range(30):                                            # heartbeats keep coming, the counter does not move
        _beat(out, clock, 1, dt=0.01, frozen=999)
        if out.mode is HapticMode.FALLBACK and fell is None:
            fell = clock.now()
    assert fell is not None and fell - t_stall <= hcfg.stall_timeout_s + 0.011
    assert out.state.render is Render.FALLBACK_OFFLINE and not out.state.alert         # alert dropped, "warnings offline"
    ch = out.drain_changes()
    assert ch[-1].cause == "HB_STALL" and ch[-1].latency_s is None and out.stats.fallbacks == 1
    _beat(out, clock, 3, start=1000)
    assert out.mode is HapticMode.NORMAL and out.state.render is Render.THREAT
    assert out.drain_changes()[0].cause == "progress resumed"


def test_absent_heartbeat_falls_back(hcfg):
    clock = FakeClock(0.0)
    out, motors, _ = _output(hcfg, clock)
    _beat(out, clock, 5)
    t_hang = clock.now()
    while out.mode is HapticMode.NORMAL:
        clock.advance(0.005)
        out.poll()
    assert clock.now() - t_hang <= hcfg.stall_timeout_s + 0.006
    assert out.drain_changes()[-1].cause == "HB_ABSENT"
    assert out.motor_states() == (True, True)                      # the offline pattern starts with a tick


def test_no_heartbeat_after_start_falls_back(hcfg):
    clock = FakeClock(0.0)
    out, _, _ = _output(hcfg, clock)
    clock.advance(hcfg.stall_timeout_s + 0.01)
    out.poll()
    assert out.mode is HapticMode.FALLBACK


def test_motor_fault_is_reported_and_recovers(hcfg):
    clock = FakeClock(0.0)
    fail = {"open": True}
    out, motors, _ = _output(hcfg, clock, fail)
    ev = out.drain_events()
    assert [(e.bit, e.active) for e in ev] == [(HealthBits.HAPTICS_FAULT, True)] and "GPIO busy" in ev[0].detail
    assert not out.up and out.state.render is Render.OFFLINE      # rendering decisions continue without motors
    _beat(out, clock, 25)
    assert out.drain_events() == [] and out.stats.open_failures >= 2      # repeated failures are counted, not repeated
    fail["open"] = False
    _beat(out, clock, 40)
    assert out.up and [(e.bit, e.active) for e in out.drain_events()] == [(HealthBits.HAPTICS_FAULT, False)]
    fail["write"] = True
    out.send_warning(_cmd(ThreatLevel.WARNING, Side.LEFT, t=clock.now()))
    assert not out.up and out.stats.write_errors == 1
    assert [(e.bit, e.active) for e in out.drain_events()] == [(HealthBits.HAPTICS_FAULT, True)]
    fail["write"] = False
    _beat(out, clock, 30, start=100)
    assert out.up and motors["left"].on is not None
    assert [(e.bit, e.active) for e in out.drain_events()] == [(HealthBits.HAPTICS_FAULT, False)]


def test_render_thread_drives_and_releases_the_motors(hcfg):
    clock = MonotonicClock()
    motors = {}

    def factory(m):
        motors[m.name] = NullMotor(m.name)
        return motors[m.name]
    out = HapticOutput(hcfg, clock, factory)
    out.start()
    try:
        out.send_health(HealthState.OK)
        out.send_warning(_cmd(ThreatLevel.ALERT, Side.RIGHT, t=clock.now()))
        for k in range(100):
            out.send_heartbeat(k)
            if out.motor_states() == (False, True):
                break
            clock.sleep(0.005)
        assert motors["right"].on is True and motors["left"].on is False and out.state.alert
    finally:
        out.stop()
    assert all(m.closed and m.on is False for m in motors.values())

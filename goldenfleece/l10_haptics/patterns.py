"""l10 pure half: what the vibration motors render, and when each one is on.

No I/O, no threads, no clock: everything here is a function of the decision, the health and the time since a
pattern began.  The motors are driven by PWM at one fixed duty cycle (team decision 2026-10-07: 100 %), so a
pattern says only WHEN a motor is on: ``count`` pulses of ``pulse_s``, ``gap_s`` apart, repeated every
``period_s``.  The full specification is docs/haptics.md.  Keep this module and that document in step.

Rendering priority (what the MCU ICD required of the firmware, now kept on the Pi):

  FALLBACK_OFFLINE  the pipeline loop stopped making progress  "warnings offline" on every motor; alert dropped
  OFFLINE           the pipeline reports health OFFLINE        "warnings offline" on every motor; no threat
  DEGRADED          health DEGRADED                            the threat pattern (if any) plus a marker on every motor
  THREAT            level > 0                                  the level's pattern on the threat side's motors
  NONE              nothing                                    every motor off

A fault is never rendered as a threat.  ``haptics_config`` refuses patterns that would allow it: fault patterns
(degraded, offline) are short ticks on every motor, threat patterns are longer pulses on the threat's side, and
the alert pattern, used only while l09 asserts the alert channel (level 3, never a fault), has the highest
on-time of all.
"""
from __future__ import annotations

import enum
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

from ..config import ConfigError, Section
from ..types import HealthState, Side, ThreatLevel

THREAT_PATTERNS = ("advisory", "warning", "alert")      # in rising intensity
FAULT_PATTERNS = ("degraded", "offline")
MOTOR_SIDES = (Side.LEFT, Side.RIGHT)


class HapticMode(str, enum.Enum):
    NORMAL = "NORMAL"
    FALLBACK = "FALLBACK"        # the pipeline loop stopped making progress: "warnings offline"


class FallbackCause(str, enum.Enum):
    HB_ABSENT = "HB_ABSENT"      # no heartbeat at all
    HB_STALL = "HB_STALL"        # heartbeats, but the loop counter did not advance


class Render(str, enum.Enum):
    NONE = "NONE"
    THREAT = "THREAT"                       # level + side pattern
    DEGRADED = "DEGRADED"                   # threat pattern (if any) plus the degraded marker
    OFFLINE = "OFFLINE"                     # "warnings offline": the pipeline says OFFLINE
    FALLBACK_OFFLINE = "FALLBACK_OFFLINE"   # "warnings offline": the pipeline loop stalled


@dataclass(frozen=True)
class Pattern:
    pulse_s: float
    gap_s: float
    count: int
    period_s: float

    @property
    def on_fraction(self) -> float:
        return self.count * self.pulse_s / self.period_s

    def is_on(self, t: float) -> bool:
        """Motor state ``t`` seconds after the pattern began.  A pattern begins with a pulse."""
        if t < 0.0:
            return False
        k, r = divmod(t % self.period_s, self.pulse_s + self.gap_s)
        return k < self.count and r < self.pulse_s

    def text(self) -> str:
        pulses = f"{self.count} × {self.pulse_s * 1000:.0f} ms" + (f" ({self.gap_s * 1000:.0f} ms apart)" if self.count > 1 else "")
        if self.period_s <= self.count * self.pulse_s + (self.count - 1) * self.gap_s:
            return f"continuous ({pulses} back to back)"
        return f"{pulses} every {self.period_s:g} s"


@dataclass(frozen=True)
class Motor:
    name: str
    pin: str            # Blinka board name, e.g. "D12" (BCM GPIO12)
    side: Side          # LEFT or RIGHT


@dataclass(frozen=True)
class HapticsConfig:
    motors: Tuple[Motor, ...]
    duty_cycle: float                   # fraction of each PWM period a motor is driven while it is on
    frequency_hz: float
    render_period_s: float
    stall_timeout_s: float
    reopen_backoff_s: Tuple[float, ...]
    patterns: Dict[str, Pattern]

    @property
    def duty_u16(self) -> int:
        """The duty cycle as PWM outputs take it: 16 bits, 65535 = 100 %."""
        return int(round(self.duty_cycle * 0xFFFF))


def _pattern(name: str, d: object) -> Pattern:
    try:
        p = Pattern(float(d["pulse_s"]), float(d["gap_s"]), int(d["count"]), float(d["period_s"]))   # type: ignore[index]
    except (KeyError, TypeError, ValueError) as e:
        raise ConfigError(f"haptics.patterns.{name}: needs pulse_s, gap_s, count and period_s ({e!r})") from e
    if p.pulse_s <= 0.0 or p.gap_s < 0.0 or p.count < 1:
        raise ConfigError(f"haptics.patterns.{name}: needs pulse_s > 0, gap_s >= 0 and count >= 1")
    if p.period_s < p.count * p.pulse_s + (p.count - 1) * p.gap_s - 1e-9:
        raise ConfigError(f"haptics.patterns.{name}: period_s is shorter than its pulses")
    return p


def check_patterns(pats: Dict[str, Pattern]) -> None:
    """The rendering requirements, enforced on the configured patterns (raises ConfigError)."""
    missing = [n for n in THREAT_PATTERNS + FAULT_PATTERNS if n not in pats]
    if missing:
        raise ConfigError(f"haptics.patterns: missing {', '.join(missing)}")
    if max(pats[n].pulse_s for n in FAULT_PATTERNS) >= min(pats[n].pulse_s for n in THREAT_PATTERNS):
        raise ConfigError("haptics.patterns: a fault must never feel like a threat: every degraded/offline pulse must be "
                          "shorter than every advisory/warning/alert pulse")
    f = [pats[n].on_fraction for n in THREAT_PATTERNS]
    if not f[0] < f[1] < f[2]:
        raise ConfigError("haptics.patterns: levels must be distinct: on-time must rise advisory < warning < alert")
    if any(pats[n].on_fraction >= f[2] for n in FAULT_PATTERNS):
        raise ConfigError("haptics.patterns: alert must be the only maximum-intensity pattern")


def haptics_config(sec: Section) -> HapticsConfig:
    """Parse and validate ``pipeline.yaml: haptics`` (raises ConfigError)."""
    raw = sec.motors
    if not isinstance(raw, list) or not raw:
        raise ConfigError("haptics.motors: at least one motor is required")
    motors = []
    for i, m in enumerate(raw):
        try:
            mo = Motor(str(m["name"]), str(m["pin"]), Side[str(m["side"]).upper()])
        except (KeyError, TypeError) as e:
            raise ConfigError(f"haptics.motors[{i}]: needs name, pin and side LEFT or RIGHT ({e!r})") from e
        if mo.side not in MOTOR_SIDES:
            raise ConfigError(f"haptics.motors[{i}]: side must be LEFT or RIGHT, not {mo.side.name}")
        motors.append(mo)
    for attr in ("name", "pin"):
        vals = [getattr(m, attr) for m in motors]
        if len(set(vals)) != len(vals):
            raise ConfigError(f"haptics.motors: duplicate {attr}")
    duty = float(sec.pwm_duty_cycle)
    if not 0.0 < duty <= 1.0:
        raise ConfigError("haptics.pwm_duty_cycle must be in (0, 1]")
    freq = float(sec.pwm_frequency_hz)
    render_period = float(sec.render_period_s)
    stall = float(sec.stall_timeout_s)
    backoff = tuple(float(b) for b in sec.reopen_backoff_s)
    if freq <= 0.0 or render_period <= 0.0 or stall <= render_period or not backoff or min(backoff) <= 0.0:
        raise ConfigError("haptics: pwm_frequency_hz, render_period_s and reopen_backoff_s must be positive, "
                          "and stall_timeout_s longer than render_period_s")
    pats = {name: _pattern(name, d) for name, d in sec.patterns.as_dict().items()}
    check_patterns(pats)
    return HapticsConfig(tuple(motors), duty, freq, render_period, stall, backoff, pats)


# --- rendering ------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class RenderState:
    render: Render
    level: ThreatLevel
    side: Side
    alert: bool
    health: HealthState
    mode: HapticMode


def render_state(mode: HapticMode, level: ThreatLevel, side: Side, alert: bool, health: HealthState) -> RenderState:
    """The rendering priority.  A fault suppresses the threat and the alert; DEGRADED keeps the threat."""
    if mode is HapticMode.FALLBACK:
        return RenderState(Render.FALLBACK_OFFLINE, ThreatLevel.NONE, Side.NONE, False, health, mode)
    if health is HealthState.OFFLINE:
        return RenderState(Render.OFFLINE, ThreatLevel.NONE, Side.NONE, False, health, mode)
    if level == ThreatLevel.NONE:
        side, alert = Side.NONE, False
    if health is HealthState.DEGRADED:
        render = Render.DEGRADED
    else:
        render = Render.THREAT if level > ThreatLevel.NONE else Render.NONE
    return RenderState(render, level, side, alert, health, mode)


def threat_pattern(rs: RenderState) -> Optional[str]:
    """The threat pattern this state renders, if any.  The alert pattern follows the alert channel only."""
    if rs.level == ThreatLevel.NONE or rs.render not in (Render.THREAT, Render.DEGRADED):
        return None
    if rs.alert:
        return "alert"
    return "warning" if rs.level >= ThreatLevel.WARNING else "advisory"


def fault_pattern(rs: RenderState) -> Optional[str]:
    return {Render.OFFLINE: "offline", Render.FALLBACK_OFFLINE: "offline", Render.DEGRADED: "degraded"}.get(rs.render)


def side_mask(side: Side, motors: Sequence[Motor]) -> Tuple[bool, ...]:
    """Which motors render a threat on ``side``.  CENTER and BOTH use every motor, and so does a side that has no
    motor (a one-motor vest still warns, without the side)."""
    if side in MOTOR_SIDES:
        mask = tuple(m.side is side for m in motors)
        if any(mask):
            return mask
    return tuple(True for _ in motors)


def describe(rs: RenderState) -> str:
    """One line for logs and the web app: what the rider feels."""
    if rs.render is Render.FALLBACK_OFFLINE:
        return "warnings offline (pipeline loop stalled)"
    if rs.render is Render.OFFLINE:
        return "warnings offline"
    tp = threat_pattern(rs)
    what = "nothing" if tp is None else f"{tp} {rs.side.name.lower()}"
    return what + (" + degraded marker" if rs.render is Render.DEGRADED else "")


class Renderer:
    """Pattern phases.  A threat pattern begins, with a pulse, whenever its pattern or side changes; the offline
    pattern or the degraded marker begins when it is entered (OFFLINE <-> FALLBACK keeps the same phase).
    Pure: the caller passes the time."""

    def __init__(self, motors: Sequence[Motor], patterns: Dict[str, Pattern]) -> None:
        self.motors = tuple(motors)
        self.patterns = patterns
        self._threat_key: Optional[Tuple[str, Side]] = None
        self._threat_t0 = 0.0
        self._fault_key: Optional[str] = None
        self._fault_t0 = 0.0

    def outputs(self, rs: RenderState, t: float) -> Tuple[bool, ...]:
        tp, fp = threat_pattern(rs), fault_pattern(rs)
        key = (tp, rs.side) if tp is not None else None
        if key != self._threat_key:
            self._threat_key, self._threat_t0 = key, t
        if fp != self._fault_key:
            self._fault_key, self._fault_t0 = fp, t
        if fp is not None and self.patterns[fp].is_on(t - self._fault_t0):
            return tuple(True for _ in self.motors)
        if tp is not None and self.patterns[tp].is_on(t - self._threat_t0):
            return side_mask(rs.side, self.motors)
        return tuple(False for _ in self.motors)

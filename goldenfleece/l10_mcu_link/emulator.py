"""Python MCU emulator implementing docs/mcu_icd.md: watchdog, fallback transitions, alert
staleness, rendering priority and the outage log.  Pure: time comes from an injected Clock
(or explicit timestamps); bytes in, bytes out.  Used by the tests and by tools/mcu_emulator_serial.py.
"""
from __future__ import annotations

import enum
from dataclasses import dataclass, field
from typing import List, Optional

from ..clock import Clock
from ..types import HealthBits, HealthState, Side, ThreatLevel
from . import protocol as P


class McuMode(str, enum.Enum):
    NORMAL = "NORMAL"
    FALLBACK = "FALLBACK"        # Pi hang / link loss: render "warnings offline"


class Render(str, enum.Enum):
    NONE = "NONE"
    THREAT = "THREAT"            # level + side pattern
    DEGRADED = "DEGRADED"        # threat pattern (if any) plus a distinct degraded marker
    OFFLINE = "OFFLINE"          # "warnings offline": Pi says OFFLINE
    FALLBACK_OFFLINE = "FALLBACK_OFFLINE"   # "warnings offline": Pi silent/stalled


class OutageCause(str, enum.Enum):
    HB_ABSENT = "HB_ABSENT"
    HB_STALL = "HB_STALL"
    ALERT_STALE = "ALERT_STALE"
    PI_OFFLINE = "PI_OFFLINE"
    PI_DEGRADED = "PI_DEGRADED"


@dataclass
class Outage:
    start_s: float
    cause: OutageCause
    end_s: Optional[float] = None

    @property
    def duration_s(self) -> Optional[float]:
        return None if self.end_s is None else self.end_s - self.start_s


@dataclass
class RenderState:
    render: Render = Render.NONE
    level: ThreatLevel = ThreatLevel.NONE
    side: Side = Side.NONE
    alert: bool = False
    health: HealthState = HealthState.OFFLINE
    mode: McuMode = McuMode.NORMAL


@dataclass
class McuEmulator:
    clock: Clock
    stall_timeout_s: float = 0.25
    absent_timeout_s: float = 0.25
    alert_stale_s: float = 0.30
    t_start: float = 0.0
    # state
    last_hb_t: Optional[float] = None
    last_loop_iter: Optional[int] = None
    last_progress_t: Optional[float] = None
    last_alert_t: Optional[float] = None
    alert_asserted: bool = False
    level: ThreatLevel = ThreatLevel.NONE
    side: Side = Side.NONE
    health: HealthState = HealthState.OFFLINE      # until the Pi says otherwise
    health_bits: HealthBits = HealthBits.NONE
    last_pseq: int = 0
    last_warn_t_decided_ms: int = 0
    mode: McuMode = McuMode.NORMAL
    outages: List[Outage] = field(default_factory=list)
    n_crc_errors: int = 0
    n_messages: int = 0
    n_alert_stale_events: int = 0
    seq_out: int = 0
    rendered: RenderState = field(default_factory=RenderState)
    render_log: List[tuple] = field(default_factory=list)
    _splitter: P.LineSplitter = field(default_factory=P.LineSplitter)
    _open_outage: Optional[Outage] = None
    _seen_first_hb: bool = False
    first_threat_after_start_t: Optional[float] = None
    restart_announced_before_threat: Optional[bool] = None

    def __post_init__(self) -> None:
        self.t_start = self.clock.now()
        self._render()

    # -- input ---------------------------------------------------------------------------------------
    def feed(self, data: bytes) -> List[P.Message]:
        msgs: List[P.Message] = []
        for line in self._splitter.feed(data):
            try:
                m = P.decode_line(line)
            except P.FrameError:
                self.n_crc_errors += 1
                continue
            self.n_messages += 1
            self._apply(m)
            msgs.append(m)
        self.poll()
        return msgs

    def _apply(self, m: P.Message) -> None:
        t = self.clock.now()
        if m.type is P.MsgType.HB:
            v = P.parse_hb(m)
            if self.last_loop_iter is None or v.loop_iter != self.last_loop_iter:
                self.last_progress_t = t
            self.last_loop_iter = v.loop_iter
            self.last_hb_t = t
            self._set_health(v.health_state, v.health_bits, t)
            self._seen_first_hb = True
        elif m.type is P.MsgType.WARN:
            v = P.parse_warn(m)
            self.level = v.level
            self.side = v.side
            self.last_pseq = v.pipeline_seq
            self.last_warn_t_decided_ms = v.t_decided_ms
            self._set_health(v.health_state, v.health_bits, t)
            if v.level >= ThreatLevel.WARNING and self.first_threat_after_start_t is None:
                self.first_threat_after_start_t = t
                self.restart_announced_before_threat = bool(self._restart_seen)
        elif m.type is P.MsgType.ALERT:
            asserted = m[0] == "1"
            self.alert_asserted = asserted
            self.last_alert_t = t if asserted else None
        elif m.type is P.MsgType.HLTH:
            self._set_health(HealthState[m[0]], HealthBits(int(m[1], 16)), t)
        elif m.type is P.MsgType.RCFG:
            self.rcfg = tuple(m.fields)

    _restart_seen: bool = False

    def _set_health(self, state: HealthState, bits: HealthBits, t: float) -> None:
        if bits & HealthBits.PIPELINE_RESTARTING:
            self._restart_seen = True
        self.health = state
        self.health_bits = bits

    # -- timers ----------------------------------------------------------------------------------------
    def poll(self) -> None:
        t = self.clock.now()
        absent = self.last_hb_t is None or (t - self.last_hb_t) > self.absent_timeout_s
        stalled = self.last_progress_t is not None and (t - self.last_progress_t) > self.stall_timeout_s
        if absent or stalled:
            if self.mode is McuMode.NORMAL:
                self.mode = McuMode.FALLBACK
                self._open(OutageCause.HB_STALL if (stalled and not absent) else OutageCause.HB_ABSENT, t)
        else:
            if self.mode is McuMode.FALLBACK:
                self.mode = McuMode.NORMAL
                self._close(t)
        # stuck / stale alert: asserted longer than alert_stale_s without refresh, or heartbeat gone -> fault, not threat
        if self.alert_asserted:
            stale = (t - (self.last_alert_t or t)) > self.alert_stale_s or self.mode is McuMode.FALLBACK
            if stale:
                self.alert_asserted = False
                self.n_alert_stale_events += 1
                if self._open_outage is None:
                    self._open(OutageCause.ALERT_STALE, t)
                    self._close(t)
        # health-driven outages (only in NORMAL mode; fallback already is an outage)
        if self.mode is McuMode.NORMAL:
            if self.health is HealthState.OFFLINE:
                if self._open_outage is None or self._open_outage.cause is not OutageCause.PI_OFFLINE:
                    self._close(t)
                    self._open(OutageCause.PI_OFFLINE, t)
            elif self.health is HealthState.DEGRADED:
                if self._open_outage is None or self._open_outage.cause is not OutageCause.PI_DEGRADED:
                    self._close(t)
                    self._open(OutageCause.PI_DEGRADED, t)
            else:
                self._close(t)
        self._render()

    def _open(self, cause: OutageCause, t: float) -> None:
        if self._open_outage is not None:
            return
        self._open_outage = Outage(start_s=t, cause=cause)
        self.outages.append(self._open_outage)

    def _close(self, t: float) -> None:
        if self._open_outage is not None:
            self._open_outage.end_s = t
            self._open_outage = None

    # -- rendering priority (ICD §6) -----------------------------------------------------------------------
    def _render(self) -> None:
        r = RenderState(level=self.level, side=self.side, alert=self.alert_asserted, health=self.health, mode=self.mode)
        if self.mode is McuMode.FALLBACK:
            r.render = Render.FALLBACK_OFFLINE
            r.alert = False
            r.level = ThreatLevel.NONE
        elif self.health is HealthState.OFFLINE:
            r.render = Render.OFFLINE
            r.alert = False
            r.level = ThreatLevel.NONE
        elif self.health is HealthState.DEGRADED:
            r.render = Render.DEGRADED          # threat pattern still shown alongside the degraded marker
        elif self.level > ThreatLevel.NONE:
            r.render = Render.THREAT
        else:
            r.render = Render.NONE
        if (not self.render_log) or self.render_log[-1][1:] != (r.render, int(r.level), r.side.value, r.alert, r.health.name):
            self.render_log.append((self.clock.now(), r.render, int(r.level), r.side.value, r.alert, r.health.name))
        self.rendered = r

    # -- output -------------------------------------------------------------------------------------------
    def status_line(self) -> bytes:
        t = self.clock.now()
        self.seq_out += 1
        hb_age = 0 if self.last_hb_t is None else int((t - self.last_hb_t) * 1000)
        return P.build_stat(self.seq_out, self.mode.value, int((t - self.t_start) * 1000), self.last_pseq, hb_age, len(self.outages))

    def outage_lines(self) -> List[bytes]:
        out = []
        for o in self.outages:
            if o.end_s is not None:
                self.seq_out += 1
                out.append(P.build_outg(self.seq_out, int((o.start_s - self.t_start) * 1000), int(o.duration_s * 1000), o.cause.value))
        return out

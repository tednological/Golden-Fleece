"""l10 adapter half: the render thread that drives the vibration motors.  The hot path never blocks.

* The pipeline loop hands over every decision (``send_warning``), every health transition (``send_health``) and,
  on every iteration, its progress counter (``send_heartbeat``).  Each is a swap under a lock plus a wake-up.
  Heartbeats are built by the ORCHESTRATOR LOOP; this module has no timer of its own that could make a stopped
  pipeline look alive (task §9.10): the render thread only READS the counter.
* Watchdog (the MCU's fallback rule, now in-process): when the counter has not advanced for ``stall_timeout_s``,
  or no heartbeat arrived at all, the render thread enters FALLBACK: alert dropped, "warnings offline" on every
  motor.  It returns to NORMAL as soon as the counter advances.  A hung or dead PROCESS stops this thread too:
  then nothing renders, systemd's watchdog restarts the pipeline and the unit's ExecStopPost turns the motors off
  (docs/haptics.md).
* Patterns come from patterns.py (pure).  A motor is written only when its state changes.
* A motor that cannot be claimed or written: every motor is released, HAPTICS_FAULT is raised (``drain_events``)
  and the thread reopens them with backoff.  Rendering decisions continue, so they are still recorded.
* Every change in what is rendered is queued with its Clock time for the orchestrator loop to record
  (``drain_changes``), with the decision -> first-motor-write latency when a new decision caused it.
"""
from __future__ import annotations

import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Deque, List, Optional, Sequence, Tuple

from ..clock import Clock
from ..types import HealthBits, HealthEvent, HealthState, Side, ThreatLevel, WarningCommand
from .motors import MotorError, MotorPort
from .patterns import (FallbackCause, HapticMode, HapticsConfig, Motor, Renderer, RenderState, describe,
                       render_state)


@dataclass(frozen=True)
class HapticChange:
    t: float
    state: RenderState
    cause: str                      # "" for a decision or health change; FallbackCause / "progress resumed" for the mode
    latency_s: Optional[float]      # decision -> motor write, when a new decision caused the change
    text: str


@dataclass
class HapticStats:
    writes: int = 0
    write_errors: int = 0
    open_failures: int = 0
    fallbacks: int = 0
    changes: int = 0
    changes_dropped: int = 0
    mode: str = HapticMode.NORMAL.value
    render: str = "OFFLINE"
    latency_samples_s: List[float] = field(default_factory=list)


class HapticOutput:
    CHANGE_QUEUE = 1024

    def __init__(self, cfg: HapticsConfig, clock: Clock, motor_factory: Callable[[Motor], MotorPort],
                 synchronous: bool = False) -> None:
        self.cfg = cfg
        self.clock = clock
        self._factory = motor_factory
        self.synchronous = synchronous
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._motors: Optional[List[MotorPort]] = None
        self._written: List[Optional[bool]] = [None] * len(cfg.motors)
        self._renderer = Renderer(cfg.motors, cfg.patterns)
        self._events: Deque[HealthEvent] = deque()
        self._changes: Deque[HapticChange] = deque(maxlen=self.CHANGE_QUEUE)
        # what the pipeline told us (written by the loop thread under _lock)
        t0 = clock.now()
        self._cmd: Optional[WarningCommand] = None
        self._health = HealthState.OFFLINE          # until the pipeline says otherwise
        self._loop_iter: Optional[int] = None
        self._hb_t = t0                             # start() restarts the grace before FALLBACK
        self._progress_t = t0
        # render-thread state
        self.mode = HapticMode.NORMAL
        self.state: Optional[RenderState] = None
        self._rendered_seq: Optional[int] = None
        self._fault_reported = False
        self._next_open_t = float("inf")            # start() opens the motors; poll() reopens them after a fault
        self._backoff_i = 0
        self.stats = HapticStats()
        self.up = False

    # -- lifecycle ------------------------------------------------------------------------------------------
    def start(self) -> None:
        t = self.clock.now()
        with self._lock:
            self._hb_t = self._progress_t = t       # the grace before FALLBACK starts now
        self._open()
        if self.synchronous:
            self.poll()
            return
        self._thread = threading.Thread(target=self._run, name="gf-haptics", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Every motor off and released."""
        self._stop.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._release()

    def _open(self) -> bool:
        made: List[MotorPort] = []
        try:
            for m in self.cfg.motors:
                made.append(self._factory(m))
        except Exception as e:  # noqa: BLE001  (a factory may raise anything; MotorError is the usual)
            for mp in made:
                self._close_quietly(mp)
            self.stats.open_failures += 1
            self._next_open_t = self.clock.now() + self.cfg.reopen_backoff_s[min(self._backoff_i, len(self.cfg.reopen_backoff_s) - 1)]
            self._backoff_i += 1
            self._report(False, f"open failed: {e}")
            return False
        self._motors = made
        self._written = [None] * len(made)          # unknown: the first poll writes every motor
        self._backoff_i = 0
        self.up = True
        self._report(True, "motors claimed: " + ", ".join(f"{m.name}={m.pin}" for m in self.cfg.motors))
        return True

    def _release(self) -> None:
        motors, self._motors = self._motors, None
        self.up = False
        for mp in motors or ():
            self._close_quietly(mp)

    @staticmethod
    def _close_quietly(mp: MotorPort) -> None:
        try:
            mp.close()
        except Exception:  # noqa: BLE001
            pass

    def _report(self, up: bool, why: str) -> None:
        """HAPTICS_FAULT on the first failure and on recovery; repeated failures are counted, not repeated."""
        if up == (not self._fault_reported):
            return
        self._fault_reported = not up
        self._events.append(HealthEvent(self.clock.now(), HealthBits.HAPTICS_FAULT, not up, "l10", why))

    # -- hot-path API (never blocks) ---------------------------------------------------------------------------
    def send_warning(self, cmd: WarningCommand) -> None:
        with self._lock:
            self._cmd = cmd
            self._health = cmd.health_state
        self._kick()

    def send_health(self, state: HealthState) -> None:
        with self._lock:
            self._health = state
        self._kick()

    def send_heartbeat(self, loop_iter: int) -> None:
        t = self.clock.now()
        with self._lock:
            if loop_iter != self._loop_iter:
                self._progress_t = t
            self._loop_iter = loop_iter
            self._hb_t = t
        self._kick()

    def _kick(self) -> None:
        if self.synchronous:
            self.poll()
        else:
            self._wake.set()

    def drain_events(self) -> List[HealthEvent]:
        """HAPTICS_FAULT transitions since the last call (orchestrator loop)."""
        return self._drain(self._events)

    def drain_changes(self) -> List[HapticChange]:
        """Changes in what is rendered since the last call, oldest first (orchestrator loop)."""
        return self._drain(self._changes)

    @staticmethod
    def _drain(q: Deque) -> list:
        out = []
        while True:
            try:
                out.append(q.popleft())
            except IndexError:
                return out

    # -- render thread -------------------------------------------------------------------------------------------
    def _run(self) -> None:
        while not self._stop.is_set():
            self.poll()
            self._wake.wait(timeout=self.cfg.render_period_s)
            self._wake.clear()
        self._release()

    def poll(self) -> None:
        """One render step: watchdog, rendering priority, pattern phase, motor writes.  The render thread calls
        this every ``render_period_s`` and on every hand-over; tests and the simulator call it directly."""
        t = self.clock.now()
        if self._motors is None and t >= self._next_open_t and not self._stop.is_set():
            self._open()
        with self._lock:
            cmd, health, hb_t, progress_t = self._cmd, self._health, self._hb_t, self._progress_t
        cause = ""
        if t - progress_t > self.cfg.stall_timeout_s:
            if self.mode is HapticMode.NORMAL:
                self.mode = HapticMode.FALLBACK
                self.stats.fallbacks += 1
                absent = t - hb_t > self.cfg.stall_timeout_s
                cause = (FallbackCause.HB_ABSENT if absent else FallbackCause.HB_STALL).value
        elif self.mode is HapticMode.FALLBACK:
            self.mode = HapticMode.NORMAL
            cause = "progress resumed"
        if cmd is None:
            rs = render_state(self.mode, ThreatLevel.NONE, Side.NONE, False, health)
        else:
            rs = render_state(self.mode, cmd.level, cmd.side, cmd.assert_alert, health)
        outs = self._renderer.outputs(rs, t)
        self._write(outs)
        if rs != self.state:
            lat = None
            if cmd is not None and cmd.seq != self._rendered_seq and not cause:
                lat = self.clock.now() - cmd.t_decided
                self.stats.latency_samples_s.append(lat)
                if len(self.stats.latency_samples_s) > 1000:
                    del self.stats.latency_samples_s[:500]
            if len(self._changes) == self.CHANGE_QUEUE:
                self.stats.changes_dropped += 1
            self._changes.append(HapticChange(t, rs, cause, lat, describe(rs)))
            self.stats.changes += 1
            self.stats.mode = rs.mode.value
            self.stats.render = rs.render.value
            self.state = rs
        if cmd is not None:
            self._rendered_seq = cmd.seq

    def _write(self, outs: Sequence[bool]) -> None:
        if self._motors is None:
            return
        for i, on in enumerate(outs):
            if self._written[i] is on:
                continue
            try:
                self._motors[i].set(on)
            except MotorError as e:
                self.stats.write_errors += 1
                self._release()
                self._next_open_t = self.clock.now() + self.cfg.reopen_backoff_s[0]
                self._report(False, f"write failed: {e}")
                return
            self._written[i] = on
            self.stats.writes += 1

    def current(self) -> Optional[HapticChange]:
        """What is rendered now, as a change record (for periodic re-recording; None before the first poll)."""
        rs = self.state
        return None if rs is None else HapticChange(self.clock.now(), rs, "", None, describe(rs))

    def motor_states(self) -> Tuple[Optional[bool], ...]:
        """What was last written to each motor (None = not written since it was claimed)."""
        return tuple(self._written)

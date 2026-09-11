"""l10 transport half: writer thread owns the port; the hot path never blocks.

* Bounded "slots": the newest WARNING / HEARTBEAT / ALERT / HEALTH / RADAR_CONFIG supersedes the
  previous one of its kind (coalescing).  Send order per wake-up: ALERT, WARN, HB, HLTH, RCFG.
* The alert channel (team decision: the UART keyword ``ALERT``) is asserted in the same code path
  as, and BEFORE, the WARNING is queued.  It is refreshed on every heartbeat while asserted so the
  MCU can detect a stale/stuck alert.  It is never asserted for a fault (l09 guarantees that;
  this module only forwards ``cmd.assert_alert``).
* Heartbeats are built by the ORCHESTRATOR LOOP and handed in here; this module has no timer
  and never beats on its own (task §9.10).
* USB re-enumeration: the writer reopens the port by its by-id path with backoff and reports
  MCU_LINK_DOWN through ``on_link_state``.
* STATUS lines from the MCU are parsed for latency measurement (last pipeline seq applied).
"""
from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Protocol

from ..clock import Clock
from ..config import Section
from ..types import HealthBits, HealthState, RadarConfigChanged, WarningCommand
from . import protocol as P


class Transport(Protocol):
    def write(self, data: bytes) -> None: ...
    def read(self, max_bytes: int, timeout_s: float) -> bytes: ...
    def close(self) -> None: ...


class TransportError(Exception):
    pass


class LoopbackTransport:
    """In-memory transport: bytes written are delivered to ``sink`` (e.g. McuEmulator.feed);
    ``inject`` queues bytes for the Pi to read."""

    def __init__(self, sink: Callable[[bytes], object]) -> None:
        self._sink = sink
        self._rx = bytearray()
        self._lock = threading.Lock()
        self.written = 0
        self.fail_writes = False

    def write(self, data: bytes) -> None:
        if self.fail_writes:
            raise TransportError("simulated port loss")
        self.written += len(data)
        self._sink(data)

    def inject(self, data: bytes) -> None:
        with self._lock:
            self._rx.extend(data)

    def read(self, max_bytes: int, timeout_s: float) -> bytes:
        with self._lock:
            out = bytes(self._rx[:max_bytes])
            del self._rx[:max_bytes]
        return out

    def close(self) -> None:
        pass


class SerialTransport:
    """pyserial by stable path (config), 8N1.  Import is lazy so pure tests never need pyserial."""

    def __init__(self, port: str, baudrate: int) -> None:
        import serial  # noqa: WPS433
        self._serial_mod = serial
        self._ser = serial.Serial(port, baudrate=baudrate, bytesize=8, parity="N", stopbits=1, timeout=0.02, write_timeout=0.2)

    def write(self, data: bytes) -> None:
        try:
            self._ser.write(data)
        except self._serial_mod.SerialException as e:
            raise TransportError(str(e)) from e

    def read(self, max_bytes: int, timeout_s: float) -> bytes:
        try:
            self._ser.timeout = timeout_s
            return self._ser.read(max_bytes)
        except self._serial_mod.SerialException as e:
            raise TransportError(str(e)) from e

    def close(self) -> None:
        try:
            self._ser.close()
        except Exception:
            pass


@dataclass
class LinkStats:
    bytes_written: int = 0
    lines_written: int = 0
    coalesced: int = 0
    write_errors: int = 0
    reconnects: int = 0
    rx_lines: int = 0
    rx_crc_errors: int = 0
    last_status: Optional[P.StatView] = None
    last_status_t: Optional[float] = None
    mcu_mode: str = "UNKNOWN"
    latency_samples_s: List[float] = field(default_factory=list)     # WARN write -> STAT ack (upper bound)


class McuLink:
    SLOT_ORDER = ("ALERT", "WARN", "HB", "HLTH", "RCFG")

    def __init__(self, link_cfg: Section, clock: Clock, transport_factory: Callable[[], Transport],
                 on_link_state: Optional[Callable[[bool, float, str], None]] = None, synchronous: bool = False) -> None:
        self.cfg = link_cfg
        self.clock = clock
        self._factory = transport_factory
        self._on_link_state = on_link_state
        self.synchronous = synchronous
        self._slots: Dict[str, bytes] = {}
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._transport: Optional[Transport] = None
        self._splitter = P.LineSplitter()
        self._seq = 0
        self._alert_asserted = False
        self._sent_t: Dict[int, float] = {}          # pipeline seq -> write time (latency)
        self._backoff = [float(b) for b in link_cfg.reconnect_backoff_s]
        self.stats = LinkStats()
        self.up = False

    # -- lifecycle ----------------------------------------------------------------------------------
    def start(self) -> None:
        if self.synchronous:
            self._open()
            return
        self._thread = threading.Thread(target=self._run, name="gf-mcu-link", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._transport is not None:
            self._transport.close()

    def _open(self) -> bool:
        try:
            self._transport = self._factory()
            self.up = True
            if self._on_link_state:
                self._on_link_state(True, self.clock.now(), "opened")
            return True
        except Exception as e:  # noqa: BLE001
            self._transport = None
            if self.up or self.stats.reconnects == 0:
                if self._on_link_state:
                    self._on_link_state(False, self.clock.now(), f"open failed: {e}")
            self.up = False
            return False

    # -- hot-path API (never blocks) -----------------------------------------------------------------
    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFFFF
        return self._seq

    def _put(self, slot: str, line: bytes) -> None:
        with self._lock:
            if slot in self._slots:
                self.stats.coalesced += 1
            self._slots[slot] = line
        if self.synchronous:
            self._flush_once()
        else:
            self._wake.set()

    def send_warning(self, cmd: WarningCommand) -> None:
        # alert channel first, in the same code path, before the WARNING is queued
        if cmd.assert_alert and not self._alert_asserted:
            self._alert_asserted = True
            self._put("ALERT", P.build_alert(self._next_seq(), True))
        elif not cmd.assert_alert and self._alert_asserted:
            self._alert_asserted = False
            self._put("ALERT", P.build_alert(self._next_seq(), False))
        self._sent_t[cmd.seq & 0xFFFFFFFF] = self.clock.now()
        if len(self._sent_t) > 256:
            for k in list(self._sent_t)[:128]:
                self._sent_t.pop(k, None)
        self._put("WARN", P.build_warn(self._next_seq(), cmd))

    def send_heartbeat(self, loop_iter: int, frames_processed: int, last_frame_number: int, state: HealthState, bits: HealthBits) -> None:
        if self._alert_asserted:
            self._put("ALERT", P.build_alert(self._next_seq(), True))    # refresh so a stale alert is detectable
        self._put("HB", P.build_hb(self._next_seq(), loop_iter, frames_processed, last_frame_number, state, bits))

    def send_health(self, state: HealthState, bits: HealthBits, onset_ms: int) -> None:
        self._put("HLTH", P.build_hlth(self._next_seq(), state, bits, onset_ms))

    def send_radar_config(self, rc: RadarConfigChanged, blind_cms: int, t_alert_ms: int, t_warn_ms: int) -> None:
        self._put("RCFG", P.build_rcfg(self._next_seq(), rc.rrai, rc.rspi, rc.baud, blind_cms, t_alert_ms, t_warn_ms))

    # -- writer thread ---------------------------------------------------------------------------------------
    def _run(self) -> None:
        bi = 0
        while not self._stop.is_set():
            if self._transport is None:
                if self._open():
                    bi = 0
                else:
                    self.stats.reconnects += 1
                    self.clock.sleep(self._backoff[min(bi, len(self._backoff) - 1)])
                    bi += 1
                    continue
            self._wake.wait(timeout=0.02)
            self._wake.clear()
            self._flush_once()
            self._read_once()

    def _flush_once(self) -> None:
        if self._transport is None and not self._open():
            return
        with self._lock:
            pending = [(s, self._slots.pop(s)) for s in self.SLOT_ORDER if s in self._slots]
        for slot, line in pending:
            try:
                self._transport.write(line)
                self.stats.bytes_written += len(line)
                self.stats.lines_written += 1
            except TransportError as e:
                self.stats.write_errors += 1
                self._drop_transport(str(e))
                with self._lock:
                    self._slots.setdefault(slot, line)
                return

    def _read_once(self) -> None:
        if self._transport is None:
            return
        try:
            data = self._transport.read(256, 0.0)
        except TransportError as e:
            self._drop_transport(str(e))
            return
        if not data:
            return
        for line in self._splitter.feed(data):
            try:
                m = P.decode_line(line)
            except P.FrameError:
                self.stats.rx_crc_errors += 1
                continue
            self.stats.rx_lines += 1
            if m.type is P.MsgType.STAT:
                try:
                    st = P.parse_stat(m)
                except (P.FrameError, ValueError):
                    continue
                now = self.clock.now()
                self.stats.last_status = st
                self.stats.last_status_t = now
                self.stats.mcu_mode = st.mode
                t_sent = self._sent_t.get(st.last_pseq)
                if t_sent is not None:
                    self.stats.latency_samples_s.append(now - t_sent)
                    if len(self.stats.latency_samples_s) > 1000:
                        del self.stats.latency_samples_s[:500]

    def poll_rx(self) -> None:
        """Synchronous mode only: read MCU -> Pi lines."""
        if self.synchronous:
            self._read_once()

    def _drop_transport(self, why: str) -> None:
        if self._transport is not None:
            try:
                self._transport.close()
            except Exception:
                pass
        self._transport = None
        if self.up and self._on_link_state:
            self._on_link_state(False, self.clock.now(), why)
        self.up = False

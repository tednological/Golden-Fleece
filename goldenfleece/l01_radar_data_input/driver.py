"""l01: DirectKld7Source -- the Pi polls the K-LD7 (PI_POLLS topology).

Adapter rules: bytes and timestamps only; no math (rule 4.1).  The reader thread owns the port.
Connect: probe baud (GBYE at each rate until a RESP) -> INIT -> GRPS -> SRPS(with the sensor's version
string) -> GRPS -> verify -> stream.  Faults are events, never silent: RESP codes 1-6, resync bytes,
frame-number gaps, poll timeouts, port loss (reconnect with backoff), RADAR_SILENT after k * T_frame.
t_header = arrival of the PDAT header's first byte, corrected for the bytes received after it in the
same read and for the configured USB latency.
"""
from __future__ import annotations

import logging
import queue
import threading
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Protocol, Union

from ..clock import Clock
from ..config import RadarConfig
from ..types import HealthBits, HealthEvent, RadarConfigChanged, RawRadarFrame
from . import protocol as K
from .source import SourceEvent

log = logging.getLogger("goldenfleece.l01")


class SerialLike(Protocol):
    baudrate: int
    timeout: float

    @property
    def in_waiting(self) -> int: ...
    def read(self, n: int = 1) -> bytes: ...
    def write(self, data: bytes) -> int: ...
    def reset_input_buffer(self) -> None: ...
    def close(self) -> None: ...


def open_pyserial(port: str, baudrate: int) -> SerialLike:
    import serial  # lazy: pure tests never need it
    return serial.Serial(port, baudrate=baudrate, bytesize=8, parity="E", stopbits=1, timeout=0.02, write_timeout=0.5)


class DriverError(Exception):
    pass


@dataclass
class DriverStats:
    frames: int = 0
    gaps: int = 0
    gap_frames_missed: int = 0
    poll_timeouts: int = 0
    resp_codes: Dict[int, int] = field(default_factory=dict)
    resync_bytes: int = 0
    bad_lengths: int = 0
    unexpected_packets: int = 0
    reconnects: int = 0
    connect_failures: int = 0
    port_errors: int = 0
    cap_hits: int = 0
    frames_without_pdat: int = 0
    busy_responses: int = 0


class DirectKld7Source:
    def __init__(self, rcfg: RadarConfig, clock: Clock, open_serial: Callable[[str, int], SerialLike] = open_pyserial,
                 queue_depth: int = 4) -> None:
        self.cfg = rcfg
        self.clock = clock
        self._open = open_serial
        self._q: "queue.Queue[RawRadarFrame]" = queue.Queue(maxsize=queue_depth)
        self._events: List[SourceEvent] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.ser: Optional[SerialLike] = None
        self.parser = K.Kld7Parser()
        self.params: Optional[K.RadarParams] = None
        self.firmware_version = ""
        self.stats = DriverStats()
        self._last_fn: Optional[int] = None
        self._seq = 0
        self._consecutive_timeouts = 0
        self._last_frame_t: Optional[float] = None
        self._silent_flag = False
        self.t_frame = rcfg.frame_duration_s[rcfg.params["RSPI"]]
        self.queue_dropped = 0

    # -- RadarFrameSource API ------------------------------------------------------------------------
    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="gf-kld7", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        self._disconnect(send_gbye=True)

    def get(self, timeout_s: float) -> Optional[RawRadarFrame]:
        try:
            return self._q.get(timeout=timeout_s)
        except queue.Empty:
            return None

    def drain_events(self) -> List[SourceEvent]:
        with self._lock:
            ev, self._events = self._events, []
        return ev

    # -- events ---------------------------------------------------------------------------------------------
    def _event(self, bit: HealthBits, active: bool, detail: str) -> None:
        with self._lock:
            self._events.append(HealthEvent(self.clock.now(), bit, active, "l01", detail))
        (log.warning if active else log.info)("l01 %s %s: %s", bit.name, "ON" if active else "off", detail)

    def _set_silent(self, silent: bool, detail: str) -> None:
        if silent != self._silent_flag:
            self._silent_flag = silent
            self._event(HealthBits.RADAR_SILENT, silent, detail)

    # -- thread ------------------------------------------------------------------------------------------------
    def _run(self) -> None:
        backoff_i = 0
        while not self._stop.is_set():
            if self.ser is None:
                if self._connect():
                    backoff_i = 0
                    self._consecutive_timeouts = 0
                else:
                    self.stats.connect_failures += 1
                    self._set_silent(True, "not connected")
                    self.clock.sleep(self.cfg.reconnect_backoff_s[min(backoff_i, len(self.cfg.reconnect_backoff_s) - 1)])
                    backoff_i += 1
                    continue
            try:
                self._poll_cycle()
            except (OSError, DriverError) as e:   # pyserial raises SerialException(IOError)
                self.stats.port_errors += 1
                self._event(HealthBits.RADAR_SILENT, True, f"port error: {e}")
                self._silent_flag = True
                self._disconnect(send_gbye=False)
                self.stats.reconnects += 1

    # -- connection ------------------------------------------------------------------------------------------------
    def _wait_packet(self, header: bytes, timeout_s: float) -> Optional[K.Packet]:
        deadline = self.clock.now() + timeout_s
        while self.clock.now() < deadline:
            n = self.ser.in_waiting
            chunk = self.ser.read(n if n > 0 else 1)
            if not chunk:
                continue
            for pk in self.parser.feed(chunk):
                if pk.header == header:
                    return pk
                if pk.header == b"RESP":
                    code = K.parse_resp(pk.payload)
                    self.stats.resp_codes[code] = self.stats.resp_codes.get(code, 0) + 1
                    if code != K.RespCode.OK:
                        return pk        # let the caller see the error code
        return None

    def _wait_resp(self, timeout_s: float) -> Optional[int]:
        pk = self._wait_packet(b"RESP", timeout_s)
        if pk is None or pk.header != b"RESP":
            return None
        code = K.parse_resp(pk.payload)
        return code

    def _probe_baud(self) -> Optional[int]:
        rates = [self.cfg.baudrate] + [r for r in self.cfg.baud_probe_order if r != self.cfg.baudrate]
        for rate in rates:
            try:
                self.ser = self._open(self.cfg.port, rate)
            except OSError as e:
                log.warning("l01 open %s @%d failed: %s", self.cfg.port, rate, e)
                self.ser = None
                return None
            self.parser = K.Kld7Parser()
            self.ser.reset_input_buffer()
            self.ser.write(K.cmd_gbye())
            code = self._wait_resp(0.15)
            if code is not None:
                log.info("l01 sensor answered GBYE at %d baud (RESP %d)", rate, code)
                self.ser.close()
                self.ser = None
                return rate
            self.ser.close()
            self.ser = None
        return None

    def _connect(self) -> bool:
        found = self._probe_baud()
        if found is None:
            log.warning("l01 no response at any baud on %s", self.cfg.port)
            return False
        try:
            self.ser = self._open(self.cfg.port, K.DEFAULT_BAUD)      # after GBYE the sensor is at 115200
        except OSError as e:
            log.warning("l01 reopen failed: %s", e)
            self.ser = None
            return False
        self.parser = K.Kld7Parser()
        self.ser.reset_input_buffer()
        self.ser.write(K.cmd_init(self.cfg.baudrate))
        code = self._wait_resp(self.cfg.resp_timeout_s)
        if code != K.RespCode.OK:
            self._event(HealthBits.RADAR_CONFIG_MISMATCH, True, f"INIT answered {code}")
            self._disconnect(send_gbye=False)
            return False
        self.ser.baudrate = self.cfg.baudrate                          # sensor switches after acknowledging
        self.clock.sleep(0.02)
        self.ser.reset_input_buffer()
        self.parser = K.Kld7Parser()
        # read the parameter structure (the version string is required by SRPS: RESP 3 otherwise)
        self.ser.write(K.cmd_grps())
        pk = self._wait_packet(b"RPST", self.cfg.resp_timeout_s)
        if pk is None or pk.header != b"RPST":
            self._event(HealthBits.RADAR_CONFIG_MISMATCH, True, "GRPS gave no RPST")
            self._disconnect(send_gbye=True)
            return False
        current = K.RadarParams.unpack(pk.payload)
        self.firmware_version = current.software_version
        desired = current.with_params(self.cfg.params)
        self.ser.write(K.cmd_srps(desired.pack()))
        code = self._wait_resp(self.cfg.resp_timeout_s)
        if code != K.RespCode.OK:
            self._event(HealthBits.RADAR_CONFIG_MISMATCH, True, f"SRPS answered {code}")
            self._disconnect(send_gbye=True)
            return False
        self.ser.write(K.cmd_grps())
        pk = self._wait_packet(b"RPST", self.cfg.resp_timeout_s)
        if pk is None or pk.header != b"RPST":
            self._event(HealthBits.RADAR_CONFIG_MISMATCH, True, "verification GRPS gave no RPST")
            self._disconnect(send_gbye=True)
            return False
        verified = K.RadarParams.unpack(pk.payload)
        if not verified.same_settings(desired):
            self._event(HealthBits.RADAR_CONFIG_MISMATCH, True, f"read-back differs: {verified.diff(desired)}")
            self._disconnect(send_gbye=True)
            return False
        self.params = verified
        self._event(HealthBits.RADAR_CONFIG_MISMATCH, False, "parameters verified")
        with self._lock:
            self._events.append(RadarConfigChanged(
                t=self.clock.now(), rrai=verified.max_range, rspi=verified.max_speed, baud=self.cfg.baudrate,
                thof=verified.threshold_offset, dedi=verified.det_direction, misp=verified.min_det_speed, masp=verified.max_det_speed,
                firmware_version=verified.software_version, frame_duration_s=self.cfg.frame_duration_s[verified.max_speed]))
        self.t_frame = self.cfg.frame_duration_s[verified.max_speed]
        self._last_fn = None
        log.info("l01 connected: fw=%s baud=%d RRAI=%d RSPI=%d THOF=%d DEDI=%d", verified.software_version, self.cfg.baudrate,
                 verified.max_range, verified.max_speed, verified.threshold_offset, verified.det_direction)
        return True

    def _disconnect(self, send_gbye: bool) -> None:
        if self.ser is None:
            return
        try:
            if send_gbye:
                self.ser.write(K.cmd_gbye())
                self._wait_resp(0.1)
        except Exception:  # noqa: BLE001
            pass
        try:
            self.ser.close()
        except Exception:  # noqa: BLE001
            pass
        self.ser = None

    # -- polling ------------------------------------------------------------------------------------------------------
    def _poll_cycle(self) -> None:
        t_send = self.clock.now()
        self.ser.write(K.cmd_gnfd(K.GNFD_WANTED))
        deadline = t_send + self.cfg.resp_timeout_s + 3.0 * self.t_frame
        targets = None
        t_header: Optional[float] = None
        fn: Optional[int] = None
        baud = self.cfg.baudrate
        while self.clock.now() < deadline:
            n = self.ser.in_waiting
            chunk = self.ser.read(n if n > 0 else 1)
            t_chunk = self.clock.now()
            if not chunk:
                continue
            pos_before = self.parser.stream_pos
            packets = self.parser.feed(chunk)
            self.stats.resync_bytes = self.parser.n_resync_bytes
            self.stats.bad_lengths = self.parser.n_bad_length
            for pk in packets:
                if pk.header == b"RESP":
                    code = K.parse_resp(pk.payload)
                    self.stats.resp_codes[code] = self.stats.resp_codes.get(code, 0) + 1
                    if code == K.RespCode.SENSOR_BUSY:
                        self.stats.busy_responses += 1
                    if code != K.RespCode.OK:
                        self._event(HealthBits.RADAR_GAPS, True, f"RESP {K.RespCode(code).name}")
                elif pk.header == b"PDAT":
                    targets = K.parse_pdat(pk.payload)
                    # bytes received after the header's first byte in this read -> subtract their transfer time
                    after = (pos_before + len(chunk)) - self.parser.last_header_pos
                    t_header = t_chunk - after * K.BITS_PER_BYTE_8E1 / baud - self.cfg.usb_latency_s
                elif pk.header == b"DONE":
                    fn = K.parse_done(pk.payload)
                    if t_header is None:
                        after = (pos_before + len(chunk)) - self.parser.last_header_pos
                        t_header = t_chunk - after * K.BITS_PER_BYTE_8E1 / baud - self.cfg.usb_latency_s
                else:
                    self.stats.unexpected_packets += 1
            if fn is not None:
                break
        if fn is None:
            self.stats.poll_timeouts += 1
            self._consecutive_timeouts += 1
            if self._last_frame_t is None or (self.clock.now() - self._last_frame_t) > self.cfg.silent_after_frames * self.t_frame:
                self._set_silent(True, f"{self._consecutive_timeouts} poll timeouts")
            if self._consecutive_timeouts >= 3:
                raise DriverError("3 consecutive poll timeouts")
            return
        self._consecutive_timeouts = 0
        if targets is None:
            self.stats.frames_without_pdat += 1
            targets = ()
        gap = 0
        if self._last_fn is not None and fn > self._last_fn + 1:
            gap = fn - self._last_fn - 1
            self.stats.gaps += 1
            self.stats.gap_frames_missed += gap
        self._last_fn = fn
        self._seq += 1
        cap = len(targets) >= self.cfg.max_targets
        self.stats.cap_hits += int(cap)
        self.stats.frames += 1
        self._last_frame_t = self.clock.now()
        self._set_silent(False, "frames flowing")
        frame = RawRadarFrame(t_header=t_header, frame_number=fn, gap=gap, rspi=self.params.max_speed, rrai=self.params.max_range,
                              targets=tuple(targets), cap_hit=cap, source_seq=self._seq)
        while True:
            try:
                self._q.put_nowait(frame)
                break
            except queue.Full:
                try:
                    self._q.get_nowait()
                    self.queue_dropped += 1
                except queue.Empty:
                    pass

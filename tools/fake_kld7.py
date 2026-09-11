"""A simulated K-LD7 behind a serial port, for driver tests and probe dry runs.

Models what the datasheet describes: boots at 115200 8E1, INIT selects a rate (RESP at the old rate,
then switch), GBYE reverts to 115200, GRPS/SRPS with version check (RESP 3), GNFD answered by RESP
immediately and by the requested messages when the NEXT free-running frame completes, DONE carrying the
frame number since reset.  Bytes written at the wrong baud are garbage to the sensor (ignored), and
bytes it sends are garbage to a host at the wrong baud.  Time comes from an injected Clock.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Set, Tuple

from goldenfleece.clock import Clock
from goldenfleece.l01_radar_data_input import protocol as K
from goldenfleece.types import RawRadarTarget

FrameGen = Callable[[int, float], Sequence[RawRadarTarget]]      # (frame_number, t) -> targets


@dataclass
class FakeKld7Serial:
    clock: Clock
    frame_gen: Optional[FrameGen] = None
    sensor_baud: int = 115200            # current sensor rate (set to a high rate to simulate a stale connection)
    version: str = "K-LD7_APP-RFB-0104"
    sensor_delay_s: float = 0.010
    frame_period_s: float = 0.029
    skip_frames: Set[int] = field(default_factory=set)      # frame numbers the sensor never reports (gap test)
    silent_from: Optional[float] = None                     # sensor stops answering after this time
    silent_until: Optional[float] = None
    reject_srps_code: Optional[int] = None                  # RESP code to answer SRPS with (None = OK)
    corrupt_every_n: int = 0                                # inject garbage before every n-th frame
    baudrate: int = 115200                                  # host side (pyserial-like attribute)
    timeout: float = 0.02
    _tx: List[Tuple[float, bytes]] = field(default_factory=list)    # (ready time, bytes) sensor -> host
    _rx: bytearray = field(default_factory=bytearray)
    _parser: K.Kld7Parser = field(default_factory=K.Kld7Parser)
    _pending_gnfd: Optional[int] = None
    _t_gnfd: Optional[float] = None
    params: K.RadarParams = None
    frames_sent: int = 0
    gnfd_count: int = 0
    closed: bool = False
    log: List[str] = field(default_factory=list)

    def __post_init__(self) -> None:
        if self.params is None:
            self.params = K.RadarParams(self.version)

    # -- pyserial-like API ------------------------------------------------------------------------
    @property
    def in_waiting(self) -> int:
        self._deliver()
        return len(self._rx)

    def reset_input_buffer(self) -> None:
        self._rx.clear()
        self._tx.clear()

    def close(self) -> None:
        self.closed = True

    def write(self, data: bytes) -> int:
        if self._silent():
            return len(data)
        if self.baudrate != self.sensor_baud:
            return len(data)                    # garbage at the sensor: ignored
        for cmd in self._parse_commands(data):
            self._handle(cmd)
        return len(data)

    def _next_ready(self) -> Optional[float]:
        cands = [t for t, _ in self._tx]
        if self._pending_gnfd is not None and not self._silent():
            fn_next = int(self._t_gnfd // self.frame_period_s) + 1
            cands.append(fn_next * self.frame_period_s + self.sensor_delay_s)
        return min(cands) if cands else None

    def read(self, n: int = 1) -> bytes:
        self._deliver()
        if not self._rx and self.timeout > 0:
            # wait (simulated) for the next ready byte or the timeout
            nxt = self._next_ready()
            now = self.clock.now()
            if nxt is not None and nxt - now <= self.timeout:
                self.clock.sleep(max(0.0, nxt - now))
            else:
                self.clock.sleep(self.timeout)
            self._deliver()
        out = bytes(self._rx[:n])
        del self._rx[:n]
        return out

    # -- sensor behaviour ------------------------------------------------------------------------------
    def _silent(self) -> bool:
        t = self.clock.now()
        return self.silent_from is not None and t >= self.silent_from and (self.silent_until is None or t < self.silent_until)

    def _deliver(self) -> None:
        now = self.clock.now()
        keep = []
        for t_ready, b in self._tx:
            if t_ready <= now:
                if self.baudrate == self.sensor_baud:
                    self._rx.extend(b)
                else:
                    self._rx.extend(bytes([0xFF ^ x for x in b]))     # garbage at the wrong host baud
            else:
                keep.append((t_ready, b))
        self._tx = keep
        # frame delivery for a pending GNFD
        if self._pending_gnfd is not None and not self._silent():
            fn_next = int(self._t_gnfd // self.frame_period_s) + 1
            t_done = fn_next * self.frame_period_s + self.sensor_delay_s
            if now >= t_done:
                self._emit_frame(fn_next, t_done)
                self._pending_gnfd = None

    def _emit_frame(self, fn: int, t_ready: float) -> None:
        if fn in self.skip_frames:
            # the sensor skipped: the host sees the next frame number later; emulate by re-arming for the next frame
            self._t_gnfd = fn * self.frame_period_s
            return
        targets = list(self.frame_gen(fn, t_ready)) if self.frame_gen else []
        payload = b"".join(K.PDAT_TARGET_STRUCT.pack(t.distance_cm, t.speed_raw, t.angle_raw, t.magnitude_raw) for t in targets[:12])
        out = b""
        if self.corrupt_every_n and fn % self.corrupt_every_n == 0:
            out += b"\x00\xffXX"
        if self._pending_gnfd & K.GNFD_PDAT:
            out += K.build_command(b"PDAT", payload)
        if self._pending_gnfd & K.GNFD_DONE:
            out += K.build_command(b"DONE", struct.pack("<I", fn))
        self._tx.append((t_ready, out))
        self.frames_sent += 1

    def _parse_commands(self, data: bytes) -> List[K.Packet]:
        return self._parser.feed_commands(data) if hasattr(self._parser, "feed_commands") else _parse_cmds(self, data)

    def _resp(self, code: int, at: Optional[float] = None) -> None:
        self._tx.append((at if at is not None else self.clock.now(), K.build_command(b"RESP", bytes([code]))))

    def _handle(self, cmd: K.Packet) -> None:
        h, pl = cmd.header, cmd.payload
        now = self.clock.now()
        self.log.append(h.decode())
        if h == b"INIT":
            idx = struct.unpack("<I", pl)[0]
            rates = {v: k for k, v in K.BAUD_INDEX.items()}
            if idx not in rates:
                self._resp(K.RespCode.INVALID_PARAMETER)
                return
            self._resp(K.RespCode.OK)
            self._deliver_now_then_switch(rates[idx])
        elif h == b"GBYE":
            self._resp(K.RespCode.OK)
            self._deliver_now_then_switch(115200)
        elif h == b"GRPS":
            self._resp(K.RespCode.OK)
            self._tx.append((now, K.build_command(b"RPST", self.params.pack())))
        elif h == b"SRPS":
            if self.reject_srps_code is not None:
                self._resp(self.reject_srps_code)
                return
            try:
                newp = K.RadarParams.unpack(pl)
            except ValueError:
                self._resp(K.RespCode.INVALID_PARAMETER)
                return
            if newp.software_version != self.version:
                self._resp(K.RespCode.INVALID_RPST_VERSION)
                return
            self.params = newp
            self._resp(K.RespCode.OK)
        elif h == b"GNFD":
            self.gnfd_count += 1
            if self._pending_gnfd is not None:
                self._resp(K.RespCode.SENSOR_BUSY)
                return
            self._resp(K.RespCode.OK)
            self._pending_gnfd = struct.unpack("<I", pl)[0]
            self._t_gnfd = now
        elif h in (b"RSPI", b"RRAI", b"THOF", b"DEDI", b"MISP", b"MASP", b"RBFR", b"TRFT", b"VISU"):
            fld = K.PARAM_NAME_TO_FIELD[h.decode()]
            self.params = self.params.__class__(**{**self.params.__dict__, fld: struct.unpack("<i", pl)[0]})
            self._resp(K.RespCode.OK)
        else:
            self._resp(K.RespCode.UNKNOWN_COMMAND)

    def _deliver_now_then_switch(self, new_baud: int) -> None:
        # RESP goes out at the old rate, then the sensor switches
        self._deliver()
        self.sensor_baud = new_baud


def _parse_cmds(self: FakeKld7Serial, data: bytes) -> List[K.Packet]:
    buf = getattr(self, "_cmdbuf", bytearray())
    buf.extend(data)
    out: List[K.Packet] = []
    while len(buf) >= 8:
        hdr = bytes(buf[:4])
        n = struct.unpack("<I", buf[4:8])[0]
        if not hdr.isalpha() or n > 64:
            del buf[:1]
            continue
        if len(buf) < 8 + n:
            break
        out.append(K.Packet(hdr, bytes(buf[8:8 + n])))
        del buf[:8 + n]
    self._cmdbuf = buf
    return out

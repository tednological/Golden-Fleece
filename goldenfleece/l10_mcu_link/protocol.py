"""MCU link protocol: pure framing, CRC and message codec.  No I/O here.

Team decision (Stage 0 answer 5): the serial link is the alert channel and the
MCU firmware matches KEYWORDS, so the wire format is ASCII lines:

    $GF1,<TYPE>,<seq>,<fields...>*<CRC16>\\n

  * GF1           protocol id + version 1
  * TYPE          keyword (ALERT WARN HB RCFG HLTH  |  STAT OUTG)
  * seq           0..65535 per direction, wraps
  * CRC16         CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over the bytes
                  between '$' and '*' (exclusive), 4 uppercase hex digits
  * max line      96 bytes including the newline (ATmega328P buffer)

The full specification is docs/mcu_icd.md.  Keep this module and the ICD in step.
"""
from __future__ import annotations

import enum
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple, Union

from ..types import HealthBits, HealthState, Side, TArrivalBucket, ThreatLevel, WarningCommand

PROTOCOL_ID = "GF1"
MAX_LINE = 96

SIDE_CODE = {Side.LEFT: "L", Side.CENTER: "C", Side.RIGHT: "R", Side.BOTH: "B", Side.NONE: "N"}
SIDE_FROM = {v: k for k, v in SIDE_CODE.items()}


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class MsgType(str, enum.Enum):
    ALERT = "ALERT"
    WARN = "WARN"
    HB = "HB"
    RCFG = "RCFG"
    HLTH = "HLTH"
    STAT = "STAT"      # MCU -> Pi
    OUTG = "OUTG"      # MCU -> Pi


@dataclass(frozen=True)
class Message:
    type: MsgType
    seq: int
    fields: Tuple[str, ...]

    def __getitem__(self, i: int) -> str:
        return self.fields[i]


class FrameError(ValueError):
    pass


# --- framing ---------------------------------------------------------------------------------
def encode_line(mtype: MsgType, seq: int, fields: Sequence[Union[str, int]]) -> bytes:
    body = ",".join([PROTOCOL_ID, mtype.value, str(seq & 0xFFFF)] + [str(f) for f in fields])
    if "*" in body or "$" in body or "\n" in body:
        raise FrameError("illegal character in payload")
    crc = crc16_ccitt_false(body.encode("ascii"))
    line = f"${body}*{crc:04X}\n".encode("ascii")
    if len(line) > MAX_LINE:
        raise FrameError(f"line too long ({len(line)} > {MAX_LINE})")
    return line


def decode_line(line: bytes) -> Message:
    """Parse one complete line (with or without the trailing newline).  Raises FrameError."""
    s = line.strip(b"\r\n")
    if not s.startswith(b"$") or b"*" not in s:
        raise FrameError("missing framing")
    body, _, crc_s = s[1:].rpartition(b"*")
    if len(crc_s) != 4:
        raise FrameError("bad crc field")
    try:
        crc_rx = int(crc_s, 16)
    except ValueError as e:
        raise FrameError("bad crc hex") from e
    if crc16_ccitt_false(body) != crc_rx:
        raise FrameError("crc mismatch")
    parts = body.decode("ascii", errors="strict").split(",")
    if len(parts) < 3 or parts[0] != PROTOCOL_ID:
        raise FrameError("bad protocol id")
    try:
        mtype = MsgType(parts[1])
    except ValueError as e:
        raise FrameError(f"unknown type {parts[1]!r}") from e
    try:
        seq = int(parts[2])
    except ValueError as e:
        raise FrameError("bad seq") from e
    return Message(mtype, seq, tuple(parts[3:]))


class LineSplitter:
    """Byte stream -> complete lines.  Resyncs on '$'.  Counts garbage."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.n_garbage_bytes = 0
        self.n_overlong = 0

    def feed(self, data: bytes) -> List[bytes]:
        out: List[bytes] = []
        self._buf.extend(data)
        while True:
            nl = self._buf.find(b"\n")
            if nl < 0:
                if len(self._buf) > 4 * MAX_LINE:
                    self.n_overlong += 1
                    self.n_garbage_bytes += len(self._buf)
                    self._buf.clear()
                return out
            line = bytes(self._buf[: nl + 1])
            del self._buf[: nl + 1]
            start = line.find(b"$")
            if start < 0:
                self.n_garbage_bytes += len(line)
                continue
            if start > 0:
                self.n_garbage_bytes += start
                line = line[start:]
            out.append(line)


# --- Pi -> MCU message builders --------------------------------------------------------------------
def ms32(t_s: float) -> int:
    return int(round(t_s * 1000.0)) & 0xFFFFFFFF


def build_alert(seq: int, asserted: bool) -> bytes:
    return encode_line(MsgType.ALERT, seq, [1 if asserted else 0])


def build_warn(seq: int, cmd: WarningCommand) -> bytes:
    return encode_line(MsgType.WARN, seq, [int(cmd.level), SIDE_CODE[cmd.side], int(cmd.t_arrival_bucket),
                                           cmd.health_state.name, f"{int(cmd.health_bits):X}", cmd.seq & 0xFFFFFFFF,
                                           ms32(cmd.t_decided)])


def build_hb(seq: int, loop_iter: int, frames_processed: int, last_frame_number: int, health_state: HealthState,
             health_bits: HealthBits) -> bytes:
    return encode_line(MsgType.HB, seq, [loop_iter & 0xFFFFFFFF, frames_processed & 0xFFFFFFFF, max(last_frame_number, 0) & 0xFFFFFFFF,
                                         health_state.name, f"{int(health_bits):X}"])


def build_rcfg(seq: int, rrai: int, rspi: int, baud: int, blind_cms: int, t_alert_ms: int, t_warn_ms: int) -> bytes:
    return encode_line(MsgType.RCFG, seq, [rrai, rspi, baud, blind_cms, t_alert_ms, t_warn_ms])


def build_hlth(seq: int, state: HealthState, bits: HealthBits, onset_ms: int) -> bytes:
    return encode_line(MsgType.HLTH, seq, [state.name, f"{int(bits):X}", onset_ms & 0xFFFFFFFF])


# --- MCU -> Pi ---------------------------------------------------------------------------------------
def build_stat(seq: int, mode: str, mcu_ms: int, last_pseq: int, hb_age_ms: int, outages: int) -> bytes:
    return encode_line(MsgType.STAT, seq, [mode, mcu_ms & 0xFFFFFFFF, last_pseq & 0xFFFFFFFF, hb_age_ms & 0xFFFF, outages & 0xFFFF])


def build_outg(seq: int, start_ms: int, dur_ms: int, cause: str) -> bytes:
    return encode_line(MsgType.OUTG, seq, [start_ms & 0xFFFFFFFF, dur_ms & 0xFFFFFFFF, cause])


# --- parsed views -----------------------------------------------------------------------------------------
@dataclass(frozen=True)
class WarnView:
    level: ThreatLevel
    side: Side
    bucket: TArrivalBucket
    health_state: HealthState
    health_bits: HealthBits
    pipeline_seq: int
    t_decided_ms: int


def parse_warn(m: Message) -> WarnView:
    if m.type is not MsgType.WARN or len(m.fields) < 7:
        raise FrameError("not a WARN")
    return WarnView(ThreatLevel(int(m[0])), SIDE_FROM[m[1]], TArrivalBucket(int(m[2])), HealthState[m[3]],
                    HealthBits(int(m[4], 16)), int(m[5]), int(m[6]))


@dataclass(frozen=True)
class HbView:
    loop_iter: int
    frames_processed: int
    last_frame_number: int
    health_state: HealthState
    health_bits: HealthBits


def parse_hb(m: Message) -> HbView:
    if m.type is not MsgType.HB or len(m.fields) < 5:
        raise FrameError("not a HB")
    return HbView(int(m[0]), int(m[1]), int(m[2]), HealthState[m[3]], HealthBits(int(m[4], 16)))


@dataclass(frozen=True)
class StatView:
    mode: str
    mcu_ms: int
    last_pseq: int
    hb_age_ms: int
    outages: int


def parse_stat(m: Message) -> StatView:
    if m.type is not MsgType.STAT or len(m.fields) < 5:
        raise FrameError("not a STAT")
    return StatView(m[0], int(m[1]), int(m[2]), int(m[3]), int(m[4]))

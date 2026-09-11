"""K-LD7 wire protocol, written from the RFbeam datasheet (Rev B, 03/2021).  Pure: no I/O, no time.

Packet (Table 9):  4-byte ASCII header | uint32 LE payload length | payload (all fields LE).
Commands (Table 11/14): INIT(4) GNFD(4) GRPS(0) SRPS(42) RFSE(0) GBYE(0) and 4-byte parameter commands.
Messages (Table 10/13): RESP(1) PDAT(0-96) DONE(4) TDAT(0-8) DDAT(6) RPST(42) RFFT(1024) RADC(3072).
PDAT target (8 B): u16 distance cm | i16 speed km/h x100 (+ receding) | i16 angle deg x100 | u16 magnitude dB x100.
RESP codes (Table 13): 0 OK, 1 unknown command, 2 invalid parameter value, 3 invalid RPST version,
4 UART error (parity, framing, noise), 5 sensor busy, 6 timeout error.
Datasheet erratum: Fig. 19 shows the GBYE example with the INIT header bytes; the header is ASCII "GBYE".

Rule 4.1: this module moves bytes only.  No unit conversion happens here (that is l03).
"""
from __future__ import annotations

import enum
import struct
from dataclasses import dataclass, replace
from typing import Dict, List, Optional, Tuple

from ..types import RawRadarTarget

HEADER_LEN = 8
MAX_PAYLOAD = 3072
MESSAGE_HEADERS = {b"RESP", b"PDAT", b"DONE", b"TDAT", b"DDAT", b"RPST", b"RFFT", b"RADC"}
EXPECTED_LEN = {b"RESP": 1, b"DONE": 4, b"DDAT": 6, b"RPST": 42, b"RFFT": 1024, b"RADC": 3072}

BAUD_INDEX: Dict[int, int] = {115200: 0, 460800: 1, 921600: 2, 2000000: 3, 3000000: 4}
DEFAULT_BAUD = 115200
BITS_PER_BYTE_8E1 = 11

GNFD_RADC, GNFD_RFFT, GNFD_PDAT, GNFD_TDAT, GNFD_DDAT, GNFD_DONE = 0x01, 0x02, 0x04, 0x08, 0x10, 0x20
GNFD_WANTED = GNFD_PDAT | GNFD_DONE          # exactly PDAT | DONE (task §8); TDAT is never requested

PDAT_TARGET_STRUCT = struct.Struct("<HhhH")
PDAT_MAX_TARGETS = 12


class RespCode(enum.IntEnum):
    OK = 0
    UNKNOWN_COMMAND = 1
    INVALID_PARAMETER = 2
    INVALID_RPST_VERSION = 3
    UART_ERROR = 4
    SENSOR_BUSY = 5
    TIMEOUT = 6


@dataclass(frozen=True)
class Packet:
    header: bytes
    payload: bytes


# --- commands -------------------------------------------------------------------------------
def build_command(header: bytes, payload: bytes = b"") -> bytes:
    if len(header) != 4:
        raise ValueError("header must be 4 bytes")
    return header + struct.pack("<I", len(payload)) + payload


def cmd_init(baud: int) -> bytes:
    return build_command(b"INIT", struct.pack("<I", BAUD_INDEX[baud]))


def cmd_gnfd(mask: int = GNFD_WANTED) -> bytes:
    return build_command(b"GNFD", struct.pack("<I", mask))


def cmd_grps() -> bytes:
    return build_command(b"GRPS")


def cmd_srps(rpst: bytes) -> bytes:
    if len(rpst) != 42:
        raise ValueError("RPST must be 42 bytes")
    return build_command(b"SRPS", rpst)


def cmd_gbye() -> bytes:
    return build_command(b"GBYE")


def cmd_param(name: str, value: int) -> bytes:
    """Single-parameter command, e.g. RSPI/RRAI/THOF (u32 LE payload; INT8 params are sign-extended by the sensor)."""
    return build_command(name.encode("ascii"), struct.pack("<i", int(value)))


# --- parameter structure (Table 12, 42 bytes) -------------------------------------------------------
_RPST_STRUCT = struct.Struct("<19sBBBBBBBBbbBBBBbBBBBHBB")

_RPST_FIELDS = ("base_frequency", "max_speed", "max_range", "threshold_offset", "tracking_filter", "vibration_suppression",
                "min_det_distance", "max_det_distance", "min_det_angle", "max_det_angle", "min_det_speed", "max_det_speed",
                "det_direction", "range_threshold", "angle_threshold", "speed_threshold", "dig1", "dig2", "dig3", "hold_time",
                "micro_det_retrigger", "micro_det_sensitivity")

PARAM_NAME_TO_FIELD = {"RBFR": "base_frequency", "RSPI": "max_speed", "RRAI": "max_range", "THOF": "threshold_offset",
                       "TRFT": "tracking_filter", "VISU": "vibration_suppression", "MIRA": "min_det_distance", "MARA": "max_det_distance",
                       "MIAN": "min_det_angle", "MAAN": "max_det_angle", "MISP": "min_det_speed", "MASP": "max_det_speed",
                       "DEDI": "det_direction", "RATH": "range_threshold", "ANTH": "angle_threshold", "SPTH": "speed_threshold",
                       "DIG1": "dig1", "DIG2": "dig2", "DIG3": "dig3", "HOLD": "hold_time", "MIDE": "micro_det_retrigger",
                       "MIDS": "micro_det_sensitivity"}


@dataclass(frozen=True)
class RadarParams:
    software_version: str
    base_frequency: int = 1
    max_speed: int = 1
    max_range: int = 1
    threshold_offset: int = 30
    tracking_filter: int = 0
    vibration_suppression: int = 2
    min_det_distance: int = 0
    max_det_distance: int = 50
    min_det_angle: int = -90
    max_det_angle: int = 90
    min_det_speed: int = 0
    max_det_speed: int = 100
    det_direction: int = 2
    range_threshold: int = 10
    angle_threshold: int = 0
    speed_threshold: int = 50
    dig1: int = 0
    dig2: int = 1
    dig3: int = 2
    hold_time: int = 1
    micro_det_retrigger: int = 0
    micro_det_sensitivity: int = 4

    @classmethod
    def unpack(cls, payload: bytes) -> "RadarParams":
        if len(payload) != 42:
            raise ValueError(f"RPST payload must be 42 bytes, got {len(payload)}")
        vals = _RPST_STRUCT.unpack(payload)
        version = vals[0].split(b"\0", 1)[0].decode("ascii", errors="replace")
        return cls(version, *vals[1:])

    def pack(self) -> bytes:
        v = self.software_version.encode("ascii")[:18].ljust(19, b"\0")
        return _RPST_STRUCT.pack(v, *[getattr(self, f) for f in _RPST_FIELDS])

    def with_params(self, params: Dict[str, int]) -> "RadarParams":
        kw = {}
        for name, value in params.items():
            if name not in PARAM_NAME_TO_FIELD:
                raise ValueError(f"unknown K-LD7 parameter {name}")
            kw[PARAM_NAME_TO_FIELD[name]] = int(value)
        return replace(self, **kw)

    def same_settings(self, other: "RadarParams") -> bool:
        return all(getattr(self, f) == getattr(other, f) for f in _RPST_FIELDS)

    def diff(self, other: "RadarParams") -> Dict[str, Tuple[int, int]]:
        return {f: (getattr(self, f), getattr(other, f)) for f in _RPST_FIELDS if getattr(self, f) != getattr(other, f)}


# --- messages -----------------------------------------------------------------------------------------
def parse_pdat(payload: bytes) -> Tuple[RawRadarTarget, ...]:
    if len(payload) % 8 != 0 or len(payload) > 96:
        raise ValueError(f"PDAT payload length {len(payload)} is not a multiple of 8 within 0..96")
    return tuple(RawRadarTarget(*PDAT_TARGET_STRUCT.unpack_from(payload, i)) for i in range(0, len(payload), 8))


def parse_done(payload: bytes) -> int:
    if len(payload) != 4:
        raise ValueError("DONE payload must be 4 bytes")
    return struct.unpack("<I", payload)[0]


def parse_resp(payload: bytes) -> int:
    if len(payload) != 1:
        raise ValueError("RESP payload must be 1 byte")
    return payload[0]


class Kld7Parser:
    """Byte stream -> packets.  Resyncs by sliding one byte on anything that is not a known message header
    with a plausible length.  Every resync is counted (quiet failures are the enemy)."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.n_resync_bytes = 0
        self.n_packets = 0
        self.n_bad_length = 0
        # byte offset (within the stream) at which the last emitted packet's header started: for timestamping
        self.stream_pos = 0
        self.last_header_pos = 0

    def feed(self, data: bytes) -> List[Packet]:
        self._buf.extend(data)
        out: List[Packet] = []
        while True:
            if len(self._buf) < HEADER_LEN:
                return out
            hdr = bytes(self._buf[:4])
            if hdr not in MESSAGE_HEADERS:
                self._slide()
                continue
            n = struct.unpack("<I", self._buf[4:8])[0]
            exp = EXPECTED_LEN.get(hdr)
            if n > MAX_PAYLOAD or (exp is not None and n != exp) or (hdr == b"PDAT" and (n % 8 or n > 96)) or (hdr == b"TDAT" and n > 8):
                self.n_bad_length += 1
                self._slide()
                continue
            if len(self._buf) < HEADER_LEN + n:
                return out
            payload = bytes(self._buf[HEADER_LEN:HEADER_LEN + n])
            self.last_header_pos = self.stream_pos
            del self._buf[:HEADER_LEN + n]
            self.stream_pos += HEADER_LEN + n
            self.n_packets += 1
            out.append(Packet(hdr, payload))

    def _slide(self) -> None:
        del self._buf[:1]
        self.stream_pos += 1
        self.n_resync_bytes += 1

    def pending(self) -> int:
        return len(self._buf)


def header_transfer_s(baud: int, n_bytes: int = 4) -> float:
    return n_bytes * BITS_PER_BYTE_8E1 / float(baud)

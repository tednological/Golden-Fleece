"""A BNO085 on SPI that behaves like the part measured on 2026-09-14, for l02 tests.

What it reproduces:
  * RESET rising edge -> boot traffic: the 276-byte advertisement after 94 ms, then a 20-byte command
    response (0xF1) and the 5-byte reset complete (0x01) 60 ms later;
  * H_INTN asserted only while a packet is due;
  * transactions are full duplex: the sensor clocks out its pending packet while it reads the host's;
  * a host packet counts only if H_INTN was asserted when the transaction began (PS0/WAKE tied high);
  * a short read leaves the rest of the packet as a continuation (length | 0x8000);
  * Set Feature (0xFD) starts or stops a report stream; a Product ID request (0xF9) is answered with 0xF8.
"""
from __future__ import annotations

import struct
from contextlib import contextmanager
from typing import Dict, List, Tuple

REPORT_Q = {0x01: 8, 0x02: 9}          # accelerometer Q8 m/s^2, gyroscope Q9 rad/s


class FakeBno085:
    BOOT_INT_DELAY_S = 0.094
    BOOT_GAP_S = 0.060
    FEATURE_RESPONSE_DELAY_S = 0.05

    def __init__(self, clock, accel=(0.0, 0.0, 9.81), gyro=(0.0, 0.0, 0.0)):
        self.clock = clock
        self.values = {0x01: accel, 0x02: gyro}
        self.queue: List[Tuple[float, bytes]] = []
        self.features: Dict[int, List[float]] = {}      # report id -> [interval_s, next_due]
        self.reset_level = True
        self.dead = False                               # unplugged: never asserts H_INTN
        self.accepted: List[bytes] = []                 # host packets the sensor acted on
        self.ignored: List[bytes] = []                  # host packets written while H_INTN was deasserted
        self.continuations = 0
        self.closed = False
        self._seq = [0] * 6
        self._report_seq = 0

    # ---- SpiPort ---------------------------------------------------------------------------------------
    def int_asserted(self) -> bool:
        if self.dead or not self.reset_level:
            return False
        self._generate_reports()
        now = self.clock.now()
        return any(due <= now for due, _ in self.queue)

    def set_reset(self, level: bool) -> None:
        if level and not self.reset_level:
            self._boot()
        self.reset_level = level

    @contextmanager
    def transaction(self):
        int_at_start = self.int_asserted()
        now = self.clock.now()
        pending, pending_due = b"", now
        if int_at_start:
            i = next(i for i, (due, _) in enumerate(self.queue) if due <= now)
            pending_due, pending = self.queue.pop(i)
        tx = bytearray()

        def exchange(out: bytes) -> bytes:
            start = len(tx)
            tx.extend(out)
            return pending[start:start + len(out)].ljust(len(out), b"\0")
        yield exchange
        if len(tx) < len(pending):                      # cut short: the rest comes back as a continuation
            rest = pending[len(tx):]
            n = len(rest) + 4
            self.queue.insert(0, (pending_due, bytes([n & 0xFF, (n >> 8) | 0x80, pending[2], pending[3]]) + rest))   # keeps its place: sent next
            self.continuations += 1
        n_tx = (tx[0] | (tx[1] << 8)) & 0x7FFF if len(tx) >= 4 else 0
        if n_tx > 4:
            pkt = bytes(tx[:n_tx])
            if int_at_start:
                self.accepted.append(pkt)
                self._command(pkt)
            else:
                self.ignored.append(pkt)

    def close(self) -> None:
        self.closed = True

    # ---- test controls ---------------------------------------------------------------------------------
    def spontaneous_reset(self) -> None:
        self._boot()

    # ---- internals -------------------------------------------------------------------------------------
    def _packet(self, channel: int, payload: bytes) -> bytes:
        n = len(payload) + 4
        seq = self._seq[channel]
        self._seq[channel] = (seq + 1) & 0xFF
        return bytes([n & 0xFF, n >> 8, channel, seq]) + payload

    def _boot(self) -> None:
        t = self.clock.now() + self.BOOT_INT_DELAY_S
        self.features.clear()
        self.queue = [(t, self._packet(0, bytes(272))),
                      (t + self.BOOT_GAP_S, self._packet(2, bytes([0xF1]) + bytes(15))),
                      (t + self.BOOT_GAP_S, self._packet(1, bytes([0x01])))]

    def _command(self, pkt: bytes) -> None:
        channel, payload = pkt[2], pkt[4:]
        now = self.clock.now()
        if channel == 2 and payload[:1] == b"\xfd":
            rid = payload[1]
            interval_s = struct.unpack_from("<I", payload, 5)[0] / 1e6
            if interval_s > 0:
                self.features[rid] = [interval_s, now + self.FEATURE_RESPONSE_DELAY_S + interval_s]
            else:
                self.features.pop(rid, None)
            self._enqueue(now + self.FEATURE_RESPONSE_DELAY_S, self._packet(2, bytes([0xFC, rid]) + bytes(15)))
        elif channel == 2 and payload[:1] == b"\xf9":
            self._enqueue(now + 0.001, self._packet(2, bytes([0xF8]) + bytes(15)))

    def _enqueue(self, due: float, pkt: bytes) -> None:
        self.queue.append((due, pkt))
        self.queue.sort(key=lambda e: e[0])

    def _generate_reports(self) -> None:
        now = self.clock.now()
        for rid, f in self.features.items():
            while f[1] <= now:
                self._enqueue(f[1], self._report(rid))
                f[1] += f[0]

    def _report(self, rid: int) -> bytes:
        q = REPORT_Q[rid]
        xyz = [int(round(v * (1 << q))) for v in self.values[rid]]
        self._report_seq = (self._report_seq + 1) & 0xFF
        body = struct.pack("<BBBBhhh", rid, self._report_seq, 3, 0, *xyz)
        return self._packet(3, bytes([0xFB, 0, 0, 0, 0]) + body)

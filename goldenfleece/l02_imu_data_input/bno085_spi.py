"""l02: BNO085 over SPI.

SH-2 content comes from adafruit-circuitpython-bno08x: the Set Feature encoding, report lengths and the
Q-point parser (``BNO08X._get_feature_enable_report``, ``_separate_batch``, ``_parse_sensor_report_data``).
The SPI *transport* is ours, because the library's cannot start this sensor (bench, 2026-09-14):

  * it writes with a plain ``spi.write`` while H_INTN is asserted, discarding the packet the sensor clocks
    out in that same transaction; the sensor re-sends the remainder as a continuation, which the library
    treats as fatal, so it never gets past "Could not read ID";
  * with PS0/WAKE tied high the BNO085 ignores host writes while H_INTN is deasserted, so every write has to
    ride on a transaction the sensor started: at boot on its boot packets, afterwards on its reports, which
    arrive every 5 ms once the gyroscope is enabled.

So every transfer is ONE chip-select assertion, full duplex: our 4-byte header goes out while theirs comes
in, then max(ours, theirs) bytes of body.  Nothing the sensor sends is dropped, and every gyroscope and
accelerometer report becomes a sample (no newest-reading-only collapse), stamped when H_INTN is seen.

Adapter rules: no math (byte framing only); sample values are exactly what the library's parser returns
(rad/s, m/s^2); time only from the Clock; health events on init failure, silence and sensor resets.
No magnetometer, no rotation vector; the game rotation vector may be logged as a diagnostic, never fused.
"""
from __future__ import annotations

import logging
import threading
from contextlib import contextmanager
from typing import Any, Callable, Dict, Iterator, List, Optional, Protocol, Set, Tuple

from ..clock import Clock
from ..config import Section
from ..types import HealthBits, HealthEvent, ImuKind, RawImuSample

log = logging.getLogger("goldenfleece.l02")

CH_COMMAND, CH_EXECUTABLE, CH_CONTROL, CH_REPORTS = 0, 1, 2, 3
EXE_RESET_COMPLETE = 0x01
REPORT_ACCEL, REPORT_GYRO, REPORT_GAME_RV = 0x01, 0x02, 0x08
KIND = {REPORT_GYRO: ImuKind.GYRO, REPORT_ACCEL: ImuKind.ACCEL, REPORT_GAME_RV: ImuKind.GAME_RV}
GAME_RV_INTERVAL_US = 50000

BOOT_INT_TIMEOUT_S = 1.0          # first H_INTN after RESET: 94 ms measured
BOOT_PACKET_TIMEOUT_S = 0.5       # the boot packet the first write rides on: 60 ms after the advertisement
REPORTS_TIMEOUT_S = 1.5           # every requested report seen at least once
RESEND_AFTER_S = 0.3              # a Set Feature whose reports have not appeared by then is sent again
SILENCE_REINIT_S = 0.5            # streaming, but no packet at all for this long: reset and start again
POLL_S = 0.0002                   # H_INTN poll period; the sensor retries an unanswered INT after ~10 ms


class SensorReset(RuntimeError):
    """The sensor rebooted on its own; its reports are off again."""


class SensorSilent(RuntimeError):
    """No packet for SILENCE_REINIT_S while streaming."""


class SpiPort(Protocol):
    """What the transport needs from the hardware."""

    def int_asserted(self) -> bool: ...
    def set_reset(self, level: bool) -> None: ...
    def transaction(self) -> Any: ...    # context manager yielding exchange(out) -> bytes; CS held throughout
    def close(self) -> None: ...


class BlinkaSpiPort:
    """SPI0 via Blinka, mode 3.  CS, INT and RESET are plain GPIOs; CS is not CE0 (the kernel owns GPIO8)."""

    def __init__(self, pins: Dict[str, str], baudrate: int) -> None:
        import board  # noqa: WPS433
        import digitalio  # noqa: WPS433
        from adafruit_bus_device.spi_device import SPIDevice  # noqa: WPS433
        made = []
        try:
            for key in ("cs", "int", "reset"):
                made.append(digitalio.DigitalInOut(getattr(board, pins[key])))
            cs, self._int, self._rst = made
            self._int.switch_to_input(pull=digitalio.Pull.UP)
            self._rst.switch_to_output(value=True)
            self._dev = SPIDevice(board.SPI(), cs, baudrate=baudrate, polarity=1, phase=1)
        except Exception:
            for pin in made:
                pin.deinit()
            raise
        self._pins = tuple(made)

    def int_asserted(self) -> bool:
        return not self._int.value           # H_INTN is active low

    def set_reset(self, level: bool) -> None:
        self._rst.value = level

    @contextmanager
    def transaction(self) -> Iterator[Callable[[bytes], bytes]]:
        with self._dev as spi:               # CS low for the whole block, across several transfers
            def exchange(out: bytes) -> bytes:
                buf = bytearray(len(out))
                spi.write_readinto(out, buf)
                return bytes(buf)
            yield exchange

    def close(self) -> None:
        for pin in self._pins:
            pin.deinit()


def open_bno085_spi(pins: Dict[str, str], spi_baudrate: int) -> BlinkaSpiPort:
    """Imports are lazy (inside BlinkaSpiPort) so pure tests never need Blinka."""
    return BlinkaSpiPort(pins, int(spi_baudrate))


def shtp_packet(channel: int, seq: int, payload: bytes) -> bytes:
    n = len(payload) + 4
    return bytes((n & 0xFF, n >> 8, channel, seq & 0xFF)) + bytes(payload)


class ShtpSpi:
    """SHTP over SPI the way the BNO085 behaves: one full-duplex chip-select assertion per transfer."""

    def __init__(self, port: SpiPort) -> None:
        self.port = port
        self._seq = [0] * 6
        self.n_continuations = 0

    def transfer(self, tx: bytes = b"") -> Optional[bytes]:
        """Clock ``tx`` out while reading the sensor's pending packet; return that packet (header included)
        or None.  A write only counts while H_INTN is asserted."""
        with self.port.transaction() as exchange:
            hdr = exchange(tx[:4].ljust(4, b"\0"))
            rx_len = (hdr[0] | (hdr[1] << 8)) & 0x7FFF
            if rx_len == 0x7FFF:             # 0xFFFF on the wire: nothing valid
                rx_len = 0
            rest = max(len(tx), rx_len) - 4
            body = exchange(tx[4:].ljust(rest, b"\0")) if rest > 0 else b""
        if rx_len < 4:
            return None
        if hdr[1] & 0x80:                    # the tail of a packet an earlier transfer cut short
            self.n_continuations += 1
            return None
        return hdr + body[:rx_len - 4]

    def send(self, channel: int, payload: bytes) -> Optional[bytes]:
        """Write one packet; call only while H_INTN is asserted.  Returns the packet received meanwhile."""
        pkt = shtp_packet(channel, self._seq[channel], payload)
        self._seq[channel] = (self._seq[channel] + 1) & 0xFF
        return self.transfer(pkt)


class Bno085SpiSource:
    def __init__(self, imu_cfg: Section, clock: Clock, device_factory: Optional[Callable[[], SpiPort]] = None) -> None:
        self.cfg = imu_cfg
        self.clock = clock
        pins = imu_cfg.pins.as_dict() if "pins" in imu_cfg else {"cs": "D5", "int": "D25", "reset": "D24"}
        self._factory = device_factory or (lambda: open_bno085_spi(pins, imu_cfg.get("spi_baudrate", 1000000)))
        self.gyro_interval_us = int(1e6 / float(imu_cfg.gyro_rate_hz))
        self.accel_interval_us = int(1e6 / float(imu_cfg.accel_rate_hz))
        self.log_game_rv = bool(imu_cfg.get("log_game_rotation_vector", False))
        self._buf: List[RawImuSample] = []
        self._events: List[HealthEvent] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.port: Optional[SpiPort] = None
        self.shtp: Optional[ShtpSpi] = None
        self._lib: Any = None
        self._streaming = False
        self._backoff = 0.25
        self._last_packet_t = 0.0
        self._seen: Set[int] = set()
        self.seq = 0
        self.n_gyro = 0
        self.n_accel = 0
        self.n_errors = 0
        self.n_reinit = 0
        self.max_buffer = 4096
        self.dropped = 0

    # ---- ImuSource ----------------------------------------------------------------------------------------
    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="gf-bno085", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        if self.port is not None:
            self.port.close()
            self.port = None

    def drain(self) -> List[RawImuSample]:
        with self._lock:
            b, self._buf = self._buf, []
        return b

    def drain_events(self) -> List[HealthEvent]:
        with self._lock:
            ev, self._events = self._events, []
        return ev

    # ---- reader thread ------------------------------------------------------------------------------------
    def _run(self) -> None:
        while not self._stop.is_set():
            self._step()

    def _step(self) -> None:
        if not self._streaming:
            if self._init_device():
                self._backoff = 0.25
            else:
                self.clock.sleep(self._backoff)
                self._backoff = min(self._backoff * 2, 4.0)
            return
        try:
            self._poll_once()
        except Exception as e:  # noqa: BLE001
            self.n_errors += 1
            self.n_reinit += 1
            self._streaming = False
            self._event(True, f"{e}; reinitialising")

    def _poll_once(self) -> None:
        if not self.port.int_asserted():
            if self.clock.now() - self._last_packet_t > SILENCE_REINIT_S:
                raise SensorSilent(f"sensor silent for {SILENCE_REINIT_S} s")
            self.clock.sleep(POLL_S)
            return
        t = self.clock.now()                 # stamped as H_INTN is seen, before the transfer
        self._handle(self.shtp.transfer(), t, streaming=True)

    # ---- startup ------------------------------------------------------------------------------------------
    def _init_device(self) -> bool:
        try:
            import adafruit_bno08x as lib  # noqa: WPS433
        except ImportError as e:
            self._event(True, f"library missing: {e}")
            return False
        self._lib = lib
        try:
            if self.port is None:
                self.port = self._factory()
                self.shtp = ShtpSpi(self.port)
            self.port.set_reset(True)
            self.clock.sleep(0.01)
            self.port.set_reset(False)
            self.clock.sleep(0.01)
            self.port.set_reset(True)
            if not self._wait_int(BOOT_INT_TIMEOUT_S):
                raise RuntimeError(f"no H_INTN within {BOOT_INT_TIMEOUT_S} s of reset (power, wiring, or P0/P1 not high)")
            self.shtp.transfer()             # the advertisement
            self._enable_reports()
        except Exception as e:  # noqa: BLE001
            self.n_errors += 1
            self._event(True, f"init failed: {e}")
            return False
        self._streaming = True
        self._last_packet_t = self.clock.now()
        self._event(False, f"initialised: gyro {self.gyro_interval_us} us, accel {self.accel_interval_us} us requested")
        return True

    def _features(self) -> List[Tuple[int, int]]:
        f = [(REPORT_GYRO, self.gyro_interval_us), (REPORT_ACCEL, self.accel_interval_us)]
        if self.log_game_rv:
            f.append((REPORT_GAME_RV, GAME_RV_INTERVAL_US))
        return f

    def _enable_reports(self) -> None:
        """Every Set Feature rides on a transfer the sensor started: the first on a boot packet, the rest on
        whatever arrives next.  Done once each requested report has been seen."""
        wanted = dict(self._features())
        queue = list(wanted)
        sent_at: Dict[int, float] = {}
        self._seen = set()
        if not self._wait_int(BOOT_PACKET_TIMEOUT_S):
            raise RuntimeError("no boot packet to carry the first Set Feature")
        deadline = self.clock.now() + REPORTS_TIMEOUT_S
        while not set(wanted) <= self._seen:
            now = self.clock.now()
            if now >= deadline:
                missing = ", ".join(f"0x{r:02X}" for r in sorted(set(wanted) - self._seen))
                raise RuntimeError(f"reports never arrived: {missing}")
            for rid, t_sent in sent_at.items():
                if rid not in self._seen and rid not in queue and now - t_sent > RESEND_AFTER_S:
                    queue.append(rid)
            if not self.port.int_asserted():
                self.clock.sleep(POLL_S)
                continue
            if queue:
                rid = queue.pop(0)
                report = self._lib.BNO08X._get_feature_enable_report(rid, wanted[rid])
                raw = self.shtp.send(CH_CONTROL, bytes(report))
                sent_at[rid] = now
            else:
                raw = self.shtp.transfer()
            self._handle(raw, now, streaming=False)

    def _wait_int(self, timeout: float) -> bool:
        deadline = self.clock.now() + timeout
        while not self.port.int_asserted():
            if self.clock.now() >= deadline:
                return False
            self.clock.sleep(POLL_S)
        return True

    # ---- packets ------------------------------------------------------------------------------------------
    def _handle(self, raw: Optional[bytes], t: float, streaming: bool) -> None:
        if raw is None:
            return
        self._last_packet_t = t
        channel = raw[2]
        if channel == CH_REPORTS:
            slices: List[Any] = []
            self._lib._separate_batch(self._lib.Packet(bytearray(raw)), slices)
            for rid, body in slices:         # in arrival order (the library's own handler pops them LIFO)
                kind = KIND.get(rid)
                if kind is None or (kind is ImuKind.GAME_RV and not self.log_game_rv):
                    continue
                values, _accuracy = self._lib._parse_sensor_report_data(body)
                self._seen.add(rid)
                self.seq += 1
                if kind is ImuKind.GYRO:
                    self.n_gyro += 1
                elif kind is ImuKind.ACCEL:
                    self.n_accel += 1
                self._push(RawImuSample(t, kind, tuple(float(v) for v in values), self.seq))
        elif streaming and (channel == CH_COMMAND or (channel == CH_EXECUTABLE and raw[4:5] == bytes([EXE_RESET_COMPLETE]))):
            raise SensorReset("sensor reset itself")

    def _event(self, active: bool, detail: str) -> None:
        with self._lock:
            self._events.append(HealthEvent(self.clock.now(), HealthBits.IMU_FAULT, active, "l02", detail))

    def _push(self, s: RawImuSample) -> None:
        with self._lock:
            if len(self._buf) >= self.max_buffer:
                self._buf.pop(0)
                self.dropped += 1
            self._buf.append(s)

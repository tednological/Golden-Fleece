"""l02: BNO085 over SPI via adafruit-circuitpython-bno08x (SPI class) and Blinka.

Adapter rules: no math; samples are emitted exactly as the library returns them (rad/s, m/s^2)
with a Clock timestamp taken when the INT line reported data ready.  Reports: raw gyroscope and
accelerometer only.  No magnetometer, no rotation vector; the game rotation vector may be logged
as a diagnostic and is never fused.  Reset / re-initialisation on fault with health events.

The library exposes the newest reading per report type; it has no per-sample timestamps or queue.
The adapter detects a new reading by object identity change after processing packets.  Whether this
sustains >= 100 Hz with usable timing is measured by tools/bno085_probe.py (task §9.2: stop and
report if not).
"""
from __future__ import annotations

import logging
import threading
from typing import Any, Callable, List, Optional

from ..clock import Clock
from ..config import Section
from ..types import HealthBits, HealthEvent, ImuKind, RawImuSample

log = logging.getLogger("goldenfleece.l02")


def open_bno085_spi(pins: dict, spi_baudrate: int) -> Any:
    """Create the library device.  Imports are lazy so pure tests never need Blinka."""
    import board  # noqa: WPS433
    import digitalio  # noqa: WPS433
    from adafruit_bno08x.spi import BNO08X_SPI  # noqa: WPS433
    spi = board.SPI()
    cs = digitalio.DigitalInOut(getattr(board, pins["cs"]))
    intp = digitalio.DigitalInOut(getattr(board, pins["int"]))
    intp.direction = digitalio.Direction.INPUT
    rst = digitalio.DigitalInOut(getattr(board, pins["reset"]))
    return BNO08X_SPI(spi, cs, intp, rst, baudrate=int(spi_baudrate))


class Bno085SpiSource:
    def __init__(self, imu_cfg: Section, clock: Clock, device_factory: Optional[Callable[[], Any]] = None) -> None:
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
        self.dev: Any = None
        self.seq = 0
        self.n_gyro = 0
        self.n_accel = 0
        self.n_errors = 0
        self.n_reinit = 0
        self.max_buffer = 4096
        self.dropped = 0

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="gf-bno085", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)

    def drain(self) -> List[RawImuSample]:
        with self._lock:
            b, self._buf = self._buf, []
        return b

    def drain_events(self) -> List[HealthEvent]:
        with self._lock:
            ev, self._events = self._events, []
        return ev

    def _event(self, active: bool, detail: str) -> None:
        with self._lock:
            self._events.append(HealthEvent(self.clock.now(), HealthBits.IMU_FAULT, active, "l02", detail))

    def _push(self, s: RawImuSample) -> None:
        with self._lock:
            if len(self._buf) >= self.max_buffer:
                self._buf.pop(0)
                self.dropped += 1
            self._buf.append(s)

    def _init_device(self) -> bool:
        try:
            from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GAME_ROTATION_VECTOR, BNO_REPORT_GYROSCOPE  # noqa: WPS433
        except ImportError as e:
            self._event(True, f"library missing: {e}")
            return False
        try:
            self.dev = self._factory()
            self.dev.enable_feature(BNO_REPORT_GYROSCOPE, report_interval=self.gyro_interval_us)
            self.dev.enable_feature(BNO_REPORT_ACCELEROMETER, report_interval=self.accel_interval_us)
            if self.log_game_rv:
                self.dev.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR, report_interval=50000)
            self._event(False, "initialised")
            return True
        except Exception as e:  # noqa: BLE001
            self.n_errors += 1
            self.dev = None
            self._event(True, f"init failed: {e}")
            return False

    def _run(self) -> None:
        backoff = 0.25
        last_g = last_a = last_q = None
        while not self._stop.is_set():
            if self.dev is None:
                if self._init_device():
                    backoff = 0.25
                else:
                    self.clock.sleep(backoff)
                    backoff = min(backoff * 2, 4.0)
                    continue
            try:
                ready = self.dev._data_ready if hasattr(self.dev, "_data_ready") else True
                if not ready:
                    self.clock.sleep(0.0005)
                    continue
                t = self.clock.now()
                self.dev._process_available_packets(max_packets=8)
                g = self.dev._readings.get(2) if hasattr(self.dev, "_readings") else self.dev.gyro
                a = self.dev._readings.get(1) if hasattr(self.dev, "_readings") else self.dev.acceleration
                if g is not None and g is not last_g:
                    last_g = g
                    self.seq += 1
                    self.n_gyro += 1
                    self._push(RawImuSample(t, ImuKind.GYRO, tuple(float(v) for v in g), self.seq))
                if a is not None and a is not last_a:
                    last_a = a
                    self.seq += 1
                    self.n_accel += 1
                    self._push(RawImuSample(t, ImuKind.ACCEL, tuple(float(v) for v in a), self.seq))
                if self.log_game_rv and hasattr(self.dev, "_readings"):
                    q = self.dev._readings.get(8)
                    if q is not None and q is not last_q:
                        last_q = q
                        self.seq += 1
                        self._push(RawImuSample(t, ImuKind.GAME_RV, tuple(float(v) for v in q), self.seq))
            except Exception as e:  # noqa: BLE001
                self.n_errors += 1
                self.n_reinit += 1
                self._event(True, f"read error, reinitialising: {e}")
                try:
                    self.dev.hard_reset()
                except Exception:  # noqa: BLE001
                    pass
                self.dev = None

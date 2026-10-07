"""l10 hardware half: one PWM output per vibration motor.  On or off; no math, no timing, no threads.

Each motor hangs off a Pi GPIO through its driver's PWM input.  "On" writes the configured duty cycle (team
decision 2026-10-07: 100 %), "off" writes 0.  On the Pi 5 Blinka's ``pwmio.PWMOut`` is lgpio software PWM;
at 100 % the line is simply held high, so the frequency does not matter.  Imports are lazy so pure tests never
need Blinka.
"""
from __future__ import annotations

from typing import Optional, Protocol


class MotorError(Exception):
    pass


class MotorPort(Protocol):
    def set(self, on: bool) -> None: ...
    def close(self) -> None: ...


class BlinkaPwmMotor:
    """``pin`` is a Blinka board name ("D12" = BCM GPIO12).  Raises MotorError if the pin cannot be claimed
    (wrong name, already in use by another process, no permission)."""

    def __init__(self, pin: str, frequency_hz: float, duty_u16: int) -> None:
        self.pin = pin
        self._duty = duty_u16
        self._pwm = None
        try:
            import board  # noqa: WPS433
            import pwmio  # noqa: WPS433
            self._board_pin = getattr(board, pin)
            self._pwm = pwmio.PWMOut(self._board_pin, frequency=frequency_hz, duty_cycle=0)
        except Exception as e:  # noqa: BLE001  (lgpio raises its own error type)
            raise MotorError(f"{pin}: {e}") from e

    def set(self, on: bool) -> None:
        try:
            self._pwm.duty_cycle = self._duty if on else 0
        except Exception as e:  # noqa: BLE001
            raise MotorError(f"{self.pin}: {e}") from e

    def close(self) -> None:
        """Off, then release the line.  Blinka's own deinit leaves lgpio's PWM running, hence the explicit 0."""
        if self._pwm is None:
            return
        try:
            self._pwm.duty_cycle = 0
        except Exception:  # noqa: BLE001
            pass
        try:
            self._pwm.deinit()
            import lgpio  # noqa: WPS433
            from adafruit_blinka.microcontroller.generic_linux.lgpio_pin import CHIP  # noqa: WPS433
            lgpio.gpio_free(CHIP, self._board_pin.id)
        except Exception:  # noqa: BLE001
            pass
        self._pwm = None


class NullMotor:
    """No hardware: remembers what it was told.  For ``run_pipeline.py --no-haptics``, the simulator and tests."""

    def __init__(self, name: str = "") -> None:
        self.name = name
        self.on: Optional[bool] = None
        self.n_writes = 0
        self.closed = False

    def set(self, on: bool) -> None:
        self.on = on
        self.n_writes += 1

    def close(self) -> None:
        self.on = False
        self.closed = True

"""The only module in the package that may import ``time``.

All time inside goldenfleece flows through a ``Clock`` object handed in by the
composition root.  Pure layers never receive a clock at all: they receive
timestamps as arguments.  Adapters receive a clock and stamp what they read.
"""
from __future__ import annotations

import time
from typing import Protocol


class Clock(Protocol):
    def now(self) -> float:
        """Seconds on a monotonic timeline."""

    def sleep(self, dt: float) -> None:
        """Block the calling thread for ``dt`` seconds (adapters only)."""


class MonotonicClock:
    """CLOCK_MONOTONIC.  Immune to wall-clock steps; shared by every thread."""

    def now(self) -> float:
        return time.clock_gettime(time.CLOCK_MONOTONIC)

    def sleep(self, dt: float) -> None:
        if dt > 0:
            time.sleep(dt)


class FakeClock:
    """Deterministic clock for tests, the simulator and replay."""

    def __init__(self, t0: float = 0.0) -> None:
        self._t = float(t0)

    def now(self) -> float:
        return self._t

    def sleep(self, dt: float) -> None:
        self.advance(dt)

    def advance(self, dt: float) -> None:
        if dt < 0:
            raise ValueError("FakeClock cannot run backwards")
        self._t += float(dt)

    def set(self, t: float) -> None:
        if t < self._t:
            raise ValueError("FakeClock cannot run backwards")
        self._t = float(t)


def wall_clock_iso() -> str:
    """Wall-clock string for recording headers only (never for computation)."""
    return time.strftime("%Y-%m-%dT%H:%M:%S%z")

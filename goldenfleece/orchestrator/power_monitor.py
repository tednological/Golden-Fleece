"""Power monitor adapter: polls the Pi firmware's undervoltage / throttle flags at a low rate on
its own thread and publishes them.  No math; bytes/flags only.

vcgencmd get_throttled bit meanings (Raspberry Pi firmware):
  0x1 under-voltage now, 0x2 arm freq capped now, 0x4 throttled now, 0x8 soft temp limit now,
  0x10000 under-voltage occurred, 0x20000 freq capped occurred, 0x40000 throttled occurred, 0x80000 soft temp limit occurred
"""
from __future__ import annotations

import subprocess
import threading
from typing import Callable, List, Optional, Tuple

from ..clock import Clock

UNDERVOLTAGE_NOW = 0x1
THROTTLED_NOW = 0x4
FREQ_CAPPED_NOW = 0x2


def parse_throttled(text: str) -> Optional[int]:
    text = text.strip()
    if "=" not in text:
        return None
    try:
        return int(text.split("=", 1)[1], 16)
    except ValueError:
        return None


class PowerMonitor:
    def __init__(self, clock: Clock, period_s: float = 1.0, reader: Optional[Callable[[], str]] = None) -> None:
        self.clock = clock
        self.period_s = period_s
        self._reader = reader or self._vcgencmd
        self._lock = threading.Lock()
        self._events: List[Tuple[float, int]] = []
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="gf-power", daemon=True)
        self.last_flags: Optional[int] = None
        self.errors = 0

    @staticmethod
    def _vcgencmd() -> str:
        return subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=2.0).stdout

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=3.0)

    def sample_once(self) -> None:
        try:
            flags = parse_throttled(self._reader())
        except Exception:  # noqa: BLE001
            flags = None
        t = self.clock.now()
        if flags is None:
            self.errors += 1
            return
        with self._lock:
            self.last_flags = flags
            self._events.append((t, flags))

    def _run(self) -> None:
        while not self._stop.is_set():
            self.sample_once()
            self.clock.sleep(self.period_s)

    def drain(self) -> List[Tuple[float, int]]:
        with self._lock:
            ev, self._events = self._events, []
        return ev

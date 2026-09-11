"""ImuSource interface and an in-memory source for sim/replay/tests."""
from __future__ import annotations

import threading
from typing import List, Protocol

from ..types import HealthEvent, RawImuSample


class ImuSource(Protocol):
    def start(self) -> None: ...
    def stop(self) -> None: ...
    def drain(self) -> List[RawImuSample]: ...
    def drain_events(self) -> List[HealthEvent]: ...


class QueueImuSource:
    def __init__(self, max_buffer: int = 4096) -> None:
        self._buf: List[RawImuSample] = []
        self._events: List[HealthEvent] = []
        self._lock = threading.Lock()
        self.max_buffer = max_buffer
        self.dropped = 0

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def push(self, s: RawImuSample) -> None:
        with self._lock:
            if len(self._buf) >= self.max_buffer:
                self._buf.pop(0)
                self.dropped += 1
            self._buf.append(s)

    def push_event(self, ev: HealthEvent) -> None:
        with self._lock:
            self._events.append(ev)

    def drain(self) -> List[RawImuSample]:
        with self._lock:
            b, self._buf = self._buf, []
        return b

    def drain_events(self) -> List[HealthEvent]:
        with self._lock:
            ev, self._events = self._events, []
        return ev

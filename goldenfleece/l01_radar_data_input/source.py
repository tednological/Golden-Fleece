"""RadarFrameSource interface (topology-independent) and an in-memory source for sim/replay/tests."""
from __future__ import annotations

import queue
import threading
from typing import List, Optional, Protocol, Union

from ..types import HealthEvent, RadarConfigChanged, RawRadarFrame

SourceEvent = Union[HealthEvent, RadarConfigChanged]


class RadarFrameSource(Protocol):
    def start(self) -> None: ...
    def stop(self) -> None: ...
    def get(self, timeout_s: float) -> Optional[RawRadarFrame]: ...
    def drain_events(self) -> List[SourceEvent]: ...


class QueueRadarSource:
    """Frames and events pushed by a producer (simulator, replay, tests).  Bounded, newest wins."""

    def __init__(self, depth: int = 4) -> None:
        self._q: "queue.Queue[RawRadarFrame]" = queue.Queue(maxsize=depth)
        self._events: List[SourceEvent] = []
        self._lock = threading.Lock()
        self.dropped = 0

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def push(self, frame: RawRadarFrame) -> None:
        while True:
            try:
                self._q.put_nowait(frame)
                return
            except queue.Full:
                try:
                    self._q.get_nowait()
                    self.dropped += 1
                except queue.Empty:
                    pass

    def push_event(self, ev: SourceEvent) -> None:
        with self._lock:
            self._events.append(ev)

    def get(self, timeout_s: float) -> Optional[RawRadarFrame]:
        try:
            return self._q.get(timeout=timeout_s if timeout_s > 0 else None) if timeout_s > 0 else self._q.get_nowait()
        except queue.Empty:
            return None

    def drain_events(self) -> List[SourceEvent]:
        with self._lock:
            ev, self._events = self._events, []
        return ev

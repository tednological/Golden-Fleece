"""Health aggregation: reason bits -> state, with onset bookkeeping.

Health is a channel separate from threat.  Every transition produces a
HealthEvent that is recorded and forwarded to the MCU.  Pure: receives
timestamps as arguments.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

from .types import DEGRADED_BITS, OFFLINE_BITS, HealthBits, HealthEvent, HealthState


def state_from_bits(bits: HealthBits) -> HealthState:
    if bits & OFFLINE_BITS:
        return HealthState.OFFLINE
    if bits & DEGRADED_BITS:
        return HealthState.DEGRADED
    return HealthState.OK


@dataclass
class HealthSnapshot:
    t: float
    state: HealthState
    bits: HealthBits
    onset: Dict[HealthBits, float] = field(default_factory=dict)


class HealthTracker:
    """Owns the current bit set; emits events on change; remembers onsets."""

    def __init__(self) -> None:
        self._bits = HealthBits.NONE
        self._onset: Dict[HealthBits, float] = {}
        self._events: List[HealthEvent] = []
        self.transitions = 0

    @property
    def bits(self) -> HealthBits:
        return self._bits

    @property
    def state(self) -> HealthState:
        return state_from_bits(self._bits)

    def set(self, bit: HealthBits, active: bool, t: float, source: str, detail: str = "") -> Optional[HealthEvent]:
        cur = bool(self._bits & bit)
        if cur == active:
            return None
        if active:
            self._bits |= bit
            self._onset[bit] = t
        else:
            self._bits &= ~bit
            self._onset.pop(bit, None)
        ev = HealthEvent(t=t, bit=bit, active=active, source=source, detail=detail)
        self._events.append(ev)
        self.transitions += 1
        return ev

    def drain_events(self) -> List[HealthEvent]:
        ev, self._events = self._events, []
        return ev

    def snapshot(self, t: float) -> HealthSnapshot:
        return HealthSnapshot(t=t, state=self.state, bits=self._bits, onset=dict(self._onset))

    def active_since(self, bit: HealthBits) -> Optional[float]:
        return self._onset.get(bit)


def bits_to_names(bits: HealthBits) -> List[str]:
    return [b.name for b in HealthBits if b != HealthBits.NONE and bits & b]

"""Per-stage and end-to-end latency statistics (p50 / p99 / max).  Pure containers."""
from __future__ import annotations

from collections import deque
from typing import Deque, Dict, Iterable, List


def percentile(xs: Iterable[float], p: float) -> float:
    s = sorted(xs)
    if not s:
        return 0.0
    return s[min(len(s) - 1, int(p * len(s)))]


class Reservoir:
    def __init__(self, maxlen: int = 4000) -> None:
        self._d: Deque[float] = deque(maxlen=maxlen)
        self.n = 0
        self.max = 0.0

    def add(self, x: float) -> None:
        self._d.append(x)
        self.n += 1
        if x > self.max:
            self.max = x

    def summary(self) -> Dict[str, float]:
        return {"p50": percentile(self._d, 0.5), "p99": percentile(self._d, 0.99), "max": self.max, "n": float(self.n)}


class LatencyStats:
    def __init__(self) -> None:
        self.stages: Dict[str, Reservoir] = {}
        self.e2e_pi = Reservoir()          # radar t_mid -> bytes handed to the transport
        self.loop = Reservoir()            # process_frame wall time
        self.header_to_decision = Reservoir()

    def add_stage(self, name: str, dt: float) -> None:
        self.stages.setdefault(name, Reservoir()).add(dt)

    def report(self) -> Dict[str, Dict[str, float]]:
        out = {k: v.summary() for k, v in self.stages.items()}
        out["e2e_pi"] = self.e2e_pi.summary()
        out["loop"] = self.loop.summary()
        out["header_to_decision"] = self.header_to_decision.summary()
        return out

    def log_line(self) -> str:
        parts: List[str] = []
        for k, v in self.report().items():
            parts.append(f"{k}: p50={v['p50']*1e3:.2f} p99={v['p99']*1e3:.2f} max={v['max']*1e3:.2f} ms")
        return " | ".join(parts)

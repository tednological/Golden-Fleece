"""Outage report: DEGRADED / OFFLINE episodes per recorded session (task §10.5).

    .venv/bin/python tools/outage_report.py recordings/*.jsonl
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from goldenfleece.health import state_from_bits                # noqa: E402
from goldenfleece.orchestrator.recording import read_recording  # noqa: E402
from goldenfleece.types import HealthBits, HealthState          # noqa: E402


@dataclass
class Episode:
    state: HealthState
    t_start: float
    t_end: Optional[float]
    causes: List[str] = field(default_factory=list)

    @property
    def duration(self) -> float:
        return (self.t_end if self.t_end is not None else self.t_start) - self.t_start


@dataclass
class SessionReport:
    path: str
    t0: float
    t_end: float
    riding_time_s: float
    episodes: List[Episode]
    drops: int
    frames: int

    def count(self, state: HealthState) -> int:
        return sum(1 for e in self.episodes if e.state is state)

    def longest(self) -> Optional[Episode]:
        return max(self.episodes, key=lambda e: e.duration) if self.episodes else None

    def total_time(self, state: HealthState) -> float:
        return sum(e.duration for e in self.episodes if e.state is state)


def analyse(path: Path | str) -> SessionReport:
    bits = HealthBits.NONE
    state = HealthState.OK
    t0: Optional[float] = None
    t_last = 0.0
    drops = 0
    frames = 0
    episodes: List[Episode] = []
    cur: Optional[Episode] = None
    for r in read_recording(path):
        k = r.get("k")
        t = float(r.get("t", t_last))
        if k in ("rf", "imu", "hev", "cmd", "pwr"):
            t_last = max(t_last, t)
        if k == "hdr":
            t0 = float(r.get("t0", 0.0))
        elif k == "rf":
            frames += 1
        elif k == "end":
            drops = int(r.get("drops", 0))
        elif k == "hev":
            bit = HealthBits[r["bit"]]
            if r["on"]:
                bits |= bit
            else:
                bits &= ~bit
            new_state = state_from_bits(bits)
            if cur is not None and r["on"] and new_state is cur.state and r["bit"] not in cur.causes:
                cur.causes.append(r["bit"])
            if new_state is not state:
                if cur is not None:
                    cur.t_end = t
                    cur = None
                if new_state is not HealthState.OK:
                    cur = Episode(new_state, t, None, [b.name for b in HealthBits if b != HealthBits.NONE and bits & b])
                    episodes.append(cur)
                state = new_state
    if cur is not None:
        cur.t_end = t_last
    if t0 is None:
        t0 = episodes[0].t_start if episodes else 0.0
    return SessionReport(str(path), t0, t_last, max(t_last - t0, 0.0), episodes, drops, frames)


def format_report(rep: SessionReport) -> str:
    lines = [f"session {rep.path}", f"  riding time     : {rep.riding_time_s:8.1f} s   frames: {rep.frames}   recorder drops: {rep.drops}",
             f"  DEGRADED        : {rep.count(HealthState.DEGRADED)} episodes, {rep.total_time(HealthState.DEGRADED):.1f} s total",
             f"  OFFLINE         : {rep.count(HealthState.OFFLINE)} episodes, {rep.total_time(HealthState.OFFLINE):.1f} s total"]
    lg = rep.longest()
    if lg:
        lines.append(f"  longest episode : {lg.state.name} {lg.duration:.1f} s at t={lg.t_start - rep.t0:.1f} s ({', '.join(lg.causes)})")
    for e in rep.episodes:
        lines.append(f"    {e.state.name:9s} t={e.t_start - rep.t0:8.1f}s dur={e.duration:7.2f}s causes={','.join(e.causes) or '-'}")
    return "\n".join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("recordings", nargs="+")
    a = ap.parse_args(argv)
    for p in a.recordings:
        print(format_report(analyse(p)))
    return 0


if __name__ == "__main__":
    sys.exit(main())

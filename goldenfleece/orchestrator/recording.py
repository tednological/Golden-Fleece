"""Recording format, writer thread and reader.

Every raw adapter output and every health transition is logged with Clock
timestamps so any session replays through l03-l09 bit-identically.  The writer
runs on its own thread with a bounded queue; drops are counted, never blocking.
Format: JSON lines (stdlib only).  First record is the session header.

Record kinds:
  hdr   {v, t_wall, t0, config_summary, firmware, git, note}
  rf    raw radar frame  {t, fn, gap, rspi, rrai, cap, seq, tg: [[wire fields of RawRadarTarget in declaration order], ...]}
  imu   raw IMU sample   {t, k2: g|a|q, v: [...], seq}
  hev   health event     {t, bit, on, src, det}
  cmd   decision         {t, seq, lvl, side, bkt, hs, bits, alert}
  rcfg  radar config     {t, rrai, rspi, baud, fw, ...}
  pwr   power flags      {t, flags}
  hap   haptics         {t, r, lvl, side, alert, hs, mode, txt, cause?, lat?, rf?}  every change in what the motors
        render (r: Render; cause: FALLBACK entry/exit reason; lat: decision -> motor write, s), and the current state
        again every recording.haptics_refresh_s without a change (rf: 1)
  lat   latency summary  {t, stage_p50, stage_p99, e2e_p50, e2e_p99, e2e_max}
  end   {t, drops, records}
"""
from __future__ import annotations

import dataclasses
import json
import queue
import threading
from pathlib import Path
from typing import Any, Dict, Iterator, Optional

from ..clock import Clock, wall_clock_iso
from ..l10_haptics.output import HapticChange
from ..types import HealthBits, HealthEvent, ImuKind, RadarConfigChanged, RawImuSample, RawRadarFrame, RawRadarTarget, WarningCommand

FORMAT_VERSION = 1


# --- encoders (pure) ----------------------------------------------------------------------------
def rec_header(t0: float, config_summary: Dict[str, Any], firmware: str = "", git: str = "", note: str = "") -> Dict[str, Any]:
    return {"k": "hdr", "v": FORMAT_VERSION, "t_wall": wall_clock_iso(), "t0": t0, "config": config_summary, "fw": firmware, "git": git, "note": note}


def rec_radar(f: RawRadarFrame) -> Dict[str, Any]:
    return {"k": "rf", "t": f.t_header, "fn": f.frame_number, "gap": f.gap, "rspi": f.rspi, "rrai": f.rrai, "cap": f.cap_hit,
            "seq": f.source_seq, "tg": [list(dataclasses.astuple(t)) for t in f.targets]}


def rec_imu(s: RawImuSample) -> Dict[str, Any]:
    return {"k": "imu", "t": s.t, "k2": s.kind.value, "v": list(s.values), "seq": s.seq}


def rec_health(e: HealthEvent) -> Dict[str, Any]:
    return {"k": "hev", "t": e.t, "bit": e.bit.name, "on": e.active, "src": e.source, "det": e.detail}


def rec_cmd(c: WarningCommand) -> Dict[str, Any]:
    return {"k": "cmd", "t": c.t_decided, "seq": c.seq, "lvl": int(c.level), "side": c.side.value, "bkt": int(c.t_arrival_bucket),
            "hs": c.health_state.name, "bits": int(c.health_bits), "alert": c.assert_alert}


def rec_rcfg(r: RadarConfigChanged) -> Dict[str, Any]:
    return {"k": "rcfg", "t": r.t, "rrai": r.rrai, "rspi": r.rspi, "baud": r.baud, "thof": r.thof, "dedi": r.dedi, "misp": r.misp,
            "masp": r.masp, "fw": r.firmware_version, "T": r.frame_duration_s}


def rec_haptic(c: HapticChange, refresh: bool = False) -> Dict[str, Any]:
    s = c.state
    r = {"k": "hap", "t": c.t, "r": s.render.value, "lvl": int(s.level), "side": s.side.value, "alert": s.alert,
         "hs": s.health.name, "mode": s.mode.value, "txt": c.text}
    if c.cause:
        r["cause"] = c.cause
    if c.latency_s is not None:
        r["lat"] = round(c.latency_s, 6)
    if refresh:
        r["rf"] = 1
    return r


def rec_power(t: float, flags: int) -> Dict[str, Any]:
    return {"k": "pwr", "t": t, "flags": flags}


# --- decoders (pure) ----------------------------------------------------------------------------------
def radar_from_rec(r: Dict[str, Any]) -> RawRadarFrame:
    return RawRadarFrame(t_header=float(r["t"]), frame_number=int(r["fn"]), gap=int(r["gap"]), rspi=int(r["rspi"]), rrai=int(r["rrai"]),
                         targets=tuple(RawRadarTarget(*[int(x) for x in tg]) for tg in r["tg"]), cap_hit=bool(r["cap"]), source_seq=int(r["seq"]))


# Bits that recordings name but this build does not.  MCU_LINK_DOWN (0x200, before the MCU was removed) was
# informational: it never changed the health state, so reading it as NONE keeps old sessions' reports unchanged.
RETIRED_BITS = frozenset({"MCU_LINK_DOWN"})


def health_bit_from_rec(name: str) -> HealthBits:
    return HealthBits.NONE if name in RETIRED_BITS else HealthBits[name]


def imu_from_rec(r: Dict[str, Any]) -> RawImuSample:
    return RawImuSample(t=float(r["t"]), kind=ImuKind(r["k2"]), values=tuple(float(v) for v in r["v"]), seq=int(r["seq"]))


def read_recording(path: Path | str) -> Iterator[Dict[str, Any]]:
    p = Path(path)
    opener = open
    if p.suffix == ".gz":
        import gzip
        opener = gzip.open
    with opener(p, "rt", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                yield json.loads(line)


# --- writer thread (adapter) ---------------------------------------------------------------------------------
class RecordingWriter:
    """Bounded-queue JSONL writer.  ``put`` never blocks; drops are counted."""

    def __init__(self, path: Path | str, clock: Clock, queue_depth: int = 4096, flush_period_s: float = 1.0) -> None:
        self.path = Path(path)
        self.clock = clock
        self._q: "queue.Queue[Optional[Dict[str, Any]]]" = queue.Queue(maxsize=queue_depth)
        self.flush_period_s = flush_period_s
        self.drops = 0
        self.written = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="gf-recorder", daemon=True)
        self._started = False

    def start(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._thread.start()
        self._started = True

    def put(self, rec: Dict[str, Any]) -> bool:
        try:
            self._q.put_nowait(rec)
            return True
        except queue.Full:
            self.drops += 1
            return False

    def close(self, timeout_s: float = 5.0) -> None:
        if not self._started:
            return
        try:
            self._q.put({"k": "end", "t": self.clock.now(), "drops": self.drops, "records": self.written + 1}, timeout=timeout_s)
        except queue.Full:
            pass
        self._stop.set()
        self._q.put(None)
        self._thread.join(timeout=timeout_s)

    def _run(self) -> None:
        with open(self.path, "a", encoding="utf-8", buffering=1 << 16) as f:
            last_flush = self.clock.now()
            while True:
                try:
                    rec = self._q.get(timeout=self.flush_period_s)
                except queue.Empty:
                    rec = "tick"
                if rec is None:
                    f.flush()
                    return
                if rec != "tick":
                    f.write(json.dumps(rec, separators=(",", ":")))
                    f.write("\n")
                    self.written += 1
                now = self.clock.now()
                if now - last_flush >= self.flush_period_s:
                    f.flush()
                    last_flush = now

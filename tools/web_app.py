"""Golden Fleece field web app: live radar targets, tracks, IMU and warnings, for a phone on the bars.

    .venv/bin/python tools/web_app.py                      # follow the live pipeline's recording, serve :8080
    .venv/bin/python tools/web_app.py --recording FILE     # review a recorded session at real-time pace
    then open http://raspberrypi.local:8080/  (or http://<the Pi's address>:8080/)

It is a separate process that never talks to the pipeline.  It follows the newest session file in
``recording.dir`` and feeds its raw radar frames and IMU samples through its own copy of the pure pipeline
(l03..l09).  Replay is bit-identical (tools/replay.py), so the detections and tracks drawn are the ones the
live pipeline computed, and a slow or crashed web app cannot touch the warnings.  The warning shown is the
pipeline's own recorded decision (``cmd`` records), not a recomputation.  Radar coordinates come from l03,
the single conversion site, so what is drawn on the rider's left is what the pipeline believes is there.

The camera card shows what the camera recorder (tools/camera_recorder.py, another separate process) reports in its
status file, and its button writes the recorder's control file: ``POST /camera {"recording": true|false}``.

The IMU card's "mark aligned" button stores the current roll, pitch and yaw as the aligned attitude and the card then
shows how far the IMU has turned from it: ``POST /imu/align {"aligned": true|false}`` (false forgets it).  It is kept
in ``<recording.dir>/imu_aligned.json`` for the page only; the pipeline never reads it.  Roll and pitch are measured
against gravity, so they survive a restart; yaw has no absolute reference (l04), so the aligned yaw only holds until
this app starts over on a new session or restarts, after which the card asks for a fresh mark.

The haptics card shows what the vibration motors render, from the pipeline's ``hap`` records (one per change, plus a
refresh every few seconds; the pipeline owns the motor pins, this app never touches them): the pattern, the render
thread's watchdog mode (FALLBACK = the pipeline loop stalled), HAPTICS_FAULT, the decision -> motor latency, and a log
of recent changes.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from goldenfleece import frames as fr                                        # noqa: E402
from goldenfleece.config import load_config                                  # noqa: E402
from goldenfleece.orchestrator.pipeline import Pipeline                      # noqa: E402
from goldenfleece.orchestrator.recording import imu_from_rec, radar_from_rec  # noqa: E402
from goldenfleece.l10_haptics.patterns import haptics_config                # noqa: E402
from goldenfleece.types import HealthBits, ImuKind                           # noqa: E402
from tools.camera_recorder import (CameraConfig, CameraConfigError, load_camera_config,   # noqa: E402
                                   read_status, set_recording_wanted)

DEG = 180.0 / math.pi
IMU_HIST_S = 8.0                 # chart window
IMU_HIST_DT = 0.05               # chart resolution
RATE_WINDOW_S = 2.0
UPDATE_S = 0.125                 # page refresh period (8 Hz)
HAP_LOG_N = 40                   # haptics card: recent changes kept for the page
TAIL_BYTES = 2_000_000           # joining a live session: start this far from its end (tracks re-form in seconds)


def _f(v: Any, n: int = 2) -> Optional[float]:
    """Round for JSON; None for anything not a finite number (JSON has no NaN/inf)."""
    try:
        v = float(v)
    except (TypeError, ValueError):
        return None
    return round(v, n) if math.isfinite(v) else None


def _wrap_deg(d: float) -> float:
    return (d + 180.0) % 360.0 - 180.0


def _bit_names(bits: int) -> List[str]:
    return [b.name for b in HealthBits if b.value and b.value & bits]


def _rate(times: Deque[float]) -> Optional[float]:
    if len(times) < 2 or times[-1] <= times[0]:
        return None
    return (len(times) - 1) / (times[-1] - times[0])


class LiveState:
    """Everything the page shows, built from recording records.  One writer (the follower), many readers."""

    def __init__(self, cfg) -> None:
        self.cfg = cfg
        self.lock = threading.Lock()
        self.gen = 0                     # bumped by every reset: the ESKF's yaw reference starts over each time
        self.aligned: Optional[Dict[str, Any]] = None
        self.align_path: Optional[Path] = None
        h = haptics_config(cfg.pipeline.haptics)
        self.motors = {"motors": [{"name": m.name, "pin": m.pin, "side": m.side.name} for m in h.motors],
                       "duty_pct": _f(h.duty_cycle * 100, 0)}
        self.reset(None)

    # ---- alignment ----------------------------------------------------------------------------------------
    def load_alignment(self, path: Optional[Path]) -> None:
        """Adopt the stored aligned attitude.  Its yaw belongs to an earlier process's reference, so it is dropped."""
        self.align_path = Path(path) if path else None
        try:
            a = json.loads(self.align_path.read_text()) if self.align_path else None
            if not (isinstance(a, dict) and all(isinstance(a.get(k), (int, float)) for k in ("roll", "pitch"))):
                a = None
        except (OSError, ValueError):
            a = None
        with self.lock:
            self.aligned = None if a is None else {"roll": float(a["roll"]), "pitch": float(a["pitch"]), "yaw": None,
                                                   "gen": None, "t_wall": a.get("t_wall"), "src": a.get("src", "")}

    def set_aligned(self, aligned: bool, src: str) -> Optional[Dict[str, Any]]:
        """Mark the current attitude as aligned (or forget it).  Raises LookupError before l04 has an attitude,
        OSError if the file cannot be written; the in-memory state only changes once the file has."""
        with self.lock:
            if not aligned:
                new = None
            else:
                m = self.imu_state
                if m.get("roll") is None or m.get("pitch") is None or m.get("health") == "NO_DATA":
                    raise LookupError("no IMU attitude yet")
                new = {"roll": m["roll"], "pitch": m["pitch"], "yaw": m.get("yaw"), "gen": self.gen,
                       "t_wall": round(time.time(), 3), "src": src}
            if self.align_path is not None:
                if new is None:
                    self.align_path.unlink(missing_ok=True)
                else:
                    self.align_path.parent.mkdir(parents=True, exist_ok=True)
                    tmp = self.align_path.with_suffix(".tmp")
                    tmp.write_text(json.dumps({k: v for k, v in new.items() if k != "gen"}) + "\n")
                    tmp.replace(self.align_path)
            self.aligned = new
            return self._aligned_view()

    def _aligned_view(self) -> Optional[Dict[str, Any]]:
        """For the page: the aligned attitude and the current one relative to it (degrees, current minus aligned)."""
        a, m = self.aligned, self.imu_state
        if a is None:
            return None
        yaw_ok = a["yaw"] is not None and a["gen"] == self.gen
        out = {"roll": a["roll"], "pitch": a["pitch"], "yaw": a["yaw"] if yaw_ok else None, "yaw_valid": yaw_ok,
               "t_wall": a["t_wall"], "src": a["src"]}
        for k in ("roll", "pitch", "yaw"):
            ref, cur = out[k], m.get(k)
            out["d_" + k] = _f(_wrap_deg(cur - ref), 1) if ref is not None and cur is not None else None
        return out

    def reset(self, session: Optional[str]) -> None:
        with self.lock:
            self.session = session
            self.gen += 1
            self.pipe = Pipeline(self.cfg)
            self.pipe.set_health(HealthBits.PIPELINE_RESTARTING, True, 0.0, "web", "join")
            self.header: Dict[str, Any] = {}
            self.frame: Dict[str, Any] = {}
            self.targets: List[Dict[str, Any]] = []
            self.tracks: List[Dict[str, Any]] = []
            self.ego: Dict[str, Any] = {}
            self.imu_state: Dict[str, Any] = {}
            self.decision: Dict[str, Any] = {}
            self.health: Dict[str, Dict[str, Any]] = {}
            self.power_flags = 0
            self.last_gyro: Optional[Tuple[float, ...]] = None
            self.last_accel: Optional[Tuple[float, ...]] = None
            self.imu_hist: Deque[Tuple[float, ...]] = deque()
            self.acc_window: Deque[Tuple[float, ...]] = deque()
            self.rf_times: Deque[float] = deque()
            self.g_times: Deque[float] = deque()
            self.a_times: Deque[float] = deque()
            self.hap_now: Optional[Dict[str, Any]] = None
            self.hap_log: Deque[Dict[str, Any]] = deque(maxlen=HAP_LOG_N)
            self.hap_n = {"changes": 0, "fallbacks": 0}
            self.hap_lat_s: Deque[float] = deque(maxlen=200)
            self.last_t: Optional[float] = None
            self.ended = False
            self.n_records = 0
            self.n_errors = 0
            self.last_error = ""
            self._hist_next = -math.inf

    # ---- records ------------------------------------------------------------------------------------------
    def feed(self, rec: Dict[str, Any]) -> None:
        k = rec.get("k")
        with self.lock:
            self.n_records += 1
            if isinstance(rec.get("t"), (int, float)):     # max: haptic changes are drained a loop iteration late
                self.last_t = float(rec["t"]) if self.last_t is None else max(self.last_t, float(rec["t"]))
            try:
                if k == "imu":
                    self._imu(rec)
                elif k == "rf":
                    self._radar(rec)
                elif k == "cmd":
                    bits = int(rec.get("bits", 0))
                    self.decision = {"t": rec["t"], "lvl": int(rec["lvl"]), "side": rec["side"], "bkt": int(rec["bkt"]),
                                     "hs": rec["hs"], "alert": bool(rec.get("alert")),
                                     "bits": _bit_names(bits)}
                elif k == "hev":
                    if rec["on"]:
                        self.health[rec["bit"]] = {"since": rec["t"], "det": rec.get("det", ""), "src": rec.get("src", "")}
                    else:
                        self.health.pop(rec["bit"], None)
                elif k == "hap":
                    self._hap(rec)
                elif k == "pwr":
                    self.power_flags = int(rec.get("flags", 0))
                elif k == "hdr":
                    self.header = rec
                elif k == "end":
                    self.ended = True
            except Exception as e:  # noqa: BLE001
                self.n_errors += 1
                self.last_error = f"{k}: {e!r}"

    def _window(self, times: Deque[float], t: float) -> None:
        times.append(t)
        while times and times[0] < t - RATE_WINDOW_S:
            times.popleft()

    def _hap(self, rec: Dict[str, Any]) -> None:
        lat = rec.get("lat")
        h = {"t": float(rec["t"]), "r": str(rec["r"]), "lvl": int(rec.get("lvl", 0)), "side": str(rec.get("side", "NONE")),
             "alert": bool(rec.get("alert")), "hs": str(rec.get("hs", "")), "mode": str(rec.get("mode", "")),
             "txt": str(rec.get("txt", "")), "cause": str(rec.get("cause", "")),
             "lat_ms": _f(float(lat) * 1000, 1) if isinstance(lat, (int, float)) else None}
        self.hap_now = h
        if rec.get("rf"):                                # a refresh: the same state again, not a change
            return
        self.hap_log.append(h)
        self.hap_n["changes"] += 1
        if h["mode"] == "FALLBACK" and h["cause"]:
            self.hap_n["fallbacks"] += 1
        if isinstance(lat, (int, float)):
            self.hap_lat_s.append(float(lat))

    def _hap_view(self) -> Dict[str, Any]:
        last = self.last_t
        ago = lambda t: _f(last - t, 2) if (t is not None and last is not None) else None   # noqa: E731
        lats = sorted(self.hap_lat_s)
        fault = self.health.get("HAPTICS_FAULT")
        return {
            **self.motors, "n": dict(self.hap_n), "fault": (fault["det"] or "fault") if fault else None,
            "now": {**{k: v for k, v in self.hap_now.items() if k != "t"}, "age_s": ago(self.hap_now["t"])} if self.hap_now else None,
            "lat_ms": {"n": len(lats), "last": _f(self.hap_lat_s[-1] * 1000, 1), "p50": _f(lats[len(lats) // 2] * 1000, 1),
                       "max": _f(lats[-1] * 1000, 1)} if lats else None,
            "log": [[ago(h["t"]), h["txt"], h["mode"], h["cause"], h["lat_ms"]] for h in self.hap_log],
        }

    def _imu(self, rec: Dict[str, Any]) -> None:
        s = imu_from_rec(rec)
        self.pipe.ingest_imu(s)
        if s.kind is ImuKind.GYRO:
            self.last_gyro = s.values
            self._window(self.g_times, s.t)
        elif s.kind is ImuKind.ACCEL:
            self.last_accel = s.values
            self._window(self.a_times, s.t)
            self.acc_window.append((s.t,) + tuple(s.values))
            while self.acc_window and self.acc_window[0][0] < s.t - 1.0:
                self.acc_window.popleft()
        if s.t >= self._hist_next and self.last_gyro and self.last_accel:
            self._hist_next = s.t + IMU_HIST_DT
            g, a = self.last_gyro, self.last_accel
            self.imu_hist.append((s.t, g[0] * DEG, g[1] * DEG, g[2] * DEG, math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)))
            while self.imu_hist and self.imu_hist[0][0] < s.t - IMU_HIST_S:
                self.imu_hist.popleft()

    def _radar(self, rec: Dict[str, Any]) -> None:
        raw = radar_from_rec(rec)
        t_now = float(rec.get("tn", raw.t_header + 0.003))
        res = self.pipe.process_frame(raw, t_now)
        self._window(self.rf_times, raw.t_header)
        cls = {c.det.raw_index: c.cls.value for c in res.clutter.kept}
        dets = {d.raw_index: d for d in res.radar.detections}
        targets = []
        for i, tg in enumerate(raw.targets):
            row: Dict[str, Any] = {"distance_cm": tg.distance_cm, "speed_raw": tg.speed_raw, "angle_raw": tg.angle_raw}
            d = dets.get(i)
            if d is None:
                row["cls"] = "clipped"                   # rejected by l03 (outside the azimuth clip, zero range)
            else:
                row.update(r=_f(d.r), az=_f(d.az * DEG, 1), v=_f(d.v_radial), x=_f(d.x), y=_f(d.y), db=_f(d.magnitude_db, 1),
                           side="LEFT" if d.y < -0.25 else "RIGHT" if d.y > 0.25 else "ON AXIS",
                           cls=cls.get(i, "rejected"))
            targets.append(row)
        self.targets = targets
        by_track = {a.track_id: a for a in res.assessments}
        tracks = []
        for tk in res.tracks.tracks:
            if tk.status.value == "DELETED":
                continue
            a = by_track.get(tk.id)
            tracks.append({"id": tk.id, "status": tk.status.value, "coast": tk.coast_reason.value, "x": _f(tk.x), "y": _f(tk.y),
                           "r": _f(tk.r), "vc": _f(tk.v_closing), "az": _f(tk.az * DEG, 1), "hits": tk.hits,
                           "level": int(a.level) if a else 0, "side": a.side.value if a else "",
                           "ta": _f(a.t_arrival, 1) if a else None})
        self.tracks = tracks
        self.frame = {"fn": raw.frame_number, "n_raw": len(raw.targets), "cap": raw.cap_hit, "gap": raw.gap, "rspi": raw.rspi}
        e = res.ego
        self.ego = {"valid": e.valid, "speed": _f(e.speed), "source": e.source.value, "reason": e.invalid_reason.value}
        m = res.imu
        yaw = fr.euler_zyx_for_display(m.q_ref_radar)[0] if m.health.value != "NO_DATA" else math.nan   # arbitrary zero
        self.imu_state = {"health": m.health.value, "roll": _f(m.roll * DEG, 1), "pitch": _f(m.pitch * DEG, 1),
                          "yaw": _f(yaw * DEG, 1), "in_motion": m.in_motion}

    # ---- snapshot -----------------------------------------------------------------------------------------
    def snapshot(self, now_mono: Optional[float], live: bool) -> Dict[str, Any]:
        with self.lock:
            imu: Dict[str, Any] = dict(self.imu_state)
            if self.last_gyro:
                imu["gyro_dps"] = [_f(v * DEG, 1) for v in self.last_gyro]
            if self.last_accel:
                a = self.last_accel
                imu["accel"] = [_f(v) for v in a]
                imu["accel_norm"] = _f(math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2))
            if self.acc_window:
                mean = [sum(r[j] for r in self.acc_window) / len(self.acc_window) for j in (1, 2, 3)]
                j = max(range(3), key=lambda i: abs(mean[i]))
                imu["accel_mean_1s"] = [_f(v) for v in mean]
                imu["up_axis"] = ("+" if mean[j] > 0 else "-") + "XYZ"[j]
            imu["gyro_hz"] = _f(_rate(self.g_times), 1)
            imu["accel_hz"] = _f(_rate(self.a_times), 1)
            imu["aligned"] = self._aligned_view()
            last = self.last_t
            hist = [[_f(p[0] - last, 2)] + [_f(v, 2) for v in p[1:]] for p in self.imu_hist] if last is not None else []
            health = [{"bit": b, "for_s": _f(last - h["since"], 1) if last is not None else None, "det": h["det"]}
                      for b, h in sorted(self.health.items())]
            return {
                "session": self.session, "live": live, "ended": self.ended, "t": _f(last, 2),
                "age_s": _f(now_mono - last, 3) if (live and now_mono is not None and last is not None) else None,
                "started": self.header.get("t_wall"), "fw": self.header.get("fw"), "git": self.header.get("git"),
                "decision": self.decision, "health": health, "power_flags": self.power_flags,
                "frame": self.frame, "radar_hz": _f(_rate(self.rf_times), 1), "targets": self.targets, "tracks": self.tracks,
                "ego": self.ego, "imu": imu, "imu_hist": hist, "haptics": self._hap_view(),
                "records": self.n_records, "errors": self.n_errors, "last_error": self.last_error,
            }


class Follower(threading.Thread):
    """Feeds records to a LiveState: the newest session in a directory (live), or one file at recorded pace."""

    def __init__(self, state: LiveState, rec_dir: Optional[Path] = None, file: Optional[str] = None,
                 speed: float = 1.0, from_start: bool = False) -> None:
        super().__init__(name="gf-web-follower", daemon=True)
        self.state = state
        self.rec_dir = Path(rec_dir) if rec_dir else None
        self.file = file
        self.speed = speed
        self.from_start = from_start
        self.stop_evt = threading.Event()

    def run(self) -> None:
        if self.file:
            self._review(Path(self.file))
        else:
            self._follow()

    def _feed_line(self, line: bytes) -> None:
        line = line.strip()
        if not line:
            return
        try:
            rec = json.loads(line)
        except ValueError:
            self.state.n_errors += 1
            return
        self.state.feed(rec)

    def _newest(self) -> Optional[Path]:
        files = list(self.rec_dir.glob("session_*.jsonl")) if self.rec_dir and self.rec_dir.is_dir() else []
        return max(files, key=lambda p: p.stat().st_mtime) if files else None

    def _follow(self) -> None:
        current: Optional[Path] = None
        f = None
        buf = b""
        next_scan = 0.0
        while not self.stop_evt.is_set():
            if time.monotonic() >= next_scan:
                next_scan = time.monotonic() + 1.0
                newest = self._newest()
                if newest is not None and newest != current:
                    if f is not None:
                        f.close()
                    current, buf = newest, b""
                    self.state.reset(newest.name)
                    f = open(newest, "rb")
                    self._feed_line(f.readline())                    # the session header
                    size = newest.stat().st_size
                    if not self.from_start and size - f.tell() > TAIL_BYTES:
                        f.seek(size - TAIL_BYTES)
                        f.readline()                                  # drop the partial line
            if f is None:
                self.stop_evt.wait(0.2)
                continue
            chunk = f.read(1 << 16)
            if not chunk:
                self.stop_evt.wait(0.05)
                continue
            buf += chunk
            *lines, buf = buf.split(b"\n")                            # keep a partial last line for later
            for line in lines:
                self._feed_line(line)
        if f is not None:
            f.close()

    def _review(self, path: Path) -> None:
        self.state.reset(path.name)
        t0 = wall0 = None
        with open(path, "rb") as f:
            for line in f:
                if self.stop_evt.is_set():
                    return
                if self.speed > 0:
                    try:
                        t = json.loads(line).get("t")
                    except ValueError:
                        t = None
                    if isinstance(t, (int, float)):
                        if t0 is None:
                            t0, wall0 = t, time.monotonic()
                        delay = wall0 + (t - t0) / self.speed - time.monotonic()
                        if delay > 0 and self.stop_evt.wait(delay):
                            return
                self._feed_line(line)
        with self.state.lock:
            self.state.ended = True


class Handler(BaseHTTPRequestHandler):
    server: "WebServer"

    def do_GET(self) -> None:  # noqa: N802
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            self._send(200, "text/html; charset=utf-8", PAGE.encode())
        elif path == "/3d":                                   # prototype 3D radar view, read per request
            self._send(200, "text/html; charset=utf-8", (Path(__file__).parent / "web_scope3d.html").read_bytes())
        elif path == "/state":
            self._send(200, "application/json", self.server.snapshot_bytes())
        elif path == "/events":
            self._events()
        else:
            self._send(404, "text/plain", b"not found")

    def do_POST(self) -> None:  # noqa: N802
        path = self.path.split("?", 1)[0]
        if path == "/camera":
            self._post_camera()
        elif path == "/imu/align":
            self._post_align()
        else:
            self._send(404, "text/plain", b"not found")

    def _body(self, key: str) -> Optional[bool]:
        """The boolean ``key`` of a small JSON body, or None after answering with the error."""
        # A JSON body cannot be sent cross-site without a CORS preflight, which this server never grants, so some
        # other web page open on the phone cannot press these buttons.
        if self.headers.get("Content-Type", "").split(";")[0].strip().lower() != "application/json":
            self._json(415, {"error": "send application/json"})
            return None
        try:
            n = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(n) if 0 < n <= 1024 else b"")
            v = body[key]
            if not isinstance(v, bool):
                raise TypeError
            return v
        except (ValueError, KeyError, TypeError):
            self._json(400, {"error": f'expected {{"{key}": true}} or {{"{key}": false}}'})
            return None

    def _post_align(self) -> None:
        aligned = self._body("aligned")
        if aligned is None:
            return
        try:
            view = self.server.state.set_aligned(aligned, f"the web app ({self.client_address[0]})")
        except LookupError as e:
            self._json(409, {"error": str(e)})
            return
        except OSError as e:
            self._json(500, {"error": f"could not store the alignment: {e}"})
            return
        self.server.invalidate()
        self._json(200, {"ok": True, "aligned": view})

    def _post_camera(self) -> None:
        cam = self.server.camera
        if cam is None:
            self._json(503, {"error": "no camera configured"})
            return
        recording = self._body("recording")
        if recording is None:
            return
        try:
            set_recording_wanted(cam, recording, f"the web app ({self.client_address[0]})")
        except OSError as e:
            self._json(500, {"error": f"could not write {cam.control_path}: {e}"})
            return
        self.server.invalidate()                            # the next event already carries the new wish
        self._json(200, {"ok": True, "camera": read_status(cam)})

    def _json(self, code: int, obj: Dict[str, Any]) -> None:
        self._send(code, "application/json", json.dumps(obj, separators=(",", ":"), allow_nan=False).encode())

    def _send(self, code: int, ctype: str, body: bytes) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _events(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        try:
            while True:
                self.wfile.write(b"data: " + self.server.snapshot_bytes() + b"\n\n")
                self.wfile.flush()
                time.sleep(UPDATE_S)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def log_message(self, *args: Any) -> None:
        pass


class WebServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, addr: Tuple[str, int], state: LiveState, live: bool, camera: Optional[CameraConfig] = None) -> None:
        super().__init__(addr, Handler)
        self.state = state
        self.live = live
        self.camera = camera
        self._cache: Tuple[float, bytes] = (-1.0, b"{}")
        self._cache_lock = threading.Lock()

    def snapshot_bytes(self) -> bytes:
        """One serialisation per UPDATE_S/2 however many phones are watching."""
        with self._cache_lock:
            now = time.monotonic()
            if now - self._cache[0] >= UPDATE_S / 2:
                snap = self.state.snapshot(time.clock_gettime(time.CLOCK_MONOTONIC), self.live)   # the pipeline's clock
                snap["camera"] = read_status(self.camera) if self.camera else None
                self._cache = (now, json.dumps(snap, separators=(",", ":"), allow_nan=False).encode())
            return self._cache[1]

    def invalidate(self) -> None:
        with self._cache_lock:
            self._cache = (-1.0, self._cache[1])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(ROOT / "config"))
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--recording", default=None, help="review this session file instead of following the live one")
    ap.add_argument("--speed", type=float, default=1.0, help="review pace; 0 = as fast as possible")
    ap.add_argument("--from-start", action="store_true", help="when joining a live session, process all of it")
    ap.add_argument("--camera-config", default=str(ROOT / "config" / "camera.yaml"),
                    help="the camera recorder's config (independent of --config, so presets keep the same camera)")
    ap.add_argument("--imu-align-file", default=None,
                    help="where the IMU card's aligned attitude is kept (default: <recording.dir>/imu_aligned.json)")
    a = ap.parse_args(argv)
    cfg = load_config(a.config)
    try:
        camera: Optional[CameraConfig] = load_camera_config(a.camera_config)
    except CameraConfigError as e:
        print(f"camera: {e}; the page shows no camera", flush=True)
        camera = None
    state = LiveState(cfg)
    rec_dir = Path(str(cfg.pipeline.recording.dir))
    state.load_alignment(Path(a.imu_align_file) if a.imu_align_file else rec_dir / "imu_aligned.json")
    follower = Follower(state, rec_dir=rec_dir, file=a.recording, speed=a.speed, from_start=a.from_start)
    follower.start()
    srv = WebServer((a.host, a.port), state, live=a.recording is None, camera=camera)
    what = f"reviewing {a.recording}" if a.recording else f"following the newest session in {rec_dir}"
    print(f"Golden Fleece web app on http://{a.host}:{a.port}/ ({what})", flush=True)
    try:
        srv.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        follower.stop_evt.set()
        srv.server_close()
    return 0


PAGE = r"""<!doctype html>
<html lang="en"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Golden Fleece · field view</title>
<style>
:root{--bg:#0b0f14;--panel:#111a23;--line:#1f2d3a;--text:#d2dde8;--dim:#7489a0;--ok:#39d3a0;--warn:#ff9a3c;--bad:#ff5c5c;
 --mono:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--text);font:14px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
header{display:flex;flex-wrap:wrap;gap:6px 10px;align-items:center;padding:8px 12px;background:var(--panel);
 border-bottom:1px solid var(--line);position:sticky;top:0;z-index:2}
header h1{font-size:15px;margin:0 6px 0 0;font-weight:650}
.chip{font:12px var(--mono);padding:2px 8px;border:1px solid var(--line);border-radius:10px;color:var(--dim);white-space:nowrap}
.chip.ok{color:var(--ok);border-color:var(--ok)} .chip.warn{color:var(--warn);border-color:var(--warn)} .chip.bad{color:var(--bad);border-color:var(--bad)}
#banner{margin:10px 12px 0;border-radius:8px;padding:12px 16px;display:flex;flex-wrap:wrap;align-items:baseline;gap:4px 16px;background:#1a2530}
#lvl,#side{font-size:30px;font-weight:800;letter-spacing:.03em}
#bsub{font:13px var(--mono);color:rgba(255,255,255,.82)}
main{display:grid;grid-template-columns:minmax(0,1.2fr) minmax(0,1fr);gap:12px;padding:12px}
@media(max-width:900px){main{grid-template-columns:minmax(0,1fr)}}
.card{background:var(--panel);border:1px solid var(--line);border-radius:8px;overflow:hidden}
.card h2{margin:0;padding:8px 12px;font-size:11px;letter-spacing:.12em;text-transform:uppercase;color:var(--dim);border-bottom:1px solid var(--line)}
.body{padding:10px 12px}
canvas{display:block;width:100%}
table{width:100%;border-collapse:collapse;font:12px var(--mono);font-variant-numeric:tabular-nums}
th,td{padding:3px 6px;text-align:right;white-space:nowrap} th{color:var(--dim);font-weight:500;border-bottom:1px solid var(--line)}
td.l,th.l{text-align:left} .scroll{overflow-x:auto}
.kv{display:grid;grid-template-columns:auto 1fr;gap:3px 12px;font:13px var(--mono)} .kv span:nth-child(odd){color:var(--dim)}
.muted{color:var(--dim)} .note{font-size:12px;color:var(--dim);padding:6px 12px 10px}
.legend{display:flex;flex-wrap:wrap;gap:4px 12px;font-size:12px;color:var(--dim);padding:6px 12px 10px}
.legend i{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:5px;vertical-align:-1px}
.zoom{display:flex;align-items:center;gap:10px;padding:6px 12px;font:12px var(--mono);color:var(--dim)}
.zoom input{flex:1;accent-color:#6fa8ff} .zoom output{min-width:44px;text-align:right;color:var(--text)}
details summary{cursor:pointer;padding:9px 12px;color:var(--dim);font-size:11px;letter-spacing:.12em;text-transform:uppercase}
details ol{margin:0;padding:0 16px 12px 32px} details li{margin:5px 0}
a.chip{text-decoration:none} .chip.rec{color:#fff;background:#a61b1b;border-color:#ff5c5c}
.col{display:flex;flex-direction:column;gap:12px;min-width:0}
.cambar{display:flex;flex-wrap:wrap;align-items:center;gap:8px 14px;margin-bottom:10px}
.cambtn{font:650 15px system-ui,-apple-system,sans-serif;padding:11px 18px;min-width:200px;border-radius:8px;cursor:pointer;
 border:1px solid #2f6f4f;background:#15392a;color:#b9f5d8}
.cambtn.stop{border-color:#ff5c5c;background:#6d1414;color:#ffe1e1} .cambtn:disabled{opacity:.5;cursor:default}
.alignbar{display:flex;flex-wrap:wrap;align-items:center;gap:8px 14px;margin-bottom:10px}
.alignbtn{font:650 15px system-ui,-apple-system,sans-serif;padding:11px 18px;border-radius:8px;cursor:pointer;
 border:1px solid #2e5a8a;background:#132a42;color:#cfe3ff} .alignbtn.clear{font-size:13px;padding:8px 12px;background:none;color:var(--dim);border-color:var(--line)}
.alignbtn:disabled{opacity:.5;cursor:default}
.ulog{max-height:260px;overflow-y:auto;margin-top:10px;border-top:1px solid var(--line);padding-top:6px;font:11.5px/1.5 var(--mono)}
.ulog div{white-space:pre-wrap;word-break:break-all} .ulog .tx{color:#9fb3c6} .ulog .er{color:var(--bad)}
.ulog b{font-weight:500;color:var(--dim);display:inline-block;min-width:64px}
.ubar{display:flex;flex-wrap:wrap;gap:6px 14px;align-items:center;margin-top:10px;font-size:12px;color:var(--dim)}
.ubar button{font:12px system-ui,sans-serif;padding:4px 10px;border-radius:6px;border:1px solid var(--line);background:none;color:var(--text);cursor:pointer}
#camstate{font:13px var(--mono)} .camerr{color:var(--bad);font-size:13px} .camerr:not(:empty){margin-top:8px}
</style></head><body>
<header><h1>Golden Fleece · field view</h1>
 <span id="conn" class="chip">connecting…</span><span id="hs" class="chip">health –</span><span id="age" class="chip">–</span>
 <span id="sess" class="chip">–</span><a id="hap" class="chip" href="#hapcard">haptics –</a><a id="cam" class="chip" href="#camcard">camera –</a>
 <a href="3d" style="color:#6fa8ff;font-size:13px">3D view (prototype)</a></header>
<div id="banner"><div id="lvl">–</div><div id="side"></div><div id="bsub">waiting for the pipeline…</div></div>
<main>
 <section class="card"><h2>Radar · top view · rider at the top, riding up the screen</h2>
  <div class="zoom"><label for="rm">view range</label><input type="range" id="rm" min="5" max="100" step="5" value="100"><output id="rmv">100 m</output></div>
  <canvas id="scope"></canvas>
  <div class="legend"><span><i style="background:#ff9a3c"></i>approaching</span><span><i style="background:#6fa8ff"></i>receding mover</span>
   <span><i style="background:#7b8a99"></i>stationary</span><span><i style="border:1px solid #7b8a99"></i>rejected</span>
   <span>ring = track, colour = threat level, dashed = coasting</span><span>▲ at the edge = beyond the view range</span></div></section>
 <div class="col">
 <section class="card"><h2>IMU</h2><div class="body">
  <div class="alignbar"><button id="alignbtn" class="alignbtn" type="button" disabled>Mark current attitude as aligned</button>
   <button id="alignclr" class="alignbtn clear" type="button" hidden>forget</button></div>
  <div class="kv" id="imukv"></div><div class="camerr" id="alignerr"></div>
  <canvas id="gchart" style="height:90px;margin-top:10px"></canvas><canvas id="achart" style="height:64px;margin-top:6px"></canvas></div></section>
 <section class="card" id="hapcard"><h2>Haptics · vibration motors</h2><div class="body">
  <div class="kv" id="hapkv"></div>
  <div class="ubar"><button id="upause" type="button">pause log</button></div>
  <div class="ulog" id="ulog"></div></div>
  <div class="note">What the motors render, as the pipeline's render thread decided it, one line per change (<code>"k":"hap"</code>
   records in the session file). FALLBACK means the pipeline loop stopped advancing and the motors switched to
   "warnings offline" on their own.</div></section>
 <section class="card" id="camcard"><h2>Camera</h2><div class="body">
  <div class="cambar"><button id="cambtn" class="cambtn" type="button" disabled>camera</button><span id="camstate" class="muted">waiting for the recorder…</span></div>
  <div class="kv" id="camkv"></div><div class="camerr" id="camerr"></div></div><div class="note" id="camnote"></div></section>
 </div>
 <section class="card"><h2>Radar targets this frame (decoded by l03)</h2><div class="scroll"><table id="tg"></table></div>
  <div class="note">v: + = moving away. az and y: + = rider's right. Raw angle is in 0.01° with + = the sensor's right = the rider's LEFT
   (datasheet sign, still to be confirmed on the bench). Only moving things appear: the sensor is a Doppler radar.</div></section>
 <section class="card"><h2>Health &amp; pipeline</h2><div class="body"><div class="kv" id="hkv"></div></div></section>
 <section class="card"><h2>Tracks</h2><div class="scroll"><table id="tk"></table></div></section>
 <section class="card"><details><summary>Bench accuracy check · before a ride</summary><ol>
  <li><b>Azimuth sign (R1).</b> Stand in front of the sensor facing it and walk toward it on <i>your</i> left, which is the rider's left.
   Dots and tracks must appear on the <b>left</b> of the scope, side <b>LEFT</b>, raw angle <b>&gt; 0</b>. If they appear on the right,
   stop and report it. Never flip a sign in code.</li>
  <li><b>Doppler sign (R2).</b> Walking toward the sensor shows v <b>&lt; 0</b> in orange (approaching); walking away shows v &gt; 0.</li>
  <li><b>Range (R3).</b> Stand at taped 3, 10 and 20 m and step in place: r should read within about 0.3 m (30 cm range bins).</li>
  <li><b>Blind band (R11).</b> Walk toward it more and more slowly and note the slowest pace that still shows a dot.</li>
  <li><b>IMU.</b> With the vest still, |a| ≈ 9.8 m/s² and "up axis" names the IMU axis pointing up. The six-orientation test then fills
   <code>config/frames.yaml</code>.</li></ol></details></section>
</main>
<script>
const LV=["NO THREAT","ADVISORY","WARNING","ALERT"],LVBG=["#1a2530","#5c4d10","#7c3a0a","#8f1717"],LVFG=["#9fb3c6","#f0d35a","#ffae5c","#ff8080"];
const RING=["#c9d6e2","#f0d35a","#ff9a3c","#ff4d4d"];
const CLS={APPROACHING:"#ff9a3c",RECEDING_MOVER:"#6fa8ff",RECEDING_UNCLASSIFIED:"#6fa8ff",STATIONARY:"#7b8a99"};
const BKT=["","arrives in > 6 s","arrives in 3–6 s","arrives in 1.5–3 s","arrives in < 1.5 s"];
const SIDE={LEFT:"◀ LEFT",RIGHT:"RIGHT ▶",CENTER:"▼ CENTRE",BOTH:"◀ BOTH ▶",NONE:""};
let S=null,lastMsg=0;
const $=id=>document.getElementById(id);
const f=(v,n=1)=>v==null||!isFinite(v)?"–":Number(v).toFixed(n);
function chip(el,txt,cls){el.textContent=txt;el.className="chip"+(cls?" "+cls:"");}
function kv(el,pairs){el.innerHTML=pairs.map(([k,v])=>`<span>${k}</span><span>${v}</span>`).join("");}
function connect(){const es=new EventSource("events");es.onmessage=m=>{lastMsg=Date.now();S=JSON.parse(m.data);render();};}
setInterval(()=>{const up=Date.now()-lastMsg<2500;chip($("conn"),up?"connected":"no link to the Pi",up?"ok":"bad");},500);
let RM=100;try{const v=+localStorage.getItem("gf.rm");if(v>=5&&v<=100)RM=v;}catch(e){}
$("rm").value=RM;$("rmv").textContent=RM+" m";
$("rm").addEventListener("input",e=>{RM=+e.target.value;$("rmv").textContent=RM+" m";
 try{localStorage.setItem("gf.rm",RM);}catch(e){}if(S)scope();});
function render(){banner();header();scope();targets();tracks();imu();haptics();health();camera();}
function banner(){const d=S.decision||{},l=d.lvl||0;
 $("banner").style.background=LVBG[l];$("lvl").textContent=LV[l];$("lvl").style.color=LVFG[l];
 $("side").textContent=l?(SIDE[d.side]||d.side):"";$("side").style.color=LVFG[l];
 const sub=[];if(!d.hs)sub.push("waiting for the pipeline…");
 if(l&&d.bkt)sub.push(BKT[d.bkt]);if(d.alert)sub.push("alert channel on");
 if(d.hs==="OFFLINE")sub.push("WARNINGS OFFLINE");else if(d.hs==="DEGRADED")sub.push("degraded: "+(S.health||[]).map(b=>b.bit.toLowerCase()).join(", "));
 $("bsub").textContent=sub.join(" · ");}
function header(){const hs=(S.decision||{}).hs;chip($("hs"),"health "+(hs||"–"),hs==="OK"?"ok":hs==="OFFLINE"?"bad":hs?"warn":"");
 if(S.live){const a=S.age_s;chip($("age"),a==null?"no data yet":a<1?"live · "+f(a*1000,0)+" ms behind":"stale · "+f(a,1)+" s old",a==null?"warn":a<1?"ok":a<3?"warn":"bad");}
 else chip($("age"),"review · t = "+f(S.t,1)+" s","warn");
 chip($("sess"),(S.session||"no session yet")+(S.ended?" · ended":""));}
function scope(){const c=$("scope"),dpr=devicePixelRatio||1,w=c.clientWidth,h=Math.round(w*0.95);
 c.style.height=h+"px";c.width=w*dpr;c.height=h*dpr;const g=c.getContext("2d");g.scale(dpr,dpr);
 g.fillStyle="#0b0f14";g.fillRect(0,0,w,h);
 const A=40*Math.PI/180,top=28,s=Math.min((h-top-24)/RM,(w/2-8)/(RM*Math.sin(A))),cx=w/2;
 const P=(x,y)=>[cx+y*s,top+x*s];               // radar frame: +x behind the rider (down), +y rider's right (right)
 g.fillStyle="#0f1b25";g.beginPath();g.moveTo(cx,top);g.arc(cx,top,RM*s,Math.PI/2-A,Math.PI/2+A);g.closePath();g.fill();
 g.strokeStyle="#22323f";g.fillStyle="#5d7185";g.font="11px ui-monospace,monospace";
 const step=RM<=10?2:RM<=25?5:RM<=50?10:25,rings=[];for(let r=step;r<=RM-step/2;r+=step)rings.push(r);rings.push(RM);
 for(const r of rings){g.beginPath();g.arc(cx,top,r*s,Math.PI/2-A,Math.PI/2+A);g.stroke();const[lx,ly]=P(r,0);g.fillText(r+" m",lx+3,ly-3);}
 g.setLineDash([3,4]);g.beginPath();g.moveTo(cx,top);g.lineTo(...P(RM,0));g.stroke();g.setLineDash([]);
 g.fillStyle="#d2dde8";g.beginPath();g.moveTo(cx,top-14);g.lineTo(cx-8,top+2);g.lineTo(cx+8,top+2);g.closePath();g.fill();
 g.fillStyle="#8da2b5";g.font="12px system-ui,sans-serif";g.textAlign="left";g.fillText("◀ rider's LEFT",6,h-7);
 g.textAlign="right";g.fillText("rider's RIGHT ▶",w-6,h-7);g.textAlign="left";
 // anything past the view range: an arrow on the outer ring along its bearing (clamped to the fan), labelled with its range
 const edge=(x,y,col,fill,txt)=>{const b=Math.max(-A,Math.min(A,Math.atan2(y,x))),dx=Math.sin(b),dy=Math.cos(b),R=RM*s,
   ex=(d,o)=>[cx+dx*d-dy*o,top+dy*d+dx*o];
  g.beginPath();g.moveTo(...ex(R-2,0));g.lineTo(...ex(R-14,-6));g.lineTo(...ex(R-14,6));g.closePath();
  if(fill){g.fillStyle=col;g.fill();}else{g.strokeStyle=col;g.lineWidth=1.2;g.stroke();g.lineWidth=1;}
  g.fillStyle=col;g.font="10px ui-monospace,monospace";g.textAlign="center";g.fillText(txt,...ex(R-26,0).map((v,i)=>v+(i?3:0)));g.textAlign="left";};
 const out=(x,y)=>Math.hypot(x,y)>RM;
 for(const t of S.targets||[]){if(t.x==null)continue;
  if(out(t.x,t.y)){edge(t.x,t.y,CLS[t.cls]||"#7b8a99",!!CLS[t.cls],f(Math.hypot(t.x,t.y),0)+" m");continue;}const[px,py]=P(t.x,t.y),col=CLS[t.cls];g.beginPath();g.arc(px,py,4.5,0,2*Math.PI);
  if(col){g.fillStyle=col;g.fill();}else{g.strokeStyle="#7b8a99";g.lineWidth=1;g.stroke();}}
 for(const k of S.tracks||[]){if(k.x==null)continue;
  if(out(k.x,k.y)){edge(k.x,k.y,RING[k.level||0],true,"#"+k.id+" "+f(Math.hypot(k.x,k.y),0)+" m");continue;}const[px,py]=P(k.x,k.y),col=RING[k.level||0];
  g.strokeStyle=col;g.lineWidth=k.status==="CONFIRMED"?2.5:1.2;g.setLineDash(k.status==="COASTING"?[4,3]:[]);
  g.beginPath();g.arc(px,py,10,0,2*Math.PI);g.stroke();g.setLineDash([]);g.lineWidth=1;
  g.fillStyle=col;g.font="11px ui-monospace,monospace";g.fillText("#"+k.id+" "+f(k.vc,1)+" m/s",px+13,py+4);}}
function targets(){const T=(S.targets||[]).slice().sort((a,b)=>(a.r??99)-(b.r??99));
 let h='<tr><th class="l">class</th><th>r m</th><th>v m/s</th><th>az °</th><th>y m</th><th class="l">side</th><th>dB</th><th>raw angle</th><th>raw speed</th></tr>';
 if(!T.length)h+='<tr><td class="l muted" colspan="9">no targets in this frame</td></tr>';
 for(const t of T){const col=CLS[t.cls]||"#7b8a99";
  h+=`<tr><td class="l" style="color:${col}">${(t.cls||"").toLowerCase().replaceAll("_"," ")}</td><td>${f(t.r,2)}</td><td>${f(t.v,2)}</td><td>${f(t.az,1)}</td><td>${f(t.y,2)}</td><td class="l">${t.side||""}</td><td>${f(t.db,0)}</td><td>${t.angle_raw}</td><td>${t.speed_raw}</td></tr>`;}
 $("tg").innerHTML=h;}
function tracks(){const K=(S.tracks||[]).slice().sort((a,b)=>(b.level-a.level)||((a.r??99)-(b.r??99)));
 let h='<tr><th class="l">track</th><th class="l">status</th><th>r m</th><th>closing m/s</th><th>az °</th><th class="l">threat</th><th>arrives s</th></tr>';
 if(!K.length)h+='<tr><td class="l muted" colspan="7">no tracks</td></tr>';
 for(const k of K)h+=`<tr><td class="l">#${k.id}</td><td class="l">${k.status.toLowerCase()}${k.coast&&k.coast!=="NONE"?" · "+k.coast.toLowerCase().replaceAll("_"," "):""}</td><td>${f(k.r,1)}</td><td>${f(k.vc,1)}</td><td>${f(k.az,0)}</td><td class="l" style="color:${RING[k.level]}">${k.level?LV[k.level].toLowerCase()+" "+(k.side||"").toLowerCase():"–"}</td><td>${f(k.ta,1)}</td></tr>`;
 $("tk").innerHTML=h;}
function chart(id,cols,colors,lo,hi,label){const c=$(id),dpr=devicePixelRatio||1,w=c.clientWidth,h=c.clientHeight;c.width=w*dpr;c.height=h*dpr;
 const g=c.getContext("2d");g.scale(dpr,dpr);g.fillStyle="#0b0f14";g.fillRect(0,0,w,h);const H=S.imu_hist||[];
 for(const j of cols)for(const p of H)if(p[j]!=null){lo=Math.min(lo,p[j]);hi=Math.max(hi,p[j]);}
 const X=t=>(t+8)/8*w,Y=v=>h-4-(v-lo)/((hi-lo)||1)*(h-8);
 if(lo<0&&hi>0){g.strokeStyle="#22323f";g.beginPath();g.moveTo(0,Y(0));g.lineTo(w,Y(0));g.stroke();}
 cols.forEach((j,i)=>{g.strokeStyle=colors[i];g.lineWidth=1.3;g.beginPath();let first=true;
  for(const p of H){if(p[j]==null)continue;const x=X(p[0]),y=Y(p[j]);if(first){g.moveTo(x,y);first=false;}else g.lineTo(x,y);}g.stroke();});
 g.fillStyle="#7489a0";g.font="11px ui-monospace,monospace";g.fillText(`${label}  [${lo.toFixed(1)}, ${hi.toFixed(1)}]  last 8 s`,6,12);}
function imu(){const m=S.imu||{},g=m.gyro_dps,a=m.accel;
 kv($("imukv"),[["gyro °/s",g?`x ${f(g[0],1)}  y ${f(g[1],1)}  z ${f(g[2],1)}`:"–"],["accel m/s²",a?`x ${f(a[0],2)}  y ${f(a[1],2)}  z ${f(a[2],2)}`:"–"],
  ["|a|",f(m.accel_norm,2)+" m/s²"],["rates",`gyro ${f(m.gyro_hz,0)} Hz · accel ${f(m.accel_hz,0)} Hz`],
  ["up axis",(m.up_axis||"–")+" (IMU axes, 1 s mean)"],["roll / pitch",`${f(m.roll,1)}° / ${f(m.pitch,1)}° (l04, radar frame)`],
  ["yaw",`${f(m.yaw,1)}° (arbitrary zero, drifts)`],
  ["l04 state",(m.health||"–")+(m.health?(m.in_motion?" · moving":" · still"):"")],...alignRows(m.aligned)]);
 $("alignbtn").disabled=alignBusy||m.roll==null||m.health==="NO_DATA";$("alignclr").hidden=!m.aligned;$("alignclr").disabled=alignBusy;
 $("alignerr").textContent=alignErr;
 chart("gchart",[1,2,3],["#ff6b6b","#39d3a0","#6fa8ff"],-20,20,"gyro x y z °/s");
 chart("achart",[4],["#f0d35a"],8.5,11,"|a| m/s²");}
let alignBusy=false,alignErr="";
function alignRows(a){if(!a)return[["vs aligned","not set · press the button with the vest in its aligned position"]];
 const d=v=>v==null?"–":(v>0?"+":"")+f(v,1)+"°",c=v=>v!=null&&Math.abs(v)>=5?"var(--warn)":"var(--ok)";
 const when=a.t_wall?new Date(a.t_wall*1000).toLocaleTimeString():"–";
 return[["Δ roll / pitch",`<span style="color:${c(a.d_roll)}">${d(a.d_roll)}</span> / <span style="color:${c(a.d_pitch)}">${d(a.d_pitch)}</span>`],
  ["Δ yaw",a.yaw_valid?`<span style="color:${c(a.d_yaw)}">${d(a.d_yaw)}</span> (drifts slowly)`:"– (lost with the restart; mark again)"],
  ["aligned at",`roll ${f(a.roll,1)}° · pitch ${f(a.pitch,1)}°${a.yaw_valid?" · yaw "+f(a.yaw,1)+"°":""} · ${when}`]];}
async function postAlign(v){alignBusy=true;alignErr="";if(S)imu();
 try{const r=await fetch("imu/align",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({aligned:v})});
  const j=await r.json().catch(()=>({}));if(!r.ok)throw new Error(j.error||"HTTP "+r.status);if(S&&S.imu)S.imu.aligned=j.aligned;}
 catch(e){alignErr="Could not mark the alignment: "+e.message;}
 finally{alignBusy=false;if(S)imu();}}
$("alignbtn").addEventListener("click",()=>{if(!S||!S.imu||!S.imu.aligned||confirm("Replace the aligned attitude with the current one?"))postAlign(true);});
$("alignclr").addEventListener("click",()=>{if(confirm("Forget the aligned attitude?"))postAlign(false);});
function health(){const d=S.decision||{},fr=S.frame||{},e=S.ego||{};
 const bits=(S.health||[]).map(b=>`${b.bit.toLowerCase()} · ${f(b.for_s,1)} s`).join("<br>")||"none";
 kv($("hkv"),[["state",d.hs||"–"],["active faults",bits],
  ["radar",`${f(S.radar_hz,1)} Hz · frame ${fr.fn??"–"} · ${fr.n_raw??0} raw${fr.cap?" · CAP HIT":""}`],
  ["ego-motion",e.valid?`valid · ${f(e.speed,1)} m/s (${(e.source||"").toLowerCase()})`:`invalid (${(e.reason||"–").toLowerCase().replaceAll("_"," ")})`],
  ["power",S.power_flags?`0x${S.power_flags.toString(16)} (under-voltage or throttling)`:"ok"],
  ["records",`${S.records} (${S.errors} errors)${S.last_error?" · "+S.last_error:""}`],["radar fw",S.fw||"–"],["git",S.git||"–"]]);}
let uPaused=false;
$("upause").addEventListener("click",()=>{uPaused=!uPaused;$("upause").textContent=uPaused?"resume log":"pause log";if(S)haptics();});
function haptics(){const h=S.haptics||{},n=h.now,el=$("hap"),col=(c,t)=>`<span style="color:${c}">${t}</span>`;
 if(h.fault)chip(el,"haptics fault","bad");else if(!n)chip(el,"haptics –","");
 else if(n.mode==="FALLBACK")chip(el,"haptics: loop stalled","bad");
 else if(n.r==="OFFLINE")chip(el,"haptics: warnings offline","warn");else chip(el,"haptics ok","ok");
 const motors=(h.motors||[]).map(m=>`${esc(m.name)} ${esc(m.pin)} (${esc(m.side).toLowerCase()})`).join(" · ");
 const rows=[["motors",(motors||"–")+(h.duty_pct!=null?` · ${h.duty_pct} % duty`:"")]];
 if(h.fault)rows.push(["fault",col("var(--bad)","cannot drive the motors · "+esc(h.fault))]);
 rows.push(["rendering",n?esc(n.txt)+(n.age_s!=null?` <span class="muted">· since ${f(n.age_s,1)} s</span>`:"")
  :col("var(--warn)","no haptics records in this session (needs a pipeline that records them)")]);
 if(n)rows.push(["watchdog",n.mode==="FALLBACK"?col("var(--bad)","FALLBACK · the pipeline loop stalled"+(n.cause?" ("+esc(n.cause).toLowerCase()+")":""))
  :col("var(--ok)","NORMAL · the pipeline loop is advancing")]);
 if(h.lat_ms)rows.push(["decision → motor",`${f(h.lat_ms.last,1)} ms last · ${f(h.lat_ms.p50,1)} median · ${f(h.lat_ms.max,1)} max`]);
 const k=h.n||{changes:0,fallbacks:0};rows.push(["changes",`${k.changes} · ${k.fallbacks} fallback${k.fallbacks===1?"":"s"}`]);
 kv($("hapkv"),rows);
 if(uPaused)return;
 const L=$("ulog"),atEnd=L.scrollHeight-L.scrollTop-L.clientHeight<8;
 L.innerHTML=(h.log||[]).map(([a,txt,mode,cause,lat])=>`<div class="${mode==="FALLBACK"?"er":"tx"}"><b>${a==null?"":"−"+f(a,1)+" s"}</b>`+
  `${esc(txt)}${cause?" · "+esc(cause).toLowerCase():""}${lat!=null?` <span class="muted">· ${f(lat,1)} ms after the decision</span>`:""}</div>`).join("")
  ||'<div class="muted">no changes yet</div>';
 if(atEnd)L.scrollTop=L.scrollHeight;}
const CAMST={recording:"recording",starting:"starting…",stopping:"stopping…",off:"off",no_camera:"camera not found",
 low_disk:"paused: card nearly full",error:"error",down:"recorder not running"};
const esc=s=>String(s??"").replace(/[&<>"]/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"}[c]));
const hms=s=>{s=Math.max(0,Math.floor(s||0));const p=v=>String(v).padStart(2,"0"),h=Math.floor(s/3600),m=Math.floor(s/60)%60;
 return h?`${h}:${p(m)}:${p(s%60)}`:`${m}:${p(s%60)}`;};
const gb=b=>b==null?"–":(b/1e9).toFixed(b<1e10?1:0)+" GB";
let camBusy=false,camErr="";
function camera(){const c=S&&S.camera,el=$("cam"),btn=$("cambtn");
 $("camcard").hidden=!c;if(!c){chip(el,"no camera","");return;}
 const st=c.state||"down",rec=st==="recording";
 if(rec)chip(el,"● REC "+hms(c.elapsed_s),"rec");else if(st==="off")chip(el,"camera off","");
 else if(st==="starting"||st==="stopping")chip(el,"camera "+CAMST[st],"warn");else chip(el,"camera: "+(CAMST[st]||st),"bad");
 // the button switches what is wanted; the line next to it says what the recorder is actually doing
 btn.textContent=c.want?"■ Stop recording":"● Start recording";btn.className="cambtn"+(c.want?" stop":"");btn.disabled=camBusy;
 const applying=st!=="down"&&c.applied_want!=null&&c.applied_want!==c.want?" · applying…":"";
 $("camstate").textContent=(rec?`● recording ${hms(c.elapsed_s)} · ${f(c.fps,1)} fps`:CAMST[st]||st)+applying;
 $("camstate").style.color=rec?"#ff8080":st==="off"?"var(--dim)":"var(--warn)";
 const room=c.rate_bps&&c.free_bytes!=null?Math.max(0,(c.free_bytes-c.min_free_bytes)/c.rate_bps/3600):null;
 kv($("camkv"),[["state",esc(c.detail||(CAMST[st]||st))],
  [rec||st==="starting"||st==="stopping"?"file":"last file",c.file?`${esc(c.file)} · ${gb(c.file_bytes)}`:"–"],["folder",esc(c.dir)],
  ["card",`${gb(c.free_bytes)} free`+(room!=null?` · room for about ${f(room,room<10?1:0)} h more`:"")+` · pauses below ${gb(c.min_free_bytes)}`],
  ["camera",esc(c.mode)]]);
 $("camerr").textContent=camErr;
 $("camnote").textContent=`Records all the time: it starts at every boot and keeps going until stopped here. A stop lasts until `+
  `someone starts it again or the Pi restarts. One file per ${Math.round((c.segment_s||300)/60)} min; recording pauses by itself `+
  `while the card has less than ${gb(c.min_free_bytes)} free, so the pipeline can always record. Nothing is ever deleted.`;}
$("cambtn").addEventListener("click",async()=>{const c=S&&S.camera;if(!c||camBusy)return;const want=!c.want;
 if(!want&&!confirm("Stop recording the camera?\n\nIt stays off until someone starts it again here, or the Pi restarts."))return;
 camBusy=true;camErr="";camera();
 try{const r=await fetch("camera",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({recording:want})});
  const j=await r.json().catch(()=>({}));if(!r.ok)throw new Error(j.error||"HTTP "+r.status);S.camera=j.camera;}
 catch(e){camErr="Could not switch the camera: "+e.message;}
 finally{camBusy=false;camera();}});
connect();
</script></body></html>
"""


if __name__ == "__main__":
    sys.exit(main())

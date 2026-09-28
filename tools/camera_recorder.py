"""Golden Fleece camera recorder: records the USB camera into "Camera Footage/" unless the web app turns it off.

    .venv/bin/python tools/camera_recorder.py              # run it (the goldenfleece-camera service does this at boot)
    .venv/bin/python tools/camera_recorder.py --status     # what the running recorder is doing
    .venv/bin/python tools/camera_recorder.py --off        # what the web app's stop button does (--on starts again)

The footage is for reviewing rides.  Nothing in the pipeline reads the camera, and this is a separate process
(deploy/goldenfleece-camera.service), so a camera fault cannot touch the warnings.  ffmpeg copies the camera's own
MJPEG frames into MKV files, one per ``segment_s`` cut on the clock, without decoding them: about 0.02 cores, and
about 22-25 GB an hour at 1080p30 (config/camera.yaml has the measurement).  An MKV cut short by a pulled battery
still plays, and the open file is synced to the card every ``sync_period_s``, so a power cut loses seconds of
footage rather than the ~30 s the kernel would otherwise hold back.  File names carry the start time and a random
run id: the Pi can boot with a stale clock, and two runs must never write the same name (ffmpeg would overwrite).

Recording is on unless someone turned it off.  The web app's button writes ``control.json`` in ``runtime_dir``;
this process applies it within half a second and reports what it is doing in ``status.json`` there, which the page
shows.  ``runtime_dir`` is under /tmp, which is emptied at boot, so "off" lasts until recording is turned back on
or the Pi restarts.  Recording also pauses by itself while the card has less than ``min_free_gb`` free, so footage
never fills the card the pipeline records to, and resumes once space is freed.  Nothing is ever deleted.
"""
from __future__ import annotations

import argparse
import ctypes
import fcntl
import json
import logging
import os
import re
import secrets
import selectors
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Deque, Dict, List, Optional, Sequence

import yaml

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "config" / "camera.yaml"

TICK_S = 0.5                        # control.json is applied within this
STATUS_PERIOD_S = 1.0
STATUS_STALE_S = 5.0                # an older status.json means the recorder is not running
SCAN_PERIOD_S = 1.0                 # looking for the file ffmpeg is writing
STOP_TIMEOUT_S = 10.0               # after SIGINT, ffmpeg gets this long to finish its file before SIGKILL
RESTART_BACKOFF_S = (1.0, 2.0, 5.0, 10.0, 30.0)
HEALTHY_RUN_S = 60.0                # a run at least this long resets the backoff
RESUME_MARGIN_BYTES = 1e9           # after a low-disk pause, resume at min_free + 1 GB (no flapping at the edge)
GB = 1e9

log = logging.getLogger("goldenfleece.camera")


class CameraConfigError(ValueError):
    pass


@dataclass(frozen=True)
class CameraConfig:
    device: str
    input_format: str
    video_size: str
    framerate: int
    footage_dir: Path
    segment_s: int
    min_free_gb: float
    sync_period_s: float
    stall_s: float
    runtime_dir: Path

    @property
    def control_path(self) -> Path:
        return self.runtime_dir / "control.json"

    @property
    def status_path(self) -> Path:
        return self.runtime_dir / "status.json"

    @property
    def mode(self) -> str:
        return f"{self.video_size} {self.input_format.upper()} at {self.framerate} fps, copied without re-encoding"


_KEYS = ("device", "input_format", "video_size", "framerate", "footage_dir", "segment_s", "min_free_gb",
         "sync_period_s", "stall_s", "runtime_dir")


def load_camera_config(path: Path | str = DEFAULT_CONFIG) -> CameraConfig:
    """Relative ``footage_dir`` and ``runtime_dir`` are taken from the repo root."""
    path = Path(path)
    try:
        d = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as e:
        raise CameraConfigError(f"{path}: {e}") from e
    if not isinstance(d, dict):
        raise CameraConfigError(f"{path}: top level must be a mapping")
    missing = [k for k in _KEYS if k not in d]
    unknown = [k for k in d if k not in _KEYS and k != "version"]
    if missing or unknown:
        raise CameraConfigError(f"{path.name}: " + "; ".join(
            ([f"missing {', '.join(missing)}"] if missing else []) + ([f"unknown {', '.join(unknown)}"] if unknown else [])))

    def under_root(v: Any) -> Path:
        p = Path(str(v)).expanduser()
        return p if p.is_absolute() else ROOT / p

    try:
        cfg = CameraConfig(device=str(d["device"]), input_format=str(d["input_format"]), video_size=str(d["video_size"]),
                           framerate=int(d["framerate"]), footage_dir=under_root(d["footage_dir"]),
                           segment_s=int(d["segment_s"]), min_free_gb=float(d["min_free_gb"]),
                           sync_period_s=float(d["sync_period_s"]), stall_s=float(d["stall_s"]),
                           runtime_dir=under_root(d["runtime_dir"]))
    except (TypeError, ValueError) as e:
        raise CameraConfigError(f"{path.name}: {e}") from e
    if not re.fullmatch(r"\d+x\d+", cfg.video_size):
        raise CameraConfigError(f"{path.name}: video_size must look like 1920x1080")
    if cfg.framerate <= 0 or cfg.segment_s < 10 or cfg.min_free_gb < 0 or cfg.sync_period_s < 0 or cfg.stall_s <= 0:
        raise CameraConfigError(f"{path.name}: framerate > 0, segment_s >= 10, stall_s > 0, min_free_gb and "
                                "sync_period_s >= 0")
    return cfg


# ---- control.json and status.json: the only link between the web app and this process -------------------------
def _write_json(path: Path, data: Dict[str, Any]) -> None:
    """Atomic replace: a reader sees the old file or the new one, never half of one."""
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}.")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(json.dumps(data, separators=(",", ":")))
        os.chmod(tmp, 0o644)
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise


def _read_json(path: Path) -> Optional[Dict[str, Any]]:
    try:
        d = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    return d if isinstance(d, dict) else None


def prepare_runtime_dir(cfg: CameraConfig) -> None:
    """Create ``runtime_dir``; refuse one that belongs to another user (under /tmp anyone could create it first)."""
    cfg.runtime_dir.mkdir(mode=0o755, parents=True, exist_ok=True)
    owner = cfg.runtime_dir.stat().st_uid
    if owner != os.getuid():
        raise PermissionError(f"{cfg.runtime_dir} belongs to uid {owner}, not to this user; remove it")


def read_control(cfg: CameraConfig) -> Dict[str, Any]:
    """What is wanted.  Recording, unless control.json says {"recording": false}: no file (as after every boot) or
    an unreadable one means recording."""
    d = _read_json(cfg.control_path) or {}
    return {"recording": d.get("recording") is not False, "src": d.get("src"), "t_wall": d.get("t_wall")}


def set_recording_wanted(cfg: CameraConfig, recording: bool, src: str) -> None:
    prepare_runtime_dir(cfg)
    _write_json(cfg.control_path, {"recording": bool(recording), "t_wall": round(time.time(), 3), "src": src})


def read_status(cfg: CameraConfig, now_wall: Optional[float] = None) -> Dict[str, Any]:
    """For the page: the recorder's status.json, whether the recorder is alive, and what is wanted (``want``)."""
    now_wall = time.time() if now_wall is None else now_wall
    st = _read_json(cfg.status_path) or {}
    t = st.get("t_wall")
    age = now_wall - t if isinstance(t, (int, float)) else None
    out = dict(st)
    out.update(want=read_control(cfg)["recording"], status_age_s=round(age, 1) if age is not None else None,
               dir=st.get("dir", str(cfg.footage_dir)), segment_s=st.get("segment_s", cfg.segment_s),
               min_free_bytes=st.get("min_free_bytes", int(cfg.min_free_gb * GB)), mode=st.get("mode", cfg.mode))
    if st.get("state") == "down":
        out["detail"] = "the camera recorder was stopped (sudo systemctl start goldenfleece-camera)"
    elif age is None or abs(age) > STATUS_STALE_S:
        out["state"] = "down"
        out["detail"] = ("no status from the camera recorder" if age is None else
                         f"the camera recorder's last status is {age:.0f} s old") + \
                        ": is the goldenfleece-camera service running?"
    return out


# ---- ffmpeg ------------------------------------------------------------------------------------------------------
def ffmpeg_args(cfg: CameraConfig, run_id: str) -> List[str]:
    """Arguments after the program name.  The camera's MJPEG frames are copied, never decoded; ``-progress`` puts
    ``frame=N`` on stdout twice a second, which is how a stalled camera is noticed."""
    pattern = str(cfg.footage_dir).replace("%", "%%") + f"/camera_%Y-%m-%dT%H%M%S_{run_id}.mkv"   # strftime'd per file
    return ["-hide_banner", "-nostdin", "-loglevel", "error", "-nostats", "-progress", "pipe:1",
            "-f", "v4l2", "-input_format", cfg.input_format, "-video_size", cfg.video_size,
            "-framerate", str(cfg.framerate), "-i", cfg.device,
            "-map", "0:v:0", "-c:v", "copy",
            "-f", "segment", "-segment_format", "matroska", "-segment_time", str(cfg.segment_s),
            "-segment_atclocktime", "1", "-reset_timestamps", "1", "-strftime", "1", pattern]


try:
    _LIBC: Optional[ctypes.CDLL] = ctypes.CDLL(None, use_errno=True)
except OSError:                                                            # pragma: no cover
    _LIBC = None
_PR_SET_PDEATHSIG = 1


def _ffmpeg_dies_with_us() -> None:
    """Runs in the child before exec (this process has no threads): if the recorder dies without stopping ffmpeg,
    the kernel sends ffmpeg SIGINT and it finishes its file instead of holding the camera forever."""
    if _LIBC is not None:
        _LIBC.prctl(ctypes.c_int(_PR_SET_PDEATHSIG), ctypes.c_ulong(int(signal.SIGINT)))


class Recorder:
    """Keeps one ffmpeg recording while recording is wanted, the camera is plugged in and the card has room.

    Single-threaded: ``step()`` decides, ``pump()`` waits up to a timeout for ffmpeg's output.  States:
    starting, recording, stopping, off, no_camera, low_disk, error, down (written on the way out)."""

    def __init__(self, cfg: CameraConfig, ffmpeg: Sequence[str] = ("ffmpeg",),
                 free_bytes: Optional[Callable[[Path], float]] = None) -> None:
        self.cfg = cfg
        self.ffmpeg = list(ffmpeg)
        self.free_bytes = free_bytes or (lambda p: shutil.disk_usage(p).free)
        self.sel = selectors.DefaultSelector()
        self.proc: Optional[subprocess.Popen] = None
        self.state, self.detail = "starting", ""
        self.control: Dict[str, Any] = {"recording": True}
        self.free: Optional[float] = None
        self.run_id = ""
        self.t_start = self.t_frame = 0.0         # monotonic: this ffmpeg started / the frame count last advanced
        self.frames = 0
        self.fps: Optional[float] = None
        self.t_stop: Optional[float] = None       # when ffmpeg was asked to stop, and the state to take after
        self.after_stop = ("off", "")
        self.failures = 0
        self.t_retry = 0.0
        self.err: Deque[str] = deque(maxlen=8)    # ffmpeg's last stderr lines, for the reason it stopped
        self.bufs: Dict[int, bytes] = {}
        self.file: Optional[Path] = None          # the file being written (after a stop: the last one written)
        self.file_fd: Optional[int] = None        # read-only handle, only for fdatasync
        self.file_bytes = 0
        self.run_bytes = 0
        self.t_scan = self.t_sync = 0.0
        self.t_status = -1e9
        self._status_key: Any = None

    # ---- the loop ----------------------------------------------------------------------------------------------
    def run(self, should_stop: Callable[[], bool]) -> None:
        try:
            while not should_stop():
                self.step()
                self.pump(TICK_S)
        finally:
            self.shutdown()

    def step(self) -> None:
        now = time.monotonic()
        control = read_control(self.cfg)
        if control["recording"] and not self.control["recording"]:
            self.failures, self.t_retry = 0, 0.0          # turned back on: try the camera again right away
        self.control = control
        self.free = self._free()
        if self.proc is not None:
            self._supervise(now)
        if self.proc is None:
            self._idle(now)
        self._write_status(now)

    def pump(self, timeout: float) -> None:
        """Wait up to ``timeout`` for ffmpeg output and take in whatever arrived."""
        if not self.sel.get_map():
            time.sleep(timeout)
            return
        for key, _ in self.sel.select(timeout):
            self._read(key.fd, key.data)

    def shutdown(self) -> None:
        if self.proc is not None:
            if self.t_stop is None:
                self._stop(time.monotonic(), "down", "the recorder is shutting down")
            try:
                self.proc.wait(timeout=STOP_TIMEOUT_S)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
            self._reap(time.monotonic(), self.proc.returncode)
        self.state, self.detail = "down", ""
        self._write_status(time.monotonic(), force=True)
        self.sel.close()

    # ---- decisions ---------------------------------------------------------------------------------------------
    def _supervise(self, now: float) -> None:
        rc = self.proc.poll()
        if rc is not None:
            self._reap(now, rc)
            return
        if self.t_stop is not None:                        # asked to stop: let it finish the file
            if now - self.t_stop > STOP_TIMEOUT_S:
                log.warning("ffmpeg did not stop within %.0f s; killing it", STOP_TIMEOUT_S)
                self.proc.kill()
            return
        if not self.control["recording"]:
            self._stop(now, "off", self._off_detail())
        elif self._low_disk(resuming=False):
            self._stop(now, "low_disk", self._low_disk_detail())
        elif now - self.t_frame > self.cfg.stall_s:
            self._stop(now, "error", f"no frames from the camera for {self.cfg.stall_s:.0f} s")
        else:
            if self.frames > 0:
                self.state, self.detail = "recording", ""
            self._track_file(now)

    def _idle(self, now: float) -> None:
        if not self.control["recording"]:
            self.state, self.detail = "off", self._off_detail()
        elif not os.path.exists(self.cfg.device):
            self.state, self.detail = "no_camera", f"{self.cfg.device} not found: is the camera plugged in?"
        elif self._low_disk(resuming=self.state == "low_disk"):
            self.state, self.detail = "low_disk", self._low_disk_detail()
        elif now >= self.t_retry:
            self._start(now)
        # otherwise an error is being shown until the retry

    def _start(self, now: float) -> None:
        self.run_id = secrets.token_hex(2)
        self.file, self.file_bytes, self.run_bytes = None, 0, 0
        try:
            self.cfg.footage_dir.mkdir(parents=True, exist_ok=True)
            self.proc = subprocess.Popen(self.ffmpeg + ffmpeg_args(self.cfg, self.run_id), stdin=subprocess.DEVNULL,
                                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True,
                                         preexec_fn=_ffmpeg_dies_with_us)
        except OSError as e:
            self.proc = None
            self.state, self.detail = "error", f"cannot start {self.ffmpeg[0]}: {e}"
            self._schedule_retry(now, 0.0)
            log.error("%s", self.detail)
            return
        for f, tag in ((self.proc.stdout, "out"), (self.proc.stderr, "err")):
            os.set_blocking(f.fileno(), False)
            self.sel.register(f.fileno(), selectors.EVENT_READ, tag)
            self.bufs[f.fileno()] = b""
        self.t_start = self.t_frame = now
        self.t_scan = self.t_sync = 0.0
        self.frames, self.fps = 0, None
        self.err.clear()
        self.state, self.detail = "starting", ""
        log.info("recording %s (%s) into %s, run %s", self.cfg.device, self.cfg.mode, self.cfg.footage_dir, self.run_id)

    def _stop(self, now: float, then: str, detail: str) -> None:
        log.info("stopping ffmpeg: %s%s", then, f" ({detail})" if detail else "")
        try:
            self.proc.send_signal(signal.SIGINT)           # ffmpeg finishes the file it is writing, then exits
        except ProcessLookupError:
            pass
        self.t_stop = now
        self.after_stop = (then, detail)
        self.state, self.detail = "stopping", ""

    def _reap(self, now: float, rc: int) -> None:
        """ffmpeg exited: because it was asked to, or on its own (camera unplugged, device busy, crash)."""
        for fd, tag in [(k.fd, k.data) for k in self.sel.get_map().values()]:
            self._read(fd, tag, drain=True)                # its last words explain an unexpected exit
            if fd in self.bufs:
                self.sel.unregister(fd)
                self.bufs.pop(fd)
        self.proc.stdout.close()
        self.proc.stderr.close()
        self._track_file(now, force=True)
        self._release_file()
        ran = now - self.t_start
        if self.t_stop is not None:
            self.state, self.detail = self.after_stop
            if self.state == "error":
                self._schedule_retry(now, ran)
        else:
            why = self.err[-1] if self.err else f"exit status {rc}"
            if not os.path.exists(self.cfg.device):
                self.state, self.detail = "no_camera", "the camera was unplugged"
            else:
                self.state, self.detail = "error", f"ffmpeg stopped: {why}"
            self._schedule_retry(now, ran)
        log.info("ffmpeg exited (%s) after %.0f s and %d frames; %s%s", rc, ran, self.frames, self.state,
                 f": {self.detail}" if self.detail else "")
        self.proc, self.t_stop = None, None

    def _schedule_retry(self, now: float, ran: float) -> None:
        self.failures = 1 if ran >= HEALTHY_RUN_S else self.failures + 1
        delay = RESTART_BACKOFF_S[min(self.failures, len(RESTART_BACKOFF_S)) - 1]
        self.t_retry = now + delay
        if self.state == "error":
            self.detail += f" (retrying in {delay:.0f} s)"

    def _free(self) -> Optional[float]:
        p = self.cfg.footage_dir
        while not p.exists() and p != p.parent:
            p = p.parent
        try:
            return float(self.free_bytes(p))
        except OSError:
            return None

    def _low_disk(self, resuming: bool) -> bool:
        floor = self.cfg.min_free_gb * GB + (RESUME_MARGIN_BYTES if resuming else 0.0)
        return self.free is not None and self.free < floor

    def _low_disk_detail(self) -> str:
        return (f"{(self.free or 0) / GB:.1f} GB free; recording pauses below {self.cfg.min_free_gb:g} GB so the "
                f"pipeline can still record, and resumes above {self.cfg.min_free_gb + RESUME_MARGIN_BYTES / GB:g} GB")

    def _off_detail(self) -> str:
        return f"turned off from {self.control.get('src') or 'the web app'}"

    # ---- ffmpeg's output -----------------------------------------------------------------------------------------
    def _read(self, fd: int, tag: str, drain: bool = False) -> None:
        while True:
            try:
                data = os.read(fd, 65536)
            except BlockingIOError:
                return
            lines = (self.bufs.get(fd, b"") + data).split(b"\n")
            self.bufs[fd] = lines.pop() if data else b""     # keep a partial last line, unless the pipe closed
            for line in lines:
                self._line(tag, line.decode("utf-8", "replace").strip())
            if not data:
                self.sel.unregister(fd)
                self.bufs.pop(fd, None)
                return
            if not drain:
                return

    def _line(self, tag: str, line: str) -> None:
        if not line:
            return
        if tag == "err":
            self.err.append(line)
            (log.debug if self.t_stop is not None else log.warning)("ffmpeg: %s", line)   # stop noise is harmless
            return
        key, _, value = line.partition("=")
        try:
            if key == "frame" and int(value) > self.frames:
                self.frames, self.t_frame = int(value), time.monotonic()
            elif key == "fps":
                self.fps = float(value)
        except ValueError:
            pass

    # ---- the file being written --------------------------------------------------------------------------------
    def _track_file(self, now: float, force: bool = False) -> None:
        """Find this run's newest file (the one being written), total the run's bytes, and fdatasync the open file
        every ``sync_period_s`` so a pulled battery loses seconds, not the kernel's ~30 s of unwritten footage."""
        if not force and now < self.t_scan:
            return
        self.t_scan = now + SCAN_PERIOD_S
        newest, total = None, 0
        for p in self.cfg.footage_dir.glob(f"camera_*_{self.run_id}.mkv"):
            try:
                st = p.stat()
            except OSError:
                continue
            total += st.st_size
            if newest is None or (st.st_mtime, p.name) > newest[0]:
                newest = ((st.st_mtime, p.name), p, st.st_size)
        self.run_bytes = total
        if newest is None:
            return
        _, path, self.file_bytes = newest
        if path != self.file:
            self._release_file()                           # the previous file is complete: sync it one last time
            self.file = path
            try:
                self.file_fd = os.open(path, os.O_RDONLY)
            except OSError:
                self.file_fd = None
        if self.file_fd is not None and self.cfg.sync_period_s > 0 and now - self.t_sync >= self.cfg.sync_period_s:
            self.t_sync = now
            try:
                os.fdatasync(self.file_fd)
            except OSError as e:
                log.warning("fdatasync %s: %s", self.file.name, e)

    def _release_file(self) -> None:
        if self.file_fd is None:
            return
        try:
            os.fdatasync(self.file_fd)
        except OSError:
            pass
        os.close(self.file_fd)
        self.file_fd = None

    # ---- status.json ---------------------------------------------------------------------------------------------
    def _write_status(self, now: float, force: bool = False) -> None:
        key = (self.state, self.detail, self.control["recording"], self.file)
        if not force and key == self._status_key and now - self.t_status < STATUS_PERIOD_S:
            return
        self._status_key, self.t_status = key, now
        active = self.proc is not None
        ran = now - self.t_start
        st = {"v": 1, "t_wall": round(time.time(), 3), "pid": os.getpid(), "state": self.state, "detail": self.detail,
              "applied_want": self.control["recording"], "device": self.cfg.device, "mode": self.cfg.mode,
              "dir": str(self.cfg.footage_dir), "segment_s": self.cfg.segment_s, "run_id": self.run_id or None,
              "file": self.file.name if self.file else None, "file_bytes": self.file_bytes if self.file else None,
              "elapsed_s": round(ran, 1) if active else None, "frames": self.frames if active else None,
              "fps": self.fps if active else None,
              "rate_bps": round(self.run_bytes / ran) if active and ran >= 10 and self.run_bytes else None,
              "free_bytes": self.free, "min_free_bytes": int(self.cfg.min_free_gb * GB)}
        try:
            prepare_runtime_dir(self.cfg)
            _write_json(self.cfg.status_path, st)
        except OSError as e:
            log.warning("cannot write %s: %s", self.cfg.status_path, e)


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(DEFAULT_CONFIG))
    ap.add_argument("--log-level", default="INFO")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--status", action="store_true", help="print what the running recorder is doing, and exit")
    g.add_argument("--on", action="store_true", help="turn recording on, as the web app's button does")
    g.add_argument("--off", action="store_true", help="turn recording off until --on, the web app or the next boot")
    a = ap.parse_args(argv)
    logging.basicConfig(level=getattr(logging, a.log_level.upper()), format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    cfg = load_camera_config(a.config)
    if a.on or a.off:
        set_recording_wanted(cfg, a.on, "the command line")
        print(f"recording turned {'on' if a.on else 'off'}; the recorder applies it within {TICK_S:g} s")
        return 0
    if a.status:
        print(json.dumps(read_status(cfg), indent=2))
        return 0
    try:
        prepare_runtime_dir(cfg)
    except PermissionError as e:
        print(f"camera recorder: {e}", file=sys.stderr)
        return 1
    lock = open(cfg.runtime_dir / "recorder.lock", "w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print("another camera recorder is already running (see --status); two would fight over the camera",
              file=sys.stderr)
        return 1
    stop: List[int] = []
    for s in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        signal.signal(s, lambda signum, frame: stop.append(signum))
    log.info("camera recorder: %s -> %s (control and status in %s)", cfg.device, cfg.footage_dir, cfg.runtime_dir)
    Recorder(cfg).run(lambda: bool(stop))
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""Camera recorder (tools/camera_recorder.py) and the web app's camera button: recording is on unless someone turned
it off; the button reaches the recorder through control.json and the page reads status.json; ffmpeg is stopped
cleanly, restarted when it fails or stalls, never outlives a killed recorder and never overwrites a file; a nearly
full card pauses recording instead of filling up.  ffmpeg is replaced by a small script that behaves like it."""
import dataclasses
import json
import os
import signal
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import pytest

from tools import camera_recorder as C
from tools import web_app as W

FAKE_FFMPEG = r'''
import os, signal, sys, time
if os.environ.get("FAKE_FFMPEG_PIDFILE"):
    open(os.environ["FAKE_FFMPEG_PIDFILE"], "w").write(str(os.getpid()))
mode = os.environ.get("FAKE_FFMPEG_MODE", "ok")
if mode == "fail":
    print("[video4linux2,v4l2 @ 0x1] Cannot open video device: Device or resource busy", file=sys.stderr, flush=True)
    sys.exit(1)
stop = []
signal.signal(signal.SIGINT, lambda *a: stop.append(1))
n = 0
with open(time.strftime(sys.argv[-1]), "ab") as f:          # ffmpeg -strftime 1: the pattern is the last argument
    while not stop:
        n += 1
        f.write(b"\0" * 4096)
        f.flush()
        if mode != "stall" or n <= 3:
            print(f"frame={n}\nfps=30.0\nprogress=continue", flush=True)
        time.sleep(0.02)
print(f"frame={n}\nprogress=end", flush=True)
'''


@pytest.fixture
def cam(tmp_path):
    dev = tmp_path / "video0"
    dev.write_text("")                                       # stands in for the camera's device node
    return C.CameraConfig(device=str(dev), input_format="mjpeg", video_size="1920x1080", framerate=30,
                          footage_dir=tmp_path / "Camera Footage", segment_s=300, min_free_gb=1.0, sync_period_s=0.1,
                          stall_s=1.0, runtime_dir=tmp_path / "run")


@pytest.fixture
def fake_ffmpeg(tmp_path):
    p = tmp_path / "fake_ffmpeg.py"
    p.write_text(FAKE_FFMPEG)
    return [sys.executable, str(p)]


def _drive(rec, pred, timeout=8.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rec.step()
        if pred():
            return True
        rec.pump(0.05)
    return False


def _status(cam):
    return json.loads(cam.status_path.read_text())


def test_repo_config_records_the_usb_camera_into_camera_footage(repo_root):
    cfg = C.load_camera_config(repo_root / "config" / "camera.yaml")
    assert cfg.device.startswith("/dev/v4l/by-id/") and "HD_USB_Camera" in cfg.device
    assert cfg.footage_dir == repo_root / "Camera Footage"
    assert cfg.input_format == "mjpeg"
    assert cfg.runtime_dir.is_relative_to("/tmp")            # emptied at boot: recording is on after every boot
    assert "Camera Footage/" in (repo_root / ".gitignore").read_text().splitlines()   # never pushed with the repo


def test_config_rejects_a_misspelt_key(repo_root, tmp_path):
    bad = tmp_path / "camera.yaml"
    bad.write_text((repo_root / "config" / "camera.yaml").read_text().replace("min_free_gb:", "min_free_gbb:"))
    with pytest.raises(C.CameraConfigError, match="missing min_free_gb; unknown min_free_gbb"):
        C.load_camera_config(bad)


def test_recording_is_wanted_unless_turned_off(cam):
    assert C.read_control(cam)["recording"]                  # no control file, as after every boot
    C.set_recording_wanted(cam, False, "test")
    assert C.read_control(cam) == {"recording": False, "src": "test", "t_wall": pytest.approx(time.time(), abs=5)}
    C.set_recording_wanted(cam, True, "test")
    assert C.read_control(cam)["recording"]
    cam.control_path.write_text("{not json")
    assert C.read_control(cam)["recording"]                  # unreadable: record rather than miss a ride


def test_ffmpeg_copies_the_mjpeg_frames_into_clock_aligned_mkv_files(cam):
    args = C.ffmpeg_args(cam, "ab12")
    line = " ".join(args)
    assert "-c:v copy" in line and "-input_format mjpeg" in line and f"-i {cam.device}" in line
    assert "-f segment -segment_format matroska -segment_time 300 -segment_atclocktime 1" in line
    assert "-strftime 1" in line and "-progress pipe:1" in line
    assert args[-1] == f"{cam.footage_dir}/camera_%Y-%m-%dT%H%M%S_ab12.mkv"
    odd = dataclasses.replace(cam, footage_dir=cam.footage_dir.parent / "100%")
    assert C.ffmpeg_args(odd, "ab12")[-1].startswith(str(odd.footage_dir).replace("%", "%%") + "/")


def test_records_by_default_stops_when_turned_off_and_never_overwrites(cam, fake_ffmpeg, monkeypatch):
    run_ids = iter(["aaaa", "bbbb"])
    monkeypatch.setattr(C.secrets, "token_hex", lambda n: next(run_ids))
    rec = C.Recorder(cam, ffmpeg=fake_ffmpeg)
    try:
        assert _drive(rec, lambda: rec.state == "recording" and rec.file is not None)
        first = rec.file
        assert first.parent == cam.footage_dir and first.name.startswith("camera_") and first.name.endswith("_aaaa.mkv")
        st = _status(cam)
        assert st["state"] == "recording" and st["file"] == first.name and st["frames"] > 0
        assert C.read_status(cam)["want"] is True
        C.set_recording_wanted(cam, False, "the web app (test)")
        assert _drive(rec, lambda: rec.state == "off")
        assert rec.proc is None and rec.detail == "turned off from the web app (test)"
        size = first.stat().st_size
        assert size > 0 and C.read_status(cam)["state"] == "off"
        C.set_recording_wanted(cam, True, "test")
        assert _drive(rec, lambda: rec.state == "recording" and rec.file is not None and rec.file != first)
        assert rec.file.name.endswith("_bbbb.mkv") and first.stat().st_size == size   # a new file; the old one intact
    finally:
        rec.shutdown()
    assert rec.proc is None and C.read_status(cam)["state"] == "down"


def test_waits_for_the_camera_and_starts_when_it_is_plugged_in(cam, fake_ffmpeg):
    os.unlink(cam.device)
    rec = C.Recorder(cam, ffmpeg=fake_ffmpeg)
    try:
        rec.step()
        assert rec.state == "no_camera" and rec.proc is None and _status(cam)["state"] == "no_camera"
        Path(cam.device).write_text("")
        assert _drive(rec, lambda: rec.state == "recording")
    finally:
        rec.shutdown()


def test_an_ffmpeg_failure_is_shown_and_retried(cam, fake_ffmpeg, monkeypatch):
    monkeypatch.setenv("FAKE_FFMPEG_MODE", "fail")
    rec = C.Recorder(cam, ffmpeg=fake_ffmpeg)
    try:
        assert _drive(rec, lambda: rec.state == "error")
        assert "Device or resource busy" in rec.detail and "retrying in 1 s" in rec.detail
        assert "Device or resource busy" in C.read_status(cam)["detail"]
        monkeypatch.setenv("FAKE_FFMPEG_MODE", "ok")
        assert _drive(rec, lambda: rec.state == "recording")       # after the 1 s backoff
    finally:
        rec.shutdown()


def test_a_camera_that_stops_delivering_frames_is_restarted(cam, fake_ffmpeg, monkeypatch):
    monkeypatch.setenv("FAKE_FFMPEG_MODE", "stall")
    rec = C.Recorder(cam, ffmpeg=fake_ffmpeg)
    try:
        assert _drive(rec, lambda: rec.state == "recording")
        assert _drive(rec, lambda: rec.state == "error")           # stall_s = 1 s without a new frame
        assert rec.proc is None and "no frames from the camera for 1 s" in rec.detail
        monkeypatch.setenv("FAKE_FFMPEG_MODE", "ok")
        assert _drive(rec, lambda: rec.state == "recording")
    finally:
        rec.shutdown()


def test_a_nearly_full_card_pauses_recording_until_space_is_freed(cam, fake_ffmpeg):
    free = [50e9]
    rec = C.Recorder(cam, ffmpeg=fake_ffmpeg, free_bytes=lambda p: free[0])
    try:
        assert _drive(rec, lambda: rec.state == "recording")
        free[0] = 0.5e9                                              # below min_free_gb = 1
        assert _drive(rec, lambda: rec.state == "low_disk")
        assert rec.proc is None and rec.detail.startswith("0.5 GB free")
        free[0] = 1.5e9                                              # above the floor but inside the 1 GB margin
        for _ in range(4):
            rec.step()
        assert rec.state == "low_disk" and rec.proc is None
        free[0] = 2.5e9
        assert _drive(rec, lambda: rec.state == "recording")
    finally:
        rec.shutdown()


def test_the_page_says_when_the_recorder_is_not_running(cam):
    s = C.read_status(cam)
    assert s["state"] == "down" and s["want"] is True and "no status from the camera recorder" in s["detail"]
    C.prepare_runtime_dir(cam)
    cam.status_path.write_text(json.dumps({"state": "recording", "t_wall": time.time() - 60}))
    s = C.read_status(cam)
    assert s["state"] == "down" and "60 s old" in s["detail"]


def test_ffmpeg_does_not_outlive_a_killed_recorder(cam, fake_ffmpeg, tmp_path, repo_root):
    """The recorder can die without stopping ffmpeg (kill -9); ffmpeg must then finish and free the camera."""
    conf = tmp_path / "camera.yaml"
    conf.write_text("\n".join(f"{k}: {json.dumps(str(v) if isinstance(v, Path) else v)}"
                              for k, v in dataclasses.asdict(cam).items()))
    driver = (f"import sys; sys.path.insert(0, {str(repo_root)!r})\n"
              "from tools import camera_recorder as C\n"
              f"C.Recorder(C.load_camera_config({str(conf)!r}), ffmpeg={fake_ffmpeg!r}).run(lambda: False)\n")
    pidfile = tmp_path / "ffmpeg.pid"
    proc = subprocess.Popen([sys.executable, "-c", driver], env={**os.environ, "FAKE_FFMPEG_PIDFILE": str(pidfile)})
    try:
        end = time.monotonic() + 10
        while time.monotonic() < end and C.read_status(cam).get("state") != "recording":
            time.sleep(0.05)
        assert C.read_status(cam)["state"] == "recording"
        ffmpeg_pid = int(pidfile.read_text())
    finally:
        proc.send_signal(signal.SIGKILL)
        proc.wait()
    end = time.monotonic() + 5
    while time.monotonic() < end:
        try:
            os.kill(ffmpeg_pid, 0)
        except ProcessLookupError:
            break
        time.sleep(0.05)
    else:
        os.kill(ffmpeg_pid, signal.SIGKILL)
        pytest.fail("ffmpeg kept running after its recorder was killed")


@pytest.fixture
def web(cfg, cam):
    srv = W.WebServer(("127.0.0.1", 0), W.LiveState(cfg), live=False, camera=cam)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"

    def post(body, ctype="application/json"):
        req = urllib.request.Request(base + "/camera", data=body, method="POST", headers={"Content-Type": ctype})
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())

    def state():
        return json.loads(urllib.request.urlopen(base + "/state", timeout=5).read())

    try:
        yield base, post, state
    finally:
        srv.shutdown()
        srv.server_close()


def test_web_app_camera_button_writes_the_control_file(cam, web):
    base, post, state = web
    page = urllib.request.urlopen(base + "/", timeout=5).read().decode()
    assert 'id="cambtn"' in page and 'fetch("camera"' in page and 'href="#camcard"' in page
    cam_state = state()["camera"]
    assert cam_state["want"] is True and cam_state["state"] == "down"          # no recorder running in this test
    assert post(b"recording=false", "application/x-www-form-urlencoded")[0] == 415   # all a cross-site form can send
    assert post(b'{"recording": "no"}')[0] == 400
    assert not cam.control_path.exists()
    code, body = post(b'{"recording": false}')
    assert code == 200 and body["ok"] and body["camera"]["want"] is False
    assert json.loads(cam.control_path.read_text())["src"] == "the web app (127.0.0.1)"
    assert state()["camera"]["want"] is False                                   # the next update already shows it
    code, body = post(b'{"recording": true}')
    assert code == 200 and body["camera"]["want"] is True


def test_the_button_stops_and_starts_a_running_recorder(cam, fake_ffmpeg, web):
    _, post, state = web
    rec = C.Recorder(cam, ffmpeg=fake_ffmpeg)
    try:
        assert _drive(rec, lambda: rec.state == "recording")
        assert state()["camera"]["state"] == "recording"
        assert post(b'{"recording": false}')[0] == 200
        assert _drive(rec, lambda: rec.state == "off")
        assert _drive(rec, lambda: state()["camera"]["state"] == "off")
        assert post(b'{"recording": true}')[0] == 200
        assert _drive(rec, lambda: state()["camera"]["state"] == "recording")
    finally:
        rec.shutdown()


def test_state_has_no_camera_without_a_camera_config(cfg):
    srv = W.WebServer(("127.0.0.1", 0), W.LiveState(cfg), live=False)
    assert json.loads(srv.snapshot_bytes())["camera"] is None
    srv.server_close()

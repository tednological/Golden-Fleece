"""Field web app: what it shows comes from l03..l09 replayed over the recording plus the pipeline's own
decisions; targets on the rider's left are drawn on the left; it tails a live session line by line; the HTTP
endpoints serve the page, valid JSON and an event stream."""
import json
import threading
import time
import urllib.request

import pytest

from tools import web_app as W
from tools.sim.run import run_scenario
from tools.sim.scenarios import BY_NAME


@pytest.fixture(scope="module")
def overtake_recording(tmp_path_factory, cfg):
    p = tmp_path_factory.mktemp("rec") / "session_overtake.jsonl"
    run_scenario(BY_NAME["overtake_left_10mps"](), cfg, record_path=str(p))
    return p


def _feed(cfg, path, every=25):
    st = W.LiveState(cfg)
    st.reset(path.name)
    snaps = []
    with open(path) as f:
        for i, line in enumerate(f):
            st.feed(json.loads(line))
            if i % every == 0:
                snaps.append(st.snapshot(None, live=False))
    snaps.append(st.snapshot(None, live=False))
    return st, snaps


def _wait(pred, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.02)
    return False


def test_car_overtaking_on_the_left_is_drawn_on_the_left(cfg, overtake_recording):
    st, snaps = _feed(cfg, overtake_recording)
    assert st.n_errors == 0
    close = [t for s in snaps for t in s["targets"] if t.get("cls") == "APPROACHING" and t["r"] < 15]
    assert close and sum(t["y"] < 0 and t["side"] == "LEFT" for t in close) >= 0.9 * len(close)
    assert any(tk["level"] >= 2 and tk["y"] < 0 for s in snaps for tk in s["tracks"])
    decisions = [s["decision"] for s in snaps if s["decision"]]
    assert max(d["lvl"] for d in decisions) >= 2 and any(d["side"] == "LEFT" for d in decisions)
    for s in snaps:
        json.dumps(s, allow_nan=False)                     # the page's JSON.parse would reject NaN


def test_imu_readout(cfg, overtake_recording):
    _, snaps = _feed(cfg, overtake_recording)
    imu = snaps[-1]["imu"]
    assert imu["gyro_hz"] > 100 and 8.5 < imu["accel_norm"] < 11.0
    assert imu["up_axis"] == "+Z" and imu["health"] == "OK"
    assert snaps[-1]["imu_hist"] and snaps[-1]["imu_hist"][-1][0] <= 0.0


def test_follower_tails_the_newest_session_line_by_line(cfg, overtake_recording, tmp_path):
    lines = overtake_recording.read_bytes().splitlines(keepends=True)
    live = tmp_path / "session_live.jsonl"
    live.write_bytes(b"".join(lines[:300]))
    st = W.LiveState(cfg)
    fol = W.Follower(st, rec_dir=tmp_path)
    fol.start()
    try:
        assert _wait(lambda: st.n_records == 300)
        with open(live, "ab") as f:
            f.write(b"".join(lines[300:600]) + lines[600][:10])    # a record cut mid-line, as the writer leaves it
        assert _wait(lambda: st.n_records == 600)
        time.sleep(0.2)
        assert st.n_records == 600 and st.n_errors == 0            # the partial line was held back, not misparsed
        with open(live, "ab") as f:
            f.write(lines[600][10:])
        assert _wait(lambda: st.n_records == 601) and st.session == "session_live.jsonl"
    finally:
        fol.stop_evt.set()
        fol.join(2.0)


def test_http_serves_page_state_and_events(cfg, overtake_recording):
    st, _ = _feed(cfg, overtake_recording, every=10**9)
    srv = W.WebServer(("127.0.0.1", 0), st, live=False)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"
    try:
        page = urllib.request.urlopen(base + "/", timeout=5).read().decode()
        assert "Golden Fleece" in page and "EventSource" in page
        page3d = urllib.request.urlopen(base + "/3d", timeout=5).read().decode()
        assert "three" in page3d and 'EventSource("events")' in page3d
        state = json.loads(urllib.request.urlopen(base + "/state", timeout=5).read())
        assert state["session"] == overtake_recording.name and "imu" in state and "targets" in state
        with urllib.request.urlopen(base + "/events", timeout=5) as r:
            first = r.readline().decode()
        assert first.startswith("data: ") and "imu" in json.loads(first[len("data: "):])
        with pytest.raises(urllib.error.HTTPError):
            urllib.request.urlopen(base + "/nope", timeout=5)
    finally:
        srv.shutdown()
        srv.server_close()

"""Task §11 invariants that are checked structurally or by grep."""
import dataclasses
import re
from pathlib import Path

import pytest

from goldenfleece import types as T

PKG = Path(__file__).resolve().parent.parent / "goldenfleece"
TOOLS = Path(__file__).resolve().parent.parent / "tools"
DOCS = Path(__file__).resolve().parent.parent / "docs"
CONFIG = Path(__file__).resolve().parent.parent / "config"

THREAD_ALLOWED = {
    "l10_mcu_link/link.py",            # the transport half of l10
    "orchestrator/recording.py",       # the recording writer
    "orchestrator/power_monitor.py",   # power-flag adapter thread
}
THREAD_ALLOWED_DIRS = ("l01_radar_data_input/", "l02_imu_data_input/")   # adapter layers


def _py_files(root: Path):
    return sorted(p for p in root.rglob("*.py"))


def _rel(p: Path) -> str:
    return str(p.relative_to(PKG)).replace("\\", "/")


def test_inv3_no_z_or_elevation_value_on_radar_types():
    radar_types = [T.RawRadarTarget, T.RawRadarFrame, T.RadarDetection, T.RadarFrame, T.ClassifiedDetection,
                   T.ClutterOutput, T.Track, T.TrackerOutput, T.ThreatAssessment]
    for tp in radar_types:
        for f in dataclasses.fields(tp):
            assert f.name != "z", f"{tp.__name__}.{f.name}"
            assert not re.fullmatch(r"elev(ation)?|el|z_[a-z]*", f.name), f"{tp.__name__}.{f.name}"
    names = {f.name for f in dataclasses.fields(T.RadarDetection)}
    assert {"elev_min", "elev_max"} <= names   # the interval is carried; no value


def test_inv4_no_third_party_kld7_library():
    for p in _py_files(PKG) + _py_files(TOOLS):
        src = p.read_text()
        assert not re.search(r"^\s*(import|from)\s+kld7\b", src, re.M), p


def test_inv4_single_radar_conversion_site():
    """Degrees->radians of the wire angle and km/h->m/s happen only in l03."""
    for p in _py_files(PKG):
        rel = _rel(p)
        if rel.startswith("l03_radar_decode/"):
            continue
        src = p.read_text()
        assert "/ 3.6" not in src and "/3.6" not in src, rel
        assert "angle_raw / 100" not in src and "angle_raw/100" not in src, rel
        assert "distance_cm / 100" not in src, rel


def test_inv5_time_only_in_clock():
    for p in _py_files(PKG):
        rel = _rel(p)
        if rel == "clock.py":
            continue
        src = p.read_text()
        assert not re.search(r"^\s*(import|from)\s+time\b", src, re.M), rel
        assert not re.search(r"^\s*(import|from)\s+datetime\b", src, re.M), rel
        assert not re.search(r"\btime\.(time|monotonic|sleep|perf_counter)\(", src), rel


def test_inv5_threading_only_in_adapters():
    for p in _py_files(PKG):
        rel = _rel(p)
        src = p.read_text()
        uses = re.search(r"^\s*(import|from)\s+(threading|queue|concurrent|asyncio|multiprocessing)\b", src, re.M)
        if uses:
            assert rel in THREAD_ALLOWED or rel.startswith(THREAD_ALLOWED_DIRS), f"{rel} uses {uses.group(2)} but is not an adapter"
    for rel in THREAD_ALLOWED:
        pass  # files may not exist yet in early stages


LANE_RE = re.compile(r"\blanes?\b|lane[-_ ]change|lane[-_]?level", re.I)
NEGATION_RE = re.compile(r"\b(not|no|never|cannot|can't|unsupportable|no lane|without|nothing)\b", re.I)


def test_inv7_no_lane_semantics_in_code():
    for p in _py_files(PKG) + _py_files(TOOLS):
        for i, line in enumerate(p.read_text().splitlines(), 1):
            if LANE_RE.search(line):
                assert NEGATION_RE.search(line), f"{p}:{i}: lane vocabulary without negation: {line.strip()}"


def test_inv7_no_lane_semantics_in_docs_and_config():
    for p in list(DOCS.glob("*.md")) + list(CONFIG.glob("*.yaml")):
        for i, line in enumerate(p.read_text().splitlines(), 1):
            if LANE_RE.search(line):
                assert NEGATION_RE.search(line), f"{p.name}:{i}: lane vocabulary without negation: {line.strip()}"


def test_inv12_no_compensating_negations_outside_l03():
    """The azimuth flip lives in l03 only.  No other pipeline module negates a radians() call or angle_raw."""
    for p in _py_files(PKG):
        rel = _rel(p)
        if rel.startswith("l03_radar_decode/") or rel == "types.py":
            continue
        src = p.read_text()
        assert not re.search(r"-\s*(math\.|np\.)?radians\(", src), rel
        assert not re.search(r"-\s*\w*\.?angle_raw", src), rel
        if not rel.startswith("l01_radar_data_input/"):
            assert "angle_raw" not in src, f"{rel}: angle_raw must not escape l01/l03"


def test_inv3_no_world_frames_in_package():
    for p in _py_files(PKG):
        if _rel(p) == "config.py":      # config.py is the guard that rejects these names
            continue
        src = p.read_text()
        for name in ("\"world\"", "'world'", "\"odom\"", "'odom'", "\"bike\"", "'bike'"):
            assert name not in src, f"{_rel(p)} mentions frame {name}"


def test_inv6_heartbeat_built_only_by_the_pipeline_loop():
    """No independent heartbeat timer: send_heartbeat is called from the orchestrator loop only, and the
    link module owns no Timer."""
    callers = []
    for p in _py_files(PKG):
        rel = _rel(p)
        src = p.read_text()
        if "send_heartbeat(" in src and "def send_heartbeat" not in src:
            callers.append(rel)
        if rel.startswith("l10_mcu_link/"):
            assert "Timer(" not in src and "sched" not in src, rel
    assert set(callers) == {"orchestrator/runner.py"}, callers
    runner = (PKG / "orchestrator" / "runner.py").read_text()
    assert "WATCHDOG" not in runner.split("def _watchdog")[0] or "notifier.watchdog()" in runner   # watchdog only via _watchdog on progress

"""tools/config_overlay.py: overrides land where they should, typos fail, invalid configs are rejected, and the
three preset scripts build configs the pipeline accepts."""
import os
import re
import subprocess
from pathlib import Path

import pytest

from goldenfleece.config import load_config
from tools import config_overlay as O

ROOT = Path(__file__).resolve().parent.parent


def test_overrides_are_applied_and_typed(tmp_path):
    changes = O.build(ROOT / "config", tmp_path / "cfg",
                      ["radar.params.RRAI=3", "radar.params.THOF=20", "pipeline.clutter.doppler_blind_band_mps=0.6"])
    cfg = load_config(tmp_path / "cfg")
    assert cfg.radar.params["RRAI"] == 3 and cfg.radar.params["THOF"] == 20
    assert cfg.pipeline.clutter.doppler_blind_band_mps == pytest.approx(0.6)
    assert cfg.radar.params["RSPI"] == load_config(ROOT / "config").radar.params["RSPI"]     # untouched values kept
    assert any("RRAI: 2 -> 3" in c for c in changes)


@pytest.mark.parametrize("bad", ["radar.params.RRAII=3", "radar.params=3", "nofile.x=1", "radar.params.RRAI"])
def test_typos_and_malformed_overrides_fail(tmp_path, bad):
    with pytest.raises(O.OverlayError):
        O.build(ROOT / "config", tmp_path / "cfg", [bad])


def test_values_the_loader_rejects_fail(tmp_path):
    assert O.main([str(tmp_path / "cfg"), "radar.params.RRAI=7"]) == 2


@pytest.mark.parametrize("script,expect", [
    ("run_preset_a.sh", {"RRAI": 3, "RSPI": 3, "THOF": 30, "DEDI": 2, "MISP": 0, "MASP": 100}),
    ("run_preset_b.sh", {"RRAI": 3, "RSPI": 3, "THOF": 20, "DEDI": 2, "MISP": 2, "MASP": 100}),
    ("run_preset_c.sh", {"RRAI": 3, "RSPI": 3, "THOF": 20, "DEDI": 1, "MISP": 2, "MASP": 100}),
])
def test_preset_scripts_build_valid_configs(script, expect):
    env = {**os.environ, "PRESET_DRY_RUN": "1"}
    r = subprocess.run(["bash", str(ROOT / script)], capture_output=True, text=True, env=env, timeout=60)
    assert r.returncode == 0, r.stderr
    name = re.search(r"preset (\w+):", r.stdout).group(1)
    cfg = load_config(Path(f"/tmp/goldenfleece_preset_{name}"))
    assert {k: cfg.radar.params[k] for k in expect} == expect
    blind = cfg.pipeline.clutter.doppler_blind_band_mps
    assert blind >= expect["MISP"] * 100 / 3.6 / 100 - 1e-9          # MISP % of 100 km/h, in m/s
    assert "RRAI 3" in r.stdout and "dry run" in r.stdout


@pytest.mark.parametrize("args,thof", [(["--thof", "15"], 15), (["--thof=10"], 10), (["--thof", "060"], 60), ([], 20)])
def test_thof_override(args, thof):
    env = {**os.environ, "PRESET_DRY_RUN": "1"}
    r = subprocess.run(["bash", str(ROOT / "run_preset_b.sh"), *args, "--no-imu"], capture_output=True, text=True, env=env, timeout=60)
    assert r.returncode == 0, r.stderr
    cfg = load_config(Path("/tmp/goldenfleece_preset_B"))
    assert cfg.radar.params["THOF"] == thof
    assert cfg.radar.params["MISP"] == 2 and cfg.radar.params["RRAI"] == 3          # the rest of preset B is kept
    assert ("THOF override" in r.stdout) == bool(args)


@pytest.mark.parametrize("args", [["--thof", "9"], ["--thof", "61"], ["--thof", "abc"], ["--thof", "15.5"], ["--thof"]])
def test_thof_override_rejects_bad_values(args):
    env = {**os.environ, "PRESET_DRY_RUN": "1"}
    r = subprocess.run(["bash", str(ROOT / "run_preset_a.sh"), *args], capture_output=True, text=True, env=env, timeout=60)
    assert r.returncode == 2 and "--thof" in r.stderr

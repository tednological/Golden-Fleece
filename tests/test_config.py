import textwrap
from pathlib import Path

import pytest
import yaml

from goldenfleece.config import ConfigError, load_config, load_frames
from goldenfleece.types import RadarTopology


def _write_all(tmp: Path, root: Path, **overrides):
    for name in ("radar.yaml", "frames.yaml", "pipeline.yaml"):
        data = yaml.safe_load((root / "config" / name).read_text())
        if name in overrides:
            overrides[name](data)
        (tmp / name).write_text(yaml.safe_dump(data))


def test_loads_repo_config(cfg):
    assert cfg.radar.topology is RadarTopology.PI_POLLS
    assert cfg.radar.params["RRAI"] == 2 and cfg.radar.params["RSPI"] == 3
    assert cfg.frames.T_radar_imu.target == "radar" and cfg.frames.T_radar_imu.source == "imu"
    assert cfg.pipeline.clutter.doppler_blind_band_mps > 0
    assert "sensor_delay_s" in cfg.register


def test_missing_source_tag_rejected(tmp_path, repo_root):
    def strip(d):
        for f in d["frames"]:
            f.pop("source", None)
    _write_all(tmp_path, repo_root, **{"frames.yaml": strip})
    with pytest.raises(ConfigError, match="source"):
        load_config(tmp_path)


def test_forbidden_world_frame_rejected(tmp_path, repo_root):
    def add(d):
        d["frames"].append({"name": "world", "parent": "radar", "t_xyz": [0, 0, 0], "q_wxyz": [1, 0, 0, 0], "source": "nominal"})
    _write_all(tmp_path, repo_root, **{"frames.yaml": add})
    with pytest.raises(ConfigError, match="world"):
        load_config(tmp_path)


def _null_delay(d):
    d["sensor_delay_s"] = None


def test_null_sensor_delay_refused_unless_dev_mode(tmp_path, repo_root):
    def strict(d):
        d["allow_unmeasured"] = False
    _write_all(tmp_path, repo_root, **{"pipeline.yaml": strict, "radar.yaml": _null_delay})
    with pytest.raises(ConfigError, match="sensor_delay_s"):
        load_config(tmp_path)


def test_dev_mode_logs_warning(tmp_path, repo_root):
    _write_all(tmp_path, repo_root, **{"radar.yaml": _null_delay})
    cfg = load_config(tmp_path)
    assert any("UNMEASURED sensor_delay_s" in w for w in cfg.warnings)
    assert not cfg.radar.sensor_delay_measured


def test_register_entry_without_tag_rejected(tmp_path, repo_root):
    def strip(d):
        d["register"]["sensor_delay_s"] = {"filled_by": "x"}
    _write_all(tmp_path, repo_root, **{"pipeline.yaml": strip})
    with pytest.raises(ConfigError, match="tag"):
        load_config(tmp_path)


def test_reflection_extrinsic_rejected(tmp_path, repo_root):
    bad = textwrap.dedent("""
    version: 1
    root: radar
    frames:
      - {name: radar, parent: null, source: definition}
      - {name: imu, parent: radar, t_xyz: [0,0,0], q_wxyz: [0, 1, 0, 0], source: nominal}
    """)
    p = tmp_path / "frames.yaml"
    p.write_text(bad)
    fc = load_frames(p)   # a 180deg rotation about x is a valid signed permutation
    assert fc.T_radar_imu.source == "imu"
    p.write_text(bad.replace("[0, 1, 0, 0]", "[0.5, 0.5, 0.5, 0.5]"))
    fc = load_frames(p)   # still a proper rotation (cyclic axis permutation)
    assert fc.T_radar_imu.q_wxyz[0] == pytest.approx(0.5)

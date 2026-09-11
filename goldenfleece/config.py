"""Configuration loading and validation.

Validation rules (task §12):
  * every extrinsic carries a ``source`` tag (definition|nominal|measured|calibrated);
  * every register entry carries a tag; required register values may not be null
    unless ``allow_unmeasured`` is true, in which case every start logs loudly;
  * rotations are orthonormal with det +1; T_radar_imu is expected to be a signed permutation.
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional

import numpy as np
import yaml

from . import frames as fr
from .types import RadarTopology, SourceTag, Transform

log = logging.getLogger("goldenfleece.config")


class ConfigError(ValueError):
    pass


class Section:
    """Read-only dotted access to a YAML mapping; missing keys raise ConfigError."""

    def __init__(self, data: Mapping[str, Any], path: str = "") -> None:
        object.__setattr__(self, "_data", dict(data))
        object.__setattr__(self, "_path", path)

    def __getattr__(self, name: str) -> Any:
        d = object.__getattribute__(self, "_data")
        if name not in d:
            raise ConfigError(f"missing config key '{object.__getattribute__(self, '_path')}.{name}'")
        v = d[name]
        if isinstance(v, Mapping):
            return Section(v, f"{object.__getattribute__(self, '_path')}.{name}")
        return v

    def __setattr__(self, name: str, value: Any) -> None:
        raise AttributeError("config sections are read-only")

    def get(self, name: str, default: Any = None) -> Any:
        return object.__getattribute__(self, "_data").get(name, default)

    def as_dict(self) -> Dict[str, Any]:
        return dict(object.__getattribute__(self, "_data"))

    def __contains__(self, name: str) -> bool:
        return name in object.__getattribute__(self, "_data")


@dataclass(frozen=True)
class RadarConfig:
    topology: RadarTopology
    port: str
    baudrate: int
    baud_probe_order: List[int]
    params: Dict[str, int]
    frame_duration_s: Dict[int, float]
    frame_duration_source: SourceTag
    sensor_delay_s: float
    sensor_delay_measured: bool
    usb_latency_s: float
    az_clip_rad: float
    elev_min_rad: float
    elev_max_rad: float
    silent_after_frames: int
    resp_timeout_s: float
    reconnect_backoff_s: List[float]
    max_targets: int


@dataclass(frozen=True)
class FramesConfig:
    root: str
    transforms: Dict[str, Transform]      # keyed by child frame name; T_parent_child
    notes: Dict[str, str]

    @property
    def T_radar_imu(self) -> Transform:
        return self.transforms["imu"]


@dataclass(frozen=True)
class Config:
    radar: RadarConfig
    frames: FramesConfig
    pipeline: Section
    allow_unmeasured: bool
    register: Dict[str, Dict[str, Any]]
    warnings: List[str] = field(default_factory=list)


_VALID_TAGS = {t.value for t in SourceTag}


def _load_yaml(path: Path) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ConfigError(f"{path}: top level must be a mapping")
    return data


def load_frames(path: Path) -> FramesConfig:
    data = _load_yaml(path)
    root = data.get("root")
    if not root:
        raise ConfigError("frames.yaml: 'root' is required")
    transforms: Dict[str, Transform] = {}
    notes: Dict[str, str] = {}
    seen = set()
    for entry in data.get("frames", []):
        name = entry.get("name")
        if not name:
            raise ConfigError("frames.yaml: every frame needs a name")
        if name in seen:
            raise ConfigError(f"frames.yaml: duplicate frame '{name}'")
        seen.add(name)
        if "source" not in entry:
            raise ConfigError(f"frames.yaml: frame '{name}' has no 'source' tag")
        if entry["source"] not in _VALID_TAGS or entry["source"] == "unvalidated":
            raise ConfigError(f"frames.yaml: frame '{name}' has invalid source tag '{entry['source']}'")
        notes[name] = str(entry.get("note", ""))
        parent = entry.get("parent")
        if parent is None:
            if name != root:
                raise ConfigError(f"frames.yaml: '{name}' has no parent but is not the root")
            continue
        if parent not in seen:
            raise ConfigError(f"frames.yaml: parent '{parent}' of '{name}' must be declared first")
        q = entry.get("q_wxyz")
        t = entry.get("t_xyz")
        if q is None or t is None or len(q) != 4 or len(t) != 3:
            raise ConfigError(f"frames.yaml: frame '{name}' needs q_wxyz[4] and t_xyz[3]")
        T = fr.make_transform(parent, name, q, t, SourceTag(entry["source"]))
        fr.validate_rotation(fr.rotation(T))
        transforms[name] = T
    if "imu" not in transforms:
        raise ConfigError("frames.yaml: an 'imu' frame under 'radar' is required")
    for forbidden in ("world", "odom", "bike", "map"):
        if forbidden in seen:
            raise ConfigError(f"frames.yaml: frame '{forbidden}' is not allowed (warnings are rider-relative)")
    return FramesConfig(root=root, transforms=transforms, notes=notes)


def load_radar(path: Path, allow_unmeasured: bool, warnings: List[str]) -> RadarConfig:
    d = _load_yaml(path)
    for k in ("port", "baudrate", "params", "frame_duration_s", "frame_duration_source"):
        if k not in d:
            raise ConfigError(f"radar.yaml: '{k}' is required")
    params = {str(k): int(v) for k, v in d["params"].items()}
    for k in ("RRAI", "RSPI"):
        if k not in params:
            raise ConfigError(f"radar.yaml: params.{k} is required")
    if not 0 <= params["RRAI"] <= 3 or not 0 <= params["RSPI"] <= 3:
        raise ConfigError("radar.yaml: RRAI and RSPI must be 0..3")
    fd = {int(k): float(v) for k, v in d["frame_duration_s"].items()}
    if set(fd) != {0, 1, 2, 3}:
        raise ConfigError("radar.yaml: frame_duration_s needs entries for RSPI 0..3")
    if d["frame_duration_source"] not in _VALID_TAGS:
        raise ConfigError("radar.yaml: frame_duration_source tag invalid")
    sd = d.get("sensor_delay_s")
    if sd is None:
        if not allow_unmeasured:
            raise ConfigError("radar.yaml: sensor_delay_s is null and allow_unmeasured is false; "
                              "measure it with tools/kld7_probe.py")
        sd = float(d.get("sensor_delay_placeholder_s", 0.0))
        warnings.append(f"UNMEASURED sensor_delay_s: using placeholder {sd*1e3:.1f} ms")
        measured = False
    else:
        measured = True
    return RadarConfig(
        topology=RadarTopology(d.get("topology", "PI_POLLS")),
        port=str(d["port"]),
        baudrate=int(d["baudrate"]),
        baud_probe_order=[int(b) for b in d.get("baud_probe_order", [115200, 460800, 921600, 2000000, 3000000])],
        params=params,
        frame_duration_s=fd,
        frame_duration_source=SourceTag(d["frame_duration_source"]),
        sensor_delay_s=float(sd),
        sensor_delay_measured=measured,
        usb_latency_s=float(d.get("usb_latency_s", 0.0)),
        az_clip_rad=float(d.get("az_clip_rad", math.radians(40))),
        elev_min_rad=float(d.get("elev_min_rad", math.radians(-17.0))),
        elev_max_rad=float(d.get("elev_max_rad", math.radians(17))),
        silent_after_frames=int(d.get("silent_after_frames", 5)),
        resp_timeout_s=float(d.get("resp_timeout_s", 0.2)),
        reconnect_backoff_s=[float(x) for x in d.get("reconnect_backoff_s", [0.25, 0.5, 1, 2, 4])],
        max_targets=int(d.get("max_targets", 12)),
    )


def load_config(config_dir: Path | str = "config") -> Config:
    config_dir = Path(config_dir)
    pdata = _load_yaml(config_dir / "pipeline.yaml")
    allow_unmeasured = bool(pdata.get("allow_unmeasured", False))
    warnings: List[str] = []
    register = pdata.get("register", {})
    if not isinstance(register, dict) or not register:
        raise ConfigError("pipeline.yaml: 'register' section is required")
    for name, entry in register.items():
        if not isinstance(entry, dict) or "tag" not in entry:
            raise ConfigError(f"pipeline.yaml: register entry '{name}' needs a tag")
        if entry["tag"] not in _VALID_TAGS:
            raise ConfigError(f"pipeline.yaml: register entry '{name}' has invalid tag '{entry['tag']}'")
    radar = load_radar(config_dir / "radar.yaml", allow_unmeasured, warnings)
    frames = load_frames(config_dir / "frames.yaml")
    pipeline = Section(pdata, "pipeline")
    for sec in ("imu", "ego", "clutter", "tracker", "threat", "policy", "blockage", "link", "orchestrator", "recording", "power", "radar_health"):
        if sec not in pdata:
            raise ConfigError(f"pipeline.yaml: section '{sec}' is required")
    if not fr.is_signed_permutation(fr.rotation(frames.T_radar_imu)):
        warnings.append("T_radar_imu is not a signed permutation; the IMU is expected to be axis-aligned")
    unval = [n for n, e in register.items() if e["tag"] in ("unvalidated", "nominal")]
    if unval:
        warnings.append("register entries not yet measured/validated: " + ", ".join(unval))
    cfg = Config(radar=radar, frames=frames, pipeline=pipeline, allow_unmeasured=allow_unmeasured,
                 register=register, warnings=warnings)
    return cfg


def log_config_warnings(cfg: Config) -> None:
    """Loud on every start (task §12.3)."""
    if cfg.allow_unmeasured:
        log.warning("allow_unmeasured=true: DEV MODE, running with placeholder/unvalidated parameters")
    for w in cfg.warnings:
        log.warning("CONFIG: %s", w)

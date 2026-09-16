"""Build a config directory from config/ plus overrides, and validate it with the real loader.

    .venv/bin/python tools/config_overlay.py OUT_DIR radar.params.RRAI=3 pipeline.clutter.doppler_blind_band_mps=0.6

Each override is FILE.dotted.key=value, where FILE is radar, pipeline or frames and value is parsed as YAML
(3 -> int, 0.6 -> float, true -> bool).  The key must already exist: a typo fails instead of adding a setting
nothing reads.  The written config is loaded with goldenfleece.config.load_config before this exits 0, so a
preset that would not start the pipeline is rejected here.  Comments are not preserved in the copy.
"""
from __future__ import annotations

import shutil
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from goldenfleece.config import ConfigError, load_config  # noqa: E402

FILES = ("radar", "pipeline", "frames")


class OverlayError(ValueError):
    pass


def parse_override(text: str) -> Tuple[str, List[str], Any]:
    if "=" not in text:
        raise OverlayError(f"'{text}': expected FILE.key=value")
    path, raw = text.split("=", 1)
    parts = path.split(".")
    if len(parts) < 2 or parts[0] not in FILES:
        raise OverlayError(f"'{text}': must start with one of {', '.join(f + '.' for f in FILES)}")
    return parts[0], parts[1:], yaml.safe_load(raw)


def apply(data: Dict[str, Any], keys: List[str], value: Any, label: str) -> Any:
    node: Any = data
    for i, k in enumerate(keys):
        if not isinstance(node, dict) or k not in node:
            raise OverlayError(f"{label}: no existing key '{'.'.join(keys[:i + 1])}'")
        if i == len(keys) - 1:
            old = node[k]
            if isinstance(old, dict):
                raise OverlayError(f"{label}: '{'.'.join(keys)}' is a section, not a value")
            node[k] = value
            return old
        node = node[k]
    return None


def build(src: Path, out: Path, overrides: List[str]) -> List[str]:
    parsed = [parse_override(o) for o in overrides]
    docs = {f: yaml.safe_load((src / f"{f}.yaml").read_text()) for f in FILES}
    changes = []
    for (f, keys, value), text in zip(parsed, overrides):
        old = apply(docs[f], keys, value, text)
        changes.append(f"{f}.{'.'.join(keys)}: {old!r} -> {value!r}")
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)
    for f in FILES:
        (out / f"{f}.yaml").write_text(yaml.safe_dump(docs[f], sort_keys=False))
    load_config(out)
    return changes


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if not argv or argv[0] in ("-h", "--help"):
        print(__doc__)
        return 0 if argv else 2
    out, overrides = Path(argv[0]), argv[1:]
    try:
        for line in build(ROOT / "config", out, overrides):
            print("  " + line)
    except (OverlayError, ConfigError) as e:
        print(f"config_overlay: {e}", file=sys.stderr)
        return 2
    print(f"  config written to {out} and validated")
    return 0


if __name__ == "__main__":
    sys.exit(main())

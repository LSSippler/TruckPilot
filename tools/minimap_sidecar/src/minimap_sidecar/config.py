"""Read [vision.minimap] from truckpilot.toml using stdlib tomllib."""

from __future__ import annotations

import sys
import tomllib
from dataclasses import dataclass
from pathlib import Path

_THIS_FILE = Path(__file__).resolve()
_REPO_ROOT = _THIS_FILE.parents[4]  # tools/minimap_sidecar/src/minimap_sidecar/
DEFAULT_TOML = _REPO_ROOT / "truckpilot.toml"


@dataclass
class MinimapConfig:
    x: int
    y: int
    w: int
    h: int
    hsv_lower: tuple[int, int, int]
    hsv_upper: tuple[int, int, int]


def load_config(toml_path: Path = DEFAULT_TOML) -> MinimapConfig:
    """Load [vision.minimap] from truckpilot.toml. Raises on missing section."""
    if not toml_path.exists():
        raise FileNotFoundError(
            f"truckpilot.toml not found at {toml_path}.\n"
            "Run `python -m minimap_calibration` first to create [vision.minimap]."
        )

    with toml_path.open("rb") as f:
        data = tomllib.load(f)

    vision = data.get("vision", {})
    mm = vision.get("minimap")
    if mm is None:
        raise KeyError(
            "[vision.minimap] section missing from truckpilot.toml.\n"
            "Run `python -m minimap_calibration` first."
        )

    def _hsv(key: str) -> tuple[int, int, int]:
        v = mm[key]
        if not isinstance(v, list) or len(v) != 3:
            raise ValueError(f"[vision.minimap].{key} must be a list of 3 integers")
        return (int(v[0]), int(v[1]), int(v[2]))

    return MinimapConfig(
        x=int(mm["x"]),
        y=int(mm["y"]),
        w=int(mm["w"]),
        h=int(mm["h"]),
        hsv_lower=_hsv("hsv_lower"),
        hsv_upper=_hsv("hsv_upper"),
    )

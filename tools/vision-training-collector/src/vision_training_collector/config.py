"""Config loader for vision-training-collector."""

from __future__ import annotations

import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

if sys.version_info >= (3, 11):
    import tomllib
else:  # pragma: no cover
    import tomli as tomllib  # type: ignore[import-not-found]


@dataclass
class YouTubeCfg:
    search_terms: list[str] = field(default_factory=list)
    channels: list[str] = field(default_factory=list)
    playlists: list[str] = field(default_factory=list)
    min_duration_seconds: int = 300
    target_resolution: str = "1080p"
    daily_quota_gb: float = 5.0
    max_results_per_search: int = 25
    # Optional browser whose cookie jar yt-dlp should use to bypass YouTube's
    # "Sign in to confirm you're not a bot" wall. One of: firefox, chrome, edge,
    # brave, opera, vivaldi, safari, chromium. None = don't pass cookies.
    cookies_browser: str | None = None


@dataclass
class CaptureCfg:
    ets2_window_title: str = "Euro Truck Simulator 2"
    hotkey_save: str = "F8"
    hotkey_quit: str = "F9"
    auto_interval_seconds: float = 2.0


@dataclass
class ExtractCfg:
    frame_interval_seconds: float = 2.0
    skip_black_threshold: float = 0.05
    skip_menu_color_count: int = 50
    target_width: int = 1920
    target_height: int = 1080


@dataclass
class DedupeCfg:
    phash_threshold: int = 5


@dataclass
class ExportCfg:
    train_ratio: float = 0.80
    val_ratio: float = 0.15
    test_ratio: float = 0.05
    seed: int = 42


@dataclass
class SceneHintCfg:
    highway_horizontal_ratio: float = 0.55
    city_edge_density: float = 0.10


@dataclass
class Config:
    youtube: YouTubeCfg = field(default_factory=YouTubeCfg)
    capture: CaptureCfg = field(default_factory=CaptureCfg)
    extract: ExtractCfg = field(default_factory=ExtractCfg)
    dedupe: DedupeCfg = field(default_factory=DedupeCfg)
    export: ExportCfg = field(default_factory=ExportCfg)
    scene_hint: SceneHintCfg = field(default_factory=SceneHintCfg)

    @classmethod
    def load(cls, path: Path) -> "Config":
        data: dict[str, Any] = {}
        if path.exists():
            with path.open("rb") as fh:
                data = tomllib.load(fh)
        return cls(
            youtube=YouTubeCfg(**data.get("youtube", {})),
            capture=CaptureCfg(**data.get("capture", {})),
            extract=ExtractCfg(**data.get("extract", {})),
            dedupe=DedupeCfg(**data.get("dedupe", {})),
            export=ExportCfg(**data.get("export", {})),
            scene_hint=SceneHintCfg(**data.get("scene_hint", {})),
        )

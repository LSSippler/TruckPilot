"""YouTube scraper via yt-dlp."""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any

from .config import Config
from .state import State

log = logging.getLogger(__name__)

_MUSIC_KEYWORDS = ("music video", "official music", "soundtrack", "lyrics", "remix")


def _yt_dlp():
    try:
        import yt_dlp  # type: ignore[import-not-found]
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError("yt-dlp is not installed. Run `pip install yt-dlp`.") from exc
    return yt_dlp


def _video_info_ok(info: dict[str, Any], min_duration: int) -> tuple[bool, str]:
    duration = info.get("duration") or 0
    if duration < min_duration:
        return False, f"too short ({duration}s)"
    title = (info.get("title") or "").lower()
    if any(k in title for k in _MUSIC_KEYWORDS):
        return False, "music-video keyword in title"
    height = info.get("height") or 0
    if height and height < 1080:
        return False, f"resolution too low ({height}p)"
    return True, "ok"


def _write_metadata(target: Path, info: dict[str, Any]) -> None:
    meta = {
        "url": info.get("webpage_url"),
        "id": info.get("id"),
        "title": info.get("title"),
        "channel": info.get("channel") or info.get("uploader"),
        "duration_seconds": info.get("duration"),
        "resolution": f"{info.get('width', '?')}x{info.get('height', '?')}",
        "upload_date": info.get("upload_date"),
        "download_date": info.get("_download_date"),
    }
    target.write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")


def _bytes_to_gb(b: int) -> float:
    return b / (1024 ** 3)


def _dir_size_bytes(p: Path) -> int:
    if not p.exists():
        return 0
    return sum(f.stat().st_size for f in p.rglob("*") if f.is_file())


def scrape(cfg: Config, raw_dir: Path, state_path: Path, dry_run: bool = False) -> dict[str, int]:
    """Download videos matching the configured search terms / channels / playlists."""
    raw_dir.mkdir(parents=True, exist_ok=True)
    state = State(state_path)
    yt_dlp = _yt_dlp()

    quota_bytes = int(cfg.youtube.daily_quota_gb * (1024 ** 3))
    downloaded_today = state.get("youtube_bytes_today", 0)

    targets: list[str] = []
    for term in cfg.youtube.search_terms:
        targets.append(f"ytsearch{cfg.youtube.max_results_per_search}:{term}")
    targets.extend(cfg.youtube.channels)
    targets.extend(cfg.youtube.playlists)

    stats = {"considered": 0, "downloaded": 0, "skipped": 0, "errors": 0}

    ydl_opts_probe: dict[str, Any] = {
        "quiet": True,
        "skip_download": True,
        "extract_flat": "in_playlist",
        "noplaylist": False,
    }

    for target in targets:
        log.info("scraping target: %s", target)
        try:
            with yt_dlp.YoutubeDL(ydl_opts_probe) as ydl:
                probe = ydl.extract_info(target, download=False)
        except Exception as exc:  # noqa: BLE001
            log.warning("probe failed for %s: %s", target, exc)
            stats["errors"] += 1
            continue

        entries = probe.get("entries") or [probe]
        for entry in entries:
            if not entry:
                continue
            stats["considered"] += 1
            video_id = entry.get("id")
            if not video_id:
                continue
            if state.has("youtube_downloaded_ids", video_id):
                stats["skipped"] += 1
                continue
            if downloaded_today >= quota_bytes:
                log.info("daily quota reached (%.2f GB)", _bytes_to_gb(downloaded_today))
                state.set("youtube_bytes_today", downloaded_today)
                return stats

            video_url = entry.get("url") or entry.get("webpage_url") or f"https://www.youtube.com/watch?v={video_id}"

            ydl_opts_info: dict[str, Any] = {"quiet": True, "skip_download": True}
            try:
                with yt_dlp.YoutubeDL(ydl_opts_info) as ydl:
                    info = ydl.extract_info(video_url, download=False)
            except Exception as exc:  # noqa: BLE001
                log.warning("info fetch failed for %s: %s", video_id, exc)
                stats["errors"] += 1
                continue

            ok, reason = _video_info_ok(info, cfg.youtube.min_duration_seconds)
            if not ok:
                log.info("skip %s: %s", video_id, reason)
                stats["skipped"] += 1
                state.add_to_set("youtube_downloaded_ids", video_id)
                continue

            if dry_run:
                log.info("[dry-run] would download %s (%s)", video_id, info.get("title"))
                stats["downloaded"] += 1
                continue

            outtmpl = str(raw_dir / "%(id)s.%(ext)s")
            ydl_opts_dl: dict[str, Any] = {
                "outtmpl": outtmpl,
                "format": "bestvideo[height<=1080]+bestaudio/best[height<=1080]/best",
                "merge_output_format": "mp4",
                "quiet": True,
                "no_warnings": True,
                "retries": 3,
            }
            try:
                with yt_dlp.YoutubeDL(ydl_opts_dl) as ydl:
                    ydl.download([video_url])
            except Exception as exc:  # noqa: BLE001
                log.error("download failed for %s: %s", video_id, exc)
                stats["errors"] += 1
                continue

            video_file = next(raw_dir.glob(f"{video_id}.*"), None)
            if video_file is None:
                stats["errors"] += 1
                continue
            size = video_file.stat().st_size
            downloaded_today += size

            from datetime import datetime, timezone
            info["_download_date"] = datetime.now(timezone.utc).isoformat()
            _write_metadata(raw_dir / f"{video_id}.json", info)
            state.add_to_set("youtube_downloaded_ids", video_id)
            state.set("youtube_bytes_today", downloaded_today)
            stats["downloaded"] += 1
            log.info("downloaded %s (%.2f MB)", video_id, size / (1024 ** 2))

    log.info("scrape stats: %s", stats)
    return stats

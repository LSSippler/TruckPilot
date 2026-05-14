#!/usr/bin/env python3
"""
TruckPilot YouTube Downloader

Standalone-Tool fuer manuelles Video-Auswaehlen.
Schreibt Videos direkt in data/raw/ des vision-training-collectors,
sodass extract-frames danach drueberlaufen kann.

Usage:
    # Einzelnes Video
    python yt_download.py https://www.youtube.com/watch?v=VIDEO_ID

    # Mehrere Videos (Datei mit URLs, eine pro Zeile)
    python yt_download.py --file urls.txt

    # Playlist
    python yt_download.py https://www.youtube.com/playlist?list=PLAYLIST_ID

    # Ziel-Ordner ueberschreiben
    python yt_download.py URL --output C:\\path\\to\\videos
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path

import yt_dlp


DEFAULT_OUTPUT = Path(__file__).parent / "data" / "raw"


def download_url(url: str, output_dir: Path, resolution: str = "1080") -> dict:
    """Download single URL or playlist. Returns metadata dict."""
    output_dir.mkdir(parents=True, exist_ok=True)

    ydl_opts = {
        "outtmpl": str(output_dir / "%(id)s.%(ext)s"),
        "format": f"bestvideo[height<={resolution}][ext=mp4]+bestaudio[ext=m4a]/best[height<={resolution}][ext=mp4]/best",
        "merge_output_format": "mp4",
        "noplaylist": False,
        "ignoreerrors": True,
        "retries": 3,
        "fragment_retries": 3,
        "extract_flat": False,
        "writeinfojson": False,
        "quiet": False,
        "no_warnings": False,
    }

    print(f"[INFO] downloading {url} -> {output_dir}")

    with yt_dlp.YoutubeDL(ydl_opts) as ydl:
        try:
            info = ydl.extract_info(url, download=True)
        except Exception as e:
            print(f"[ERROR] {url}: {e}", file=sys.stderr)
            return {"url": url, "success": False, "error": str(e)}

    if info is None:
        return {"url": url, "success": False, "error": "no info extracted"}

    # Handle playlist (info has 'entries') vs single video
    entries = info.get("entries") if "entries" in info else [info]
    entries = [e for e in entries if e is not None]

    results = []
    for entry in entries:
        video_id = entry.get("id", "unknown")
        title = entry.get("title", "")
        duration = entry.get("duration", 0)
        channel = entry.get("channel", "")
        url_actual = entry.get("webpage_url", url)

        # Metadata-Datei pro Video
        meta_path = output_dir / f"{video_id}.metadata.json"
        meta = {
            "id": video_id,
            "title": title,
            "channel": channel,
            "duration_seconds": duration,
            "url": url_actual,
            "download_timestamp": datetime.now().isoformat(),
            "resolution_target": resolution,
        }
        meta_path.write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")

        print(f"[OK] {video_id} - {title} ({duration}s)")
        results.append(meta)

    return {"url": url, "success": True, "videos": results}


def read_url_file(path: Path) -> list[str]:
    """Read URLs from a file, one per line. Skip empty + #-comments."""
    if not path.exists():
        raise FileNotFoundError(f"URL file not found: {path}")
    urls = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        urls.append(line)
    return urls


def main() -> int:
    parser = argparse.ArgumentParser(description="TruckPilot YouTube Downloader")
    parser.add_argument("urls", nargs="*", help="YouTube URLs (video or playlist)")
    parser.add_argument("--file", type=Path, help="File containing URLs (one per line, # comments allowed)")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help=f"Output dir (default: {DEFAULT_OUTPUT})")
    parser.add_argument("--resolution", default="1080", help="Max resolution (default: 1080)")
    args = parser.parse_args()

    urls = list(args.urls)
    if args.file:
        urls.extend(read_url_file(args.file))

    if not urls:
        parser.error("No URLs provided. Pass URLs as args or use --file.")

    print(f"[INFO] {len(urls)} URL(s) to process")
    print(f"[INFO] output: {args.output.resolve()}")

    summary = {"total": len(urls), "ok": 0, "fail": 0, "videos_downloaded": 0}
    for url in urls:
        result = download_url(url, args.output, args.resolution)
        if result["success"]:
            summary["ok"] += 1
            summary["videos_downloaded"] += len(result.get("videos", []))
        else:
            summary["fail"] += 1

    print("\n=== SUMMARY ===")
    print(json.dumps(summary, indent=2))
    return 0 if summary["fail"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

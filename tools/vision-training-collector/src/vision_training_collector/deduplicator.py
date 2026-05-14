"""Perceptual-hash deduplication."""

from __future__ import annotations

import logging
import shutil
from pathlib import Path

from PIL import Image

from .config import Config

log = logging.getLogger(__name__)


def _phash(path: Path):
    import imagehash  # local import keeps tests light
    with Image.open(path) as img:
        return imagehash.phash(img)


def dedupe_dir(src_dir: Path, dst_dir: Path, cfg: Config) -> dict[str, int]:
    """Walk src_dir for images and copy only non-duplicates into dst_dir.

    Similarity is measured by pHash hamming distance <= cfg.dedupe.phash_threshold
    against the set of kept hashes.
    """
    dst_dir.mkdir(parents=True, exist_ok=True)
    exts = {".jpg", ".jpeg", ".png"}
    files = sorted([p for p in src_dir.rglob("*") if p.suffix.lower() in exts])
    log.info("dedupe scanning %d files in %s", len(files), src_dir)

    kept_hashes = []
    stats = {"input": len(files), "kept": 0, "dropped": 0, "errors": 0}
    threshold = cfg.dedupe.phash_threshold

    for f in files:
        try:
            h = _phash(f)
        except Exception as exc:  # noqa: BLE001
            log.warning("phash failed for %s: %s", f, exc)
            stats["errors"] += 1
            continue
        duplicate = any((h - prev) <= threshold for prev in kept_hashes)
        if duplicate:
            stats["dropped"] += 1
            continue
        kept_hashes.append(h)
        rel = f.relative_to(src_dir)
        target = dst_dir / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(f, target)
        stats["kept"] += 1

    log.info("dedupe stats: %s", stats)
    return stats

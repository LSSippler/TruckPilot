"""CLI: `python -m vision_pipeline_capture ...`."""

from __future__ import annotations

import logging
import sys
import time
from typing import Any

import click
from rich.console import Console

from . import DEFAULT_FPS, DEFAULT_JPEG_QUALITY, DEFAULT_SHM_NAME, configure_logging

console = Console()


@click.group()
@click.option("--verbose", is_flag=True)
def cli(verbose: bool) -> None:
    configure_logging(logging.DEBUG if verbose else logging.INFO)


@cli.command("start")
@click.option("--shm-name", default=DEFAULT_SHM_NAME, show_default=True)
@click.option("--fps", default=DEFAULT_FPS, show_default=True, type=int)
@click.option("--quality", default=DEFAULT_JPEG_QUALITY, show_default=True, type=int)
@click.option("--window-title", default="Euro Truck Simulator 2", show_default=True)
@click.option("--pause-hotkey", default="F8", show_default=True)
@click.option("--quit-hotkey", default="F9", show_default=True)
def start_cmd(
    shm_name: str,
    fps: int,
    quality: int,
    window_title: str,
    pause_hotkey: str,
    quit_hotkey: str,
) -> None:
    """Start the capture producer. F8 pauses/resumes, F9 quits."""
    from .capture import run_capture
    stats = run_capture(
        shm_name=shm_name,
        fps=fps,
        jpeg_quality=quality,
        window_title=window_title,
        pause_hotkey=pause_hotkey,
        quit_hotkey=quit_hotkey,
    )
    console.print(
        f"published={stats.frames_published} skipped={stats.frames_skipped} "
        f"avg_fps={stats.frames_published / max(time.monotonic() - stats.started_at, 1e-3):.2f}"
    )


@cli.command("stats")
@click.option("--shm-name", default=DEFAULT_SHM_NAME, show_default=True)
@click.option("--seconds", default=5, type=int, show_default=True)
def stats_cmd(shm_name: str, seconds: int) -> None:
    """Tail the producer's SHM for `seconds` and print observed rate / size."""
    if sys.platform != "win32":
        console.print("[red]stats requires Windows[/red]")
        sys.exit(1)
    import mmap
    from .shm_writer import read_header

    try:
        mm = mmap.mmap(-1, 0, tagname=shm_name)
    except OSError as exc:
        console.print(f"[red]cannot open SHM '{shm_name}': {exc}[/red]")
        sys.exit(2)

    last_seq = -1
    observed = 0
    sizes: list[int] = []
    t0 = time.monotonic()
    last_print = t0
    try:
        while time.monotonic() - t0 < seconds:
            _, _, seq, _, w, h, jpeg_size = read_header(mm)
            if seq and not (seq & 1) and seq != last_seq:
                observed += 1
                sizes.append(jpeg_size)
                last_seq = seq
                now = time.monotonic()
                if now - last_print >= 1.0:
                    fps = observed / max(now - t0, 1e-3)
                    avg = sum(sizes[-30:]) / max(len(sizes[-30:]), 1)
                    console.print(
                        f"seq={seq} frame_id={seq // 2} fps={fps:.1f} "
                        f"avg_jpeg={avg / 1024:.1f}KB last={w}x{h}"
                    )
                    last_print = now
            time.sleep(0.01)
    finally:
        mm.close()

    if not observed:
        console.print("[yellow]no committed frames observed[/yellow]")
        return
    fps = observed / max(time.monotonic() - t0, 1e-3)
    avg = sum(sizes) / len(sizes)
    console.print(
        f"frames={observed} avg_fps={fps:.2f} avg_jpeg={avg / 1024:.1f}KB "
        f"min_jpeg={min(sizes) / 1024:.1f}KB max_jpeg={max(sizes) / 1024:.1f}KB"
    )


def main(argv: list[str] | None = None) -> Any:
    return cli(argv)


if __name__ == "__main__":
    cli()

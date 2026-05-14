"""CLI entry: `python -m vision_training_collector`."""

from __future__ import annotations

import logging
from pathlib import Path

import click
from rich.console import Console
from rich.table import Table

from . import DATA_ROOT, DEFAULT_CONFIG, configure_logging
from .config import Config

console = Console()
log = logging.getLogger(__name__)


def _load(config_path: Path) -> Config:
    return Config.load(config_path)


def _state_path() -> Path:
    return DATA_ROOT / "state.json"


@click.group()
@click.option(
    "--config",
    "config_path",
    type=click.Path(path_type=Path),
    default=DEFAULT_CONFIG,
    show_default=True,
)
@click.option("--verbose", is_flag=True)
@click.pass_context
def cli(ctx: click.Context, config_path: Path, verbose: bool) -> None:
    configure_logging(logging.DEBUG if verbose else logging.INFO)
    ctx.ensure_object(dict)
    ctx.obj["config"] = _load(config_path)
    ctx.obj["config_path"] = config_path


@cli.command("scrape-youtube")
@click.option("--dry-run", is_flag=True, help="probe only, no downloads")
@click.pass_context
def scrape_youtube(ctx: click.Context, dry_run: bool) -> None:
    from .youtube_scraper import scrape
    cfg: Config = ctx.obj["config"]
    stats = scrape(cfg, DATA_ROOT / "raw", _state_path(), dry_run=dry_run)
    console.print(stats)


@cli.command("capture-live")
@click.option("--auto", is_flag=True, help="auto-save every interval seconds")
@click.pass_context
def capture_live_cmd(ctx: click.Context, auto: bool) -> None:
    from .ets2_capture import capture_live
    cfg: Config = ctx.obj["config"]
    saved = capture_live(cfg, DATA_ROOT / "raw" / "live", auto=auto)
    console.print(f"Saved {saved} frames.")


@cli.command("extract-frames")
@click.pass_context
def extract_frames_cmd(ctx: click.Context) -> None:
    from .frame_extractor import extract_all
    cfg: Config = ctx.obj["config"]
    stats = extract_all(DATA_ROOT / "raw", DATA_ROOT / "frames", cfg, _state_path())
    console.print(stats)


@cli.command("dedupe")
@click.pass_context
def dedupe_cmd(ctx: click.Context) -> None:
    from .deduplicator import dedupe_dir
    cfg: Config = ctx.obj["config"]
    stats = dedupe_dir(DATA_ROOT / "frames", DATA_ROOT / "deduped", cfg)
    console.print(stats)


@cli.command("export")
@click.pass_context
def export_cmd(ctx: click.Context) -> None:
    from .exporter import export
    cfg: Config = ctx.obj["config"]
    stats = export(DATA_ROOT / "deduped", DATA_ROOT / "final", cfg)
    console.print(stats)


@cli.command("pre-label")
@click.option("--input", "input_dir", type=click.Path(path_type=Path), required=True)
@click.option("--output", "output_dir", type=click.Path(path_type=Path), required=True)
@click.option("--model", "model_path", type=click.Path(path_type=Path), required=True)
@click.option("--mapping", "mapping_path", type=click.Path(path_type=Path), default=None)
@click.option("--conf-auto", type=float, default=None, help="override auto-accept threshold")
@click.option("--conf-review", type=float, default=None, help="override review-min threshold")
@click.option("--dry-run", is_flag=True)
def pre_label_cmd(
    input_dir: Path,
    output_dir: Path,
    model_path: Path,
    mapping_path: Path | None,
    conf_auto: float | None,
    conf_review: float | None,
    dry_run: bool,
) -> None:
    from .pre_label import ClassMapping, pre_label_directory
    mapping_file = mapping_path or (DEFAULT_CONFIG.parent / "class_mapping.yaml")
    mapping = ClassMapping.from_yaml(mapping_file)
    if conf_auto is not None:
        mapping.auto_accept = conf_auto
    if conf_review is not None:
        mapping.review_min = conf_review
    report = pre_label_directory(input_dir, output_dir, mapping, model_path, dry_run=dry_run)
    console.print(report)


@cli.command("pipeline")
@click.option("--skip-scrape", is_flag=True)
@click.option("--skip-capture", is_flag=True, default=True, help="skip live capture by default")
@click.pass_context
def pipeline_cmd(ctx: click.Context, skip_scrape: bool, skip_capture: bool) -> None:
    cfg: Config = ctx.obj["config"]
    if not skip_scrape:
        from .youtube_scraper import scrape
        scrape(cfg, DATA_ROOT / "raw", _state_path())
    if not skip_capture:
        from .ets2_capture import capture_live
        capture_live(cfg, DATA_ROOT / "raw" / "live", auto=True)
    from .frame_extractor import extract_all
    extract_all(DATA_ROOT / "raw", DATA_ROOT / "frames", cfg, _state_path())
    from .deduplicator import dedupe_dir
    dedupe_dir(DATA_ROOT / "frames", DATA_ROOT / "deduped", cfg)
    from .exporter import export
    export(DATA_ROOT / "deduped", DATA_ROOT / "final", cfg)


@cli.command("stats")
def stats_cmd() -> None:
    exts = {".jpg", ".jpeg", ".png"}
    table = Table(title="Vision-Training-Collector Stats")
    table.add_column("Stage")
    table.add_column("Items", justify="right")
    table.add_column("Bytes", justify="right")
    for label, sub in [
        ("raw videos", DATA_ROOT / "raw"),
        ("frames", DATA_ROOT / "frames"),
        ("deduped", DATA_ROOT / "deduped"),
        ("final/images", DATA_ROOT / "final" / "images"),
    ]:
        if not sub.exists():
            table.add_row(label, "0", "0")
            continue
        if "videos" in label:
            files = [p for p in sub.iterdir() if p.is_file() and p.suffix.lower() in {".mp4", ".mkv", ".webm"}]
        else:
            files = [p for p in sub.rglob("*") if p.is_file() and p.suffix.lower() in exts]
        total_bytes = sum(p.stat().st_size for p in files)
        table.add_row(label, str(len(files)), f"{total_bytes / (1024 ** 2):.1f} MB")
    console.print(table)


if __name__ == "__main__":
    cli(obj={})

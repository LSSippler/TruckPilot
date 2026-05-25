"""Entry point: python -m minimap_sidecar"""

from __future__ import annotations

import argparse
import logging
import sys
import textwrap
from pathlib import Path

from .config import DEFAULT_TOML
from . import DEFAULT_FPS, DEFAULT_SHM_NAME
from .main import run_sidecar


def main() -> None:
    parser = argparse.ArgumentParser(
        description="TruckPilot VMM-2: Minimap sidecar — route-line detection → SHM",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=textwrap.dedent("""\
            Examples:
              python -m minimap_sidecar
              python -m minimap_sidecar --fps 5 --debug
              python -m minimap_sidecar --toml /path/to/truckpilot.toml
        """),
    )
    parser.add_argument(
        "--toml", type=Path, default=DEFAULT_TOML,
        help=f"Path to truckpilot.toml (default: {DEFAULT_TOML})",
    )
    parser.add_argument(
        "--fps", type=int, default=DEFAULT_FPS,
        help=f"Capture rate in Hz (default: {DEFAULT_FPS})",
    )
    parser.add_argument(
        "--shm", default=DEFAULT_SHM_NAME,
        help=f"Named SHM region (default: {DEFAULT_SHM_NAME})",
    )
    parser.add_argument(
        "--window", default="Euro Truck Simulator 2",
        help="ETS2 window title substring",
    )
    parser.add_argument(
        "--debug", action="store_true",
        help="Show OpenCV preview window with detected points",
    )
    parser.add_argument(
        "--log-level", default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
        datefmt="%H:%M:%S",
    )

    sys.exit(run_sidecar(
        toml_path=args.toml,
        fps=args.fps,
        shm_name=args.shm,
        window_title=args.window,
        debug_preview=args.debug,
    ))


main()

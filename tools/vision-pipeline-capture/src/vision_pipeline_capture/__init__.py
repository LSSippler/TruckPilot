"""TruckPilot Phase 6.5c.1 - DXcam to SHM frame producer."""

from __future__ import annotations

import logging

__version__ = "0.1.0"

DEFAULT_SHM_NAME = "TruckPilotFrame"  # mmap on Windows resolves to Local\<name>
DEFAULT_FPS = 10
DEFAULT_JPEG_QUALITY = 85
DEFAULT_BUFFER_BYTES = 2 * 1024 * 1024 + 64  # 2 MiB payload + 64 B header
HEADER_BYTES = 64
HEADER_MAGIC = b"TPF1"  # TruckPilot Frame v1
HEADER_VERSION = 1


def configure_logging(level: int = logging.INFO) -> None:
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

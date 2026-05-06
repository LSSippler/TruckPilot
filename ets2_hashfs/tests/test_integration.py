"""Integration tests (require real .scs file)."""

from __future__ import annotations

import os
import pytest
from ets2_hashfs import HashFsReader

SCS_PATH = os.environ.get("ETS2_BASE_SCS", "")


@pytest.mark.skipif(not SCS_PATH or not os.path.isfile(SCS_PATH),
                    reason="ETS2_BASE_SCS env var not set or file not found")
class TestRealArchive:
    """Tests that require a real ETS2 .scs archive."""

    def test_opens_and_has_many_entries(self) -> None:
        with HashFsReader(SCS_PATH) as archive:
            assert archive.num_entries > 100_000, \
                f"expected >100k entries, got {archive.num_entries}"

    def test_road_sii_readable(self) -> None:
        with HashFsReader(SCS_PATH) as archive:
            data = archive.read_file("def/world/road.sii")
            assert len(data) > 100
            text = data.decode("utf-8", errors="replace")
            assert "SiiNunit" in text or "road_look" in text

    def test_map_sector_exists(self) -> None:
        with HashFsReader(SCS_PATH) as archive:
            found = False
            for x in range(-5, 6):
                for z in range(-5, 6):
                    path = f"map/europe/sec{'+' if x >= 0 else ''}{x:04}{'+' if z >= 0 else ''}{z:04}.base"
                    if archive.has_file(path):
                        data = archive.read_file(path)
                        assert len(data) > 1024, f"{path} too small: {len(data)} bytes"
                        found = True
                        break
                if found:
                    break
            assert found, "no map sector found in range ±5"

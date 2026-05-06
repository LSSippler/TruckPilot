"""ETS2 HashFS v2 archive reader.

Usage::

    from ets2_hashfs import HashFsReader

    with HashFsReader("base.scs") as archive:
        data = archive.read_file("def/world/road.sii")
        print(data[:100])
"""

from __future__ import annotations

from .reader import HashFsReader

__all__ = ["HashFsReader"]

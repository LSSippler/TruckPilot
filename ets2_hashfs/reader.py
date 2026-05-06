"""HashFS v2 archive reader.

Opens .scs files, indexes all entries, and provides file extraction
by logical path or CityHash64 hash.
"""

from __future__ import annotations

import struct
import zlib
from typing import BinaryIO, Dict, List, Optional, Tuple

from .cityhash import cityhash64
from .entry import (
    ChunkType,
    EntryTableEntry,
    MainMetadata,
    parse_entry_table,
    parse_metadata,
    read_chunk_types,
)
from .exceptions import EntryNotFound, HashFsError
from .header import HashFsHeader


class HashFsReader:
    """Reads an SCS HashFS v2 archive."""

    def __init__(self, path: str) -> None:
        self._path = path
        self._file: BinaryIO = open(path, "rb")
        self._header: HashFsHeader = HashFsHeader.read(self._file)

        # Read and decompress entry table.
        self._file.seek(self._header.entry_table_start)
        compressed_entry_table = self._file.read(self._header.entry_table_length)
        entry_table_raw = zlib.decompress(compressed_entry_table)
        self._entries: List[EntryTableEntry] = parse_entry_table(entry_table_raw)

        # Read and decompress metadata table.
        self._file.seek(self._header.metadata_table_start)
        compressed_meta = self._file.read(self._header.metadata_table_length)
        self._metadata: bytes = zlib.decompress(compressed_meta)

        # Build hash→entry index.
        self._by_hash: Dict[int, EntryTableEntry] = {}
        for e in self._entries:
            self._by_hash[e.hash] = e

    @property
    def num_entries(self) -> int:
        """Total number of entries in the archive."""
        return len(self._entries)

    @property
    def header(self) -> HashFsHeader:
        """The parsed archive header."""
        return self._header

    def read_by_hash(self, hash_val: int) -> bytes:
        """Read the file data for the given CityHash64 hash.

        Returns the decompressed file content.
        """
        entry = self._by_hash.get(hash_val)
        if entry is None:
            raise EntryNotFound(f"hash 0x{hash_val:016X}")

        return self._read_entry_data(entry)

    def _normalize_path(self, logical_path: str) -> bytes:
        """Normalize a logical path for hash lookup.

        Removes leading slash, normalizes separators, and prepends salt.
        """
        # Remove leading /
        p = logical_path.lstrip("/")
        # Normalize backslashes to forward slashes
        p = p.replace("\\", "/")
        # Salt prepending (if non-zero)
        if self._header.salt != 0:
            salt_bytes = self._header.salt.to_bytes(2, "little")
            return salt_bytes + p.encode("utf-8")
        return p.encode("utf-8")

    def read_file(self, path: str) -> bytes:
        """Read the file data for the given logical path.

        The path is normalized, then hashed with CityHash64 and looked up.
        """
        normalized = self._normalize_path(path)
        h = cityhash64(normalized)
        return self.read_by_hash(h)

    def has_file(self, path: str) -> bool:
        """Check if a file exists in the archive."""
        normalized = self._normalize_path(path)
        h = cityhash64(normalized)
        return h in self._by_hash

    def debug_hash(self, path: str) -> dict:
        """Return debug info for a path lookup.

        Useful for understanding why a path is not found.
        """
        normalized = self._normalize_path(path)
        h = cityhash64(normalized)
        found = h in self._by_hash
        entry = self._by_hash.get(h)
        info = {
            "path": path,
            "normalized": normalized,
            "normalized_repr": repr(normalized),
            "salt": self._header.salt,
            "hash": f"0x{h:016x}",
            "found": found,
        }
        if entry is not None:
            info["entry"] = {
                "metadata_index": entry.metadata_index,
                "metadata_count": entry.metadata_count,
                "flags": entry.flags,
            }
        return info

    def list_entries(self) -> List[Tuple[int, int, bool, int]]:
        """Return (hash, size, is_compressed, offset) for all entries."""
        result: list[tuple[int, int, bool, int]] = []
        for e in self._entries:
            try:
                meta = self._get_metadata(e)
                result.append((e.hash, meta.size, meta.is_compressed, meta.offset))
            except (ValueError, IndexError):
                pass
        return result

    def read_directory(self, path: str) -> List[str]:
        """Read a directory listing.

        Directory entries contain a binary listing::

            count u32 | string_lengths byte[count] | strings[]

        Strings starting with '/' are subdirectories (the '/' is stripped).
        """
        data = self.read_file(path)
        if len(data) < 4:
            return []
        count = struct.unpack_from("<I", data, 0)[0]
        if count == 0 or count > 100000:
            return []
        lengths = list(data[4:4 + count])
        strings: list[str] = []
        off = 4 + count
        for ln in lengths:
            if off + ln > len(data):
                break
            s = data[off:off + ln].decode("utf-8", errors="replace")
            if s.startswith("/"):
                s = s[1:]
            strings.append(s)
            off += ln
        return strings

    def _get_metadata(self, entry: EntryTableEntry) -> MainMetadata:
        """Get the MainMetadata for an entry, handling chunk headers."""
        # Skip the chunk-type headers (4 bytes each) to reach MainMetadata.
        meta_index = entry.metadata_index + entry.metadata_count
        return parse_metadata(self._metadata, meta_index)

    def _read_entry_data(self, entry: EntryTableEntry) -> bytes:
        """Read and decompress the file data for a single entry."""
        if entry.is_directory:
            # Directory listings use the same metadata structure.
            meta = self._get_metadata(entry)
            self._file.seek(meta.offset)
            raw = self._file.read(meta.compressed_size if meta.is_compressed and meta.compressed_size > 0 else meta.size)
            if meta.is_compressed and meta.compressed_size > 0 and meta.compressed_size != meta.size:
                return zlib.decompress(raw)
            return raw

        # Check chunk types: skip Image chunks.
        types = read_chunk_types(self._metadata, entry.metadata_index, entry.metadata_count)
        if types and types[0] == ChunkType.IMAGE:
            raise HashFsError(f"image entries not supported (hash 0x{entry.hash:016X})")

        meta = self._get_metadata(entry)
        self._file.seek(meta.offset)

        if meta.is_compressed and meta.compressed_size > 0 and meta.compressed_size != meta.size:
            raw = self._file.read(meta.compressed_size)
            return zlib.decompress(raw)
        else:
            return self._file.read(meta.size)

    def close(self) -> None:
        """Close the underlying file handle."""
        self._file.close()

    def __enter__(self) -> "HashFsReader":
        return self

    def __exit__(self, *args: object) -> None:
        self.close()

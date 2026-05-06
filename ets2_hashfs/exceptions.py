"""HashFS-specific exceptions."""

from __future__ import annotations


class HashFsError(Exception):
    """Base exception for HashFS operations."""
    pass


class UnsupportedVersion(HashFsError):
    """The archive uses a HashFS version that is not supported."""
    def __init__(self, version: int) -> None:
        super().__init__(f"unsupported HashFS version: {version}")


class EntryNotFound(HashFsError):
    """A logical path could not be found in the archive."""
    def __init__(self, path: str) -> None:
        super().__init__(f"entry not found: {path}")


class InvalidMagic(HashFsError):
    """The file does not start with the expected SCS# magic."""
    def __init__(self, found: bytes) -> None:
        super().__init__(f"invalid magic: {found!r}")

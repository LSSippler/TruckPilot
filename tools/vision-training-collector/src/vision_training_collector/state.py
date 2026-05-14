"""Resume-state tracking via state.json."""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any

log = logging.getLogger(__name__)


class State:
    def __init__(self, path: Path) -> None:
        self.path = path
        self._data: dict[str, Any] = {}
        if path.exists():
            try:
                self._data = json.loads(path.read_text(encoding="utf-8"))
            except json.JSONDecodeError:
                log.warning("state.json corrupt, starting fresh: %s", path)
                self._data = {}

    def get(self, key: str, default: Any = None) -> Any:
        return self._data.get(key, default)

    def set(self, key: str, value: Any) -> None:
        self._data[key] = value
        self.flush()

    def add_to_set(self, key: str, value: str) -> None:
        s = set(self._data.get(key, []))
        s.add(value)
        self._data[key] = sorted(s)
        self.flush()

    def has(self, key: str, value: str) -> bool:
        return value in set(self._data.get(key, []))

    def flush(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self._data, indent=2, sort_keys=True), encoding="utf-8")
        tmp.replace(self.path)

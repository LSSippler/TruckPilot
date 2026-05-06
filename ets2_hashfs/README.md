# ets2_hashfs — Python HashFS v2 Reader

Reads ETS2 `.scs` archives (HashFS v2 with CityHash64), used since ETS2 v1.50.

## Usage

```python
from ets2_hashfs import HashFsReader

with HashFsReader("base.scs") as archive:
    print(f"Entries: {archive.num_entries}")

    # Read by logical path
    data = archive.read_file("def/world/road.sii")
    print(data[:100].decode())

    # List directory
    files = archive.read_directory("/def/world")
    print(files[:10])
```

## CLI

```bash
python -m ets2_hashfs base.scs
```

Extract map sectors:

```bash
python -m ets2_hashfs extract base_map.scs --sectors --out /tmp/ets2_sectors
```

## Tests

```bash
pytest ets2_hashfs/tests/
# With real archive:
ETS2_BASE_SCS=/path/to/base.scs pytest ets2_hashfs/tests/
```

## Requirements

Python 3.9+ standard library only. No external dependencies.

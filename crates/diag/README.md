# truckpilot-diag

Diagnostic binaries for TruckPilot.

## Bins

### `truckpilot-diag`

Pre-flight checks for the daemon (telemetry, IPC, etc.). Run before a live session.

### `truckpilot-item-census`

Phase 5.25 pre-check. Quantifies Two-Node-Item volume (Terrain, Buildings,
Curve) across the production load order (base + workshop mods, last-wins)
and produces a GO/SKIP recommendation per type before any 5.25b/5.25c
edge-generation work.

```powershell
cargo run --release --bin truckpilot-item-census -- `
  --ets2-dir "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2" `
  --graph graph.json
```

Optional: `--mods-dir <PATH>` (defaults to `~/Documents/Euro Truck Simulator 2/mod`),
`--output <PATH>` (defaults to `outputs/item_census.txt`).

Output: total / both-resolved / unique-new / cross-sector counts plus
per-type GO/SKIP advice and a Curve-locator H1 sanity check. A copy is
mirrored to `outputs/claude/item_census.txt`.

The Buildings row is a hard sanity check against Phase 5.25a's published
606 items / 483 both-resolved figures — mismatch exits with code 2.

### `truckpilot-archive-audit`

Phase 5.26a Reframe-B. Per-archive parse-success / item-counts / node
connectivity / cross-archive road resolution, plus an H1/H2/H3 hypothesis
ranking for the DLC-edge-extraction bottleneck.

```powershell
cargo run --release --bin truckpilot-archive-audit -- `
  --ets2-dir "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2" `
  --graph graph.json
```

Output: `outputs/archive_audit.txt` (mirrored to `outputs/claude/`).
Caveat: in environments where ProMods (or any large map mod) is installed,
ProMods owns the majority of `.base` sector paths via last-wins and the
H1 base-vs-DLC ratio becomes statistically unreliable. Read the per-archive
tables directly for the real signal.

### `truckpilot-vis-uid-audit`

Phase 5.26a Reframe-C (H4a probe). Re-runs the Phase 5.13 vis_uids
hypothesis test against the full multi-archive load order. Reports
resolve rates, byte-pattern histogram, and a hypothetical
edge-generation gain. Phase 5.13's "disjoint namespace" verdict is
re-verified at scale (vis_uids consistently sit in the `0x4a-0x5C`
top-byte range, never `0x00XX` where Node-UIDs live).

```powershell
cargo run --release --bin truckpilot-vis-uid-audit -- `
  --ets2-dir "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2" `
  --graph graph.json
```

Output: `outputs/vis_uid_audit.txt` (mirrored to `outputs/claude/`).

### `road-drop-audit`

Phase 6.2b-Diag-3. Instruments the full parse pipeline to trace every point
where a road item is silently discarded before it can contribute an edge to the
routing graph. Two drop layers are probed:

- **Sector-level** (`parse_sector_with_tracer`): `RoadParseFailed`,
  `SectorHandlerError`, `UnknownItemType`
- **Graph-level** (`GraphBuilder::analyze_roads_for_audit`): `BothUnresolved`,
  `OneUnresolved`

```powershell
cargo run --release -p truckpilot-diag --bin road-drop-audit -- `
  --ets2-dir "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2" `
  --output-dir outputs/2026-05-17 `
  --focus-city Berlin `
  --hex-dump-limit 64
```

Outputs to `--output-dir`:
- `road_drop_audit.md` — Markdown summary with per-category counts, top-20
  sector hot-spots, auto-diagnosis text, and optional focus-city snap-node check
- `road_drop_audit.json` — all `DropEvent` records as structured JSON
- `road_drop_<City>_trace.md` — detailed per-city trace (only with `--focus-city`)

`--focus-city` accepts any city name from the built-in list
(Berlin, Hamburg, Wien, Paris, Amsterdam, Köln, Frankfurt, München, Prag, Warschau).
`--focus-sector` additionally filters sector-level events by filename substring.

The tracer infrastructure (`crates/map-parser/src/drop_tracer.rs`) has
zero overhead in the production parser path — the `None` branch is compiled
away entirely.

### `truckpilot-anchor-junction-audit`

Phase 5.26d (H4b probe). Counts how many Single-Anchor-Items
(Sign, Model, FarModel, MapOverlay, VisibilityArea, BusStop) reference
the same Node-UID. If the count is significant, those nodes are
implicit junctions hidden from the graph builder.

```powershell
cargo run --release --bin truckpilot-anchor-junction-audit -- `
  --ets2-dir "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2" `
  --graph graph.json
```

Uses `audit_sector` to walk items and direct-slices Node-UIDs at known
byte offsets per type — no cursor, no production-code touch. Output:
`outputs/h4b_anchor_junction_audit.txt` (mirrored to `outputs/claude/`).


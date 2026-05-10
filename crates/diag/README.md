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

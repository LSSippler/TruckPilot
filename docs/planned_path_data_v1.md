# PlannedPathData v1

Read-only planned path model for TruckPilot overlay, CLI, and future
lane/ACC consumers. **v1 does not drive steering, engage, or the route
resolver.**

## Why PlannedPathData

ETS2LA centralises path geometry, lane offsets, curvature, junction hints,
and safety context in a `PlannedPathData`-style structure shared by
pathfinding, LaneAssist, ACC, and overlay. TruckPilot already has map graph
edges, PrefabAiPath, NavCurves, Hermite splines, arc-length LUT, SplineIndex,
and lane-follower primitives — but no single serialisable view for debug and
downstream plugins.

PlannedPathData v1 unifies those concepts **without copying ETS2LA code** and
without reading ETS2 process memory.

## ETS2LA-inspired concepts (no code copy)

| Concept | TruckPilot v1 field |
|---------|---------------------|
| PrefabPath / junction | `PlannedPathItemKind::Junction`, `prefab_uid` |
| NavCurve | `PlannedPathItemKind::NavCurve`, `curve_index` |
| Hermite / spline samples | `PathPoint` polyline per item |
| InterpolateLane / lateral offset | `lane_offset_m`, `lateral_offset_m` |
| Path curvature | `curvature_1pm` on item and points |
| Path semaphore | `semaphore_hint` (display string only) |
| Relative path / crosstrack | `NearestPathPoint::crosstrack_m` |

## Data sources (v1)

| `PlannedPathSource` | Status |
|---------------------|--------|
| `mock` | **Active** — synthetic fixture (`build_mock_fixture_v1`) |
| `offline_graph` | Planned — graph.json route |
| `route_blackboard` | Planned — ETS2 SHM waypoints |
| `prefab_ai_path` / `navcurve` | Planned — map-parser segments |

Overlay / `truckpilot-status --overlay` currently attach **mock geometry**
plus a **live safety mirror** from SHM preflight (`route_valid`,
`resolver_safe`, `input_allowed`, …).

## Safety block

`PlannedPathSafety` mirrors preflight display fields:

- `drive_allowed_display_only` is **display only** — not engage, not
  lane-keeper, not steering.
- `reasons` lists blockers including `unknown` qualifiers from CLI preflight.

Path `valid: true` means geometry is structurally usable; safety may still
block drive display.

## Not for control in v1

- No resolver activation
- No ETS2 memory reads beyond existing SHM
- No DLL hotpath changes
- No `sendCommand`, engage, or lane-keeper gates from overlay JSON
- LaneFollower / ACC do **not** consume PlannedPathData yet

## Module locations

| Crate | Path |
|-------|------|
| Data model + fixture | `crates/plugin-api/src/planned_path.rs` |
| Overlay builder | `crates/telemetry/src/planned_path_overlay.rs` |
| Snapshot wire-up | `crates/telemetry/src/overlay_snapshot.rs` |
| UI panel | `crates/ui/src/components/overlay/PlannedPathDebugPanel.tsx` |

## CLI / JSON

```text
cargo run -p truckpilot-telemetry --bin truckpilot-status -- --overlay
```

Optional top-level field `planned_path` (mock geometry + live safety).

Example fixture export:

```text
outputs/2026-06-22/planned_path_fixture.json
```

Library helper:

```rust
use truckpilot_plugin_api::planned_path::{build_mock_fixture_v1, planned_path_to_json};
println!("{}", planned_path_to_json(&build_mock_fixture_v1()));
```

## Overlay display (v1)

When `planned_path` is present in the snapshot, `PlannedPathDebugPanel`
shows text-only stats:

- valid, source, item count
- current item id/kind
- nearest crosstrack
- curvature min/max
- junction/prefab and semaphore hint counts
- drive allowed (display only) + reason strings

No AR world-to-screen path lines in v1.

## Next steps

1. PlannedPathData from offline graph fixture (real node UIDs)
2. PrefabAiPath / junction coverage report
3. Curvature lookahead layer in overlay canvas
4. Junction debug layer on schematic map
5. LaneFollower consumes PlannedPathData (with explicit gates, later)

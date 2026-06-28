# Internal Path Visualization

Read-only top-down debug map for TruckPilot overlay snapshots. Inspired by
ETS2LA Internal Visualization — **no steering, no engage, no control gates**.

## What it shows

- **PlannedPathData** segments (road, junction, lane change, nav curve) as colored polylines
- **LaneDebugSnapshot** center/left/right lines (dashed when lane model invalid)
- Graph **node IDs** as yellow dots with labels
- **Truck marker** at nearest projection or path start
- Crosstrack / heading error text when `nearest` is present
- `safety.drive_allowed_display_only` as text only (not authorization)

## Data sources

Uses existing `OverlaySnapshot` JSON only:

- `snapshot.planned_path` (optional)
- `snapshot.lane`
- `snapshot.status` (no speed field today — shows `—`)

No daemon connection, no ETS2 memory reads, no new SHM layouts.

## Activation

```
/overlay?overlay_snapshot=fixture&overlay_visualization=internal
```

### Layout editor (panel positions + map zoom)

Press **F8** in the overlay to toggle layout editor mode (or open with
`&overlay_editor=1` for dev). While active:

- **Drag** any panel by its amber handle to reposition (saved in `localStorage`)
- **Internal visualization**: mouse wheel or **+ / −** adjusts map zoom
- **Reset** restores default layout
- **F8** again exits editor mode (overlay becomes click-through again in Tauri)

Positions persist across sessions via `truckpilot.overlay_layout_v1`.

Without `overlay_visualization=internal` the overlay stays unchanged (lane debug canvas only when snapshot mode active).

Paste workflow still works:

```
cargo run -p truckpilot-telemetry --bin truckpilot-status -- --overlay
→ localStorage → /overlay?overlay_snapshot=storage&overlay_visualization=internal
```

## Colors

| Kind | Color |
|------|-------|
| road_edge (current) | light green |
| road_edge | gray |
| junction / prefab | yellow / orange |
| lane_change | magenta |
| nav_curve | cyan |
| lane center | cyan overlay |
| lane edges | orange |

MOCK sources show a **MOCK** badge; `offline_graph` shows **OFFLINE**. Invalid lane model → dashed lines.

## Why not steering

Visualization is a debug lens on PlannedPathData v1. LaneAssist/ACC must not
consume this React component — future control plugins read structured data from
the daemon/graph layer with explicit gates, not overlay URL flags.

## Next steps

1. ~~Fill PlannedPath from offline graph fixture (real node UIDs)~~ ✓
2. Richer junction/prefab coverage labels
3. Curvature heat along polylines
4. Optional full-screen dev route (not in-game overlay)
5. Lane-Keeper / ACC integration much later

## Files

| File | Role |
|------|------|
| `internal-path-viz.ts` | bounds, transform, model builder |
| `InternalPathVisualization.tsx` | SVG panel |
| `overlay-snapshot.fixture.json` | compact `planned_path` for dev |

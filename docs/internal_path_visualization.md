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

### Fixture mode (offline graph dev data)

```
/overlay?overlay_snapshot=fixture&overlay_visualization=internal
```

### Live overlay mode (Tauri HUD + optional CLI paste)

```
/overlay?overlay_visualization=internal
```

On the normal overlay route the panel uses the **live feed**: it polls
`localStorage` for JSON pasted from `truckpilot-status --overlay` every 2s.
Blackboard panels (ACC, preflight, etc.) continue to update via IPC in parallel.

Explicit live flag (same behavior):

```
/overlay?overlay_snapshot=live&overlay_visualization=internal
```

When no `planned_path` is present in the live snapshot, the panel shows:

`No PlannedPathData in live snapshot`

plus a compact debug line (`lane` / `status` / `preflight` present yes/no).
This is read-only UI — no backend changes required until the daemon exposes
live `planned_path` continuously.

Storage paste workflow:

```
cargo run -p truckpilot-telemetry --bin truckpilot-status -- --overlay
→ localStorage → /overlay?overlay_snapshot=storage&overlay_visualization=internal
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

MOCK sources show a **MOCK** badge; `offline_graph` shows **OFFLINE**. Feed badges:
**FIXTURE** (embedded fixture), **STORAGE** (pasted JSON), **LIVE** (normal overlay route).
Invalid lane model → dashed lines.

## Curvature and junction stats (read-only)

The panel shows compact diagnostics derived from `planned_path.items`:

- Curvature min / max / average / |max| and L/M/H severity counts (display thresholds only)
- Item kind counts (road, junction, lane change, nav curve, prefab_uid, semaphore, curve_index)
- Current item metadata (nodes, prefab, curve index, semaphore hint, κ, length)
- High-severity segments use a warning stroke and midpoint ring — not a drive gate

Display thresholds (1/m): low &lt; 0.005, medium ≥ 0.005, high ≥ 0.01.

## Curvature heatmap (read-only)

Optional visual layer on planned-path segments (default **on**):

| Severity | Overlay |
|----------|---------|
| low | subtle green halo — kind colors stay primary |
| medium | amber wider stroke + midpoint tick |
| high | red halo + tick + midpoint ring |
| unknown | dimmed base stroke, neutral gray halo |

Legend shows low / medium / high / unknown and **display-only**. Thresholds are not ACC or lane-keeper gates — no control output, no engage side effects.

**Toggle:** In production overlay (click-through), heatmap state is shown as text only (`Heatmap: on/off`). The button toggle is available in layout editor mode (`overlay_editor=1` or **F8**). Without editor, use URL:

```
&overlay_heatmap=0   # heatmap off
&overlay_heatmap=1   # heatmap on (default)
```

When heatmap is off, segment styling falls back to the pre-heatmap severity strokes from the stats panel era.

## Why not steering

Visualization is a debug lens on PlannedPathData v1. LaneAssist/ACC must not
consume this React component — future control plugins read structured data from
the daemon/graph layer with explicit gates, not overlay URL flags.

## Next steps

1. ~~Fill PlannedPath from offline graph fixture (real node UIDs)~~ ✓
2. ~~Curvature/junction read-only stats in panel~~ ✓
3. ~~Curvature heatmap along polylines (read-only overlay)~~ ✓
4. Live overlay feed (storage poll + missing-path UI) ~~✓~~
5. Continuous live `planned_path` from daemon/telemetry (follow-up)
6. Richer junction/prefab coverage labels

## Files

| File | Role |
|------|------|
| `internal-path-viz.ts` | bounds, transform, model builder |
| `InternalPathVisualization.tsx` | SVG panel |
| `useOverlaySnapshotFeed.ts` | fixture / storage / live snapshot resolution |
| `overlay-snapshot.fixture.json` | compact `planned_path` for dev |

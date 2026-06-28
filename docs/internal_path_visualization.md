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
`localStorage` for JSON from `truckpilot-status --overlay` (paste) or from the
continuous CLI producer (`--overlay-loop`) every 2s. Blackboard panels (ACC,
preflight, etc.) continue to update via IPC in parallel.

Continuous read-only producer (CLI + Tauri file bridge):

```
cargo run -p truckpilot-telemetry --bin truckpilot-status -- --overlay-loop
```

Writes overlay JSON (including `planned_path`, `source=offline_graph`) every 2s
to `%LOCALAPPDATA%/TruckPilot/overlay_snapshot.json` (Windows) or
`~/.local/share/TruckPilot/overlay_snapshot.json`. The Tauri overlay polls this
file via `read_overlay_snapshot_file` — no manual paste required when using the
Tauri overlay window. Optional:

```
--overlay-interval-ms 2000
--overlay-out path/to/overlay_snapshot.json
```

Single-shot validation:

```
cargo run -p truckpilot-telemetry --bin truckpilot-status -- --overlay
```

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
5. Continuous read-only `planned_path` via `truckpilot-status --overlay-loop` ~~✓~~
6. Tauri file bridge into live overlay feed ~~✓~~ (manual smoke pass documented below)
7. Continuous live route `planned_path` from safe daemon/graph source
8. Richer junction/prefab coverage labels

## Manual smoke test — Tauri file bridge (2026-06-22)

Read-only verification on `dev-clean-base` after MR !18. No code changes required.

**Setup**

1. Terminal A: `cargo run -p truckpilot-telemetry --bin truckpilot-status -- --overlay-loop`
   → writes `%LOCALAPPDATA%\TruckPilot\overlay_snapshot.json` every 2s (Linux:
   `~/.local/share/TruckPilot/overlay_snapshot.json`)
2. Terminal B: `cd crates/ui && npm run tauri dev`
3. Open **Tauri TruckPilot window** (not a plain Chrome tab):
   `/overlay?overlay_visualization=internal`

**Verified chain**

| Step | Check |
|------|-------|
| CLI producer | JSON includes `planned_path`, `source=offline_graph`, 5 items |
| Tauri command | `read_overlay_snapshot_file` returns full JSON string |
| Frontend poll | `pollLiveOverlaySnapshotAsync` → `planned_path.items.length === 5` |
| Expected UI | Feed badge **LIVE** · source badge **OFFLINE**, segments + heatmap, `Drive (display): no` |

**DevTools (Tauri webview only)**

```js
const raw = await window.__TAURI_INTERNALS__.invoke("read_overlay_snapshot_file");
JSON.parse(raw).planned_path?.source; // "offline_graph"
```

Do **not** use `document.body.innerText` for badge checks — the panel renders in SVG;
use `textContent` or visual confirmation instead.

**Chrome / Vite browser tab (`localhost:1420` without Tauri)**

`window.__TAURI__` is absent; the file bridge does not run. The live feed falls back to
`localStorage` paste or an empty shell showing `No PlannedPathData in live snapshot`.
This is expected — **Tauri WebView is required** for the bridge test.

**Safety (unchanged)**

Read-only: no steering, no engage, no lane-keeper/ACC activation, no resolver activation,
no ETS2 memory reads, no SHM/DLL layout changes. Display-only overlay diagnostics.

## Files

| File | Role |
|------|------|
| `internal-path-viz.ts` | bounds, transform, model builder |
| `InternalPathVisualization.tsx` | SVG panel |
| `useOverlaySnapshotFeed.ts` | fixture / storage / live snapshot resolution |
| `overlay-snapshot-live.ts` | Tauri loop file → live feed poll + fallbacks |
| `overlay-snapshot.fixture.json` | compact `planned_path` for dev |

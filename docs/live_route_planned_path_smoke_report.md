# Live Route PlannedPath Smoke Report

Date: 2026-06-28
Branch: feature/live-route-planned-path-smoke-report
Base: dev-clean-base @ 7e4defc

## Goal

Verify that TruckPilot can produce and visualize `planned_path` from a real ETS2 route through the existing read-only live pipeline.

Expected stack:

```text
ETS2 route / SHM route snapshot
→ truckpilot-status --overlay-loop
→ %LOCALAPPDATA%/TruckPilot/overlay_snapshot.json
→ Tauri read_overlay_snapshot_file
→ useOverlaySnapshotFeed live mode
→ InternalPathVisualization
```

## Safety

This smoke test is read-only.

No steering.
No engage.
No lane-keeper activation.
No ACC activation.
No resolver activation from this test.
No pattern scanning.
No new memory reads.
No DLL hotpath changes.
No SHM layout changes.

## Setup

### Terminal 1

```powershell
cargo run -p truckpilot-telemetry --bin truckpilot-status -- --overlay-loop
```

### Terminal 2

```powershell
cd crates\ui
npm run tauri dev
```

### Tauri URL

```text
/overlay?overlay_visualization=internal
```

Optional:

```text
/overlay?overlay_visualization=internal&overlay_heatmap=0
/overlay?overlay_visualization=internal&overlay_editor=1
```

## Test Case A — No ETS2 route / fallback

### Expected

```text
planned_path.source = offline_graph
planned_path_producer.status = offline_fixture or live_route_unavailable
Internal Viz badges: LIVE · OFFLINE
Drive (display): no
```

### Observed

```text
TODO
```

### overlay_snapshot.json excerpt

```json
TODO
```

## Test Case B — ETS2 route active

Steps:

1. Start ETS2.
2. Load profile.
3. Set a route in GPS/navigation.
4. Confirm TruckPilot SHM/status can see route data.
5. Run `truckpilot-status --overlay-loop`.
6. Open Tauri overlay URL.

### Expected

```text
planned_path.source = route_blackboard
planned_path_producer.status = live_route_attached
items.length >= 1
points.length >= 2
Internal Viz badges: LIVE · ROUTE_BLACKBOARD or LIVE · LIVE
Drive (display): no
```

### Observed

```text
TODO
```

### overlay_snapshot.json excerpt

```json
TODO
```

## Route Blackboard Fields

Record what is visible:

```text
route_valid:
waypoint_count:
route_hash:
producer.status:
producer.source:
producer.reason:
planned_path.source:
planned_path.route_id:
planned_path.items:
planned_path.points:
nearest:
```

## Internal Visualization Check

Expected visual state:

```text
LIVE feed badge visible
planned_path source badge visible
segments visible
heatmap visible by default
Drive (display): no
no steering/control activation
```

Observed:

```text
TODO
```

## Problems / Findings

```text
TODO
```

## Conclusion

Choose one:

```text
PASS: route_blackboard planned_path attached and rendered.
PARTIAL: fallback works, but live route was unavailable.
FAIL: live snapshot or visualization did not work.
```

## Follow-up

Possible next steps after this report:

```text
Improve route_blackboard segment classification
Add nearest/current item from truck position
Add curvature from live route geometry
Add junction/prefab enrichment when MapGraph is available
Keep all changes read-only until gates are stable
```

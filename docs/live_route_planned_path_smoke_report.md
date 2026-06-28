# Live Route PlannedPath Smoke Report

Date: 2026-06-28
Branch: docs/live-route-planned-path-smoke-results
Base: dev-clean-base @ 3f36361 (includes producer !20 + smoke template !21)

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

Conditions: ETS2 not running, no route blackboard SHM, no telemetry SHM (`verdict: unavailable`). Stale `--overlay-loop` from pre-!20 binary was stopped; fresh `truckpilot-status --overlay` run on current `dev-clean-base` build.

### Expected

```text
planned_path.source = offline_graph
planned_path_producer.status = offline_fixture or live_route_unavailable
Internal Viz badges: LIVE · OFFLINE
Drive (display): no
```

### Observed

```text
planned_path.source = offline_graph
planned_path_producer.status = offline_fixture
planned_path_producer.source = offline_graph
planned_path_producer.reason = (absent)
planned_path.items.length = 5
planned_path.items[0].points.length = 7
status.route_valid = false
status.waypoint_count = 0
status.route_bb_available = false
status.resolve_status = unavailable
planned_path.safety.drive_allowed_display_only = false
lane_keeper_allowed = false
```

### overlay_snapshot.json excerpt

```json
{
  "planned_path": {
    "valid": true,
    "source": "offline_graph",
    "route_id": "offline-graph-mini-v1",
    "current_index": 1,
    "item_count": 5,
    "first_item": {
      "id": 1,
      "kind": "road_edge",
      "point_count": 7
    }
  },
  "planned_path_producer": {
    "status": "offline_fixture",
    "source": "offline_graph"
  },
  "status": {
    "route_valid": false,
    "waypoint_count": 0,
    "route_bb_available": false,
    "resolver_off": false,
    "resolve_status": "unavailable",
    "verdict": "unavailable"
  }
}
```

**Result:** PASS (fallback path behaves as designed on current build).

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
NOT EXECUTED — ETS2 process not running during this smoke session (2026-06-28).
No route blackboard SHM present (route_bb_available = false).
Cannot validate live_route_attached without active ETS2 route + telemetry DLL publishing waypoints.

Prior stale overlay-loop (started before producer !20 merge) showed legacy producer.status = attached
with offline_graph — replaced by stopping process and re-running current build for Case A.
```

### overlay_snapshot.json excerpt

```json
{
  "note": "Case B requires ETS2 running with GPS route and telemetry DLL active.",
  "observed_during_session": {
    "route_bb_available": false,
    "route_valid": false,
    "waypoint_count": 0,
    "planned_path.source": "offline_graph",
    "planned_path_producer.status": "offline_fixture"
  }
}
```

**Result:** BLOCKED (environment — not a code regression in Case A).

## Route Blackboard Fields

Record what is visible (Case A session; Case B not available):

```text
route_valid: false
waypoint_count: 0
route_hash: (not present — no route BB)
producer.status: offline_fixture
producer.source: offline_graph
producer.reason: (absent)
planned_path.source: offline_graph
planned_path.route_id: offline-graph-mini-v1
planned_path.items: 5
planned_path.points: 35 total across items (first item: 7 points)
nearest: present on offline fixture (display-only projection)
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
Case A JSON path verified on current build (offline_fixture + offline_graph).
Tauri Internal Viz not re-run in this session (ETS2 absent).
Prior bridge smoke (MR !18, 2026-06-22) confirmed LIVE · OFFLINE badges + 5 segments
via pollLiveOverlaySnapshotAsync → items.length = 5 in Tauri webview.
Case B visualization (LIVE · LIVE source badge for route_blackboard) pending ETS2 route session.
```

## Problems / Findings

```text
1. Long-running --overlay-loop must be restarted after merging producer !20 — old binary
   still emitted planned_path_producer.status = attached instead of offline_fixture.
2. Case B blocked: ETS2 not running; no route blackboard SHM during measurement window.
3. truckpilot-status exits 1 when SHM unavailable — expected; JSON overlay output still valid.
4. Live route attachment requires: ETS2 + telemetry DLL + valid route in SHM + resolver not
   in safe-off mode — to be verified in a follow-up session with game running.
```

## Conclusion

```text
PARTIAL: fallback works on current build (offline_fixture + offline_graph, drive display false).
Live route_blackboard attachment not verified — ETS2 route session required for Case B PASS.
```

## Follow-up

Possible next steps after this report:

```text
Re-run Case B with ETS2 running, GPS route set, telemetry DLL loaded, and resolver publishing waypoints.
Confirm planned_path.source = route_blackboard and producer.status = live_route_attached.
Capture Tauri Internal Viz screenshot / textContent check for LIVE · LIVE badges.
Improve route_blackboard segment classification once live path is confirmed.
Add nearest/current item from truck position on live polyline.
Add curvature from live route geometry.
Add junction/prefab enrichment when MapGraph is available.
Keep all changes read-only until gates are stable.
```

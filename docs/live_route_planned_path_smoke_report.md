# Live Route PlannedPath Smoke Report

Date: 2026-06-28
Branch: docs/live-route-planned-path-ets2-route-results
Base: dev-clean-base @ 1d1ee41 (includes smoke results !22)

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

Branch: `docs/live-route-planned-path-ets2-route-results` (Case-B measurement only — no code changes).

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

### Observed (2026-06-28, Case-B branch)

```text
Environment: ETS2 process not running (Get-Process eurotrucks2 → empty).
Telemetry/route SHM unavailable (verdict: unavailable).
Fresh truckpilot-status --overlay on current dev-clean-base build (post !22).
Tauri Internal Viz not opened — no live game session to validate LIVE · LIVE badges.

Diagnosis block (overlay JSON):
  planned_path_source = offline_graph
  route_id = offline-graph-mini-v1
  items = 5
  first_item_points = 7
  producer_status = offline_fixture
  producer_source = offline_graph
  producer_reason = (absent)
  route_bb_available = False
  route_valid = False
  waypoint_count = 0
  resolver_off = False
  resolve_status = unavailable
  verdict = unavailable
  drive_display = False
```

Root cause from status fields (not guessed):

```text
route_bb_available = false  → no route blackboard SHM reader input
route_valid = false         → no valid route in SHM
waypoint_count = 0          → no waypoints published
producer_reason = absent    → offline_fixture path (no live route candidate)
resolver_off = false        → N/A (SHM absent, not safe-off mode)
verdict = unavailable       → no telemetry/route SHM present
```

### overlay_snapshot.json excerpt

```json
{
  "planned_path": {
    "valid": true,
    "source": "offline_graph",
    "route_id": "offline-graph-mini-v1",
    "item_count": 5,
    "first_item_points": 7
  },
  "planned_path_producer": {
    "status": "offline_fixture",
    "source": "offline_graph"
  },
  "status": {
    "route_bb_available": false,
    "route_valid": false,
    "waypoint_count": 0,
    "resolver_off": false,
    "resolve_status": "unavailable",
    "verdict": "unavailable"
  },
  "preflight": {
    "drive_allowed_display": false
  }
}
```

**Result:** BLOCKED (environment — ETS2 + telemetry DLL + GPS route required).

### Prior attempt (!22, same day)

```text
First Case B attempt (MR !22) also blocked — ETS2 not running.
Same diagnosis fields; fallback offline_fixture confirmed stable.
```

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
Case A JSON path verified (offline_fixture + offline_graph, LIVE · OFFLINE in prior bridge smoke).
Case B Tauri check not performed — ETS2 not running; overlay-loop/Tauri dev not started for live route session.
LIVE · LIVE source badge for route_blackboard remains unverified.
```

## Problems / Findings

```text
1. Case B blocked twice (MR !22 + Case-B branch): ETS2 process absent during measurement.
2. Without route_bb_available, producer correctly stays offline_fixture — not a UI/producer bug.
3. truckpilot-status exits 1 when SHM unavailable — expected; overlay JSON still valid.
4. Next Case B attempt requires: ETS2 running, profile loaded, GPS route set, telemetry DLL active,
   fresh --overlay-loop after game is up, then Tauri /overlay?overlay_visualization=internal.
5. If route_bb_available=true but live_route_attached still fails, record producer_reason,
   resolver_off, and waypoint_count before any code changes.
```

## Conclusion

```text
PARTIAL: fallback stable (Case A PASS). Case B BLOCKED — environment (no ETS2/route SHM).
live_route_attached / route_blackboard not verified. No producer or UI fix attempted in this branch.
```

## Follow-up

```text
Re-run Case B on Windows with ETS2 + GPS route + telemetry DLL + fresh --overlay-loop.
If planned_path.source = route_blackboard and producer.status = live_route_attached,
commit results on a new docs branch (do not reuse merged MRs).
If attach fails despite route_bb_available=true, document producer_reason/resolver_off/waypoint_count
before considering producer changes.
```

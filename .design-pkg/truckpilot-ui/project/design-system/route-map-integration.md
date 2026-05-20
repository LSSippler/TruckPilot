# RouteMap — Integration Guide

## 1. Dashboard wiring

Drop `RouteMap.tsx` into `crates/ui/src/components/` and pull every prop from
the blackboard / telemetry store. The component itself does **no** subscribing
— that's intentional; one Zustand selector pass per render keeps the data path
explicit and testable.

```tsx
// crates/ui/src/routes/Dashboard.tsx (excerpt)
import { RouteMap } from "@/components/RouteMap";
import { useBlackboard } from "@/state/blackboard";
import { useTelemetry } from "@/state/telemetry";
import { useEngageState } from "@/state/engage";

export function RoutePreviewCard() {
  // Live telemetry — high frequency, but only the truck marker re-renders.
  const truckPos     = useTelemetry((s) => s.position);          // {x, z} | null
  const truckHeading = useTelemetry((s) => s.headingRad);
  const truckSpeed   = useTelemetry((s) => s.speedKmh);

  // Map subset — 1 Hz from daemon. Selectors return stable refs.
  const nearbyEdges  = useBlackboard((s) => s.get("map.nearby_edges")  ?? []);
  const nearbyNodes  = useBlackboard((s) => s.get("map.nearby_nodes")  ?? []);

  // Route — event-driven, changes on plan/replan.
  const route        = useBlackboard((s) => s.get("route.waypoints")              ?? []);
  const goal         = useBlackboard((s) => s.get("route.goal_position")          ?? null);
  const maneuverPos  = useBlackboard((s) => s.get("route.next_maneuver_position") ?? null);
  const maneuverDist = useBlackboard((s) => s.get("route.next_maneuver_distance") ?? null);
  const offRoute     = useBlackboard((s) => s.get("route.off_route")              ?? false);

  const engaged = useEngageState() === "engaged";

  return (
    <Card className="col-span-8">
      <h3 className="text-fg-muted text-xs uppercase tracking-wider mb-3">
        Route preview
      </h3>
      <RouteMap
        truckPosition={truckPos}
        truckHeading={truckHeading}
        truckSpeed={truckSpeed}
        nearbyEdges={nearbyEdges}
        nearbyNodes={nearbyNodes}
        routeWaypoints={route}
        goalPosition={goal}
        nextManeuverPosition={maneuverPos}
        nextManeuverDistance={maneuverDist}
        isEngaged={engaged}
        isOffRoute={offRoute}
        height={320}
      />
    </Card>
  );
}
```

### Selector tips

- **`nearbyEdges` must be referentially stable** between unrelated blackboard
  updates. Use a shallow-equality selector or `useShallow` from Zustand. If
  the array reference changes on every tick, `useMemo` inside the component
  rebuilds the road path on every truck-move and you lose the performance
  guarantee.
- **`truckPosition` is allowed to change every frame.** Only the outer SVG
  `<g transform>` and the truck `<g>` re-render; the heavy road/route paths
  are memoized on map-data identity.

---

## 2. Daemon-side: blackboard contract

| Key                                | Type                                  | Rate          | Producer                          |
| ---------------------------------- | ------------------------------------- | ------------- | --------------------------------- |
| `map.nearby_edges`                 | `MapEdge[]`                           | 1 Hz          | `map-publisher` plugin (new)      |
| `map.nearby_nodes`                 | `MapNode[]`                           | 1 Hz          | `map-publisher` plugin (new)      |
| `map.viewport_bbox`                | `{x1, z1, x2, z2}`                    | on truck-move | `map-publisher` plugin (new)      |
| `route.waypoints`                  | `RouteWaypoint[]`                     | on plan/replan| `router-plugin` (extended)        |
| `route.goal_position`              | `{x, z} \| null`                      | on set-goal   | `router-plugin` (extended)        |
| `route.next_maneuver_position`     | `{x, z} \| null`                      | on maneuver   | `router-plugin` (extended)        |
| `route.next_maneuver_distance`     | `f32 \| null` (meters)                | continuous    | `router-plugin` (extended)        |
| `route.off_route`                  | `bool`                                | on route-check| `router-plugin` (extended)        |

### `map-publisher` plugin (new)

Recommended over fattening `router-plugin`. Single responsibility: spatial
subset of `graph.json` around the truck.

```rust
// crates/plugins/map-publisher/src/lib.rs (sketch)
//
// Tick @ 1 Hz, or on every truck movement > 50m since last publish.
//
// 1. Read truck position from blackboard (truck.position).
// 2. Pick the right viewport radius from blackboard (map.viewport_radius_m).
//    Default 2500m — comfortably covers the 5km top-zoom level of the UI.
// 3. Query spatial index for all edges + nodes whose AABB intersects bbox.
// 4. Strip down to the wire format: only uid, position, road_type. Drop
//    bezier control points beyond the visible segment.
// 5. Publish three keys (edges, nodes, bbox) in one transaction.
//
// Spatial index: build an R-tree at daemon startup from graph.json. Trade
// 200–400 MB of RAM for O(log n + k) queries. The R-tree only needs the AABB
// of each edge, not the geometry — geometry lookup is a HashMap by uid.
```

Wire format mirrors `crates/ui/src/types/map.ts` 1:1 — same field names, same
JSON shape. Serialize with serde.

### `router-plugin` additions

Extend the existing router so it doesn't just compute the path internally but
exposes it. After a plan succeeds:

```rust
bb.set("route.waypoints", &waypoints);          // Vec<RouteWaypoint>
bb.set("route.goal_position", &goal_pos);       // Option<WorldPoint>
```

Each tick (or whenever the truck advances past the current next-maneuver
node):

```rust
let next = next_maneuver(&waypoints, &truck_pos);
bb.set("route.next_maneuver_position", &next.map(|m| m.position));
bb.set("route.next_maneuver_distance", &next.map(|m| dist(&truck_pos, &m.position)));
bb.set("route.off_route", &is_off_route(&waypoints, &truck_pos));
```

---

## 3. Implementation order (matches the brief)

### Phase 1 — Static skeleton (delivered)
- ✅ `RouteMap.tsx` with the full props interface.
- ✅ Pan (pointer drag), zoom (wheel + buttons), follow-truck toggle, debug toggle.
- ✅ All visuals via design-system tokens; no hardcoded color.
- ✅ Empty states (no telemetry / no route / off-route).
- ✅ `prefers-reduced-motion` handled via `motion-reduce:[animation:none]`.

**To test today,** wire it up with hardcoded data:
```tsx
const fakeEdges: MapEdge[] = [/* ~50 edges around (0, 0) */];
const fakeNodes: MapNode[] = [/* ~80 nodes */];
const fakeRoute: RouteWaypoint[] = [/* 10 waypoints */];
<RouteMap
  truckPosition={{ x: 0, z: 0 }}
  truckHeading={0}
  truckSpeed={62}
  nearbyEdges={fakeEdges}
  nearbyNodes={fakeNodes}
  routeWaypoints={fakeRoute}
  goalPosition={fakeRoute.at(-1) ?? null}
  nextManeuverPosition={fakeRoute[3] ?? null}
  nextManeuverDistance={180}
  isEngaged
  isOffRoute={false}
/>
```

### Phase 2 — Live data (UI side)
- Add `useTelemetry` selectors for `position`, `headingRad`, `speedKmh`.
- Subscribe to the new blackboard keys via your existing store.
- Wrap the route + map selectors with `useShallow` (or equivalent) for stable refs.
- Verify in DevTools: dragging through ETS2 streets, the road paths' DOM nodes never re-mount.

### Phase 3 — Daemon-side spatial publisher
- New `map-publisher` plugin with an R-tree over the static graph.
- Publish at 1 Hz or on movement-threshold.
- Profile: target < 4 ms per publish at the daemon's tick budget.

### Phase 4 — Route layer
- Extend `router-plugin` to publish `route.*` keys after every plan and on tick.
- Off-route detection: simple perpendicular-distance check against the path.

### Phase 5 — Polish
- Tune visual contrast in light theme (currently uses the same tokens; verify
  that `text-muted` at 30% opacity is still readable on light backgrounds —
  if not, bump to 40%).
- Add an entry to `notify.*` for "off-route" warning toast (Phase 7 of the
  main guide).

---

## 4. Performance notes

| Concern                                | Mitigation                                                                                          |
| -------------------------------------- | --------------------------------------------------------------------------------------------------- |
| Road path rebuilt on every truck tick  | `useMemo` keyed on `nearbyEdges` identity; outer `<g>` transform alone updates the camera.          |
| 500+ SVG `<circle>` debug nodes        | Debug layer is opt-in; default off.                                                                 |
| `prefers-reduced-motion`               | Both pulse animations use `motion-reduce:[animation:none]`.                                         |
| Idle (truck not moving)                | Component does no `requestAnimationFrame` and no `setInterval`. Zero CPU when nothing changes.      |
| Wheel + drag firing many React updates | State changes are coalesced by React. If you see stutter, throttle `setViewCenter` via `useEvent` + `requestAnimationFrame` — but only if measured. |
| Large `nearbyEdges`                    | Bucketed by `road_type` into one `<path>` per bucket — far fewer DOM nodes than per-edge `<line>`s. |

---

## 5. What's intentionally not in scope

- **Country borders / water / terrain** — the spec excludes them.
- **Road labels & city names** — except the goal pin, which carries no label
  in this version. Add later if requested.
- **Tile loading** — single in-memory spatial subset is enough until the data
  set exceeds a million visible edges. It won't.
- **3D perspective / tilt** — out of scope.

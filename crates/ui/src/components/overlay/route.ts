// Phase 6.6a-2 — route waypoints for the overlay polyline.
//
// `router.waypoints` is a blackboard key holding JSON [[x,z],...] in world
// metres (planned route, start→goal). It changes only on a replan, so the
// overlay reads it via the shared 500 ms blackboard poll (subscribeBlackboardKeys
// in ProjectionCanvas), not the fast pose poll.

/** Blackboard key carrying the planned route as JSON [[x,z],...] (world metres). */
export const ROUTE_BB_KEY = "router.waypoints";

/** A single route waypoint: [x, z] in world metres (no Y — flat for 6.6a-2). */
export type RoutePoint = readonly [number, number];

/** Parse the `router.waypoints` JSON string into [x,z] pairs. Returns [] for
 *  undefined / empty / malformed input, and silently drops non-numeric points,
 *  so a bad blackboard value degrades to "draw nothing" rather than crashing. */
export function parseRoute(raw: string | undefined): RoutePoint[] {
  if (!raw) return [];
  let v: unknown;
  try {
    v = JSON.parse(raw);
  } catch {
    return [];
  }
  if (!Array.isArray(v)) return [];
  const out: RoutePoint[] = [];
  for (const p of v) {
    if (
      Array.isArray(p) &&
      p.length >= 2 &&
      typeof p[0] === "number" &&
      typeof p[1] === "number" &&
      Number.isFinite(p[0]) &&
      Number.isFinite(p[1])
    ) {
      out.push([p[0], p[1]]);
    }
  }
  return out;
}

/** Contiguous slice of the route within `rangeM` (straight-line) of the truck,
 *  centred on the nearest waypoint and extending in BOTH directions. Taking a
 *  contiguous window (rather than filtering individual points) keeps the
 *  polyline connected; the near-plane clip then drops the behind-camera half,
 *  so only the part ahead of the truck is actually drawn. Robust to route
 *  ordering (start→goal vs reversed). Returns [] for routes shorter than 2. */
export function forwardWindow(
  route: RoutePoint[],
  truckX: number,
  truckZ: number,
  rangeM: number,
): RoutePoint[] {
  if (route.length < 2) return [];
  const r2 = rangeM * rangeM;
  const d2 = (p: RoutePoint) => (p[0] - truckX) ** 2 + (p[1] - truckZ) ** 2;

  let nearest = 0;
  let best = Infinity;
  for (let i = 0; i < route.length; i++) {
    const p = route[i];
    if (!p) continue;
    const d = d2(p);
    if (d < best) {
      best = d;
      nearest = i;
    }
  }

  let lo = nearest;
  for (let i = nearest - 1; i >= 0; i--) {
    const p = route[i];
    if (!p || d2(p) > r2) break;
    lo = i;
  }
  let hi = nearest;
  for (let i = nearest + 1; i < route.length; i++) {
    const p = route[i];
    if (!p || d2(p) > r2) break;
    hi = i;
  }
  return route.slice(lo, hi + 1);
}

/** Foot-point (nearest point) of [tx,tz] on the segment [ax,az]→[bx,bz],
 *  clamped to the segment endpoints. */
export function segmentFootPoint(
  ax: number, az: number,
  bx: number, bz: number,
  tx: number, tz: number,
): RoutePoint {
  const dx = bx - ax;
  const dz = bz - az;
  const len2 = dx * dx + dz * dz;
  if (len2 === 0) return [ax, az];
  const t = Math.max(0, Math.min(1, ((tx - ax) * dx + (tz - az) * dz) / len2));
  return [ax + t * dx, az + t * dz];
}

/** Interpolate intermediate points along a polyline so no two adjacent points
 *  are more than `stepM` apart. Endpoints are always preserved. */
export function densify(points: RoutePoint[], stepM: number): RoutePoint[] {
  if (points.length < 2) return [...points];
  const out: RoutePoint[] = [];
  for (let i = 0; i < points.length - 1; i++) {
    const a = points[i]!;
    const b = points[i + 1]!;
    out.push(a);
    const dx = b[0] - a[0];
    const dz = b[1] - a[1];
    const len = Math.hypot(dx, dz);
    if (len > stepM) {
      const n = Math.ceil(len / stepM);
      for (let j = 1; j < n; j++) {
        const t = j / n;
        out.push([a[0] + t * dx, a[1] + t * dz]);
      }
    }
  }
  out.push(points[points.length - 1]!);
  return out;
}

/** Interpolation step for densify: no adjacent route points further apart than this. */
const DENSIFY_STEP_M = 12;

/** Prepare the route slice for projection:
 *  1. `forwardWindow` to get the ±`rangeM` vertex window.
 *  2. Find the nearest segment foot-point in the window (clamps to first vertex
 *     when the truck is ahead of the entire route).
 *  3. Prepend the truck's world position so the polyline starts at the truck,
 *     not at the first (possibly distant) waypoint. The truck→foot-point segment
 *     is near-clipped by ProjectionCanvas, drawing from the truck's feet forward.
 *  4. Densify to ≤ DENSIFY_STEP_M per segment for a smooth visible line.
 *
 *  Lane-Keeper's Hermite/Catmull foot-point helpers are Rust-only; this 2D
 *  segment projection is implemented here independently. */
export function prepareRoute(
  route: RoutePoint[],
  truckX: number,
  truckZ: number,
  rangeM: number,
): RoutePoint[] {
  const slice = forwardWindow(route, truckX, truckZ, rangeM);
  if (slice.length === 0) return [];

  // Find the nearest segment foot-point in the slice.
  let bestDist2 = Infinity;
  let bestFoot: RoutePoint = slice[0]!;
  let bestSegIdx = 0;
  for (let i = 0; i < slice.length - 1; i++) {
    const a = slice[i]!;
    const b = slice[i + 1]!;
    const foot = segmentFootPoint(a[0], a[1], b[0], b[1], truckX, truckZ);
    const d2 = (foot[0] - truckX) ** 2 + (foot[1] - truckZ) ** 2;
    if (d2 < bestDist2) {
      bestDist2 = d2;
      bestFoot = foot;
      bestSegIdx = i;
    }
  }

  // Build polyline: truck pos → foot-point → remaining vertices.
  // When foot-point coincides with truck (≤1 m), skip it to avoid a duplicate.
  const truckPt: RoutePoint = [truckX, truckZ];
  const pts: RoutePoint[] =
    bestDist2 < 1.0
      ? [truckPt, ...slice.slice(bestSegIdx + 1)]
      : [truckPt, bestFoot, ...slice.slice(bestSegIdx + 1)];

  if (pts.length < 2) return pts;
  return densify(pts, DENSIFY_STEP_M);
}

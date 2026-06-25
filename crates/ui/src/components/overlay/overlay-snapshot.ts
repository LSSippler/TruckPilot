// Read-only overlay snapshot types + loader for `truckpilot-status --overlay` JSON.
//
// Activation (no daemon required):
//   /overlay?overlay_snapshot=fixture     → embedded fixture (MOCK lane lines)
//   /overlay?overlay_snapshot=storage     → localStorage paste from CLI output
//   localStorage key `truckpilot.overlay_snapshot_json` = full JSON string
//
// Paste workflow:
//   cargo run -p truckpilot-telemetry --bin truckpilot-status -- --overlay > snap.json
//   paste into DevTools → localStorage.setItem("truckpilot.overlay_snapshot_json", `<json>`)
//   open /overlay?overlay_snapshot=storage
//
// File import (SnapshotDebugPanel → "Import JSON"):
//   validates via parseOverlaySnapshot, writes localStorage, re-renders like storage mode.

import fixtureJson from "./overlay-snapshot.fixture.json";

export type StatusVerdict = "safe_cold" | "hot" | "unavailable";
export type LaneDataSource = "mock" | "route_blackboard";

export interface MapPoint2D {
  x: number;
  z: number;
}

export interface SplineSegment {
  start_idx: number;
  end_idx: number;
  length_m: number;
}

export interface OverlayStatusSnapshot {
  dll_active: boolean;
  perf_shm_available: boolean;
  route_bb_available: boolean;
  telemetry_shm_present: boolean;
  diag_level: string;
  resolver_off: boolean;
  resolve_status: string;
  resolver_attempts: number;
  input_disabled: boolean;
  input_enabled: boolean;
  worker_asleep: boolean;
  worker_walk_count: number;
  worker_wake_set_event_count: number;
  worker_parked_skip_count: number;
  pattern_scan_count: number;
  frame_cb_count: number;
  frame_cb_us_max: number;
  frame_cb_over_1000us: number;
  route_valid: boolean;
  waypoint_count: number;
  verdict: StatusVerdict;
  reasons?: string[];
}

export interface LaneDebugSnapshot {
  lane_model_valid: boolean;
  ego_offset_m: number;
  centerline_points: MapPoint2D[];
  left_lane_points: MapPoint2D[];
  right_lane_points: MapPoint2D[];
  curvature: number;
  lookahead_m: number;
  node_ids: number[];
  spline_segments: SplineSegment[];
  source: LaneDataSource;
  confidence: number;
}

/** Full payload from `truckpilot-status --overlay`. Read-only — never drives control. */
export interface OverlaySnapshot {
  status: OverlayStatusSnapshot;
  lane: LaneDebugSnapshot;
  /** Display-only gate indicator; must NOT enable lane keeper from the UI. */
  lane_keeper_allowed: boolean;
  verdict: StatusVerdict;
}

export const OVERLAY_SNAPSHOT_STORAGE_KEY = "truckpilot.overlay_snapshot_json";

const FIXTURE = fixtureJson as OverlaySnapshot;

function isRecord(v: unknown): v is Record<string, unknown> {
  return typeof v === "object" && v !== null && !Array.isArray(v);
}

function num(v: unknown): number | null {
  return typeof v === "number" && Number.isFinite(v) ? v : null;
}

function str(v: unknown): string | null {
  return typeof v === "string" ? v : null;
}

function parseMapPoint(v: unknown): MapPoint2D | null {
  if (!isRecord(v)) return null;
  const x = num(v.x);
  const z = num(v.z);
  if (x == null || z == null) return null;
  return { x, z };
}

function parseMapPoints(v: unknown): MapPoint2D[] | null {
  if (!Array.isArray(v)) return null;
  const out: MapPoint2D[] = [];
  for (const item of v) {
    const p = parseMapPoint(item);
    if (!p) return null;
    out.push(p);
  }
  return out;
}

function parseSplineSegments(v: unknown): SplineSegment[] | null {
  if (!Array.isArray(v)) return null;
  const out: SplineSegment[] = [];
  for (const item of v) {
    if (!isRecord(item)) return null;
    const start_idx = num(item.start_idx);
    const end_idx = num(item.end_idx);
    const length_m = num(item.length_m);
    if (start_idx == null || end_idx == null || length_m == null) return null;
    out.push({ start_idx, end_idx, length_m });
  }
  return out;
}

function parseVerdict(v: unknown): StatusVerdict | null {
  if (v === "safe_cold" || v === "hot" || v === "unavailable") return v;
  return null;
}

function parseLaneSource(v: unknown): LaneDataSource | null {
  if (v === "mock" || v === "route_blackboard") return v;
  return null;
}

/** Parse JSON from `truckpilot-status --overlay`. Returns null on malformed input. */
export function parseOverlaySnapshot(raw: string): OverlaySnapshot | null {
  if (!raw.trim()) return null;
  let parsed: unknown;
  try {
    parsed = JSON.parse(raw);
  } catch {
    return null;
  }
  if (!isRecord(parsed) || !isRecord(parsed.status) || !isRecord(parsed.lane)) {
    return null;
  }

  const statusObj = parsed.status;
  const laneObj = parsed.lane;
  const verdict = parseVerdict(parsed.verdict) ?? parseVerdict(statusObj.verdict);
  if (!verdict) return null;

  const centerline = parseMapPoints(laneObj.centerline_points);
  const left = parseMapPoints(laneObj.left_lane_points);
  const right = parseMapPoints(laneObj.right_lane_points);
  const segments = parseSplineSegments(laneObj.spline_segments);
  const source = parseLaneSource(laneObj.source);
  if (!centerline || !left || !right || !segments || !source) return null;

  const nodeIdsRaw = laneObj.node_ids;
  if (!Array.isArray(nodeIdsRaw)) return null;
  const node_ids: number[] = [];
  for (const id of nodeIdsRaw) {
    const n = num(id);
    if (n == null) return null;
    node_ids.push(n);
  }

  const lane_model_valid = laneObj.lane_model_valid === true;
  const ego_offset_m = num(laneObj.ego_offset_m) ?? 0;
  const curvature = num(laneObj.curvature) ?? 0;
  const lookahead_m = num(laneObj.lookahead_m) ?? 0;
  const confidence = num(laneObj.confidence) ?? 0;

  const statusVerdict = parseVerdict(statusObj.verdict) ?? verdict;

  return {
    verdict,
    lane_keeper_allowed: parsed.lane_keeper_allowed === true,
    status: {
      dll_active: statusObj.dll_active === true,
      perf_shm_available: statusObj.perf_shm_available === true,
      route_bb_available: statusObj.route_bb_available === true,
      telemetry_shm_present: statusObj.telemetry_shm_present === true,
      diag_level: str(statusObj.diag_level) ?? "unknown",
      resolver_off: statusObj.resolver_off === true,
      resolve_status: str(statusObj.resolve_status) ?? "unknown",
      resolver_attempts: num(statusObj.resolver_attempts) ?? 0,
      input_disabled: statusObj.input_disabled === true,
      input_enabled: statusObj.input_enabled === true,
      worker_asleep: statusObj.worker_asleep === true,
      worker_walk_count: num(statusObj.worker_walk_count) ?? 0,
      worker_wake_set_event_count: num(statusObj.worker_wake_set_event_count) ?? 0,
      worker_parked_skip_count: num(statusObj.worker_parked_skip_count) ?? 0,
      pattern_scan_count: num(statusObj.pattern_scan_count) ?? 0,
      frame_cb_count: num(statusObj.frame_cb_count) ?? 0,
      frame_cb_us_max: num(statusObj.frame_cb_us_max) ?? 0,
      frame_cb_over_1000us: num(statusObj.frame_cb_over_1000us) ?? 0,
      route_valid: statusObj.route_valid === true,
      waypoint_count: num(statusObj.waypoint_count) ?? 0,
      verdict: statusVerdict,
      reasons: Array.isArray(statusObj.reasons)
        ? statusObj.reasons.filter((r): r is string => typeof r === "string")
        : undefined,
    },
    lane: {
      lane_model_valid,
      ego_offset_m,
      centerline_points: centerline,
      left_lane_points: left,
      right_lane_points: right,
      curvature,
      lookahead_m,
      node_ids,
      spline_segments: segments,
      source,
      confidence,
    },
  };
}

/** Embedded fixture matching `truckpilot-status --overlay` without SHM. */
export function loadOverlaySnapshotFixture(): OverlaySnapshot {
  return FIXTURE;
}

/** Read a snapshot pasted into localStorage (CLI output). */
export function readOverlaySnapshotFromStorage(): OverlaySnapshot | null {
  try {
    const raw = localStorage.getItem(OVERLAY_SNAPSHOT_STORAGE_KEY);
    if (!raw) return null;
    return parseOverlaySnapshot(raw);
  } catch {
    return null;
  }
}

/** True when `/overlay?overlay_snapshot=…` enables snapshot debug UI (not `off`). */
export function isOverlaySnapshotDebugMode(search: URLSearchParams): boolean {
  const mode = search.get("overlay_snapshot");
  if (!mode || mode === "off" || mode === "0") return false;
  return true;
}

/** Validate JSON, persist raw string to localStorage, return parsed snapshot or null. */
export function saveOverlaySnapshotToStorage(raw: string): OverlaySnapshot | null {
  const trimmed = raw.trim();
  const snap = parseOverlaySnapshot(trimmed);
  if (!snap) return null;
  try {
    localStorage.setItem(OVERLAY_SNAPSHOT_STORAGE_KEY, trimmed);
  } catch {
    return null;
  }
  return snap;
}

/** Resolve snapshot source from overlay route search params. */
export function resolveOverlaySnapshot(search: URLSearchParams): OverlaySnapshot | null {
  const mode = search.get("overlay_snapshot");
  if (mode === "fixture" || mode === "mock" || mode === "1") {
    return loadOverlaySnapshotFixture();
  }
  if (mode === "storage" || mode === "local") {
    return readOverlaySnapshotFromStorage();
  }
  if (mode === "off" || mode === "0") {
    return null;
  }
  // Implicit storage when user pasted JSON without an explicit mode flag.
  return readOverlaySnapshotFromStorage();
}

/** Map world X/Z points into a schematic box for read-only lane debug drawing. */
export function mapLanePointsToSchematic(
  points: MapPoint2D[],
  width: number,
  height: number,
  padding: number,
): [number, number][] {
  if (points.length === 0 || width <= padding * 2 || height <= padding * 2) {
    return [];
  }
  let minX = Infinity;
  let maxX = -Infinity;
  let minZ = Infinity;
  let maxZ = -Infinity;
  for (const p of points) {
    minX = Math.min(minX, p.x);
    maxX = Math.max(maxX, p.x);
    minZ = Math.min(minZ, p.z);
    maxZ = Math.max(maxZ, p.z);
  }
  const spanX = Math.max(maxX - minX, 1);
  const spanZ = Math.max(maxZ - minZ, 1);
  const innerW = width - padding * 2;
  const innerH = height - padding * 2;
  const scale = Math.min(innerW / spanX, innerH / spanZ);
  const usedW = spanX * scale;
  const usedH = spanZ * scale;
  const ox = padding + (innerW - usedW) / 2;
  const oy = padding + (innerH - usedH) / 2;

  return points.map((p) => [
    ox + (p.x - minX) * scale,
    oy + (maxZ - p.z) * scale,
  ]);
}

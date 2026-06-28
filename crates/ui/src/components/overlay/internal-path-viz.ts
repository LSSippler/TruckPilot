import type {
  LaneDebugSnapshot,
  MapPoint2D,
  OverlaySnapshot,
  OverlaySnapshotFeed,
  PlannedPathData,
} from "./overlay-snapshot";

/** Horizontal map coordinate for top-down visualization (ETS2 X/Z). */
export interface VizPoint2D {
  x: number;
  z: number;
}

export interface VizBounds {
  minX: number;
  maxX: number;
  minZ: number;
  maxZ: number;
}

export interface VizViewport {
  width: number;
  height: number;
  padding: number;
  bounds: VizBounds;
  scale: number;
  offsetX: number;
  offsetY: number;
}

export type PlannedPathItemKind =
  | "road_edge"
  | "prefab_path"
  | "nav_curve"
  | "lane_change"
  | "junction"
  | "unknown";

export type VizSourceBadge = "MOCK" | "OFFLINE" | "LIVE" | "UNKNOWN" | null;

export type VizFeedBadge = "FIXTURE" | "STORAGE" | "LIVE" | null;

/** Display-only curvature bands (1/m) — not control gates. */
export const DISPLAY_CURVATURE_MEDIUM_1PM = 0.005;
export const DISPLAY_CURVATURE_HIGH_1PM = 0.01;

export type CurvatureSeverity = "low" | "medium" | "high" | "unknown";

export interface CurvatureStats {
  count: number;
  min: number | null;
  max: number | null;
  average: number | null;
  absMax: number | null;
  lowCount: number;
  mediumCount: number;
  highCount: number;
  unknownCount: number;
}

export interface PathKindStats {
  total: number;
  roadEdge: number;
  junction: number;
  laneChange: number;
  navCurve: number;
  prefabPath: number;
  prefabUidCount: number;
  semaphoreHint: number;
  withCurveIndex: number;
}

export interface CurrentItemMeta {
  id: number;
  kind: string;
  nodeRange: string | null;
  prefabUid: number | null;
  curveIndex: number | null;
  semaphoreHint: string | null;
  curvature1pm: number | null;
  lengthM: number | null;
  severity: CurvatureSeverity;
}

export interface SegmentStyle {
  stroke: string;
  width: number;
  dashed: boolean;
  severity: CurvatureSeverity;
  opacity: number;
}

/** Read-only heatmap overlay on top of kind-colored segments (display-only). */
export interface HeatmapOverlayStyle {
  stroke: string;
  width: number;
  opacity: number;
  showMidpointRing: boolean;
  showTick: boolean;
  dimBase: boolean;
}

export interface HeatmapLegendEntry {
  severity: CurvatureSeverity;
  color: string;
  label: string;
}

export const CURVATURE_HEATMAP_LEGEND: HeatmapLegendEntry[] = [
  { severity: "low", color: "rgba(134,239,172,0.55)", label: "low" },
  { severity: "medium", color: "rgba(251,191,36,0.85)", label: "medium" },
  { severity: "high", color: "rgba(248,113,113,0.95)", label: "high" },
  { severity: "unknown", color: "rgba(113,113,122,0.55)", label: "unknown" },
];

const DEFAULT_BOUNDS: VizBounds = {
  minX: -10,
  maxX: 10,
  minZ: -10,
  maxZ: 10,
};

/** True when `/overlay?overlay_visualization=internal`. */
export function isInternalPathVisualizationEnabled(
  search: URLSearchParams,
): boolean {
  return search.get("overlay_visualization") === "internal";
}

/** Read-only heatmap on/off from `overlay_heatmap` URL param (default on). */
export function resolveHeatmapEnabledFromSearch(
  search: URLSearchParams,
): boolean {
  const v = search.get("overlay_heatmap");
  if (v == null || v === "") return true;
  if (v === "0" || v === "false" || v === "off") return false;
  return true;
}

export function mapPointsToViz(points: MapPoint2D[]): VizPoint2D[] {
  return points.map((p) => ({ x: p.x, z: p.z }));
}

/** Collect all X/Z points from planned path items and optional lane debug lines. */
export function collectVisualizationPoints(
  plannedPath: PlannedPathData | undefined,
  lane: LaneDebugSnapshot | undefined,
): VizPoint2D[] {
  const out: VizPoint2D[] = [];

  if (plannedPath) {
    for (const item of plannedPath.items) {
      for (const p of item.points ?? []) {
        out.push({ x: p.x, z: p.z });
      }
    }
  }

  if (lane) {
    out.push(...mapPointsToViz(lane.centerline_points));
    out.push(...mapPointsToViz(lane.left_lane_points));
    out.push(...mapPointsToViz(lane.right_lane_points));
  }

  return out;
}

/** Compute axis-aligned bounds; empty input returns a small default box. */
export function computeBounds(points: VizPoint2D[]): VizBounds {
  if (points.length === 0) return { ...DEFAULT_BOUNDS };

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

  if (!Number.isFinite(minX)) return { ...DEFAULT_BOUNDS };

  const padX = Math.max((maxX - minX) * 0.08, 2);
  const padZ = Math.max((maxZ - minZ) * 0.08, 2);

  return {
    minX: minX - padX,
    maxX: maxX + padX,
    minZ: minZ - padZ,
    maxZ: maxZ + padZ,
  };
}

/** Build top-down viewport mapping (world X/Z → SVG coords, Z up on screen). */
export function createViewportTransform(
  bounds: VizBounds,
  width: number,
  height: number,
  padding: number,
): VizViewport {
  const innerW = Math.max(width - padding * 2, 1);
  const innerH = Math.max(height - padding * 2, 1);
  const spanX = Math.max(bounds.maxX - bounds.minX, 1);
  const spanZ = Math.max(bounds.maxZ - bounds.minZ, 1);
  const scale = Math.min(innerW / spanX, innerH / spanZ);
  const usedW = spanX * scale;
  const usedH = spanZ * scale;
  const offsetX = padding + (innerW - usedW) / 2;
  const offsetY = padding + (innerH - usedH) / 2;

  return {
    width,
    height,
    padding,
    bounds,
    scale,
    offsetX,
    offsetY,
  };
}

/** Scale map content around the viewport center (layout editor zoom). */
export function applyViewportZoom(viewport: VizViewport, zoom: number): VizViewport {
  if (!Number.isFinite(zoom) || zoom === 1) return viewport;
  const spanX = Math.max(viewport.bounds.maxX - viewport.bounds.minX, 1);
  const spanZ = Math.max(viewport.bounds.maxZ - viewport.bounds.minZ, 1);
  const scale = viewport.scale * zoom;
  const innerW = viewport.width - viewport.padding * 2;
  const innerH = viewport.height - viewport.padding * 2;
  const usedW = spanX * scale;
  const usedH = spanZ * scale;
  return {
    ...viewport,
    scale,
    offsetX: viewport.padding + (innerW - usedW) / 2,
    offsetY: viewport.padding + (innerH - usedH) / 2,
  };
}

export function mapWorldToSvg(
  p: VizPoint2D,
  vp: VizViewport,
): [number, number] {
  const x = vp.offsetX + (p.x - vp.bounds.minX) * vp.scale;
  const y = vp.offsetY + (vp.bounds.maxZ - p.z) * vp.scale;
  return [x, y];
}

export function normalizeKind(kind: string): PlannedPathItemKind {
  if (
    kind === "road_edge" ||
    kind === "prefab_path" ||
    kind === "nav_curve" ||
    kind === "lane_change" ||
    kind === "junction"
  ) {
    return kind;
  }
  return "unknown";
}

/** Collect per-item curvature values (1/m) when present. */
export function collectCurvatureValues(
  items: PlannedPathData["items"] | undefined,
): number[] {
  if (!items?.length) return [];
  return items
    .map((i) => i.curvature_1pm)
    .filter((c): c is number => c != null && Number.isFinite(c));
}

/** Display-only severity from item curvature (1/m). */
export function classifyCurvatureSeverity(
  curvature1pm: number | null | undefined,
): CurvatureSeverity {
  if (curvature1pm == null || !Number.isFinite(curvature1pm)) return "unknown";
  const abs = Math.abs(curvature1pm);
  if (abs >= DISPLAY_CURVATURE_HIGH_1PM) return "high";
  if (abs >= DISPLAY_CURVATURE_MEDIUM_1PM) return "medium";
  return "low";
}

/** Aggregate curvature diagnostics for overlay stats (read-only). */
export function computeCurvatureStats(
  items: PlannedPathData["items"] | undefined,
): CurvatureStats {
  const values = collectCurvatureValues(items);
  const stats: CurvatureStats = {
    count: values.length,
    min: null,
    max: null,
    average: null,
    absMax: null,
    lowCount: 0,
    mediumCount: 0,
    highCount: 0,
    unknownCount: 0,
  };
  for (const item of items ?? []) {
    const sev = classifyCurvatureSeverity(item.curvature_1pm);
    if (sev === "low") stats.lowCount += 1;
    else if (sev === "medium") stats.mediumCount += 1;
    else if (sev === "high") stats.highCount += 1;
    else stats.unknownCount += 1;
  }
  if (values.length === 0) return stats;
  let min = values[0]!;
  let max = values[0]!;
  let sum = 0;
  let absMax = 0;
  for (const v of values) {
    min = Math.min(min, v);
    max = Math.max(max, v);
    sum += v;
    absMax = Math.max(absMax, Math.abs(v));
  }
  stats.min = min;
  stats.max = max;
  stats.average = sum / values.length;
  stats.absMax = absMax;
  return stats;
}

/** Read-only kind / junction / prefab counters from planned path items. */
export function computePathKindStats(
  items: PlannedPathData["items"] | undefined,
): PathKindStats {
  const stats: PathKindStats = {
    total: 0,
    roadEdge: 0,
    junction: 0,
    laneChange: 0,
    navCurve: 0,
    prefabPath: 0,
    prefabUidCount: 0,
    semaphoreHint: 0,
    withCurveIndex: 0,
  };
  if (!items?.length) return stats;
  stats.total = items.length;
  for (const item of items) {
    switch (normalizeKind(item.kind)) {
      case "road_edge":
        stats.roadEdge += 1;
        break;
      case "junction":
        stats.junction += 1;
        break;
      case "lane_change":
        stats.laneChange += 1;
        break;
      case "nav_curve":
        stats.navCurve += 1;
        break;
      case "prefab_path":
        stats.prefabPath += 1;
        break;
      default:
        break;
    }
    if (item.prefab_uid != null) stats.prefabUidCount += 1;
    if (item.semaphore_hint) stats.semaphoreHint += 1;
    if (item.curve_index != null) stats.withCurveIndex += 1;
  }
  return stats;
}

export function resolveCurrentItemMeta(
  plannedPath: PlannedPathData | undefined,
): CurrentItemMeta | null {
  if (!plannedPath?.items.length) return null;
  const idx = Math.min(
    Math.max(plannedPath.current_index, 0),
    plannedPath.items.length - 1,
  );
  const item = plannedPath.items[idx];
  if (!item) return null;
  const nodeRange =
    item.node_uid_start != null && item.node_uid_end != null
      ? `${item.node_uid_start}→${item.node_uid_end}`
      : item.node_uid_start != null
        ? String(item.node_uid_start)
        : null;
  return {
    id: item.id,
    kind: item.kind,
    nodeRange,
    prefabUid: item.prefab_uid ?? null,
    curveIndex: item.curve_index ?? null,
    semaphoreHint: item.semaphore_hint ?? null,
    curvature1pm: item.curvature_1pm ?? null,
    lengthM: item.length_m ?? null,
    severity: classifyCurvatureSeverity(item.curvature_1pm),
  };
}

export function formatCurvatureStatsLine(stats: CurvatureStats): string {
  if (stats.count === 0) {
    return `Curv: none (unknown ${stats.unknownCount})`;
  }
  return (
    `Curv 1/m: min ${stats.min!.toFixed(4)} max ${stats.max!.toFixed(4)} ` +
    `avg ${stats.average!.toFixed(4)} |abs| ${stats.absMax!.toFixed(4)} ` +
    `· L${stats.lowCount} M${stats.mediumCount} H${stats.highCount}`
  );
}

export function formatPathKindStatsLine(stats: PathKindStats): string {
  if (stats.total === 0) return "Items: 0";
  return (
    `Items ${stats.total}: road ${stats.roadEdge} junc ${stats.junction} ` +
    `lc ${stats.laneChange} nav ${stats.navCurve} pref ${stats.prefabUidCount} ` +
    `sem ${stats.semaphoreHint} cidx ${stats.withCurveIndex}`
  );
}

export function formatCurrentItemMetaLine(meta: CurrentItemMeta): string {
  const parts = [`#${meta.id} ${meta.kind}`];
  if (meta.nodeRange) parts.push(`n ${meta.nodeRange}`);
  if (meta.prefabUid != null) parts.push(`pref ${meta.prefabUid}`);
  if (meta.curveIndex != null) parts.push(`c ${meta.curveIndex}`);
  if (meta.semaphoreHint) parts.push(meta.semaphoreHint);
  if (meta.curvature1pm != null) {
    parts.push(`κ ${meta.curvature1pm.toFixed(4)} (${meta.severity})`);
  } else {
    parts.push(`κ — (${meta.severity})`);
  }
  if (meta.lengthM != null) parts.push(`${meta.lengthM.toFixed(1)} m`);
  return `Current: ${parts.join(" · ")}`;
}

/** Display-only heatmap overlay for a curvature severity band. */
export function heatmapOverlayForSeverity(
  severity: CurvatureSeverity,
  heatmapEnabled: boolean,
): HeatmapOverlayStyle | null {
  if (!heatmapEnabled) return null;
  switch (severity) {
    case "low":
      return {
        stroke: "rgba(134,239,172,0.45)",
        width: 4,
        opacity: 0.35,
        showMidpointRing: false,
        showTick: false,
        dimBase: false,
      };
    case "medium":
      return {
        stroke: "rgba(251,191,36,0.85)",
        width: 6,
        opacity: 0.5,
        showMidpointRing: false,
        showTick: true,
        dimBase: false,
      };
    case "high":
      return {
        stroke: "rgba(248,113,113,0.95)",
        width: 7,
        opacity: 0.65,
        showMidpointRing: true,
        showTick: true,
        dimBase: false,
      };
    case "unknown":
    default:
      return {
        stroke: "rgba(113,113,122,0.45)",
        width: 3,
        opacity: 0.3,
        showMidpointRing: false,
        showTick: false,
        dimBase: true,
      };
  }
}

/** Segment stroke styles by PlannedPath item kind (read-only debug palette). */
export function segmentStyleForKind(
  kind: string,
  options: {
    current: boolean;
    dimmed: boolean;
    severity?: CurvatureSeverity;
    heatmapEnabled?: boolean;
  },
): SegmentStyle {
  const dashed = options.dimmed;
  const severity = options.severity ?? "unknown";
  const heatmapEnabled = options.heatmapEnabled ?? false;
  let width = options.current ? 3.5 : 2;
  let opacity = 1;

  const base: Record<PlannedPathItemKind, Omit<SegmentStyle, "dashed" | "width" | "severity" | "opacity">> = {
    road_edge: { stroke: options.current ? "#86efac" : "#6b7280" },
    prefab_path: { stroke: "#fb923c" },
    junction: { stroke: "#fbbf24" },
    lane_change: { stroke: "#f472b6" },
    nav_curve: { stroke: "#22d3ee" },
    unknown: { stroke: "#a1a1aa" },
  };

  let stroke = base[normalizeKind(kind)].stroke;
  if (!heatmapEnabled) {
    switch (severity) {
      case "high":
        width += 1;
        stroke = options.current ? "#fda4af" : "#f87171";
        break;
      case "medium":
        width += 0.5;
        break;
      case "unknown":
        stroke = "#71717a";
        break;
      case "low":
      default:
        break;
    }
  } else if (severity === "unknown") {
    opacity = 0.45;
  }

  return { stroke, width, dashed, severity, opacity };
}

export function resolveTruckWorldPoint(
  snapshot: OverlaySnapshot,
): VizPoint2D | null {
  const pp = snapshot.planned_path;
  if (pp?.nearest) {
    const item = pp.items.find((i) => i.id === pp.nearest!.item_id);
    const pts = item?.points;
    if (pts && pts.length > 0) {
      const idx = Math.min(
        pts.length - 1,
        Math.max(
          0,
          Math.round(
            (pp.nearest.distance_along_m / Math.max(item!.length_m, 1)) *
              (pts.length - 1),
          ),
        ),
      );
      const p = pts[idx];
      if (p) return { x: p.x, z: p.z };
    }
  }

  if (pp && pp.items.length > 0) {
    const cur = pp.items[pp.current_index] ?? pp.items[0];
    const p = cur?.points?.[0];
    if (p) return { x: p.x, z: p.z };
  }

  const lane = snapshot.lane;
  if (lane.centerline_points.length > 0) {
    const p = lane.centerline_points[0]!;
    return { x: p.x, z: p.z };
  }

  return null;
}

export function formatPositionLabel(p: VizPoint2D | null): string {
  if (!p) return "—";
  return `${p.x.toFixed(1)}, ${p.z.toFixed(1)}`;
}

export function formatSpeedLabel(_snapshot: OverlaySnapshot): string {
  // Overlay status snapshot has no truck speed — display-only placeholder.
  return "—";
}

export function resolveFeedBadge(feed: OverlaySnapshotFeed): VizFeedBadge {
  if (feed === "fixture") return "FIXTURE";
  if (feed === "storage") return "STORAGE";
  if (feed === "live") return "LIVE";
  return null;
}

/** Path-data source badge from planned_path.source (display-only, not inferred). */
export function resolveSourceBadge(
  plannedPath: PlannedPathData | undefined,
): VizSourceBadge {
  if (!plannedPath) return null;
  switch (plannedPath.source) {
    case "offline_graph":
      return "OFFLINE";
    case "mock":
      return "MOCK";
    case "route_blackboard":
    case "navcurve":
    case "prefab_ai_path":
      return "LIVE";
    default:
      return "UNKNOWN";
  }
}

export function formatLiveSnapshotDebugLine(snapshot: OverlaySnapshot): string {
  const lane = snapshot.lane ? "yes" : "no";
  const status = snapshot.status ? "yes" : "no";
  const preflight =
    snapshot.status != null &&
    typeof snapshot.status === "object" &&
    "preflight" in snapshot.status
      ? "yes"
      : "no";
  return `Snap: lane ${lane} · status ${status} · preflight ${preflight}`;
}

export function resolveMissingPlannedPathMessage(
  feed: OverlaySnapshotFeed,
  hasPlannedPath: boolean,
): string | null {
  if (hasPlannedPath) return null;
  if (feed === "live") return "No PlannedPathData in live snapshot";
  return "No PlannedPathData";
}

export interface InternalVizModel {
  hasPlannedPath: boolean;
  feedBadge: VizFeedBadge;
  sourceBadge: VizSourceBadge;
  missingPlannedPathMessage: string | null;
  liveDebugLine: string | null;
  dimmed: boolean;
  viewport: VizViewport;
  curvatureStats: CurvatureStats | null;
  kindStats: PathKindStats | null;
  curvatureStatsLine: string | null;
  kindStatsLine: string | null;
  currentItem: CurrentItemMeta | null;
  currentItemLine: string | null;
  heatmapEnabled: boolean;
  plannedSegments: Array<{
    id: number;
    kind: string;
    label: string;
    points: [number, number][];
    style: SegmentStyle;
    heatmap: HeatmapOverlayStyle | null;
    isCurrent: boolean;
    severity: CurvatureSeverity;
  }>;
  lanePolylines: Array<{
    role: "center" | "left" | "right";
    points: [number, number][];
  }>;
  nodeMarkers: Array<{ x: number; y: number; label: string }>;
  truck: { x: number; y: number } | null;
  nearestText: string | null;
  driveDisplay: string;
}

export function buildInternalVizModel(
  snapshot: OverlaySnapshot,
  width: number,
  height: number,
  padding = 28,
  zoom = 1,
  heatmapEnabled = true,
  feed: OverlaySnapshotFeed = "fixture",
): InternalVizModel {
  const pp = snapshot.planned_path;
  const lane = snapshot.lane;
  const points = collectVisualizationPoints(pp, lane);
  const bounds = computeBounds(points);
  const viewport = applyViewportZoom(
    createViewportTransform(bounds, width, height, padding),
    zoom,
  );

  const dimmed = !lane.lane_model_valid || (pp != null && !pp.valid);
  const feedBadge = resolveFeedBadge(feed);
  const sourceBadge = resolveSourceBadge(pp);
  const hasPlannedPath = pp != null && pp.items.length > 0;
  const missingPlannedPathMessage = resolveMissingPlannedPathMessage(feed, hasPlannedPath);
  const liveDebugLine =
    feed === "live" && !hasPlannedPath ? formatLiveSnapshotDebugLine(snapshot) : null;
  const curvatureStats = pp ? computeCurvatureStats(pp.items) : null;
  const kindStats = pp ? computePathKindStats(pp.items) : null;
  const currentItem = resolveCurrentItemMeta(pp);
  const curvatureStatsLine = curvatureStats
    ? formatCurvatureStatsLine(curvatureStats)
    : null;
  const kindStatsLine = kindStats ? formatPathKindStatsLine(kindStats) : null;
  const currentItemLine = currentItem ? formatCurrentItemMetaLine(currentItem) : null;

  const plannedSegments =
    pp?.items.map((item, idx) => {
      const worldPts = (item.points ?? []).map((p) => ({ x: p.x, z: p.z }));
      const svgPts = worldPts.map((p) => mapWorldToSvg(p, viewport));
      const isCurrent = idx === pp.current_index;
      const severity = classifyCurvatureSeverity(item.curvature_1pm);
      const labelParts = [`#${item.id}`];
      if (item.node_uid_start != null) labelParts.push(`n${item.node_uid_start}`);
      if (item.curve_index != null) labelParts.push(`c${item.curve_index}`);
      if (severity === "high") labelParts.push("H");
      return {
        id: item.id,
        kind: item.kind,
        label: labelParts.join(" "),
        points: svgPts,
        style: segmentStyleForKind(item.kind, {
          current: isCurrent,
          dimmed,
          severity,
          heatmapEnabled,
        }),
        heatmap: heatmapOverlayForSeverity(severity, heatmapEnabled),
        isCurrent,
        severity,
      };
    }) ?? [];

  const toSvgLine = (pts: MapPoint2D[]) =>
    mapPointsToViz(pts).map((p) => mapWorldToSvg(p, viewport));

  const lanePolylines: InternalVizModel["lanePolylines"] = [
    { role: "left", points: toSvgLine(lane.left_lane_points) },
    { role: "center", points: toSvgLine(lane.centerline_points) },
    { role: "right", points: toSvgLine(lane.right_lane_points) },
  ];

  const nodeMarkers: InternalVizModel["nodeMarkers"] = [];
  lane.centerline_points.forEach((p, i) => {
    const [x, y] = mapWorldToSvg({ x: p.x, z: p.z }, viewport);
    const id = lane.node_ids[i];
    nodeMarkers.push({ x, y, label: id != null ? String(id) : "" });
  });

  const truckWorld = resolveTruckWorldPoint(snapshot);
  const truck = truckWorld
    ? (() => {
        const [x, y] = mapWorldToSvg(truckWorld, viewport);
        return { x, y };
      })()
    : null;

  let nearestText: string | null = null;
  if (pp?.nearest) {
    nearestText = `ct ${pp.nearest.crosstrack_m.toFixed(2)} m · hdg ${pp.nearest.heading_error_rad.toFixed(3)} rad`;
  }

  const driveDisplay = pp?.safety.drive_allowed_display_only
    ? "yes (display)"
    : "no";

  return {
    hasPlannedPath,
    feedBadge,
    sourceBadge,
    missingPlannedPathMessage,
    liveDebugLine,
    dimmed,
    heatmapEnabled,
    viewport,
    curvatureStats,
    kindStats,
    curvatureStatsLine,
    kindStatsLine,
    currentItem,
    currentItemLine,
    plannedSegments,
    lanePolylines,
    nodeMarkers,
    truck,
    nearestText,
    driveDisplay,
  };
}

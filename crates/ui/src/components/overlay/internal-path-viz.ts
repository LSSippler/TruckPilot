import type {
  LaneDebugSnapshot,
  MapPoint2D,
  OverlaySnapshot,
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

export type VizSourceBadge = "MOCK" | "OFFLINE" | null;

export interface SegmentStyle {
  stroke: string;
  width: number;
  dashed: boolean;
}

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

/** Segment stroke styles by PlannedPath item kind (read-only debug palette). */
export function segmentStyleForKind(
  kind: string,
  options: { current: boolean; dimmed: boolean },
): SegmentStyle {
  const dashed = options.dimmed;
  const width = options.current ? 3.5 : 2;

  const base: Record<PlannedPathItemKind, Omit<SegmentStyle, "dashed" | "width">> = {
    road_edge: { stroke: options.current ? "#86efac" : "#6b7280" },
    prefab_path: { stroke: "#fb923c" },
    junction: { stroke: "#fbbf24" },
    lane_change: { stroke: "#f472b6" },
    nav_curve: { stroke: "#22d3ee" },
    unknown: { stroke: "#a1a1aa" },
  };

  const style = base[normalizeKind(kind)];
  return { ...style, width, dashed };
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

export function resolveSourceBadge(
  plannedPath: PlannedPathData | undefined,
): VizSourceBadge {
  if (plannedPath?.source === "offline_graph") return "OFFLINE";
  if (plannedPath?.source === "mock") return "MOCK";
  return null;
}

export interface InternalVizModel {
  hasPlannedPath: boolean;
  sourceBadge: VizSourceBadge;
  dimmed: boolean;
  viewport: VizViewport;
  plannedSegments: Array<{
    id: number;
    kind: string;
    label: string;
    points: [number, number][];
    style: SegmentStyle;
    isCurrent: boolean;
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
  const sourceBadge = resolveSourceBadge(pp);

  const plannedSegments =
    pp?.items.map((item, idx) => {
      const worldPts = (item.points ?? []).map((p) => ({ x: p.x, z: p.z }));
      const svgPts = worldPts.map((p) => mapWorldToSvg(p, viewport));
      const isCurrent = idx === pp.current_index;
      const labelParts = [`#${item.id}`];
      if (item.node_uid_start != null) labelParts.push(`n${item.node_uid_start}`);
      if (item.curve_index != null) labelParts.push(`c${item.curve_index}`);
      return {
        id: item.id,
        kind: item.kind,
        label: labelParts.join(" "),
        points: svgPts,
        style: segmentStyleForKind(item.kind, { current: isCurrent, dimmed }),
        isCurrent,
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
    hasPlannedPath: pp != null && pp.items.length > 0,
    sourceBadge,
    dimmed,
    viewport,
    plannedSegments,
    lanePolylines,
    nodeMarkers,
    truck,
    nearestText,
    driveDisplay,
  };
}

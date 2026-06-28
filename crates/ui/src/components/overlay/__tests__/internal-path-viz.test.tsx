import { describe, expect, it } from "vitest";
import { renderToStaticMarkup } from "react-dom/server";
import { InternalPathVisualization } from "@/components/overlay/InternalPathVisualization";
import {
  buildInternalVizModel,
  classifyCurvatureSeverity,
  collectCurvatureValues,
  collectVisualizationPoints,
  computeBounds,
  computeCurvatureStats,
  computePathKindStats,
  createViewportTransform,
  DISPLAY_CURVATURE_HIGH_1PM,
  DISPLAY_CURVATURE_MEDIUM_1PM,
  isInternalPathVisualizationEnabled,
  mapWorldToSvg,
  resolveCurrentItemMeta,
} from "@/components/overlay/internal-path-viz";
import {
  loadOverlaySnapshotFixture,
  parseOverlaySnapshot,
  type OverlaySnapshot,
} from "@/components/overlay/overlay-snapshot";

describe("internal-path-viz utilities", () => {
  it("computeBounds returns default for empty path", () => {
    const b = computeBounds([]);
    expect(b.minX).toBeLessThan(b.maxX);
    expect(b.minZ).toBeLessThan(b.maxZ);
  });

  it("collectVisualizationPoints merges planned path and lane", () => {
    const snap = loadOverlaySnapshotFixture();
    const pts = collectVisualizationPoints(snap.planned_path, snap.lane);
    expect(pts.length).toBeGreaterThan(10);
  });

  it("transform maps points into viewport", () => {
    const bounds = computeBounds([
      { x: 0, z: 0 },
      { x: 100, z: 200 },
    ]);
    const vp = createViewportTransform(bounds, 400, 300, 20);
    const [x, y] = mapWorldToSvg({ x: 0, z: 0 }, vp);
    expect(x).toBeGreaterThanOrEqual(20);
    expect(y).toBeGreaterThanOrEqual(20);
    expect(x).toBeLessThan(380);
    expect(y).toBeLessThan(280);
  });

  it("buildInternalVizModel highlights current item and offline badge", () => {
    const snap = loadOverlaySnapshotFixture();
    const model = buildInternalVizModel(snap, 420, 380);
    expect(model.hasPlannedPath).toBe(true);
    expect(model.plannedSegments.length).toBe(5);
    expect(model.plannedSegments.some((s) => s.isCurrent)).toBe(true);
    expect(model.sourceBadge).toBe("OFFLINE");
    expect(model.driveDisplay).toBe("no");
  });

  it("buildInternalVizModel respects zoom scale", () => {
    const snap = loadOverlaySnapshotFixture();
    const base = buildInternalVizModel(snap, 420, 380, 28, 1);
    const zoomed = buildInternalVizModel(snap, 420, 380, 28, 2);
    expect(zoomed.viewport.scale).toBeGreaterThan(base.viewport.scale);
  });
});

describe("curvature stats", () => {
  it("returns empty stats for missing items", () => {
    const stats = computeCurvatureStats(undefined);
    expect(stats.count).toBe(0);
    expect(stats.min).toBeNull();
    expect(stats.unknownCount).toBe(0);
  });

  it("computes offline fixture curvature aggregates", () => {
    const snap = loadOverlaySnapshotFixture();
    const items = snap.planned_path!.items;
    const values = collectCurvatureValues(items);
    expect(values.length).toBeGreaterThan(0);
    expect(Math.max(...values)).toBeGreaterThan(0);

    const stats = computeCurvatureStats(items);
    expect(stats.count).toBe(5);
    expect(stats.min).not.toBeNull();
    expect(stats.max).not.toBeNull();
    expect(stats.average).not.toBeNull();
    expect(stats.absMax).not.toBeNull();
    expect(stats.highCount + stats.mediumCount + stats.lowCount).toBe(5);
  });

  it("classifies display-only severity bands", () => {
    expect(classifyCurvatureSeverity(undefined)).toBe("unknown");
    expect(classifyCurvatureSeverity(0)).toBe("low");
    expect(classifyCurvatureSeverity(DISPLAY_CURVATURE_MEDIUM_1PM)).toBe("medium");
    expect(classifyCurvatureSeverity(DISPLAY_CURVATURE_HIGH_1PM)).toBe("high");
    expect(classifyCurvatureSeverity(0.02)).toBe("high");
  });

  it("applies high severity styling on offline fixture segments", () => {
    const snap = loadOverlaySnapshotFixture();
    const model = buildInternalVizModel(snap, 420, 380);
    expect(model.plannedSegments.some((s) => s.severity === "high")).toBe(true);
    expect(
      model.plannedSegments.some((s) => s.severity === "high" && s.style.width >= 3),
    ).toBe(true);
  });
});

describe("junction and prefab stats", () => {
  it("counts kinds and metadata on offline fixture", () => {
    const snap = loadOverlaySnapshotFixture();
    const stats = computePathKindStats(snap.planned_path!.items);
    expect(stats.total).toBe(5);
    expect(stats.roadEdge).toBe(2);
    expect(stats.junction).toBe(1);
    expect(stats.laneChange).toBe(1);
    expect(stats.navCurve).toBe(1);
    expect(stats.prefabUidCount).toBe(1);
    expect(stats.semaphoreHint).toBe(2);
    expect(stats.withCurveIndex).toBe(1);
  });

  it("resolves current item metadata", () => {
    const snap = loadOverlaySnapshotFixture();
    const meta = resolveCurrentItemMeta(snap.planned_path);
    expect(meta?.id).toBe(2);
    expect(meta?.kind).toBe("road_edge");
    expect(meta?.nodeRange).toBe("10002→10003");
    expect(meta?.curvature1pm).not.toBeNull();
  });
});

describe("isInternalPathVisualizationEnabled", () => {
  it("is true only for overlay_visualization=internal", () => {
    expect(
      isInternalPathVisualizationEnabled(
        new URLSearchParams("overlay_visualization=internal"),
      ),
    ).toBe(true);
    expect(
      isInternalPathVisualizationEnabled(new URLSearchParams("overlay_snapshot=fixture")),
    ).toBe(false);
  });
});

describe("InternalPathVisualization", () => {
  it("renders without planned_path (lane only)", () => {
    const minimal: OverlaySnapshot = {
      ...loadOverlaySnapshotFixture(),
      planned_path: undefined,
    };
    const html = renderToStaticMarkup(
      <InternalPathVisualization snapshot={minimal} />,
    );
    expect(html).toContain("No PlannedPathData");
    expect(html).toContain("Internal Visualization");
    expect(html).not.toContain("Curv 1/m");
  });

  it("renders fixture with stats, OFFLINE badge, and drive display no", () => {
    const snap = loadOverlaySnapshotFixture();
    const html = renderToStaticMarkup(<InternalPathVisualization snapshot={snap} />);
    expect(html).toContain("OFFLINE");
    expect(html).not.toContain(">MOCK<");
    expect(html).toContain("n10001");
    expect(html).toContain("truck");
    expect(html).toContain("Curv 1/m");
    expect(html).toContain("Items 5");
    expect(html).toContain("Current:");
    expect(html).toContain("Drive (display): no");
  });

  it("parses fixture planned_path points from JSON", () => {
    const snap = parseOverlaySnapshot(
      JSON.stringify(loadOverlaySnapshotFixture()),
    );
    expect(snap?.planned_path?.items[0]?.points?.length).toBeGreaterThan(0);
    expect(snap?.planned_path?.source).toBe("offline_graph");
    expect(snap?.planned_path?.items.some((i) => i.prefab_uid != null)).toBe(true);
    expect(snap?.planned_path?.items.some((i) => i.curve_index != null)).toBe(true);
    expect(snap?.planned_path?.items.some((i) => (i.curvature_1pm ?? 0) > 0)).toBe(true);
  });
});

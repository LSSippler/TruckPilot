import { describe, expect, it } from "vitest";
import type { ReactElement } from "react";
import { renderToStaticMarkup } from "react-dom/server";
import { MemoryRouter } from "react-router-dom";
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
  formatLiveSnapshotDebugLine,
  heatmapOverlayForSeverity,
  isInternalPathVisualizationEnabled,
  mapWorldToSvg,
  resolveCurrentItemMeta,
  resolveFeedBadge,
  resolveHeatmapEnabledFromSearch,
  resolveMissingPlannedPathMessage,
  resolveSourceBadge,
  segmentStyleForKind,
} from "@/components/overlay/internal-path-viz";
import {
  loadOverlaySnapshotFixture,
  parseOverlaySnapshot,
  createLiveOverlaySnapshotShell,
  type OverlaySnapshot,
} from "@/components/overlay/overlay-snapshot";

function renderInternalViz(
  ui: ReactElement,
  search = "overlay_snapshot=fixture&overlay_visualization=internal",
) {
  return renderToStaticMarkup(
    <MemoryRouter initialEntries={[`/overlay?${search}`]}>{ui}</MemoryRouter>,
  );
}

function renderInternalVizWithFeed(
  snapshot: OverlaySnapshot,
  feed: "fixture" | "storage" | "live",
  search = "overlay_visualization=internal",
) {
  return renderInternalViz(
    <InternalPathVisualization snapshot={snapshot} feed={feed} />,
    search,
  );
}

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
    const model = buildInternalVizModel(snap, 420, 380, 28, 1, true, "fixture");
    expect(model.hasPlannedPath).toBe(true);
    expect(model.plannedSegments.length).toBe(5);
    expect(model.plannedSegments.some((s) => s.isCurrent)).toBe(true);
    expect(model.feedBadge).toBe("FIXTURE");
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

  it("applies high severity styling on offline fixture segments when heatmap off", () => {
    const snap = loadOverlaySnapshotFixture();
    const model = buildInternalVizModel(snap, 420, 380, 28, 1, false);
    expect(model.plannedSegments.some((s) => s.severity === "high")).toBe(true);
    expect(
      model.plannedSegments.some((s) => s.severity === "high" && s.style.width >= 3),
    ).toBe(true);
  });
});

describe("live snapshot feed", () => {
  it("resolveFeedBadge maps feed modes", () => {
    expect(resolveFeedBadge("fixture")).toBe("FIXTURE");
    expect(resolveFeedBadge("live")).toBe("LIVE");
    expect(resolveFeedBadge("storage")).toBe("STORAGE");
    expect(resolveFeedBadge("off")).toBeNull();
  });

  it("resolveMissingPlannedPathMessage distinguishes live vs fixture", () => {
    expect(resolveMissingPlannedPathMessage("live", false)).toBe(
      "No PlannedPathData in live snapshot",
    );
    expect(resolveMissingPlannedPathMessage("fixture", false)).toBe("No PlannedPathData");
    expect(resolveMissingPlannedPathMessage("live", true)).toBeNull();
  });

  it("formatLiveSnapshotDebugLine reports snapshot sections", () => {
    const line = formatLiveSnapshotDebugLine(createLiveOverlaySnapshotShell());
    expect(line).toContain("lane yes");
    expect(line).toContain("status yes");
    expect(line).toContain("preflight no");
  });

  it("live shell renders LIVE badge and missing planned_path message", () => {
    const html = renderInternalVizWithFeed(createLiveOverlaySnapshotShell(), "live");
    expect(html).toContain("LIVE");
    expect(html).toContain("No PlannedPathData in live snapshot");
    expect(html).toContain("Snap: lane yes");
    expect(html).toContain("Drive (display): no");
  });

  it("live feed with planned_path shows LIVE and OFFLINE source badges", () => {
    const snap = loadOverlaySnapshotFixture();
    const html = renderInternalVizWithFeed(snap, "live");
    expect(html).toContain("LIVE");
    expect(html).toContain("OFFLINE");
    expect(html).toContain("Curv 1/m");
    expect(html).not.toContain("No PlannedPathData in live snapshot");
  });

  it("resolveSourceBadge uses explicit planned_path source only", () => {
    expect(resolveSourceBadge(undefined)).toBeNull();
    expect(
      resolveSourceBadge({ ...loadOverlaySnapshotFixture().planned_path!, source: "mock" }),
    ).toBe("MOCK");
    expect(
      resolveSourceBadge({
        ...loadOverlaySnapshotFixture().planned_path!,
        source: "route_blackboard",
      }),
    ).toBe("LIVE");
  });
});

describe("curvature heatmap", () => {
  it("adds heatmap overlay for high curvature segments", () => {
    const snap = loadOverlaySnapshotFixture();
    const model = buildInternalVizModel(snap, 420, 380, 28, 1, true);
    const high = model.plannedSegments.filter((s) => s.severity === "high");
    expect(high.length).toBeGreaterThan(0);
    expect(high.every((s) => s.heatmap?.showMidpointRing)).toBe(true);
    expect(high.every((s) => s.heatmap?.showTick)).toBe(true);
  });

  it("dims unknown curvature and keeps kind stroke readable with heatmap on", () => {
    const style = segmentStyleForKind("junction", {
      current: false,
      dimmed: false,
      severity: "unknown",
      heatmapEnabled: true,
    });
    expect(style.stroke).toBe("#fbbf24");
    expect(style.opacity).toBeLessThan(1);
    const overlay = heatmapOverlayForSeverity("unknown", true);
    expect(overlay?.dimBase).toBe(true);
  });

  it("disables heatmap overlays when toggled off in model", () => {
    const snap = loadOverlaySnapshotFixture();
    const model = buildInternalVizModel(snap, 420, 380, 28, 1, false);
    expect(model.heatmapEnabled).toBe(false);
    expect(model.plannedSegments.every((s) => s.heatmap == null)).toBe(true);
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

describe("resolveHeatmapEnabledFromSearch", () => {
  it("defaults to on and accepts overlay_heatmap=0/1", () => {
    expect(resolveHeatmapEnabledFromSearch(new URLSearchParams(""))).toBe(true);
    expect(
      resolveHeatmapEnabledFromSearch(new URLSearchParams("overlay_heatmap=0")),
    ).toBe(false);
    expect(
      resolveHeatmapEnabledFromSearch(new URLSearchParams("overlay_heatmap=1")),
    ).toBe(true);
    expect(
      resolveHeatmapEnabledFromSearch(new URLSearchParams("overlay_heatmap=off")),
    ).toBe(false);
  });
});

describe("InternalPathVisualization", () => {
  it("renders without planned_path (lane only)", () => {
    const minimal: OverlaySnapshot = {
      ...loadOverlaySnapshotFixture(),
      planned_path: undefined,
    };
    const html = renderInternalVizWithFeed(minimal, "fixture");
    expect(html).toContain("No PlannedPathData");
    expect(html).toContain("Internal Visualization");
    expect(html).not.toContain("Curv 1/m");
  });

  it("renders fixture with stats, FIXTURE/OFFLINE badges, heatmap legend, and drive display no", () => {
    const snap = loadOverlaySnapshotFixture();
    const html = renderInternalVizWithFeed(snap, "fixture");
    expect(html).toContain("FIXTURE");
    expect(html).toContain("OFFLINE");
    expect(html).not.toContain(">MOCK<");
    expect(html).toContain("n10001");
    expect(html).toContain("truck");
    expect(html).toContain("Curv 1/m");
    expect(html).toContain("Items 5");
    expect(html).toContain("Current:");
    expect(html).toContain("Drive (display): no");
    expect(html).toContain("κ heatmap");
    expect(html).toContain("display-only");
    expect(html).toContain("Heatmap: on");
    expect(html).not.toContain("<button");
  });

  it("shows non-interactive heatmap off label when overlay_heatmap=0", () => {
    const snap = loadOverlaySnapshotFixture();
    const html = renderInternalViz(
      <InternalPathVisualization snapshot={snap} />,
      "overlay_snapshot=fixture&overlay_visualization=internal&overlay_heatmap=0",
    );
    expect(html).toContain("Heatmap: off");
    expect(html).not.toContain("κ heatmap");
    expect(html).not.toContain("<button");
  });

  it("renders clickable heatmap toggle only in layout editor mode", () => {
    const snap = loadOverlaySnapshotFixture();
    const html = renderInternalViz(
      <InternalPathVisualization snapshot={snap} editorMode />,
    );
    expect(html).toContain("<button");
    expect(html).toContain("Heatmap: on");
  });

  it("keeps item kind labels visible with heatmap enabled", () => {
    const snap = loadOverlaySnapshotFixture();
    const html = renderInternalViz(<InternalPathVisualization snapshot={snap} />);
    expect(html).toContain("#4");
    expect(html).toContain("n10004");
    expect(html).toContain(" H");
  });

  it("does not change drive_allowed_display_only text", () => {
    const snap = loadOverlaySnapshotFixture();
    expect(snap.planned_path?.safety.drive_allowed_display_only).toBe(false);
    const html = renderInternalViz(<InternalPathVisualization snapshot={snap} />);
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

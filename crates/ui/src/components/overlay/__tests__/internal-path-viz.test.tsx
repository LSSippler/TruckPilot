import { describe, expect, it } from "vitest";
import { renderToStaticMarkup } from "react-dom/server";
import { InternalPathVisualization } from "@/components/overlay/InternalPathVisualization";
import {
  buildInternalVizModel,
  collectVisualizationPoints,
  computeBounds,
  createViewportTransform,
  isInternalPathVisualizationEnabled,
  mapWorldToSvg,
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
    const model = buildInternalVizModel(snap, 420, 320);
    expect(model.hasPlannedPath).toBe(true);
    expect(model.plannedSegments.length).toBe(5);
    expect(model.plannedSegments.some((s) => s.isCurrent)).toBe(true);
    expect(model.sourceBadge).toBe("OFFLINE");
  });

  it("buildInternalVizModel respects zoom scale", () => {
    const snap = loadOverlaySnapshotFixture();
    const base = buildInternalVizModel(snap, 420, 320, 28, 1);
    const zoomed = buildInternalVizModel(snap, 420, 320, 28, 2);
    expect(zoomed.viewport.scale).toBeGreaterThan(base.viewport.scale);
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
  });

  it("renders fixture with segment labels and OFFLINE badge", () => {
    const snap = loadOverlaySnapshotFixture();
    const html = renderToStaticMarkup(<InternalPathVisualization snapshot={snap} />);
    expect(html).toContain("OFFLINE");
    expect(html).not.toContain(">MOCK<");
    expect(html).toContain("n10001");
    expect(html).toContain("truck");
  });

  it("parses fixture planned_path points from JSON", () => {
    const snap = parseOverlaySnapshot(
      JSON.stringify(loadOverlaySnapshotFixture()),
    );
    expect(snap?.planned_path?.items[0]?.points?.length).toBeGreaterThan(0);
    expect(snap?.planned_path?.source).toBe("offline_graph");
  });
});

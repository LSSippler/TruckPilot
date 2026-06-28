import { describe, expect, it, beforeEach } from "vitest";
import {
  clampVizZoom,
  defaultPanelLayout,
  loadOverlayLayout,
  nudgeVizZoom,
  resetOverlayLayout,
  resolvePanelLayout,
  saveOverlayLayout,
  setPanelLayout,
  OVERLAY_LAYOUT_STORAGE_KEY,
} from "@/components/overlay/overlay-layout";

describe("overlay-layout", () => {
  beforeEach(() => {
    localStorage.removeItem(OVERLAY_LAYOUT_STORAGE_KEY);
  });

  it("defaults internal viz to bottom-right", () => {
    const layout = defaultPanelLayout("internal-viz", 1280, 800);
    expect(layout.x).toBeGreaterThan(800);
    expect(layout.y).toBeGreaterThan(400);
    expect(layout.zoom).toBe(1);
  });

  it("persists panel positions in localStorage", () => {
    const state = setPanelLayout(loadOverlayLayout(), "acc", { x: 40, y: 50 });
    saveOverlayLayout(state);
    const loaded = loadOverlayLayout();
    expect(loaded.panels.acc).toEqual({ x: 40, y: 50 });
  });

  it("resolvePanelLayout prefers saved over default", () => {
    const state = setPanelLayout(loadOverlayLayout(), "nav", { x: 300, y: 400 });
    const layout = resolvePanelLayout("nav", state, 1280, 800);
    expect(layout).toEqual({ x: 300, y: 400 });
  });

  it("clamps viz zoom", () => {
    expect(clampVizZoom(99)).toBeLessThan(99);
    expect(clampVizZoom(0.01)).toBeGreaterThan(0.01);
    expect(nudgeVizZoom(1, 0.5)).toBeGreaterThan(1);
  });

  it("reset clears storage", () => {
    saveOverlayLayout(setPanelLayout(loadOverlayLayout(), "state", { x: 1, y: 2 }));
    resetOverlayLayout();
    expect(loadOverlayLayout().panels.state).toBeUndefined();
  });
});

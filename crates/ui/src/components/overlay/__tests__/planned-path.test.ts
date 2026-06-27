import { describe, expect, it } from "vitest";
import { plannedPathViewStats } from "@/components/overlay/planned-path";
import type { PlannedPathData } from "@/components/overlay/overlay-snapshot";

const MOCK: PlannedPathData = {
  valid: true,
  source: "mock",
  route_id: "mock-fixture-v1",
  current_index: 1,
  lookahead_m: 80,
  items: [
    { id: 1, kind: "road_edge", length_m: 120, curvature_1pm: 0 },
    { id: 2, kind: "road_edge", length_m: 70, curvature_1pm: 0.022 },
    { id: 3, kind: "junction", length_m: 50, semaphore_hint: "priority_merge" },
    { id: 4, kind: "lane_change", length_m: 28, curvature_1pm: 0.002 },
    { id: 5, kind: "nav_curve", length_m: 75, semaphore_hint: "red_hold" },
  ],
  nearest: { item_id: 2, distance_along_m: 18.5, crosstrack_m: -0.12, heading_error_rad: 0.04, confidence: 0.92 },
  safety: {
    route_valid: false,
    lane_model_valid: false,
    resolver_safe: true,
    telemetry_fresh: true,
    input_allowed: false,
    drive_allowed_display_only: false,
    reasons: ["route invalid", "input disabled"],
  },
};

describe("plannedPathViewStats", () => {
  it("summarizes mock fixture for debug panel", () => {
    const stats = plannedPathViewStats(MOCK);
    expect(stats).not.toBeNull();
    expect(stats!.itemCount).toBe(5);
    expect(stats!.currentItemLabel).toContain("#2");
    expect(stats!.junctionPrefabCount).toBe(1);
    expect(stats!.semaphoreHintCount).toBe(2);
    expect(stats!.driveAllowedDisplay).toBe("no");
  });

  it("returns null when planned path missing", () => {
    expect(plannedPathViewStats(undefined)).toBeNull();
  });

  it("surfaces offline_graph source with curvature and junction counts", () => {
    const offline: PlannedPathData = {
      ...MOCK,
      source: "offline_graph",
      route_id: "offline-graph-mini-v1",
      items: [
        { id: 1, kind: "road_edge", length_m: 120, curvature_1pm: 0 },
        { id: 2, kind: "road_edge", length_m: 63.6, curvature_1pm: 0.0083 },
        { id: 3, kind: "junction", length_m: 50, semaphore_hint: "semaphore #7" },
        { id: 4, kind: "lane_change", length_m: 28.3, curvature_1pm: 0.001 },
        { id: 5, kind: "nav_curve", length_m: 75, curvature_1pm: 0.002, semaphore_hint: "semaphore #12" },
      ],
    };
    const stats = plannedPathViewStats(offline);
    expect(stats).not.toBeNull();
    expect(stats!.source).toBe("offline_graph");
    expect(stats!.junctionPrefabCount).toBe(1);
    expect(stats!.semaphoreHintCount).toBe(2);
    expect(stats!.curvatureRange).not.toBe("—");
  });
});

import { describe, expect, it } from "vitest";
import fixtureJson from "@/components/overlay/overlay-snapshot.fixture.json";
import {
  isOverlaySnapshotStandaloneMode,
  loadOverlaySnapshotFixture,
  mapLanePointsToSchematic,
  parseOverlaySnapshot,
  resolveOverlaySnapshot,
  type OverlaySnapshot,
} from "@/components/overlay/overlay-snapshot";

describe("parseOverlaySnapshot", () => {
  it("parses fixture JSON from truckpilot-status --overlay", () => {
    const snap = parseOverlaySnapshot(JSON.stringify(fixtureJson));
    expect(snap).not.toBeNull();
    expect(snap!.verdict).toBe("unavailable");
    expect(snap!.lane.source).toBe("mock");
    expect(snap!.lane.lane_model_valid).toBe(false);
    expect(snap!.lane_keeper_allowed).toBe(false);
    expect(snap!.status.route_valid).toBe(false);
    expect(snap!.lane.centerline_points.length).toBe(5);
    expect(snap!.lane.spline_segments.length).toBe(4);
  });

  it("parses core_readiness from status block", () => {
    const snap = parseOverlaySnapshot(JSON.stringify(fixtureJson));
    expect(snap!.status.core_readiness).toEqual({
      graph_ready: null,
      spline_index_ready: null,
      plugins_ready: null,
      lane_detection_ready: null,
      truckpilot_system_ready: null,
      available: false,
    });
  });

  it("returns null for malformed JSON", () => {
    expect(parseOverlaySnapshot("")).toBeNull();
    expect(parseOverlaySnapshot("{")).toBeNull();
    expect(parseOverlaySnapshot('{"verdict":"nope"}')).toBeNull();
  });

  it("lane_keeper_allowed false when route or lane invalid", () => {
    const snap = loadOverlaySnapshotFixture();
    expect(snap.lane_keeper_allowed).toBe(false);
    expect(snap.status.route_valid).toBe(false);
    expect(snap.lane.lane_model_valid).toBe(false);
  });
});

describe("resolveOverlaySnapshot", () => {
  it("loads embedded fixture via query flag", () => {
    const snap = resolveOverlaySnapshot(new URLSearchParams("overlay_snapshot=fixture"));
    expect(snap).not.toBeNull();
    expect(snap!.lane.source).toBe("mock");
  });

  it("returns null when explicitly off", () => {
    expect(resolveOverlaySnapshot(new URLSearchParams("overlay_snapshot=off"))).toBeNull();
  });
});

describe("isOverlaySnapshotStandaloneMode", () => {
  it("is true for fixture, mock, and storage modes", () => {
    expect(isOverlaySnapshotStandaloneMode(new URLSearchParams("overlay_snapshot=fixture"))).toBe(
      true,
    );
    expect(isOverlaySnapshotStandaloneMode(new URLSearchParams("overlay_snapshot=mock"))).toBe(
      true,
    );
    expect(isOverlaySnapshotStandaloneMode(new URLSearchParams("overlay_snapshot=storage"))).toBe(
      true,
    );
  });

  it("is false when off or on normal overlay route", () => {
    expect(isOverlaySnapshotStandaloneMode(new URLSearchParams("overlay_snapshot=off"))).toBe(
      false,
    );
    expect(isOverlaySnapshotStandaloneMode(new URLSearchParams(""))).toBe(false);
  });
});

describe("mapLanePointsToSchematic", () => {
  it("maps lane points into the schematic box", () => {
    const snap = loadOverlaySnapshotFixture();
    const mapped = mapLanePointsToSchematic(snap.lane.centerline_points, 300, 200, 14);
    expect(mapped.length).toBe(5);
    for (const [x, y] of mapped) {
      expect(x).toBeGreaterThanOrEqual(0);
      expect(y).toBeGreaterThanOrEqual(0);
      expect(x).toBeLessThanOrEqual(300);
      expect(y).toBeLessThanOrEqual(200);
    }
  });
});

describe("fixture shape", () => {
  it("matches OverlaySnapshot fields used by the UI", () => {
    const snap = fixtureJson as OverlaySnapshot;
    expect(snap.status.diag_level).toBeDefined();
    expect(snap.status.resolve_status).toBeDefined();
    expect(typeof snap.status.frame_cb_us_max).toBe("number");
    expect(snap.lane.left_lane_points.length).toBe(snap.lane.centerline_points.length);
    expect(snap.lane.right_lane_points.length).toBe(snap.lane.centerline_points.length);
  });
});

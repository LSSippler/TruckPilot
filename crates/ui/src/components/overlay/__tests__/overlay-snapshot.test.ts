import { describe, expect, it } from "vitest";
import fixtureJson from "@/components/overlay/overlay-snapshot.fixture.json";
import {
  isOverlaySnapshotDebugMode,
  loadOverlaySnapshotFixture,
  mapLanePointsToSchematic,
  OVERLAY_SNAPSHOT_STORAGE_KEY,
  parseOverlaySnapshot,
  resolveOverlaySnapshot,
  saveOverlaySnapshotToStorage,
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

describe("isOverlaySnapshotDebugMode", () => {
  it("is true for fixture and storage modes", () => {
    expect(isOverlaySnapshotDebugMode(new URLSearchParams("overlay_snapshot=fixture"))).toBe(true);
    expect(isOverlaySnapshotDebugMode(new URLSearchParams("overlay_snapshot=storage"))).toBe(true);
  });

  it("is false when off or absent", () => {
    expect(isOverlaySnapshotDebugMode(new URLSearchParams("overlay_snapshot=off"))).toBe(false);
    expect(isOverlaySnapshotDebugMode(new URLSearchParams(""))).toBe(false);
  });
});

describe("saveOverlaySnapshotToStorage", () => {
  it("persists valid JSON and returns parsed snapshot", () => {
    localStorage.removeItem(OVERLAY_SNAPSHOT_STORAGE_KEY);
    const raw = JSON.stringify(fixtureJson);
    const snap = saveOverlaySnapshotToStorage(raw);
    expect(snap).not.toBeNull();
    expect(snap!.lane.source).toBe("mock");
    expect(localStorage.getItem(OVERLAY_SNAPSHOT_STORAGE_KEY)).toBe(raw);
  });

  it("returns null for invalid JSON without writing storage", () => {
    localStorage.setItem(OVERLAY_SNAPSHOT_STORAGE_KEY, "keep");
    expect(saveOverlaySnapshotToStorage("{not valid")).toBeNull();
    expect(localStorage.getItem(OVERLAY_SNAPSHOT_STORAGE_KEY)).toBe("keep");
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

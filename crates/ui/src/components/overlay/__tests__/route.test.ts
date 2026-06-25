import { describe, expect, it } from "vitest";
import {
  densify,
  forwardWindow,
  parseRoute,
  prepareRoute,
  segmentFootPoint,
  type RoutePoint,
} from "@/components/overlay/route";

describe("parseRoute", () => {
  it("parses a [[x,z],...] JSON string", () => {
    expect(parseRoute("[[1,2],[3,4]]")).toEqual([
      [1, 2],
      [3, 4],
    ]);
  });

  it("returns [] for undefined / empty / malformed input", () => {
    expect(parseRoute(undefined)).toEqual([]);
    expect(parseRoute("")).toEqual([]);
    expect(parseRoute("not json")).toEqual([]);
    expect(parseRoute('{"a":1}')).toEqual([]);
  });

  it("drops non-numeric / short points but keeps the rest", () => {
    expect(parseRoute("[[1],[2,3],[\"a\",4],[5,6]]")).toEqual([
      [2, 3],
      [5, 6],
    ]);
  });
});

describe("segmentFootPoint", () => {
  it("returns midpoint when truck is beside segment", () => {
    const [fx, fz] = segmentFootPoint(0, 0, 0, 100, 5, 50);
    expect(fx).toBeCloseTo(0);
    expect(fz).toBeCloseTo(50);
  });

  it("clamps to start when truck is before segment", () => {
    const [fx, fz] = segmentFootPoint(0, 100, 0, 200, 0, 0);
    expect(fx).toBeCloseTo(0);
    expect(fz).toBeCloseTo(100);
  });

  it("clamps to end when truck is past segment", () => {
    const [fx, fz] = segmentFootPoint(0, 0, 0, 100, 0, 200);
    expect(fx).toBeCloseTo(0);
    expect(fz).toBeCloseTo(100);
  });

  it("returns start for degenerate zero-length segment", () => {
    const [fx, fz] = segmentFootPoint(5, 7, 5, 7, 0, 0);
    expect(fx).toBeCloseTo(5);
    expect(fz).toBeCloseTo(7);
  });
});

describe("densify", () => {
  it("passthrough when points are already close", () => {
    const pts: RoutePoint[] = [[0, 0], [0, 10]];
    expect(densify(pts, 12)).toEqual([[0, 0], [0, 10]]);
  });

  it("inserts intermediates for long segments", () => {
    const pts: RoutePoint[] = [[0, 0], [0, 120]];
    const out = densify(pts, 12);
    // ceil(120/12) = 10 segments → 9 intermediates + 2 endpoints = 11 points
    expect(out.length).toBe(11);
    expect(out[0]).toEqual([0, 0]);
    expect(out[out.length - 1]).toEqual([0, 120]);
    // Each successive z increases
    for (let i = 1; i < out.length; i++) {
      expect(out[i]![1]).toBeGreaterThan(out[i - 1]![1]);
    }
  });

  it("preserves single-point array", () => {
    expect(densify([[1, 2]], 12)).toEqual([[1, 2]]);
  });

  it("preserves empty array", () => {
    expect(densify([], 12)).toEqual([]);
  });
});

describe("prepareRoute", () => {
  it("starts at truck when route begins 190 m ahead", () => {
    // Reproduces the live bug: route starts 190 m ahead, truck at origin.
    const route: RoutePoint[] = [
      [0, 190], [0, 410], [0, 600],
    ];
    const out = prepareRoute(route, 0, 0, 600);
    // First point must be the truck position
    expect(out[0]).toEqual([0, 0]);
    // Last point is the last waypoint
    expect(out[out.length - 1]).toEqual([0, 600]);
    // Dense: no gap > 12 m
    for (let i = 1; i < out.length; i++) {
      const a = out[i - 1]!;
      const b = out[i]!;
      const dist = Math.hypot(b[0] - a[0], b[1] - a[1]);
      expect(dist).toBeLessThanOrEqual(12.01);
    }
  });

  it("uses foot-point when truck is beside a segment", () => {
    // Truck is 5 m to the right of the route mid-segment.
    const route: RoutePoint[] = [
      [0, 0], [0, 100], [0, 200],
    ];
    const out = prepareRoute(route, 5, 50, 600);
    // First point = truck, second = foot-point (0, 50) within ~1 m
    expect(out[0]).toEqual([5, 50]);
    expect(out[1]![0]).toBeCloseTo(0, 1);
    expect(out[1]![1]).toBeCloseTo(50, 1);
  });

  it("returns [] for empty route", () => {
    expect(prepareRoute([], 0, 0, 600)).toEqual([]);
  });
});

describe("forwardWindow", () => {
  const route: RoutePoint[] = [
    [0, 0],
    [0, 100],
    [0, 200],
    [0, 300],
    [0, 400],
    [0, 500],
  ];

  it("returns a contiguous slice within range of the truck", () => {
    // Truck near [0,200], range 150 m → nearest is index 2; neighbours within
    // 150 m are index 1 (100 m) and index 3 (100 m); index 0/4 are 200 m away.
    expect(forwardWindow(route, 0, 200, 150)).toEqual([
      [0, 100],
      [0, 200],
      [0, 300],
    ]);
  });

  it("widens the window with a larger range", () => {
    expect(forwardWindow(route, 0, 0, 250)).toEqual([
      [0, 0],
      [0, 100],
      [0, 200],
    ]);
  });

  it("returns [] for routes shorter than two points", () => {
    expect(forwardWindow([[0, 0]], 0, 0, 300)).toEqual([]);
    expect(forwardWindow([], 0, 0, 300)).toEqual([]);
  });
});

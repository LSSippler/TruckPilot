import { describe, expect, it } from "vitest";
import {
  cabinCameraPos,
  cameraIntrinsics,
  clipSegmentToNear,
  headingForward,
  pointAhead,
  worldToScreen,
  type Vec3,
} from "@/lib/projection";

const W = 1920;
const H = 1080;
const ORIGIN: Vec3 = [0, 0, 0];

// These mirror the invariants of the reference renderer
// (crates/overlay/src/projection.rs) so the TS convention matches it exactly.
describe("projection — intrinsics", () => {
  it("FOV=90° → f = w/2, principal point centred", () => {
    const k = cameraIntrinsics(90, W, H);
    expect(k.f).toBeCloseTo(W / 2, 3); // tan(45°)=1
    expect(k.cx).toBeCloseTo(W / 2, 3);
    expect(k.cy).toBeCloseTo(H / 2, 3);
  });
});

describe("projection — heading convention", () => {
  it("heading=0 looks North (−Z)", () => {
    const f = headingForward(0);
    expect(f[0]).toBeCloseTo(0, 5);
    expect(f[2]).toBeCloseTo(-1, 5);
  });
  it("heading=0.75 looks East (+X)", () => {
    const f = headingForward(0.75);
    expect(f[0]).toBeCloseTo(1, 5);
    expect(f[2]).toBeCloseTo(0, 5);
  });
  it("heading=0.25 looks West (−X)", () => {
    const f = headingForward(0.25);
    expect(f[0]).toBeCloseTo(-1, 5);
    expect(f[2]).toBeCloseTo(0, 5);
  });
});

describe("projection — world_to_screen", () => {
  const k = cameraIntrinsics(90, W, H);

  it("point straight ahead (North) projects to screen centre", () => {
    const s = worldToScreen([0, 0, -10], ORIGIN, 0, 0, 0, k);
    expect(s).not.toBeNull();
    expect(s!.x).toBeCloseTo(W / 2, 1);
    expect(s!.y).toBeCloseTo(H / 2, 1);
    expect(s!.depth).toBeCloseTo(10, 5);
  });

  it("point above the camera projects above centre (smaller y)", () => {
    const s = worldToScreen([0, 1, -10], ORIGIN, 0, 0, 0, k);
    expect(s).not.toBeNull();
    expect(s!.y).toBeLessThan(H / 2);
  });

  it("point behind the camera returns null (near-plane clip)", () => {
    const s = worldToScreen([0, 0, 10], ORIGIN, 0, 0, 0, k);
    expect(s).toBeNull();
  });

  it("respects heading: facing East, a point to the East is centred ahead", () => {
    const s = worldToScreen([10, 0, 0], ORIGIN, 0.75, 0, 0, k);
    expect(s).not.toBeNull();
    expect(s!.x).toBeCloseTo(W / 2, 1);
    expect(s!.y).toBeCloseTo(H / 2, 1);
    expect(s!.depth).toBeCloseTo(10, 5);
  });
});

// World-anchoring / lateral-sign tests. These are OFF the optical axis (x ≠ 0),
// so they catch the camera-X mirror that the on-axis tests above are blind to.
// Both would FAIL against the mirrored reference convention (px = cx + f·x/z)
// and PASS against the un-mirrored one (px = cx − f·x/z). This is the pair that
// would have caught the "marker slides to the wrong side when steering" bug.
describe("projection — lateral sign / no camera-X mirror", () => {
  const k = cameraIntrinsics(90, W, H);

  it("a point to the physical right (East, facing North) projects right of centre", () => {
    // diag.rs MIRROR-CHECK: physical-right point ⇒ px > cx; px < cx ⇒ X-mirror.
    const s = worldToScreen([5, 0, -50], ORIGIN, 0, 0, 0, k); // 5 m East, 50 m North
    expect(s).not.toBeNull();
    expect(s!.x).toBeGreaterThan(k.cx);
  });

  it("turning the truck left swings a straight-ahead world point to the RIGHT", () => {
    // Camera fixed; a world-fixed point dead ahead at heading=0. When the truck
    // yaws left (heading increases toward West), that point must swing RIGHT on
    // screen and not stay centred — exactly the world-anchored counter-rotation.
    const ahead: Vec3 = [0, 0, -50];
    const centred = worldToScreen(ahead, ORIGIN, 0, 0, 0, k);
    const turned = worldToScreen(ahead, ORIGIN, 0.05, 0, 0, k);
    expect(centred).not.toBeNull();
    expect(turned).not.toBeNull();
    expect(centred!.x).toBeCloseTo(k.cx, 1);
    expect(turned!.x).toBeGreaterThan(centred!.x + 1); // moved, to the right
  });
});

describe("projection — cabin offset + test point placement", () => {
  it("cabin camera sits up and forward at heading=0", () => {
    const cam = cabinCameraPos([100, 5, 200], 0);
    expect(cam[0]).toBeCloseTo(100, 5); // no x shift at heading=0
    expect(cam[1]).toBeCloseTo(6.5, 5); // +1.5 up
    expect(cam[2]).toBeCloseTo(199.5, 5); // -0.5 forward (North = −Z)
  });

  it("a 50 m-ahead point keeps truck ground height and lands North", () => {
    const p = pointAhead([100, 5, 200], 0, 50);
    expect(p[0]).toBeCloseTo(100, 5);
    expect(p[1]).toBeCloseTo(5, 5); // ground height, not cabin height
    expect(p[2]).toBeCloseTo(150, 5); // 50 m North (−Z)
  });
});

// Near-plane clipping for line segments (Phase 6.6a-2 polyline). Inputs are
// already in camera space (+z = depth ahead).
describe("projection — clipSegmentToNear", () => {
  const NEAR = 0.1;

  it("both endpoints behind the near plane → null (whole segment dropped)", () => {
    expect(clipSegmentToNear([0, 0, -5], [1, 0, -1], NEAR)).toBeNull();
  });

  it("both endpoints in front → returned unchanged", () => {
    const r = clipSegmentToNear([0, 0, 5], [1, 0, 10], NEAR);
    expect(r).not.toBeNull();
    expect(r![0]).toEqual([0, 0, 5]);
    expect(r![1]).toEqual([1, 0, 10]);
  });

  it("first endpoint behind → moved onto the near plane, far end unchanged", () => {
    const r = clipSegmentToNear([0, 0, 0], [1, 0, 10], NEAR);
    expect(r).not.toBeNull();
    expect(r![0][2]).toBeCloseTo(NEAR, 5); // clipped to z = near
    expect(r![1]).toEqual([1, 0, 10]);
  });

  it("second endpoint behind → moved onto the near plane, near end unchanged", () => {
    const r = clipSegmentToNear([1, 0, 10], [0, 0, 0], NEAR);
    expect(r).not.toBeNull();
    expect(r![0]).toEqual([1, 0, 10]);
    expect(r![1][2]).toBeCloseTo(NEAR, 5);
  });
});

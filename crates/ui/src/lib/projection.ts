// Phase 6.6a-1 — Clean-room world→screen projection for the Tauri overlay.
//
// CONVENTION — re-derived here from standard pinhole geometry. The axis/angle
// convention is ADOPTED from crates/overlay/src/projection.rs (read as a
// reference only; no code copied) so this TS projection matches the existing
// DX11 renderer and the line does not end up mirrored/rotated:
//
//   ETS2 world space (left-handed): X = East(+), Y = Up(+), Z = South(+)/North(−).
//   heading/pitch/roll are SCS-SDK units in 0..1 turns (× 2π → radians).
//   Camera rotation R (camera→world) = Ry(π + heading)·Rx(pitch)·Rz(roll)
//     (glam EulerRot::YXZ, intrinsic). The +π yaw maps heading=0 ⇒ look −Z
//     (North) and heading=0.75 ⇒ look +X (East).
//   world→camera = Rᵀ. Pinhole: +Z is depth into the screen, screen-Y inverted.
//
// SCREEN-X IS NEGATED vs the reference (px = cx − f·x/z, not +). ETS2 world is
// left-handed (X=East), so the right-handed camera math mirrors the lateral
// axis: a point to the physical right (East when facing North) would land LEFT
// of centre. The reference (crates/overlay/src/projection.rs + ar_renderer.rs)
// leaves this mirror in — its own diag.rs MIRROR-CHECK documents it ("px < cx ⇒
// CAMERA-X MIRROR"), but the renderer was never fixed because a symmetric road
// ribbon hides it. A discrete marker exposes it (wrong side, slides off-road
// when steering), so we un-mirror here. This is a DELIBERATE divergence from
// the reference's world_to_screen; the rotation/inversion itself is unchanged.
//
// NOTE on pitch/roll units: the reference treats ALL three euler values as
// 0..1 turns (×2π). On flat ground pitch≈roll≈0, so the 6.6a-1 test marker is
// unaffected; verify the unit on a real slope before trusting tilt accuracy.

export type Vec3 = readonly [number, number, number];

/** Fixed horizontal field of view (deg). ETS2's real cockpit FOV differs and
 *  varies per truck/zoom — this is a first guess; the reference renderer ships
 *  a calibration wizard for the same reason. Promote to a slider later. */
export const FOV_H_DEG = 75;
/** Near plane (m). Points at/behind this are not drawn. Matches the reference. */
export const NEAR = 0.1;
/** Cabin camera offset above the truck reference point (m).
 *  From crates/overlay/src/telemetry.rs (CABIN_Y_OFFSET). */
export const CABIN_Y_OFFSET = 1.5;
/** Cabin camera offset forward along heading (m).
 *  From crates/overlay/src/telemetry.rs (CABIN_FORWARD_OFFSET). */
export const CABIN_FORWARD_OFFSET = 0.5;

const TAU = Math.PI * 2;

export interface CameraIntrinsics {
  /** Focal length in pixels (fx == fy, square pixels assumed). */
  readonly f: number;
  /** Principal point X (screen centre), pixels. */
  readonly cx: number;
  /** Principal point Y (screen centre), pixels. */
  readonly cy: number;
}

/** Build intrinsics from horizontal FOV and the canvas size in CSS pixels. */
export function cameraIntrinsics(fovHDeg: number, w: number, h: number): CameraIntrinsics {
  const f = w / 2 / Math.tan((fovHDeg * (Math.PI / 180)) / 2);
  return { f, cx: w / 2, cy: h / 2 };
}

/** Unit forward direction on the ground plane for a heading (SCS 0..1 turns).
 *  forward = (−sin h, 0, −cos h): heading=0 → (0,0,−1)=North, 0.75 → (1,0,0)=East. */
export function headingForward(heading: number): Vec3 {
  const h = heading * TAU;
  return [-Math.sin(h), 0, -Math.cos(h)];
}

/** Cabin/head camera position from the truck reference point + heading.
 *  Mirrors compute_head_pos() in crates/overlay/src/telemetry.rs. */
export function cabinCameraPos(truck: Vec3, heading: number): Vec3 {
  const h = heading * TAU;
  return [
    truck[0] - Math.sin(h) * CABIN_FORWARD_OFFSET,
    truck[1] + CABIN_Y_OFFSET,
    truck[2] - Math.cos(h) * CABIN_FORWARD_OFFSET,
  ];
}

/** A world point `dist` metres ahead of the truck along its heading, at the
 *  truck's ground height (truck Y). Used to drop the 6.6a-1 test markers. */
export function pointAhead(truck: Vec3, heading: number, dist: number): Vec3 {
  const fwd = headingForward(heading);
  return [truck[0] + fwd[0] * dist, truck[1], truck[2] + fwd[2] * dist];
}

// ── rotation ──────────────────────────────────────────────────────────────
// We only ever need Rᵀ·d (world→camera), so we build R = Ry(yaw)·Rx(p)·Rz(r)
// from elementary right-handed matrices (glam convention) and apply Rᵀ
// directly — no matrix inversion needed (R is orthonormal, so R⁻¹ = Rᵀ).

/** Transform a world-space delta (worldPt − camPos) into camera space using the
 *  camera's heading/pitch/roll (SCS 0..1 turns). Returns [x,y,z] with +z = depth. */
function worldDeltaToCamera(d: Vec3, heading: number, pitch: number, roll: number): Vec3 {
  const yaw = Math.PI + heading * TAU;
  const p = pitch * TAU;
  const r = roll * TAU;

  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  const cp = Math.cos(p);
  const sp = Math.sin(p);
  const cr = Math.cos(r);
  const sr = Math.sin(r);

  // Rx·Rz, row-major (m<row><col>):
  //   Rx = [[1,0,0],[0,cp,-sp],[0,sp,cp]]
  //   Rz = [[cr,-sr,0],[sr,cr,0],[0,0,1]]
  const m00 = cr;
  const m01 = -sr;
  const m02 = 0;
  const m10 = cp * sr;
  const m11 = cp * cr;
  const m12 = -sp;
  const m20 = sp * sr;
  const m21 = sp * cr;
  const m22 = cp;

  // R = Ry·(Rx·Rz), with Ry = [[cy,0,sy],[0,1,0],[-sy,0,cy]] (r<row><col>).
  const r00 = cy * m00 + sy * m20;
  const r01 = cy * m01 + sy * m21;
  const r02 = cy * m02 + sy * m22;
  const r10 = m10;
  const r11 = m11;
  const r12 = m12;
  const r20 = -sy * m00 + cy * m20;
  const r21 = -sy * m01 + cy * m21;
  const r22 = -sy * m02 + cy * m22;

  // p_cam = Rᵀ·d  →  p_cam[i] = Σ_j R[j][i]·d[j].
  const [d0, d1, d2] = d;
  return [
    r00 * d0 + r10 * d1 + r20 * d2,
    r01 * d0 + r11 * d1 + r21 * d2,
    r02 * d0 + r12 * d1 + r22 * d2,
  ];
}

/** Transform a world-space point into camera space (before projection). +z is
 *  depth into the screen. Exposed so callers can near-clip a line segment in
 *  camera space (see clipSegmentToNear) before projecting its endpoints. */
export function worldToCamera(
  worldPt: Vec3,
  camPos: Vec3,
  heading: number,
  pitch: number,
  roll: number,
): Vec3 {
  const d: Vec3 = [worldPt[0] - camPos[0], worldPt[1] - camPos[1], worldPt[2] - camPos[2]];
  return worldDeltaToCamera(d, heading, pitch, roll);
}

export interface ScreenPoint {
  x: number;
  y: number;
  /** Camera-space Z (metres ahead of the camera). */
  depth: number;
}

/** Project an already-in-camera-space point to screen pixels. Returns null at
 *  or behind the near plane. Screen-X is negated (left-handed world un-mirror,
 *  see module header). */
export function projectCameraPoint(
  c: Vec3,
  intr: CameraIntrinsics,
  near: number = NEAR,
): ScreenPoint | null {
  if (c[2] < near) return null;
  return {
    x: intr.cx - intr.f * (c[0] / c[2]),
    y: intr.cy - intr.f * (c[1] / c[2]),
    depth: c[2],
  };
}

/** Project a world-space point onto the screen. Returns null when the point is
 *  at or behind the near plane. `depth` is the camera-space Z (metres ahead). */
export function worldToScreen(
  worldPt: Vec3,
  camPos: Vec3,
  heading: number,
  pitch: number,
  roll: number,
  intr: CameraIntrinsics,
  near: number = NEAR,
): ScreenPoint | null {
  return projectCameraPoint(worldToCamera(worldPt, camPos, heading, pitch, roll), intr, near);
}

/** Clip a line segment whose endpoints are ALREADY in camera space to the near
 *  plane. Returns null when both endpoints are behind `near`; otherwise the
 *  segment with any behind-endpoint moved onto the near plane (linear interp).
 *  Mirrors clip_segment_to_near in crates/overlay/src/projection.rs. Clipping
 *  in camera space (not screen space) is what stops a behind-camera endpoint
 *  from projecting to a garbage pixel and streaking across the frame. */
export function clipSegmentToNear(
  c0: Vec3,
  c1: Vec3,
  near: number = NEAR,
): [Vec3, Vec3] | null {
  const behind0 = c0[2] < near;
  const behind1 = c1[2] < near;
  if (behind0 && behind1) return null;
  if (!behind0 && !behind1) return [c0, c1];
  // Exactly one endpoint behind → move it onto the near plane along the segment.
  const t = (near - c0[2]) / (c1[2] - c0[2]);
  const mid: Vec3 = [
    c0[0] + (c1[0] - c0[0]) * t,
    c0[1] + (c1[1] - c0[1]) * t,
    c0[2] + (c1[2] - c0[2]) * t,
  ];
  return behind0 ? [mid, c1] : [c0, mid];
}

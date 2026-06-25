import { useEffect, useRef } from "react";
import {
  cabinCameraPos,
  cameraIntrinsics,
  clipSegmentToNear,
  projectCameraPoint,
  worldToCamera,
  FOV_H_DEG,
  type Vec3,
} from "@/lib/projection";
import { subscribeBlackboardKeys } from "@/lib/ipc";
import { useBlackboardStore } from "@/stores/blackboard";
import { readLivePose, startPosePoll } from "./pose";
import { ROUTE_BB_KEY, prepareRoute, parseRoute, type RoutePoint } from "./route";

// Phase 6.6a-2 — transparent, click-through canvas that projects the planned
// route (router.waypoints) onto the road in the cockpit. Painted UNDER the
// panels (first child of Overlay.tsx) and pointer-events:none, so the 6.5
// overlay is unchanged.
//
// The 6.6a-1 test markers (20/50/100 m + anchor/re-anchor logic) are GONE: the
// route is what we draw now, and waypoints are already fixed world coordinates,
// so no per-frame anchoring is needed. The projection math stays covered by
// src/lib/__tests__/projection.test.ts.

/** Only draw the route this far ahead of the truck (m). 600 m covers the typical
 *  spacing of router.waypoints (~200 m apart) with enough look-ahead to show a
 *  curve. Beyond this the line would streak toward the distant goal. */
const ROUTE_RANGE_M = 600;

const DEBUG_DIAG = true;

export function ProjectionCanvas() {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    let raf = 0;
    let cssW = 0;
    let cssH = 0;
    let dpr = 1;
    // Re-parse the route JSON only when the blackboard string actually changes
    // (it updates ~2 Hz and is identical between replans).
    let routeRaw: string | undefined;
    let route: RoutePoint[] = [];

    const resize = () => {
      dpr = window.devicePixelRatio || 1;
      cssW = canvas.clientWidth;
      cssH = canvas.clientHeight;
      canvas.width = Math.max(1, Math.round(cssW * dpr));
      canvas.height = Math.max(1, Math.round(cssH * dpr));
    };
    resize();
    const ro = new ResizeObserver(resize);
    ro.observe(canvas);

    const stopPoll = startPosePoll();
    // Route rides the shared 500 ms blackboard poll (2 Hz) — plenty for a
    // route that only changes on replan.
    const unsubRoute = subscribeBlackboardKeys([ROUTE_BB_KEY]);

    const frame = () => {
      raf = requestAnimationFrame(frame);

      // Draw in CSS pixels; the DPR-scaled backing store keeps the line crisp.
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, cssW, cssH);

      const pose = readLivePose();
      if (!pose || cssW < 2 || cssH < 2) return;

      const raw = useBlackboardStore.getState().values[ROUTE_BB_KEY];
      if (raw !== routeRaw) {
        routeRaw = raw;
        route = parseRoute(raw);
      }
      if (route.length < 2) return; // no/empty route → clean no-op

      const intr = cameraIntrinsics(FOV_H_DEG, cssW, cssH);
      const cam = cabinCameraPos(pose.pos, pose.heading);
      const truckY = pose.pos[1]; // flat Y (6.6a-2); real graph-Y is 6.6a-2b

      const slice = prepareRoute(route, pose.pos[0], pose.pos[2], ROUTE_RANGE_M);

      if (DEBUG_DIAG) {
        drawDiag(ctx, cssH, route, slice, pose, cam, truckY);
      }

      if (slice.length < 2) return;

      // Project each waypoint to camera space, clip each segment to the near
      // plane there, then project the (clipped) endpoints to screen.
      ctx.beginPath();
      let prevCam: Vec3 | null = null;
      for (const wp of slice) {
        const c = worldToCamera([wp[0], truckY, wp[1]], cam, pose.heading, pose.pitch, pose.roll);
        if (prevCam) {
          const clipped = clipSegmentToNear(prevCam, c);
          if (clipped) {
            const s0 = projectCameraPoint(clipped[0], intr);
            const s1 = projectCameraPoint(clipped[1], intr);
            if (s0 && s1) {
              ctx.moveTo(s0.x, s0.y);
              ctx.lineTo(s1.x, s1.y);
            }
          }
        }
        prevCam = c;
      }
      ctx.lineWidth = 2.5;
      ctx.lineJoin = "round";
      ctx.lineCap = "round";
      ctx.strokeStyle = "rgba(34, 211, 238, 0.85)"; // cyan — visible on tarmac
      ctx.stroke();
    };
    raf = requestAnimationFrame(frame);

    return () => {
      cancelAnimationFrame(raf);
      ro.disconnect();
      stopPoll();
      unsubRoute();
    };
  }, []);

  return (
    <canvas
      ref={canvasRef}
      className="pointer-events-none absolute inset-0 h-full w-full"
    />
  );
}

// ── 6.6a Counterflow-Diagnose (read-only; remove after verdict) ──────────────
//
// Ziel: unterscheiden ob Route auf Gegenfahrbahn (Skalarprodukte ≈ -1) oder
// nur seitlich versetzt auf gleicher Fahrtrichtung (Skalarprodukte ≈ +1).

interface DiagPose {
  pos: Vec3;
  heading: number;
  pitch: number;
  roll: number;
}

function nearestIdx(route: RoutePoint[], tx: number, tz: number): number {
  let idx = 0;
  let best = Infinity;
  for (let i = 0; i < route.length; i++) {
    const p = route[i];
    if (!p) continue;
    const d = (p[0] - tx) ** 2 + (p[1] - tz) ** 2;
    if (d < best) { best = d; idx = i; }
  }
  return idx;
}

function drawDiag(
  ctx: CanvasRenderingContext2D,
  cssH: number,
  route: RoutePoint[],
  slice: RoutePoint[],
  pose: DiagPose,
  _cam: Vec3,
  _truckY: number,
) {
  const f2 = (n: number) => n.toFixed(2);
  const f1 = (n: number) => n.toFixed(1);

  // Truck heading forward vector (world XZ). SCS 0..1 turns, CCW from North.
  // headingForward: (hx,hz) = (-sin(h·2π), -cos(h·2π)), h=0 → North = (0,-1).
  const TAU = Math.PI * 2;
  const hRad = pose.heading * TAU;
  const hx = -Math.sin(hRad);
  const hz = -Math.cos(hRad);

  const tx = pose.pos[0];
  const tz = pose.pos[2];
  const ni = nearestIdx(route, tx, tz);

  // Lateral offset: project (nearest_wp - truck) onto perpendicular of heading.
  // Right-perpendicular in ETS2 XZ (left-handed): (hz, -hx).
  // Positive = nearest wp is to the RIGHT of truck heading.
  const np = route[ni];
  const latSign =
    np ? (hz * (np[0] - tx) + (-hx) * (np[1] - tz)) : 0;
  const latDist = np ? Math.hypot(np[0] - tx - hx * ((np[0] - tx) * hx + (np[1] - tz) * hz),
                                   np[1] - tz - hz * ((np[0] - tx) * hx + (np[1] - tz) * hz)) : 0;
  const latLabel = latSign >= 0 ? "R" : "L";

  const lines: string[] = [
    `── Counterflow-Diagnose ──  route n=${route.length}  window=${slice.length}`,
    `truck  x=${f1(tx)} z=${f1(tz)}  hdg=${pose.heading.toFixed(3)}`,
    `hdg-vec  hx=${f2(hx)} hz=${f2(hz)}  (${f2(hx)}·east + ${f2(hz)}·south)`,
    `nearest wp#${ni}  dist=${f1(np ? Math.hypot(np[0]-tx, np[1]-tz) : 0)}m  lat=${f1(latDist)}m ${latLabel}`,
    ``,
    `seg#  dot(hdg,seg)  verdict            seg-vec (dx  dz)`,
  ];

  // First 5 raw route segments starting from nearest.
  const segEnd = Math.min(ni + 5, route.length - 1);
  for (let i = ni; i < segEnd; i++) {
    const a = route[i], b = route[i + 1];
    if (!a || !b) continue;
    const sdx0 = b[0] - a[0];
    const sdz0 = b[1] - a[1];
    const slen = Math.hypot(sdx0, sdz0);
    if (slen < 0.001) continue;
    const sdx = sdx0 / slen;
    const sdz = sdz0 / slen;
    const dot = hx * sdx + hz * sdz;
    const verdict = dot < -0.5 ? "<<< COUNTERFLOW" : dot > 0.5 ? "same dir" : "perpendicular?";
    lines.push(`  ${i}→${i+1}  dot=${f2(dot).padStart(6)}  ${verdict.padEnd(16)}  (${f2(sdx)}  ${f2(sdz)})`);
  }

  ctx.save();
  ctx.font = "12px ui-monospace, SFMono-Regular, Menlo, monospace";
  ctx.textBaseline = "top";
  const lh = 15;
  const pad = 6;
  const boxW = 640;
  const boxH = lines.length * lh + pad * 2;
  const x0 = 8;
  const y0 = cssH - boxH - 8;
  ctx.fillStyle = "rgba(0,0,0,0.72)";
  ctx.fillRect(x0, y0, boxW, boxH);
  lines.forEach((ln, i) => {
    const isCounterflow = ln.includes("COUNTERFLOW");
    ctx.fillStyle = isCounterflow ? "rgba(255,100,100,1)" : "rgba(180,255,180,0.96)";
    ctx.fillText(ln, x0 + pad, y0 + pad + i * lh);
  });
  ctx.restore();
}

import { useEffect, useRef } from "react";
import {
  mapLanePointsToSchematic,
  type MapPoint2D,
  type OverlaySnapshot,
} from "./overlay-snapshot";

const BOX_W = 300;
const BOX_H = 200;
const PAD = 14;

function strokePolyline(
  ctx: CanvasRenderingContext2D,
  points: [number, number][],
  style: { color: string; width: number; dashed: boolean },
) {
  if (points.length < 2) return;
  ctx.beginPath();
  ctx.moveTo(points[0]![0], points[0]![1]);
  for (let i = 1; i < points.length; i++) {
    const p = points[i];
    if (!p) continue;
    ctx.lineTo(p[0], p[1]);
  }
  ctx.lineWidth = style.width;
  ctx.strokeStyle = style.color;
  ctx.setLineDash(style.dashed ? [6, 4] : []);
  ctx.stroke();
  ctx.setLineDash([]);
}

function drawSegmentTicks(
  ctx: CanvasRenderingContext2D,
  centerline: [number, number][],
  segments: OverlaySnapshot["lane"]["spline_segments"],
) {
  ctx.fillStyle = "rgba(250, 204, 21, 0.95)";
  for (const seg of segments) {
    const p = centerline[seg.start_idx];
    if (!p) continue;
    ctx.beginPath();
    ctx.arc(p[0], p[1], 3, 0, Math.PI * 2);
    ctx.fill();
  }
}

function drawNodeLabels(
  ctx: CanvasRenderingContext2D,
  centerline: [number, number][],
  nodeIds: number[],
) {
  ctx.font = "9px ui-monospace, SFMono-Regular, Menlo, monospace";
  ctx.fillStyle = "rgba(255,255,255,0.75)";
  ctx.textAlign = "center";
  ctx.textBaseline = "bottom";
  for (let i = 0; i < centerline.length; i++) {
    const p = centerline[i];
    const id = nodeIds[i];
    if (!p || id == null) continue;
    ctx.fillText(String(id), p[0], p[1] - 5);
  }
}

/// Read-only schematic lane debug (center/left/right + spline ticks).
/// Dashed when `lane_model_valid=false`; MOCK badge when `source=mock`.
export function LaneDebugCanvas({ snapshot }: { snapshot: OverlaySnapshot }) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.round(BOX_W * dpr);
    canvas.height = Math.round(BOX_H * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, BOX_W, BOX_H);

    const { lane } = snapshot;
    const dashed = !lane.lane_model_valid;
    const isMock = lane.source === "mock";

    ctx.fillStyle = "rgba(0,0,0,0.72)";
    ctx.fillRect(0, 0, BOX_W, BOX_H);
    ctx.strokeStyle = "rgba(255,255,255,0.12)";
    ctx.strokeRect(0.5, 0.5, BOX_W - 1, BOX_H - 1);

    ctx.font = "11px ui-monospace, SFMono-Regular, Menlo, monospace";
    ctx.textBaseline = "top";
    ctx.fillStyle = isMock ? "rgba(251, 191, 36, 0.95)" : "rgba(180,255,180,0.95)";
    ctx.fillText(isMock ? "LANE DEBUG · MOCK" : "LANE DEBUG", PAD, 6);
    ctx.fillStyle = "rgba(255,255,255,0.55)";
    ctx.fillText(
      dashed ? "invalid model — dashed" : "valid model",
      PAD,
      20,
    );

    const toScreen = (pts: MapPoint2D[]) =>
      mapLanePointsToSchematic(pts, BOX_W, BOX_H, PAD + 18);

    const left = toScreen(lane.left_lane_points);
    const center = toScreen(lane.centerline_points);
    const right = toScreen(lane.right_lane_points);

    strokePolyline(ctx, left, {
      color: "rgba(251, 146, 60, 0.85)",
      width: 1.5,
      dashed,
    });
    strokePolyline(ctx, right, {
      color: "rgba(251, 146, 60, 0.85)",
      width: 1.5,
      dashed,
    });
    strokePolyline(ctx, center, {
      color: dashed ? "rgba(34, 211, 238, 0.55)" : "rgba(34, 211, 238, 0.95)",
      width: 2.5,
      dashed,
    });

    drawSegmentTicks(ctx, center, lane.spline_segments);
    drawNodeLabels(ctx, center, lane.node_ids);
  }, [snapshot]);

  return (
    <canvas
      ref={canvasRef}
      width={BOX_W}
      height={BOX_H}
      className="pointer-events-none absolute bottom-3 right-3 rounded-md shadow-lg ring-1 ring-white/10"
      aria-label="Lane debug schematic"
    />
  );
}

import { useEffect, useRef } from "react";

interface MiniMapProps {
  position: [number, number, number];
  routePoints?: [number, number][];
  worldBounds?: { minX: number; maxX: number; minZ: number; maxZ: number };
}

const DEFAULT_BOUNDS = { minX: -100_000, maxX: 100_000, minZ: -100_000, maxZ: 100_000 };

export function MiniMap({ position, routePoints, worldBounds = DEFAULT_BOUNDS }: MiniMapProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
      canvas.width = w * dpr;
      canvas.height = h * dpr;
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    const toX = (x: number) => ((x - worldBounds.minX) / (worldBounds.maxX - worldBounds.minX)) * w;
    const toY = (z: number) => ((z - worldBounds.minZ) / (worldBounds.maxZ - worldBounds.minZ)) * h;

    ctx.strokeStyle = "rgba(120,120,120,0.35)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(w / 2, 0);
    ctx.lineTo(w / 2, h);
    ctx.moveTo(0, h / 2);
    ctx.lineTo(w, h / 2);
    ctx.stroke();

    if (routePoints && routePoints.length > 1) {
      ctx.strokeStyle = "rgba(99,102,241,0.85)";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      const first = routePoints[0];
      if (first) ctx.moveTo(toX(first[0]), toY(first[1]));
      for (let i = 1; i < routePoints.length; i += 1) {
        const p = routePoints[i];
        if (!p) continue;
        ctx.lineTo(toX(p[0]), toY(p[1]));
      }
      ctx.stroke();
    }

    const px = toX(position[0]);
    const py = toY(position[2]);
    ctx.fillStyle = "#10b981";
    ctx.beginPath();
    ctx.arc(px, py, 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = "rgba(16,185,129,0.4)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(px, py, 9, 0, Math.PI * 2);
    ctx.stroke();
  }, [position, routePoints, worldBounds]);

  return (
    <canvas
      ref={canvasRef}
      className="h-32 w-full rounded-md border bg-muted/30"
      role="img"
      aria-label="Mini map"
    />
  );
}

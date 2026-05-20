// crates/ui/src/components/RouteMap.tsx
//
// Example:
//   <RouteMap
//     truckPosition={{ x: truck.x, z: truck.z }}
//     truckHeading={truck.heading}
//     truckSpeed={telemetry.speedKmh}
//     nearbyEdges={mapEdges}
//     nearbyNodes={mapNodes}
//     routeWaypoints={routeWaypoints}
//     goalPosition={goal}
//     nextManeuverPosition={maneuverPos}
//     nextManeuverDistance={maneuverDist}
//     isEngaged={engageState === "engaged"}
//     isOffRoute={offRoute}
//   />
//
// ETS2LA-style top-down map. Pan + 5 discrete zoom levels + follow-truck +
// debug overlay. World coords (meters) project to screen via a single matrix
// transform on the outer <g>, so road / route paths only re-memoize when the
// map data itself changes — not on truck-position ticks.

import * as React from "react";
import { MapPinOff, Crosshair, Bug, Plus, Minus, Flag } from "lucide-react";
import type {
  MapEdge,
  MapNode,
  RouteWaypoint,
  WorldPoint,
} from "@/types/map";
import { cn } from "@/lib/utils";

// ---- Zoom levels: meters visible across the smaller container dimension. ---
const ZOOM_LEVELS_M = [250, 500, 1000, 2500, 5000] as const;
const DEFAULT_ZOOM_IDX = 2;

export interface RouteMapProps {
  // Live telemetry
  truckPosition: WorldPoint | null;
  truckHeading: number | null;      // radians, 0 = +Z (north), clockwise
  truckSpeed: number;               // km/h, for header readout
  // Map data (1 Hz spatial subset from daemon)
  nearbyEdges: MapEdge[];
  nearbyNodes: MapNode[];
  // Route data
  routeWaypoints: RouteWaypoint[];
  goalPosition: WorldPoint | null;
  nextManeuverPosition: WorldPoint | null;
  nextManeuverDistance: number | null;   // meters
  // UI state
  isEngaged: boolean;
  isOffRoute: boolean;
  // Per-card settings (controlled or initial)
  followTruck?: boolean;            // controlled; omit for uncontrolled (default true)
  showDebugLayer?: boolean;         // controlled; omit for uncontrolled (default false)
  onFollowTruckChange?: (v: boolean) => void;
  onShowDebugLayerChange?: (v: boolean) => void;
  // Container
  height?: number;                  // default 320
  className?: string;
}

const ROAD_STROKE: Record<NonNullable<MapEdge["road_type"]>, number> = {
  highway: 2.0,
  road:    1.4,
  prefab:  1.0,
  bezier:  1.0,
};

export function RouteMap({
  truckPosition,
  truckHeading,
  truckSpeed,
  nearbyEdges,
  nearbyNodes,
  routeWaypoints,
  goalPosition,
  nextManeuverPosition,
  nextManeuverDistance,
  isEngaged,
  isOffRoute,
  followTruck: followProp,
  showDebugLayer: debugProp,
  onFollowTruckChange,
  onShowDebugLayerChange,
  height = 320,
  className,
}: RouteMapProps) {
  // ---------- uncontrolled fallbacks ----------
  const [followState, setFollowState] = React.useState(true);
  const [debugState, setDebugState] = React.useState(false);
  const follow = followProp ?? followState;
  const debug  = debugProp ?? debugState;
  const setFollow = (v: boolean) => {
    setFollowState(v);
    onFollowTruckChange?.(v);
  };
  const setDebug = (v: boolean) => {
    setDebugState(v);
    onShowDebugLayerChange?.(v);
  };

  // ---------- zoom + manual view center ----------
  const [zoomIdx, setZoomIdx] = React.useState<number>(DEFAULT_ZOOM_IDX);
  const [viewCenter, setViewCenter] = React.useState<WorldPoint | null>(null);
  // when follow is true, viewCenter is forced to truck; when user pans we detach.

  // ---------- measure container ----------
  const wrapRef = React.useRef<HTMLDivElement | null>(null);
  const [size, setSize] = React.useState({ w: 600, h: height });
  React.useEffect(() => {
    const el = wrapRef.current;
    if (!el || typeof ResizeObserver === "undefined") return;
    const ro = new ResizeObserver(([entry]) => {
      const cr = entry.contentRect;
      setSize({ w: Math.max(1, cr.width), h: Math.max(1, cr.height) });
    });
    ro.observe(el);
    return () => ro.disconnect();
  }, []);

  // ---------- projection ----------
  const visibleMeters = ZOOM_LEVELS_M[zoomIdx];
  const scale = Math.min(size.w, size.h) / visibleMeters; // px per meter

  const center: WorldPoint =
    follow && truckPosition
      ? truckPosition
      : viewCenter ?? truckPosition ?? { x: 0, z: 0 };

  // SVG transform: world (x, z) → pixel. Y flipped (ETS2 +Z = north → SVG up).
  // matrix(a b c d e f) ⇒ [ a c e ; b d f ]
  // x' = scale*x + 0*z + (W/2 - scale*cx)
  // y' = 0*x + (-scale)*z + (H/2 + scale*cz)
  const matrix = `matrix(${scale} 0 0 ${-scale} ${size.w / 2 - scale * center.x} ${size.h / 2 + scale * center.z})`;

  // Same math, exposed as a function. Markers render OUTSIDE the matrix group
  // so their pixel sizes are independent of zoom.
  const project = React.useCallback(
    (p: WorldPoint) => ({
      px: size.w / 2 + scale * (p.x - center.x),
      py: size.h / 2 - scale * (p.z - center.z),
    }),
    [scale, center.x, center.z, size.w, size.h],
  );

  // ---------- memoized geometry (depends only on map data) ----------
  const roadsByType = React.useMemo(() => {
    const buckets: Record<string, string[]> = { highway: [], road: [], prefab: [], bezier: [] };
    for (const e of nearbyEdges) {
      const t = e.road_type ?? "road";
      buckets[t].push(`M${e.from.x} ${e.from.z}L${e.to.x} ${e.to.z}`);
    }
    return buckets;
  }, [nearbyEdges]);

  const routePath = React.useMemo(() => {
    if (routeWaypoints.length < 2) return "";
    return (
      "M" +
      routeWaypoints.map((p) => `${p.x.toFixed(1)} ${p.z.toFixed(1)}`).join("L")
    );
  }, [routeWaypoints]);

  // ---------- pan ----------
  const dragRef = React.useRef<{ startX: number; startY: number; startCenter: WorldPoint } | null>(null);

  const onPointerDown = (e: React.PointerEvent<HTMLDivElement>) => {
    if (e.button !== 0) return;
    (e.target as Element).setPointerCapture?.(e.pointerId);
    dragRef.current = {
      startX: e.clientX,
      startY: e.clientY,
      startCenter: { ...center },
    };
  };

  const onPointerMove = (e: React.PointerEvent<HTMLDivElement>) => {
    const d = dragRef.current;
    if (!d) return;
    const dx = (e.clientX - d.startX) / scale;
    const dy = (e.clientY - d.startY) / scale;
    // moving the mouse RIGHT pans the world LEFT relative to camera (center moves left)
    setFollow(false);
    setViewCenter({
      x: d.startCenter.x - dx,
      z: d.startCenter.z + dy,   // y flipped: dragging down increases z (north)
    });
  };

  const onPointerUp = () => { dragRef.current = null; };

  // ---------- zoom (wheel) ----------
  const onWheel: React.WheelEventHandler<HTMLDivElement> = (e) => {
    e.preventDefault();
    const dir = e.deltaY > 0 ? 1 : -1;
    setZoomIdx((i) => Math.max(0, Math.min(ZOOM_LEVELS_M.length - 1, i + dir)));
  };

  // ---------- empty states ----------
  const showNoTelemetry = !truckPosition;
  const showNoRoute     = !showNoTelemetry && routeWaypoints.length < 2;

  // ---------- approach pulse ----------
  const approaching =
    nextManeuverPosition != null &&
    nextManeuverDistance != null &&
    nextManeuverDistance < 200;

  // ---------- truck heading (radians → SVG degrees) ----------
  // ETS2 heading: 0 rad = +Z (north). We rotate the marker so its nose points
  // along that direction. Inside the matrix-transformed group, "up" is +Z, so
  // a rotate(0) marker points up. Heading goes clockwise (east, south, west).
  const headingDeg =
    truckHeading != null ? (truckHeading * 180) / Math.PI : 0;

  return (
    <div
      ref={wrapRef}
      onPointerDown={onPointerDown}
      onPointerMove={onPointerMove}
      onPointerUp={onPointerUp}
      onPointerCancel={onPointerUp}
      onWheel={onWheel}
      className={cn(
        "relative w-full bg-surface-elevated border border-subtle rounded-sm overflow-hidden select-none",
        dragRef.current ? "cursor-grabbing" : follow ? "cursor-default" : "cursor-grab",
        className,
      )}
      style={{ height, touchAction: "none" }}
      role="img"
      aria-label="Route preview"
    >
      <svg
        width="100%"
        height="100%"
        viewBox={`0 0 ${size.w} ${size.h}`}
        preserveAspectRatio="xMidYMid slice"
      >
        <g transform={matrix}>
          {/* ── Roads (background) ──────────────────────────────────────── */}
          <g stroke="var(--text-muted)" strokeOpacity={0.30} fill="none" strokeLinecap="round" strokeLinejoin="round">
            {roadsByType.bezier.length > 0 && (
              <path d={roadsByType.bezier.join(" ")} strokeWidth={ROAD_STROKE.bezier} vectorEffect="non-scaling-stroke" />
            )}
            {roadsByType.prefab.length > 0 && (
              <path d={roadsByType.prefab.join(" ")} strokeWidth={ROAD_STROKE.prefab} vectorEffect="non-scaling-stroke" />
            )}
            {roadsByType.road.length > 0 && (
              <path d={roadsByType.road.join(" ")} strokeWidth={ROAD_STROKE.road} vectorEffect="non-scaling-stroke" />
            )}
            {roadsByType.highway.length > 0 && (
              <path d={roadsByType.highway.join(" ")} strokeWidth={ROAD_STROKE.highway} strokeOpacity={0.45} vectorEffect="non-scaling-stroke" />
            )}
          </g>

          {/* ── Planned route ───────────────────────────────────────────── */}
          {routePath && (
            <path
              d={routePath}
              fill="none"
              stroke="var(--brand)"
              strokeWidth={3}
              strokeOpacity={isOffRoute ? 0.35 : 1}
              strokeLinecap="round"
              strokeLinejoin="round"
              vectorEffect="non-scaling-stroke"
            />
          )}
        </g>

        {/* ── Markers in PIXEL space — sizes independent of zoom. ─────── */}

        {debug && (
          <g fill="var(--text-muted)" opacity={0.4}>
            {nearbyNodes.map((n) => {
              const { px, py } = project(n.position);
              return (
                <circle key={n.uid} cx={px} cy={py} r={n.is_junction ? 2.4 : 1.4} />
              );
            })}
          </g>
        )}

        {goalPosition && (() => {
          const { px, py } = project(goalPosition);
          return (
            <g transform={`translate(${px} ${py})`}>
              <circle r={7} fill="var(--surface-card)" stroke="#A78BFA" strokeWidth={2} />
              <circle r={2.6} fill="#A78BFA" />
            </g>
          );
        })()}

        {nextManeuverPosition && (() => {
          const { px, py } = project(nextManeuverPosition);
          return (
            <g transform={`translate(${px} ${py})`}>
              <circle
                r={approaching ? 12 : 8}
                fill="var(--brand-soft)"
                stroke="var(--brand)"
                strokeWidth={1.5}
                className={approaching ? "[animation:pulse-soft_2.4s_ease-in-out_infinite] motion-reduce:[animation:none]" : undefined}
              />
              <circle r={2} fill="var(--brand)" />
            </g>
          );
        })()}

        {truckPosition && (() => {
          const { px, py } = project(truckPosition);
          return (
            <g transform={`translate(${px} ${py}) rotate(${headingDeg})`}>
              {isEngaged && (
                <circle
                  r={16}
                  fill="none"
                  stroke="var(--brand)"
                  strokeOpacity={0.5}
                  strokeWidth={1.5}
                  className="[animation:pulse-soft_2.4s_ease-in-out_infinite] motion-reduce:[animation:none]"
                />
              )}
              {/* Body */}
              <circle r={5} fill="var(--brand)" />
              {/* Heading indicator — triangle pointing along +Z (screen up at rotation 0) */}
              <path d="M0 -12 L4.5 -3 L-4.5 -3 Z" fill="var(--brand)" />
            </g>
          );
        })()}
      </svg>

      {/* ── Top-left badges ───────────────────────────────────────────── */}
      <div className="absolute top-2 left-2 flex items-center gap-1.5 pointer-events-none">
        <span className="inline-flex items-center gap-1.5 h-5 px-1.5 rounded-sm border border-subtle bg-surface-overlay font-mono text-[10px] uppercase tracking-wider text-fg-muted">
          <span className={cn("w-1.5 h-1.5 rounded-full", truckPosition ? "bg-success" : "bg-fg-muted")} />
          {truckPosition ? "live" : "offline"}
        </span>
        {truckPosition && (
          <span className="inline-flex items-center h-5 px-1.5 rounded-sm border border-subtle bg-surface-overlay font-mono text-[10px] text-fg-secondary tabular-nums">
            {Math.round(truckSpeed)} <span className="text-fg-muted ml-0.5">km/h</span>
          </span>
        )}
      </div>

      {/* ── Top-right zoom controls ───────────────────────────────────── */}
      <div className="absolute top-2 right-2 inline-flex flex-col rounded-sm border border-subtle bg-surface-overlay overflow-hidden">
        <button
          type="button"
          onClick={() => setZoomIdx((i) => Math.max(0, i - 1))}
          disabled={zoomIdx === 0}
          aria-label="Zoom in"
          className="w-6 h-6 inline-flex items-center justify-center text-fg-secondary hover:text-fg hover:bg-surface-elevated disabled:opacity-40 disabled:hover:bg-transparent transition-colors"
        >
          <Plus size={12} />
        </button>
        <button
          type="button"
          onClick={() => setZoomIdx((i) => Math.min(ZOOM_LEVELS_M.length - 1, i + 1))}
          disabled={zoomIdx === ZOOM_LEVELS_M.length - 1}
          aria-label="Zoom out"
          className="w-6 h-6 inline-flex items-center justify-center text-fg-secondary hover:text-fg hover:bg-surface-elevated disabled:opacity-40 disabled:hover:bg-transparent transition-colors border-t border-subtle"
        >
          <Minus size={12} />
        </button>
      </div>

      {/* ── Center empty states ───────────────────────────────────────── */}
      {(showNoTelemetry || showNoRoute || isOffRoute) && (
        <div className="absolute inset-x-0 bottom-9 flex justify-center pointer-events-none">
          <div className="inline-flex items-center gap-2 px-2.5 h-7 rounded-sm border border-subtle bg-surface-overlay font-sans text-xs text-fg-muted">
            {showNoTelemetry ? (
              <>
                <MapPinOff size={12} />
                Waiting for telemetry
              </>
            ) : showNoRoute ? (
              <>
                <Flag size={12} />
                Set destination to see route
              </>
            ) : (
              <>
                <Crosshair size={12} />
                Off route — recalculating…
              </>
            )}
          </div>
        </div>
      )}

      {/* ── Footer controls ───────────────────────────────────────────── */}
      <div className="absolute inset-x-0 bottom-0 h-7 px-2 flex items-center gap-3 border-t border-subtle bg-surface-overlay/80 backdrop-blur-[2px]">
        <span className="font-mono text-[10px] text-fg-muted tabular-nums">
          1:{Math.round(visibleMeters / Math.max(1, size.h / 100))}
        </span>
        <span className="font-mono text-[10px] text-fg-muted">
          {visibleMeters >= 1000 ? `${visibleMeters / 1000} km` : `${visibleMeters} m`}
        </span>

        <span className="ml-auto inline-flex items-center gap-3">
          <button
            type="button"
            onClick={() => {
              setFollow(!follow);
              if (!follow) setViewCenter(null);
            }}
            className={cn(
              "inline-flex items-center gap-1 h-5 px-1.5 rounded-sm font-mono text-[10px] uppercase tracking-wider",
              follow ? "text-brand" : "text-fg-muted hover:text-fg",
            )}
            aria-pressed={follow}
            title="Center on truck"
          >
            <Crosshair size={11} />
            follow {follow ? "on" : "off"}
          </button>
          <button
            type="button"
            onClick={() => setDebug(!debug)}
            className={cn(
              "inline-flex items-center gap-1 h-5 px-1.5 rounded-sm font-mono text-[10px] uppercase tracking-wider",
              debug ? "text-brand" : "text-fg-muted hover:text-fg",
            )}
            aria-pressed={debug}
            title="Show map-node debug layer"
          >
            <Bug size={11} />
            debug {debug ? "on" : "off"}
          </button>
        </span>
      </div>
    </div>
  );
}

export default RouteMap;

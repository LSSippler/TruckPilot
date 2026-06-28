import { useCallback, useEffect, useMemo, useState, type ReactElement } from "react";
import { useSearchParams } from "react-router-dom";
import {
  buildInternalVizModel,
  CURVATURE_HEATMAP_LEGEND,
  formatPositionLabel,
  formatSpeedLabel,
  resolveHeatmapEnabledFromSearch,
  resolveTruckWorldPoint,
  type InternalVizModel,
  type VizFeedBadge,
  type VizSourceBadge,
} from "./internal-path-viz";
import {
  clampVizZoom,
  nudgeVizZoom,
  VIZ_ZOOM_STEP,
} from "./overlay-layout";
import type { OverlaySnapshot, OverlaySnapshotFeed } from "./overlay-snapshot";
import { cn } from "@/lib/utils";

const BOX_W = 420;
const BOX_H = 380;

function GridLines({ model }: { model: InternalVizModel }) {
  const { viewport } = model;
  const lines: ReactElement[] = [];
  const steps = 4;
  for (let i = 0; i <= steps; i++) {
    const t = i / steps;
    const x = viewport.padding + t * (viewport.width - viewport.padding * 2);
    const y = viewport.padding + t * (viewport.height - viewport.padding * 2);
    lines.push(
      <line
        key={`v-${i}`}
        x1={x}
        y1={viewport.padding}
        x2={x}
        y2={viewport.height - viewport.padding}
        stroke="rgba(255,255,255,0.06)"
        strokeWidth={1}
      />,
    );
    lines.push(
      <line
        key={`h-${i}`}
        x1={viewport.padding}
        y1={y}
        x2={viewport.width - viewport.padding}
        y2={y}
        stroke="rgba(255,255,255,0.06)"
        strokeWidth={1}
      />,
    );
  }
  return <g aria-hidden>{lines}</g>;
}

function Legend() {
  const entries = [
    { color: "#86efac", label: "road / current" },
    { color: "#fbbf24", label: "junction" },
    { color: "#f472b6", label: "lane change" },
    { color: "#22d3ee", label: "nav curve" },
    { color: "#fb923c", label: "lane edges" },
  ];
  return (
    <g transform={`translate(${BOX_W - 118}, 52)`}>
      {entries.map((e, i) => (
        <g key={e.label} transform={`translate(0, ${i * 14})`}>
          <line x1={0} y1={6} x2={14} y2={6} stroke={e.color} strokeWidth={2} />
          <text x={18} y={9} fill="rgba(255,255,255,0.65)" fontSize={9}>
            {e.label}
          </text>
        </g>
      ))}
    </g>
  );
}

function CurvatureHeatmapLegend({ enabled }: { enabled: boolean }) {
  if (!enabled) return null;
  return (
    <g transform={`translate(${BOX_W - 118}, 132)`} aria-label="Curvature heatmap legend">
      <text x={0} y={0} fill="rgba(255,255,255,0.5)" fontSize={8}>
        κ heatmap
      </text>
      {CURVATURE_HEATMAP_LEGEND.map((e, i) => (
        <g key={e.severity} transform={`translate(0, ${8 + i * 12})`}>
          <line x1={0} y1={6} x2={12} y2={6} stroke={e.color} strokeWidth={3} strokeLinecap="round" />
          <text x={16} y={9} fill="rgba(255,255,255,0.6)" fontSize={8}>
            {e.label}
          </text>
        </g>
      ))}
      <text x={0} y={62} fill="rgba(255,255,255,0.4)" fontSize={7}>
        display-only
      </text>
    </g>
  );
}

function HeatmapControl({
  enabled,
  interactive,
  onToggle,
}: {
  enabled: boolean;
  interactive: boolean;
  onToggle?: () => void;
}) {
  const label = `Heatmap: ${enabled ? "on" : "off"}`;
  if (!interactive) {
    return (
      <text
        x={BOX_W - 12}
        y={16}
        textAnchor="end"
        fill={enabled ? "rgba(251, 191, 36, 0.75)" : "rgba(255,255,255,0.45)"}
        fontSize={9}
        aria-label={`Curvature heatmap ${enabled ? "on" : "off"} (display only)`}
      >
        {label}
      </text>
    );
  }
  return (
    <foreignObject x={BOX_W - 108} y={4} width={96} height={22}>
      <button
        type="button"
        onClick={onToggle}
        className={cn(
          "pointer-events-auto rounded px-1.5 py-0.5 text-[9px] font-medium leading-none",
          enabled
            ? "bg-amber-500/20 text-amber-200 ring-1 ring-amber-400/40"
            : "bg-white/5 text-white/50 ring-1 ring-white/10",
        )}
        aria-pressed={enabled}
        aria-label={`Curvature heatmap ${enabled ? "on" : "off"}`}
      >
        {label}
      </button>
    </foreignObject>
  );
}

function PolylinePath({
  points,
  stroke,
  width,
  dashed,
  opacity = 1,
}: {
  points: [number, number][];
  stroke: string;
  width: number;
  dashed: boolean;
  opacity?: number;
}) {
  if (points.length < 2) return null;
  const d = points.map((p, i) => `${i === 0 ? "M" : "L"} ${p[0]} ${p[1]}`).join(" ");
  return (
    <path
      d={d}
      fill="none"
      stroke={stroke}
      strokeWidth={width}
      strokeOpacity={opacity}
      strokeDasharray={dashed ? "6 4" : undefined}
      strokeLinecap="round"
      strokeLinejoin="round"
    />
  );
}

function segmentMidpoint(points: [number, number][]): [number, number] | null {
  if (points.length === 0) return null;
  return points[Math.floor(points.length / 2)] ?? null;
}

function StatsBlock({ model }: { model: InternalVizModel }) {
  if (!model.hasPlannedPath) return null;
  let y = model.feedBadge || model.sourceBadge ? 102 : 78;
  if (model.feedBadge && model.sourceBadge) y = 102;
  else if (model.feedBadge || model.sourceBadge) y = 90;
  const lines = [
    model.curvatureStatsLine,
    model.kindStatsLine,
    model.currentItemLine,
  ].filter(Boolean) as string[];
  return (
    <g aria-label="Path diagnostics">
      {lines.map((line) => {
        const el = (
          <text x={12} y={y} fill="rgba(200,220,255,0.72)" fontSize={8}>
            {line}
          </text>
        );
        y += 11;
        return <g key={line}>{el}</g>;
      })}
    </g>
  );
}

function FeedBadge({ badge }: { badge: VizFeedBadge }) {
  if (!badge) return null;
  const colors: Record<NonNullable<VizFeedBadge>, string> = {
    FIXTURE: "rgba(196, 181, 253, 0.95)",
    STORAGE: "rgba(148, 163, 184, 0.95)",
    LIVE: "rgba(56, 189, 248, 0.95)",
  };
  return (
    <text x={12} y={32} fill={colors[badge]} fontSize={10} fontWeight={700}>
      {badge}
    </text>
  );
}

function SourceBadge({ badge, y = 32 }: { badge: VizSourceBadge; y?: number }) {
  if (!badge) return null;
  const colors: Record<NonNullable<VizSourceBadge>, string> = {
    OFFLINE: "rgba(134, 239, 172, 0.95)",
    MOCK: "rgba(251, 191, 36, 0.95)",
    LIVE: "rgba(56, 189, 248, 0.95)",
    UNKNOWN: "rgba(161, 161, 170, 0.9)",
  };
  return (
    <text x={12} y={y} fill={colors[badge]} fontSize={10} fontWeight={700}>
      {badge}
    </text>
  );
}

/// Read-only top-down path visualization (PlannedPath + lane debug). No control side effects.
export function InternalPathVisualization({
  snapshot,
  feed = "fixture",
  editorMode = false,
  zoom = 1,
  onZoomChange,
}: {
  snapshot: OverlaySnapshot;
  feed?: OverlaySnapshotFeed;
  editorMode?: boolean;
  zoom?: number;
  onZoomChange?: (zoom: number) => void;
}) {
  const [searchParams] = useSearchParams();
  const heatmapFromUrl = resolveHeatmapEnabledFromSearch(searchParams);
  const mapZoom = clampVizZoom(zoom);
  const [editorHeatmap, setEditorHeatmap] = useState(heatmapFromUrl);
  useEffect(() => {
    setEditorHeatmap(heatmapFromUrl);
  }, [heatmapFromUrl]);
  const heatmapEnabled = editorMode ? editorHeatmap : heatmapFromUrl;
  const model = useMemo(
    () => buildInternalVizModel(snapshot, BOX_W, BOX_H, 28, mapZoom, heatmapEnabled, feed),
    [snapshot, mapZoom, heatmapEnabled, feed],
  );
  const toggleHeatmap = useCallback(() => {
    setEditorHeatmap((v) => !v);
  }, []);
  const position = formatPositionLabel(resolveTruckWorldPoint(snapshot));
  const speed = formatSpeedLabel(snapshot);
  const hasBadges = model.feedBadge != null || model.sourceBadge != null;
  const headerY = hasBadges ? (model.feedBadge && model.sourceBadge ? 58 : 46) : 34;

  const onWheel = useCallback(
    (e: React.WheelEvent<HTMLDivElement>) => {
      if (!editorMode || !onZoomChange) return;
      e.preventDefault();
      e.stopPropagation();
      const delta = e.deltaY < 0 ? VIZ_ZOOM_STEP : -VIZ_ZOOM_STEP;
      onZoomChange(nudgeVizZoom(mapZoom, delta));
    },
    [editorMode, mapZoom, onZoomChange],
  );

  return (
    <div
      className={cn(
        "overflow-hidden rounded-md shadow-lg ring-1 ring-white/15",
        editorMode && onZoomChange ? "pointer-events-auto" : "pointer-events-none",
      )}
      aria-label="Internal path visualization"
      onWheel={onWheel}
    >
      <svg
        width={BOX_W}
        height={BOX_H}
        viewBox={`0 0 ${BOX_W} ${BOX_H}`}
        className="block bg-black/75"
      >
        <rect width={BOX_W} height={BOX_H} fill="rgba(0,0,0,0.78)" />
        <rect
          x={0.5}
          y={0.5}
          width={BOX_W - 1}
          height={BOX_H - 1}
          fill="none"
          stroke="rgba(255,255,255,0.12)"
        />

        <text x={12} y={18} fill="rgba(255,255,255,0.95)" fontSize={12} fontWeight={600}>
          Internal Visualization
        </text>
        <HeatmapControl
          enabled={heatmapEnabled}
          interactive={editorMode}
          onToggle={toggleHeatmap}
        />
        <FeedBadge badge={model.feedBadge} />
        <SourceBadge badge={model.sourceBadge} y={model.feedBadge ? 44 : 32} />
        <text x={12} y={headerY} fill="rgba(255,255,255,0.6)" fontSize={10}>
          Speed: {speed} · Pos: {position}
        </text>
        {editorMode ? (
          <text x={12} y={headerY + 14} fill="rgba(251, 191, 36, 0.8)" fontSize={9}>
            Zoom: {mapZoom.toFixed(2)}× · Mausrad / + −
          </text>
        ) : null}
        {model.missingPlannedPathMessage ? (
          <text x={12} y={62} fill="rgba(255,255,255,0.55)" fontSize={10}>
            {model.missingPlannedPathMessage}
          </text>
        ) : null}
        {model.liveDebugLine ? (
          <text x={12} y={74} fill="rgba(180,200,220,0.65)" fontSize={8}>
            {model.liveDebugLine}
          </text>
        ) : null}
        {model.nearestText ? (
          <text x={12} y={76} fill="rgba(180,255,180,0.75)" fontSize={9}>
            {model.nearestText}
          </text>
        ) : null}
        <StatsBlock model={model} />
        <text x={12} y={BOX_H - 10} fill="rgba(255,255,255,0.5)" fontSize={9}>
          Drive (display): {model.driveDisplay}
        </text>

        <GridLines model={model} />
        <Legend />
        <CurvatureHeatmapLegend enabled={heatmapEnabled} />

        {model.lanePolylines.map((line) => (
          <PolylinePath
            key={line.role}
            points={line.points}
            stroke={
              line.role === "center"
                ? model.dimmed
                  ? "rgba(34,211,238,0.45)"
                  : "rgba(34,211,238,0.85)"
                : "rgba(251,146,60,0.55)"
            }
            width={line.role === "center" ? 1.5 : 1}
            dashed={model.dimmed}
          />
        ))}

        {model.plannedSegments.map((seg) => (
          <g key={`seg-${seg.id}`}>
            {seg.heatmap ? (
              <PolylinePath
                points={seg.points}
                stroke={seg.heatmap.stroke}
                width={seg.heatmap.width}
                dashed={false}
                opacity={seg.heatmap.opacity}
              />
            ) : null}
            <PolylinePath
              points={seg.points}
              stroke={seg.style.stroke}
              width={seg.style.width}
              dashed={seg.style.dashed}
              opacity={seg.style.opacity}
            />
            {seg.points.length > 0 ? (
              <>
                {seg.points.map((p, i) => (
                  <circle
                    key={`${seg.id}-pt-${i}`}
                    cx={p[0]}
                    cy={p[1]}
                    r={seg.isCurrent ? 3 : 2}
                    fill={seg.style.stroke}
                    opacity={seg.style.opacity * 0.85}
                  />
                ))}
                <text
                  x={seg.points[0]![0] + 4}
                  y={seg.points[0]![1] - 4}
                  fill="rgba(255,255,255,0.55)"
                  fontSize={8}
                >
                  {seg.label}
                </text>
                {seg.heatmap?.showTick
                  ? (() => {
                      const mid = segmentMidpoint(seg.points);
                      if (!mid) return null;
                      return (
                        <line
                          x1={mid[0] - 3}
                          y1={mid[1]}
                          x2={mid[0] + 3}
                          y2={mid[1]}
                          stroke={seg.heatmap!.stroke}
                          strokeWidth={2}
                          strokeLinecap="round"
                        />
                      );
                    })()
                  : null}
                {(seg.heatmap?.showMidpointRing ||
                  (!heatmapEnabled && seg.severity === "high")) &&
                seg.points.length > 0
                  ? (() => {
                      const mid = segmentMidpoint(seg.points);
                      if (!mid) return null;
                      return (
                        <circle
                          cx={mid[0]}
                          cy={mid[1]}
                          r={4}
                          fill="none"
                          stroke={
                            seg.heatmap?.showMidpointRing
                              ? seg.heatmap.stroke
                              : "rgba(248,113,113,0.9)"
                          }
                          strokeWidth={1.5}
                        />
                      );
                    })()
                  : null}
              </>
            ) : null}
          </g>
        ))}

        {model.nodeMarkers.map((n, i) => (
          <g key={`node-${i}`}>
            <circle cx={n.x} cy={n.y} r={2.5} fill="rgba(250,204,21,0.9)" />
            {n.label ? (
              <text
                x={n.x}
                y={n.y - 5}
                textAnchor="middle"
                fill="rgba(255,255,255,0.7)"
                fontSize={8}
              >
                {n.label}
              </text>
            ) : null}
          </g>
        ))}

        {model.truck ? (
          <g>
            <polygon
              points={`${model.truck.x},${model.truck.y - 7} ${model.truck.x - 5},${model.truck.y + 5} ${model.truck.x + 5},${model.truck.y + 5}`}
              fill="rgba(255,255,255,0.95)"
              stroke="rgba(0,0,0,0.6)"
              strokeWidth={1}
            />
            <text
              x={model.truck.x}
              y={model.truck.y + 16}
              textAnchor="middle"
              fill="rgba(255,255,255,0.8)"
              fontSize={8}
            >
              truck
            </text>
          </g>
        ) : null}
      </svg>
    </div>
  );
}

export { BOX_W as INTERNAL_VIZ_BOX_W, BOX_H as INTERNAL_VIZ_BOX_H };

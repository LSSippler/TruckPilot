import { Panel, Row } from "./Panel";
import { plannedPathFromSnapshot, plannedPathViewStats } from "./planned-path";
import type { OverlaySnapshot } from "./overlay-snapshot";

/// Read-only planned-path debug panel (text only — no world-to-screen lines in v1).
export function PlannedPathDebugPanel({
  snapshot,
}: {
  snapshot: OverlaySnapshot | null;
}) {
  const data = plannedPathFromSnapshot(snapshot);
  const stats = plannedPathViewStats(data);

  if (!stats) {
    return null;
  }

  return (
    <Panel
      title="Planned path v1"
      className="min-w-[13rem] ring-violet-400/25 pointer-events-auto"
    >
      <div className="mb-1 rounded bg-violet-500/15 px-1.5 py-0.5 text-[10px] font-bold tracking-wider text-violet-200">
        READ-ONLY · MOCK GEOMETRY
      </div>
      <Row label="Valid" value={stats.valid ? "yes" : "no"} />
      <Row label="Source" value={stats.source} />
      <Row label="Items" value={String(stats.itemCount)} />
      <Row label="Current" value={stats.currentItemLabel} />
      <Row label="Crosstrack" value={stats.nearestCrosstrack} />
      <Row label="Curvature" value={stats.curvatureRange} />
      <Row label="Junction/prefab" value={String(stats.junctionPrefabCount)} />
      <Row label="Semaphore hints" value={String(stats.semaphoreHintCount)} />
      <Row
        label="Drive (display)"
        value={stats.driveAllowedDisplay}
        hint="not engage authorization"
      />
      {data?.safety.reasons.length ? (
        <p className="mt-1 text-[10px] leading-snug text-white/55">
          {data.safety.reasons.join(", ")}
        </p>
      ) : null}
    </Panel>
  );
}

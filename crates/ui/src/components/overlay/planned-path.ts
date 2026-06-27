import type { OverlaySnapshot, PlannedPathData } from "./overlay-snapshot";

export type { PlannedPathData } from "./overlay-snapshot";

export interface PlannedPathViewStats {
  valid: boolean;
  source: string;
  itemCount: number;
  currentItemLabel: string;
  nearestCrosstrack: string;
  curvatureRange: string;
  junctionPrefabCount: number;
  semaphoreHintCount: number;
  driveAllowedDisplay: string;
}

function kindLabel(k: string): string {
  return k.replace(/_/g, " ");
}

/** Read-only stats for overlay debug panel — never used for control. */
export function plannedPathViewStats(
  data: PlannedPathData | undefined,
): PlannedPathViewStats | null {
  if (!data) return null;

  const current = data.items[data.current_index];
  const curvatures = data.items
    .map((i) => i.curvature_1pm)
    .filter((c): c is number => typeof c === "number" && Number.isFinite(c));

  const minC = curvatures.length ? Math.min(...curvatures) : null;
  const maxC = curvatures.length ? Math.max(...curvatures) : null;
  const curvatureRange =
    minC != null && maxC != null
      ? `${minC.toExponential(2)} … ${maxC.toExponential(2)} 1/m`
      : "—";

  const junctionPrefabCount = data.items.filter(
    (i) => i.kind === "junction" || i.kind === "prefab_path",
  ).length;
  const semaphoreHintCount = data.items.filter((i) => i.semaphore_hint).length;

  return {
    valid: data.valid,
    source: data.source,
    itemCount: data.items.length,
    currentItemLabel: current
      ? `#${current.id} ${kindLabel(current.kind)}`
      : `#${data.current_index} —`,
    nearestCrosstrack:
      data.nearest != null
        ? `${data.nearest.crosstrack_m.toFixed(2)} m`
        : "—",
    curvatureRange,
    junctionPrefabCount,
    semaphoreHintCount,
    driveAllowedDisplay: data.safety.drive_allowed_display_only
      ? "yes (display)"
      : "no",
  };
}

export function plannedPathFromSnapshot(
  snap: OverlaySnapshot | null,
): PlannedPathData | undefined {
  return snap?.planned_path;
}

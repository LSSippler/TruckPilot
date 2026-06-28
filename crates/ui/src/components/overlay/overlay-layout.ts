/** Persisted overlay panel positions + internal-viz zoom (localStorage). */

export const OVERLAY_LAYOUT_STORAGE_KEY = "truckpilot.overlay_layout_v1";
export const OVERLAY_LAYOUT_EDITOR_KEY = "F8";

export type OverlayPanelId =
  | "notification"
  | "acc"
  | "readiness"
  | "preflight"
  | "snapshot-debug"
  | "planned-path"
  | "state"
  | "nav"
  | "vehicle"
  | "lane-debug"
  | "internal-viz";

export interface OverlayPanelLayout {
  x: number;
  y: number;
  /** Map zoom for internal visualization only (1 = fit bounds). */
  zoom?: number;
}

export interface OverlayLayoutState {
  version: 1;
  panels: Partial<Record<OverlayPanelId, OverlayPanelLayout>>;
}

const PANEL_BOX: Record<OverlayPanelId, { w: number; h: number }> = {
  notification: { w: 240, h: 44 },
  acc: { w: 240, h: 88 },
  readiness: { w: 240, h: 96 },
  preflight: { w: 240, h: 120 },
  "snapshot-debug": { w: 240, h: 140 },
  "planned-path": { w: 240, h: 160 },
  state: { w: 240, h: 88 },
  nav: { w: 240, h: 72 },
  vehicle: { w: 240, h: 72 },
  "lane-debug": { w: 300, h: 200 },
  "internal-viz": { w: 420, h: 380 },
};

/** Left-column stack defaults (matches legacy Overlay layout). */
const LEFT_STACK: OverlayPanelId[] = [
  "notification",
  "acc",
  "readiness",
  "preflight",
  "snapshot-debug",
  "planned-path",
  "state",
  "nav",
  "vehicle",
];

const LEFT_GAP = 8;
const LEFT_X = 12;
const TOP_Y = 12;

export const VIZ_ZOOM_MIN = 0.35;
export const VIZ_ZOOM_MAX = 4;
export const VIZ_ZOOM_STEP = 0.12;

export function clampVizZoom(zoom: number): number {
  if (!Number.isFinite(zoom)) return 1;
  return Math.min(VIZ_ZOOM_MAX, Math.max(VIZ_ZOOM_MIN, zoom));
}

export function nudgeVizZoom(current: number, delta: number): number {
  return clampVizZoom(current + delta);
}

function isRecord(v: unknown): v is Record<string, unknown> {
  return typeof v === "object" && v !== null && !Array.isArray(v);
}

function parsePanelLayout(v: unknown): OverlayPanelLayout | null {
  if (!isRecord(v)) return null;
  const x = typeof v.x === "number" && Number.isFinite(v.x) ? v.x : null;
  const y = typeof v.y === "number" && Number.isFinite(v.y) ? v.y : null;
  if (x == null || y == null) return null;
  const zoom =
    typeof v.zoom === "number" && Number.isFinite(v.zoom)
      ? clampVizZoom(v.zoom)
      : undefined;
  return { x, y, zoom };
}

export function loadOverlayLayout(): OverlayLayoutState {
  try {
    const raw = localStorage.getItem(OVERLAY_LAYOUT_STORAGE_KEY);
    if (!raw) return { version: 1, panels: {} };
    const parsed: unknown = JSON.parse(raw);
    if (!isRecord(parsed) || parsed.version !== 1 || !isRecord(parsed.panels)) {
      return { version: 1, panels: {} };
    }
    const panels: OverlayLayoutState["panels"] = {};
    for (const [key, value] of Object.entries(parsed.panels)) {
      const layout = parsePanelLayout(value);
      if (layout) panels[key as OverlayPanelId] = layout;
    }
    return { version: 1, panels };
  } catch {
    return { version: 1, panels: {} };
  }
}

export function saveOverlayLayout(state: OverlayLayoutState): void {
  try {
    localStorage.setItem(OVERLAY_LAYOUT_STORAGE_KEY, JSON.stringify(state));
  } catch {
    // ignore quota / private mode
  }
}

export function defaultPanelLayout(
  id: OverlayPanelId,
  viewportW: number,
  viewportH: number,
): OverlayPanelLayout {
  if (id === "lane-debug" || id === "internal-viz") {
    const box = PANEL_BOX[id];
    return {
      x: Math.max(12, viewportW - box.w - 12),
      y: Math.max(12, viewportH - box.h - 12),
      zoom: id === "internal-viz" ? 1 : undefined,
    };
  }

  let y = TOP_Y;
  for (const panelId of LEFT_STACK) {
    if (panelId === id) {
      return { x: LEFT_X, y };
    }
    y += PANEL_BOX[panelId].h + LEFT_GAP;
  }

  return { x: LEFT_X, y: TOP_Y };
}

export function resolvePanelLayout(
  id: OverlayPanelId,
  state: OverlayLayoutState,
  viewportW: number,
  viewportH: number,
): OverlayPanelLayout {
  const saved = state.panels[id];
  if (saved) {
    return {
      ...saved,
      zoom: saved.zoom != null ? clampVizZoom(saved.zoom) : saved.zoom,
    };
  }
  return defaultPanelLayout(id, viewportW, viewportH);
}

export function setPanelLayout(
  state: OverlayLayoutState,
  id: OverlayPanelId,
  layout: OverlayPanelLayout,
): OverlayLayoutState {
  return {
    version: 1,
    panels: {
      ...state.panels,
      [id]: {
        x: layout.x,
        y: layout.y,
        zoom: layout.zoom != null ? clampVizZoom(layout.zoom) : undefined,
      },
    },
  };
}

export function resetOverlayLayout(): OverlayLayoutState {
  try {
    localStorage.removeItem(OVERLAY_LAYOUT_STORAGE_KEY);
  } catch {
    // ignore
  }
  return { version: 1, panels: {} };
}

export function isOverlayLayoutEditorForced(search: URLSearchParams): boolean {
  const v = search.get("overlay_editor");
  return v === "1" || v === "true";
}

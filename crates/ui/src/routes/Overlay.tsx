import { useCallback, useEffect, useMemo, useState, type ReactNode } from "react";

import { subscribeBlackboardKeys } from "@/lib/ipc";

import { OVERLAY_BB_KEYS } from "@/components/overlay/overlay-lib";

import { AccPanel } from "@/components/overlay/AccPanel";

import { StatePanel } from "@/components/overlay/StatePanel";

import { NotificationBar } from "@/components/overlay/NotificationBar";

import { NavPanel } from "@/components/overlay/NavPanel";

import { VehiclePanel } from "@/components/overlay/VehiclePanel";

import { ReadinessPanel } from "@/components/overlay/ReadinessPanel";

import { PreflightPanel } from "@/components/overlay/PreflightPanel";

import { InternalPathVisualization } from "@/components/overlay/InternalPathVisualization";

import { isInternalPathVisualizationEnabled } from "@/components/overlay/internal-path-viz";

import { PlannedPathDebugPanel } from "@/components/overlay/PlannedPathDebugPanel";

import { LaneDebugCanvas } from "@/components/overlay/LaneDebugCanvas";

import { SnapshotDebugPanel } from "@/components/overlay/SnapshotDebugPanel";

import { OverlayDraggablePanel } from "@/components/overlay/OverlayDraggablePanel";

import { OverlayLayoutEditorBanner } from "@/components/overlay/OverlayLayoutEditorBanner";

import {

  nudgeVizZoom,

  resolvePanelLayout,

  type OverlayPanelId,

  VIZ_ZOOM_STEP,

} from "@/components/overlay/overlay-layout";

import { useOverlayLayoutEditor } from "@/components/overlay/useOverlayLayoutEditor";

import {

  isOverlaySnapshotDebugMode,

  isOverlaySnapshotStandaloneMode,

  resolveOverlaySnapshot,

  type OverlaySnapshot,

} from "@/components/overlay/overlay-snapshot";



const PANEL_LABELS: Record<OverlayPanelId, string> = {

  notification: "Notifications",

  acc: "ACC",

  readiness: "Readiness",

  preflight: "Preflight",

  "snapshot-debug": "Snapshot",

  "planned-path": "Planned path",

  state: "State",

  nav: "Navigation",

  vehicle: "Vehicle",

  "lane-debug": "Lane debug",

  "internal-viz": "Internal viz",

};



function OverlayPanel({

  id,

  editorMode,

  layoutState,

  viewport,

  persistPanel,

  interactive = false,

  visible = true,

  children,

}: {

  id: OverlayPanelId;

  editorMode: boolean;

  layoutState: ReturnType<typeof useOverlayLayoutEditor>["layoutState"];

  viewport: { w: number; h: number };

  persistPanel: ReturnType<typeof useOverlayLayoutEditor>["persistPanel"];

  interactive?: boolean;

  visible?: boolean;

  children: ReactNode;

}) {

  if (!visible) return null;

  const layout = resolvePanelLayout(id, layoutState, viewport.w, viewport.h);

  return (

    <OverlayDraggablePanel

      id={id}

      label={PANEL_LABELS[id]}

      editorMode={editorMode}

      layout={layout}

      onLayoutChange={(next) => persistPanel(id, next)}

      interactive={interactive}

    >

      {children}

    </OverlayDraggablePanel>

  );

}



/// Transparent, click-through HUD overlay (Phase 6.5a). Rendered in the

/// dedicated `overlay` Tauri window (route `/overlay`). It reuses the existing

/// Rust IpcBridge: telemetry arrives via the global `core-event` broadcast and

/// blackboard keys via this window's own instance of the existing poller — NO

/// second daemon connection (the WebSocket stays single, in Rust).

export function Overlay() {

  const search = useMemo(

    () => new URLSearchParams(window.location.search),

    [],

  );

  const snapshotDebugMode = isOverlaySnapshotDebugMode(search);

  const snapshotStandalone = isOverlaySnapshotStandaloneMode(search);

  const internalVisualization = isInternalPathVisualizationEnabled(search);

  const [snapshot, setSnapshot] = useState<OverlaySnapshot | null>(() =>

    resolveOverlaySnapshot(search),

  );

  const { editorMode, layoutState, viewport, persistPanel, resetLayout } =

    useOverlayLayoutEditor(search);



  const internalVizLayout = useMemo(

    () => resolvePanelLayout("internal-viz", layoutState, viewport.w, viewport.h),

    [layoutState, viewport.h, viewport.w],

  );



  const onInternalVizZoomChange = useCallback(

    (zoom: number) => {

      persistPanel("internal-viz", { ...internalVizLayout, zoom });

    },

    [internalVizLayout, persistPanel],

  );



  useEffect(() => {

    if (!editorMode || !internalVisualization) return;

    const onKey = (e: KeyboardEvent) => {

      if (e.key !== "+" && e.key !== "=" && e.key !== "-" && e.key !== "_") return;

      if (e.ctrlKey || e.altKey || e.metaKey) return;

      const tag = (e.target as HTMLElement | null)?.tagName;

      if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT") return;

      e.preventDefault();

      const delta =

        e.key === "-" || e.key === "_"

          ? -VIZ_ZOOM_STEP

          : VIZ_ZOOM_STEP;

      onInternalVizZoomChange(

        nudgeVizZoom(internalVizLayout.zoom ?? 1, delta),

      );

    };

    window.addEventListener("keydown", onKey);

    return () => window.removeEventListener("keydown", onKey);

  }, [editorMode, internalVisualization, internalVizLayout.zoom, onInternalVizZoomChange]);



  useEffect(() => {

    document.documentElement.classList.add("overlay-active");

    const unsubscribe = snapshotStandalone

      ? () => {}

      : subscribeBlackboardKeys(OVERLAY_BB_KEYS);

    return () => {

      document.documentElement.classList.remove("overlay-active");

      unsubscribe();

    };

  }, [snapshotStandalone]);



  const panelProps = {

    editorMode,

    layoutState,

    viewport,

    persistPanel,

  };



  return (

    <div

      className={

        editorMode

          ? "fixed inset-0 select-none text-xs"

          : "pointer-events-none fixed inset-0 select-none text-xs"

      }

    >

      {snapshot && !internalVisualization ? (

        <OverlayPanel id="lane-debug" visible {...panelProps}>

          <LaneDebugCanvas snapshot={snapshot} />

        </OverlayPanel>

      ) : null}

      {snapshot && internalVisualization ? (

        <OverlayPanel id="internal-viz" visible {...panelProps}>

          <InternalPathVisualization

            snapshot={snapshot}

            editorMode={editorMode}

            zoom={internalVizLayout.zoom ?? 1}

            onZoomChange={onInternalVizZoomChange}

          />

        </OverlayPanel>

      ) : null}



      <OverlayPanel id="notification" visible {...panelProps}>

        <NotificationBar />

      </OverlayPanel>

      <OverlayPanel id="acc" visible {...panelProps}>

        <AccPanel />

      </OverlayPanel>

      {!snapshotStandalone ? (

        <OverlayPanel id="readiness" visible {...panelProps}>

          <ReadinessPanel />

        </OverlayPanel>

      ) : null}

      {!snapshotStandalone ? (

        <OverlayPanel id="preflight" visible {...panelProps}>

          <PreflightPanel />

        </OverlayPanel>

      ) : null}

      {snapshotDebugMode ? (

        <OverlayPanel id="snapshot-debug" visible interactive {...panelProps}>

          <SnapshotDebugPanel

            snapshot={snapshot}

            onSnapshotImported={setSnapshot}

          />

        </OverlayPanel>

      ) : null}

      {snapshot?.planned_path ? (

        <OverlayPanel id="planned-path" visible interactive {...panelProps}>

          <PlannedPathDebugPanel snapshot={snapshot} />

        </OverlayPanel>

      ) : null}

      <OverlayPanel id="state" visible {...panelProps}>

        <StatePanel />

      </OverlayPanel>

      <OverlayPanel id="nav" visible {...panelProps}>

        <NavPanel />

      </OverlayPanel>

      <OverlayPanel id="vehicle" visible {...panelProps}>

        <VehiclePanel />

      </OverlayPanel>



      <OverlayLayoutEditorBanner editorMode={editorMode} onReset={resetLayout} />
    </div>
  );
}
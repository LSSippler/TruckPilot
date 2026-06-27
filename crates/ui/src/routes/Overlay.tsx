import { useEffect, useMemo, useState } from "react";
import { subscribeBlackboardKeys } from "@/lib/ipc";
import { OVERLAY_BB_KEYS } from "@/components/overlay/overlay-lib";
import { AccPanel } from "@/components/overlay/AccPanel";
import { StatePanel } from "@/components/overlay/StatePanel";
import { NotificationBar } from "@/components/overlay/NotificationBar";
import { NavPanel } from "@/components/overlay/NavPanel";
import { VehiclePanel } from "@/components/overlay/VehiclePanel";
import { ReadinessPanel } from "@/components/overlay/ReadinessPanel";
import { LaneDebugCanvas } from "@/components/overlay/LaneDebugCanvas";
import { SnapshotDebugPanel } from "@/components/overlay/SnapshotDebugPanel";
import {
  isOverlaySnapshotDebugMode,
  isOverlaySnapshotStandaloneMode,
  resolveOverlaySnapshot,
  type OverlaySnapshot,
} from "@/components/overlay/overlay-snapshot";

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
  const [snapshot, setSnapshot] = useState<OverlaySnapshot | null>(() =>
    resolveOverlaySnapshot(search),
  );

  useEffect(() => {
    // Make THIS window's document see-through (only affects the overlay webview;
    // the window itself is `transparent: true` on the Rust side).
    document.documentElement.classList.add("overlay-active");
    // Snapshot fixture/storage/mock: read-only local data — no daemon blackboard poll.
    const unsubscribe = snapshotStandalone
      ? () => {}
      : subscribeBlackboardKeys(OVERLAY_BB_KEYS);
    return () => {
      document.documentElement.classList.remove("overlay-active");
      unsubscribe();
    };
  }, [snapshotStandalone]);

  // Layout (Phase 6.5b): EINE Safe-Zone-Spalte am linken Rand, oben verankert und
  // gestapelt. ETS2 besitzt sein eigenes HUD — Karte/Minimap unten-rechts,
  // Tacho/Dashboard unten-mitte, Toll/Job-Notifications oben-mitte und die
  // Spiegel-Cams oben/seitlich-oben. Alle bleiben frei. `max-h` + `overflow`
  // verhindern, dass die Spalte je aus dem Viewport läuft (das in 6.5a unten
  // abgeschnittene State-Panel). Reihenfolge: Speedlimit-Warnung zuerst (gut
  // sichtbar), dann ACC, State, Nav, Vehicle.
  return (
    <div className="pointer-events-none fixed inset-0 select-none text-xs">
      {snapshot ? <LaneDebugCanvas snapshot={snapshot} /> : null}
      <div className="absolute left-3 top-3 flex max-h-[calc(100vh-1.5rem)] w-60 flex-col gap-2 overflow-hidden">
        <NotificationBar />
        <AccPanel />
        {!snapshotStandalone ? <ReadinessPanel /> : null}
        {snapshotDebugMode ? (
          <SnapshotDebugPanel
            snapshot={snapshot}
            onSnapshotImported={setSnapshot}
          />
        ) : null}
        <StatePanel />
        <NavPanel />
        <VehiclePanel />
      </div>
    </div>
  );
}

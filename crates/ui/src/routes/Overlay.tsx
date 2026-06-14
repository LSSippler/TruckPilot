import { useEffect } from "react";
import { subscribeBlackboardKeys } from "@/lib/ipc";
import { OVERLAY_BB_KEYS } from "@/components/overlay/overlay-lib";
import { AccPanel } from "@/components/overlay/AccPanel";
import { StatePanel } from "@/components/overlay/StatePanel";
import { NotificationBar } from "@/components/overlay/NotificationBar";
import { NavPanel } from "@/components/overlay/NavPanel";
import { VehiclePanel } from "@/components/overlay/VehiclePanel";

/// Transparent, click-through HUD overlay (Phase 6.5a). Rendered in the
/// dedicated `overlay` Tauri window (route `/overlay`). It reuses the existing
/// Rust IpcBridge: telemetry arrives via the global `core-event` broadcast and
/// blackboard keys via this window's own instance of the existing poller — NO
/// second daemon connection (the WebSocket stays single, in Rust).
export function Overlay() {
  useEffect(() => {
    // Make THIS window's document see-through (only affects the overlay webview;
    // the window itself is `transparent: true` on the Rust side).
    document.documentElement.classList.add("overlay-active");
    // Register the keys our panels render so the shared 500 ms poller fetches
    // exactly these (and no more).
    const unsubscribe = subscribeBlackboardKeys(OVERLAY_BB_KEYS);
    return () => {
      document.documentElement.classList.remove("overlay-active");
      unsubscribe();
    };
  }, []);

  // Layout (Phase 6.5b): EINE Safe-Zone-Spalte am linken Rand, oben verankert und
  // gestapelt. ETS2 besitzt sein eigenes HUD — Karte/Minimap unten-rechts,
  // Tacho/Dashboard unten-mitte, Toll/Job-Notifications oben-mitte und die
  // Spiegel-Cams oben/seitlich-oben. Alle bleiben frei. `max-h` + `overflow`
  // verhindern, dass die Spalte je aus dem Viewport läuft (das in 6.5a unten
  // abgeschnittene State-Panel). Reihenfolge: Speedlimit-Warnung zuerst (gut
  // sichtbar), dann ACC, State, Nav, Vehicle.
  return (
    <div className="pointer-events-none fixed inset-0 select-none text-xs">
      <div className="absolute left-3 top-3 flex max-h-[calc(100vh-1.5rem)] w-60 flex-col gap-2 overflow-hidden">
        <NotificationBar />
        <AccPanel />
        <StatePanel />
        <NavPanel />
        <VehiclePanel />
      </div>
    </div>
  );
}

import { useEffect, useMemo } from "react";
import { Navigate, Route, Routes, useLocation } from "react-router-dom";
import { Toaster } from "sonner";
import { RouteShell } from "@/components/layout/RouteShell";
import { Dashboard } from "@/routes/Dashboard";
import { Plugins } from "@/routes/Plugins";
import { ModManager } from "@/routes/ModManager";
import { PidTuning } from "@/routes/PidTuning";
import { Settings } from "@/routes/Settings";
import { Logs } from "@/routes/Logs";
import { Blackboard } from "@/routes/Blackboard";
import { ExternalDashboard } from "@/routes/ExternalDashboard";
import { Overlay } from "@/routes/Overlay";
import { isOverlaySnapshotStandaloneMode } from "@/components/overlay/overlay-snapshot";
import { initIpcSubscriptions } from "@/lib/ipc";
import { daemonGetAutoStart, daemonStart } from "@/lib/tauri-bridge";
import { HotkeyHandler } from "@/components/HotkeyHandler";
import { AutopilotToastWatcher } from "@/components/AutopilotToastWatcher";

/** Defer daemon spawn so UI paints first; graph.json load is CPU-heavy. */
const DAEMON_AUTOSTART_DELAY_MS = 3_000;

export function App() {
  const location = useLocation();
  const isOverlayRoute = location.pathname.startsWith("/overlay");
  const skipIpc = useMemo(() => {
    if (!isOverlayRoute) return false;
    return isOverlaySnapshotStandaloneMode(new URLSearchParams(location.search));
  }, [isOverlayRoute, location.search]);

  useEffect(() => {
    if (skipIpc) return;
    const teardown = initIpcSubscriptions();
    return () => {
      void teardown.then((fn) => fn());
    };
  }, [skipIpc]);

  // Only the main window may auto-start the daemon. Overlay webviews (fixture or
  // live) must never spawn truckpilot-core — start manually or from Settings.
  useEffect(() => {
    if (isOverlayRoute || skipIpc) return;

    let cancelled = false;
    let timeoutId: number | undefined;

    void (async () => {
      const enabled = await daemonGetAutoStart();
      if (!enabled || cancelled) return;
      timeoutId = window.setTimeout(() => {
        if (!cancelled) void daemonStart();
      }, DAEMON_AUTOSTART_DELAY_MS);
    })();

    return () => {
      cancelled = true;
      if (timeoutId !== undefined) window.clearTimeout(timeoutId);
    };
  }, [isOverlayRoute, skipIpc]);

  // The overlay window renders chrome-less and must NOT mount global chrome
  // (toasts/hotkeys) — it shares this App but lives in its own webview.
  const isOverlay = isOverlayRoute;

  return (
    <>
      {!isOverlay && <HotkeyHandler />}
      {!isOverlay && <AutopilotToastWatcher />}
      <Routes>
        <Route element={<RouteShell />}>
          <Route index element={<Dashboard />} />
          <Route path="plugins" element={<Plugins />} />
          <Route path="mods" element={<ModManager />} />
          <Route path="pid" element={<PidTuning />} />
          <Route path="settings" element={<Settings />} />
          <Route path="logs" element={<Logs />} />
          <Route path="blackboard" element={<Blackboard />} />
        </Route>
        <Route path="external-dashboard" element={<ExternalDashboard />} />
        <Route path="overlay" element={<Overlay />} />
        <Route path="*" element={<Navigate to="/" replace />} />
      </Routes>
      {!isOverlay && <Toaster richColors position="bottom-right" closeButton />}
    </>
  );
}

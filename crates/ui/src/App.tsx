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
import { HotkeyHandler } from "@/components/HotkeyHandler";
import { AutopilotToastWatcher } from "@/components/AutopilotToastWatcher";

export function App() {
  const location = useLocation();
  const skipIpc = useMemo(() => {
    if (!location.pathname.startsWith("/overlay")) return false;
    return isOverlaySnapshotStandaloneMode(new URLSearchParams(location.search));
  }, [location.pathname, location.search]);

  useEffect(() => {
    if (skipIpc) return;
    const teardown = initIpcSubscriptions();
    return () => {
      void teardown.then((fn) => fn());
    };
  }, [skipIpc]);

  // The overlay window renders chrome-less and must NOT mount global chrome
  // (toasts/hotkeys) — it shares this App but lives in its own webview.
  const isOverlay = location.pathname.startsWith("/overlay");

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

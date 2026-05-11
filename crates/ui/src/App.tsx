import { useEffect } from "react";
import { Navigate, Route, Routes } from "react-router-dom";
import { Toaster } from "sonner";
import { RouteShell } from "@/components/layout/RouteShell";
import { Dashboard } from "@/routes/Dashboard";
import { Plugins } from "@/routes/Plugins";
import { ModManager } from "@/routes/ModManager";
import { PidTuning } from "@/routes/PidTuning";
import { Settings } from "@/routes/Settings";
import { Logs } from "@/routes/Logs";
import { ExternalDashboard } from "@/routes/ExternalDashboard";
import { initIpcSubscriptions } from "@/lib/ipc";

export function App() {
  useEffect(() => {
    const teardown = initIpcSubscriptions();
    return () => {
      void teardown.then((fn) => fn());
    };
  }, []);

  return (
    <>
      <Routes>
        <Route element={<RouteShell />}>
          <Route index element={<Dashboard />} />
          <Route path="plugins" element={<Plugins />} />
          <Route path="mods" element={<ModManager />} />
          <Route path="pid" element={<PidTuning />} />
          <Route path="settings" element={<Settings />} />
          <Route path="logs" element={<Logs />} />
        </Route>
        <Route path="external-dashboard" element={<ExternalDashboard />} />
        <Route path="*" element={<Navigate to="/" replace />} />
      </Routes>
      <Toaster richColors position="bottom-right" closeButton />
    </>
  );
}

import { Outlet } from "react-router-dom";
import { useHotkeys } from "react-hotkeys-hook";
import { Sidebar } from "@/components/layout/Sidebar";
import { TopBar } from "@/components/layout/TopBar";
import { StatusBar } from "@/components/layout/StatusBar";
import { openExternalDashboard } from "@/lib/tauri-bridge";

export function RouteShell() {
  useHotkeys("f2", () => void openExternalDashboard(), { preventDefault: true });

  return (
    <div className="flex h-full">
      <Sidebar />
      <div className="flex h-full flex-1 flex-col">
        <TopBar />
        <main className="flex-1 overflow-y-auto p-6">
          <Outlet />
        </main>
        <StatusBar />
      </div>
    </div>
  );
}

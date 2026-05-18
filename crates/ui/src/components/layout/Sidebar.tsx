import { useEffect } from "react";
import {
  Activity,
  Boxes,
  Database,
  Map,
  ScrollText,
  Settings as SettingsIcon,
  Truck,
  SlidersHorizontal,
} from "lucide-react";
import { NavGroup } from "@/components/sidebar/NavGroup";
import { NavItem } from "@/components/sidebar/NavItem";
import { ETS2StatusIndicator, type ETS2State } from "@/components/ETS2StatusIndicator";
import { useConnectionStore } from "@/stores/connection";
import { useBlackboardStore } from "@/stores/blackboard";
import { subscribeBlackboardKeys } from "@/lib/ipc";

const TELE_KEY = ["telemetry.available"] as const;

function useEts2State(): ETS2State {
  const status = useConnectionStore((s) => s.status);
  const teleAvailable = useBlackboardStore((s) => s.values["telemetry.available"]);
  if (status !== "connected") return "disconnected";
  if (teleAvailable === "true") return "connected";
  return "waiting";
}

export function Sidebar() {
  useEffect(() => subscribeBlackboardKeys(TELE_KEY), []);
  const ets2State = useEts2State();

  return (
    <aside
      className="flex h-full flex-col border-r border-subtle"
      style={{ width: "var(--sidebar-w)", background: "var(--surface-base)" }}
    >
      {/* Logo strip */}
      <div
        className="flex items-center gap-2 px-3 shrink-0 border-b border-subtle"
        style={{ height: "var(--header-h)" }}
      >
        <Truck className="size-4 text-brand shrink-0" />
        <span className="text-sm font-sans font-medium text-fg tracking-tight">TruckPilot</span>
        <span className="ml-auto font-mono text-[10px] text-fg-muted">v0.2.0</span>
      </div>

      {/* Navigation */}
      <nav className="flex-1 overflow-y-auto py-3 px-2 flex flex-col gap-4">
        <NavGroup label="Operation">
          <NavItem to="/" end icon={<Activity size={14} />}>Dashboard</NavItem>
          <NavItem to="/blackboard" icon={<Database size={14} />}>Blackboard</NavItem>
          <NavItem to="/logs" icon={<ScrollText size={14} />}>Logs</NavItem>
        </NavGroup>

        <NavGroup label="Configuration">
          <NavItem to="/plugins" icon={<Boxes size={14} />}>Plugins</NavItem>
          <NavItem to="/pid" icon={<SlidersHorizontal size={14} />}>PID Tuning</NavItem>
          <NavItem to="/mods" icon={<Map size={14} />}>Mods</NavItem>
          <NavItem to="/settings" icon={<SettingsIcon size={14} />}>Settings</NavItem>
        </NavGroup>
      </nav>

      {/* ETS2 status foot */}
      <ETS2StatusIndicator state={ets2State} className="border-t border-subtle shrink-0" />
    </aside>
  );
}

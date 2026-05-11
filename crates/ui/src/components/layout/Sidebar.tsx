import { NavLink } from "react-router-dom";
import { Activity, Boxes, Gauge, Map, ScrollText, Settings as SettingsIcon, Truck } from "lucide-react";
import { cn } from "@/lib/utils";

interface NavItem {
  to: string;
  label: string;
  icon: React.ComponentType<{ className?: string }>;
  end?: boolean;
}

const NAV: NavItem[] = [
  { to: "/", label: "Dashboard", icon: Activity, end: true },
  { to: "/plugins", label: "Plugins", icon: Boxes },
  { to: "/mods", label: "Mods", icon: Map },
  { to: "/pid", label: "PID Tuning", icon: Gauge },
  { to: "/settings", label: "Settings", icon: SettingsIcon },
  { to: "/logs", label: "Logs", icon: ScrollText },
];

export function Sidebar() {
  return (
    <aside className="flex h-full w-56 flex-col border-r bg-sidebar text-sidebar-foreground">
      <div className="flex h-14 items-center gap-2 border-b px-4">
        <Truck className="size-5 text-sidebar-primary" />
        <span className="text-base font-semibold">TruckPilot</span>
      </div>
      <nav className="flex-1 space-y-1 p-2">
        {NAV.map(({ to, label, icon: Icon, end }) => (
          <NavLink
            key={to}
            to={to}
            end={end}
            className={({ isActive }) =>
              cn(
                "flex items-center gap-3 rounded-md px-3 py-2 text-sm transition-colors",
                "hover:bg-sidebar-accent hover:text-sidebar-accent-foreground",
                isActive && "bg-sidebar-accent text-sidebar-accent-foreground font-medium"
              )
            }
          >
            <Icon className="size-4" />
            <span>{label}</span>
          </NavLink>
        ))}
      </nav>
      <div className="border-t p-3 text-xs text-muted-foreground">v0.2.0</div>
    </aside>
  );
}

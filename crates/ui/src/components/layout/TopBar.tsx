import { useEffect } from "react";
import { Monitor, Moon, Sun } from "lucide-react";
import { Button } from "@/components/ui/button";
import { useSettingsStore, type Theme } from "@/stores/settings";
import { openExternalDashboard } from "@/lib/tauri-bridge";

function applyTheme(theme: Theme) {
  if (typeof document === "undefined") return;
  const root = document.documentElement;
  root.classList.remove("dark", "light");
  if (theme === "dark") root.classList.add("dark");
  else if (theme === "light") root.classList.add("light");
  else {
    const prefersDark = window.matchMedia("(prefers-color-scheme: dark)").matches;
    root.classList.add(prefersDark ? "dark" : "light");
  }
}

export function TopBar() {
  const theme = useSettingsStore((s) => s.theme);
  const setTheme = useSettingsStore((s) => s.setTheme);

  useEffect(() => {
    applyTheme(theme);
    if (theme !== "system") return;
    const mq = window.matchMedia("(prefers-color-scheme: dark)");
    const onChange = () => applyTheme("system");
    mq.addEventListener("change", onChange);
    return () => mq.removeEventListener("change", onChange);
  }, [theme]);

  const ThemeIcon = theme === "dark" ? Moon : theme === "light" ? Sun : Monitor;
  const cycleTheme = () => setTheme(theme === "system" ? "light" : theme === "light" ? "dark" : "system");

  return (
    <header className="flex h-14 items-center justify-between border-b px-4">
      <div className="text-sm text-muted-foreground">Self-Driving for Euro Truck Simulator 2</div>
      <div className="flex items-center gap-2">
        <Button variant="outline" size="sm" onClick={() => void openExternalDashboard()}>
          Pop out dashboard (F2)
        </Button>
        <Button variant="ghost" size="icon" onClick={cycleTheme} aria-label="Toggle theme">
          <ThemeIcon className="size-4" />
        </Button>
      </div>
    </header>
  );
}

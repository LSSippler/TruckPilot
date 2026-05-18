import { useEffect } from "react";
import { toast } from "sonner";
import { useSettingsStore } from "@/stores/settings";
import { useConnectionStore } from "@/stores/connection";
import { sendCommand } from "@/lib/ipc";

/// Global F5/F6 hotkey handler. Renders no DOM — attaches a window-level
/// keydown listener. Active anywhere in the app (Logs/Settings/etc.) as long
/// as the focused element isn't an editable field.
export function HotkeyHandler() {
  const engageKey = useSettingsStore((s) => s.hotkeyEngage);
  const disengageKey = useSettingsStore((s) => s.hotkeyDisengage);
  const status = useConnectionStore((s) => s.status);

  useEffect(() => {
    const isEditable = (el: EventTarget | null): boolean => {
      if (!(el instanceof HTMLElement)) return false;
      if (el.isContentEditable) return true;
      const tag = el.tagName;
      return tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT";
    };

    const onKey = (e: KeyboardEvent) => {
      if (isEditable(e.target)) return;
      // Match by key code (e.key === "F5") — modifier-free.
      if (e.ctrlKey || e.altKey || e.metaKey) return;
      if (status !== "connected") return;
      if (e.key === engageKey) {
        e.preventDefault();
        toast.info(`Engage (${engageKey})`);
        void sendCommand({ type: "autopilot_engage" });
      } else if (e.key === disengageKey) {
        e.preventDefault();
        toast.info(`Disengage (${disengageKey})`);
        void sendCommand({ type: "autopilot_disengage" });
      }
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [engageKey, disengageKey, status]);

  return null;
}

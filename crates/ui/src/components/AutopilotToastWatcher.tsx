import { useEffect, useRef } from "react";
import { toast } from "sonner";
import { useAutopilotStore } from "@/stores/autopilot";
import { useConnectionStore } from "@/stores/connection";
import type { AutopilotState } from "@/lib/types";

export function AutopilotToastWatcher() {
  const apState = useAutopilotStore((s) => s.state);
  const faultReason = useAutopilotStore((s) => s.faultReason);
  const connStatus = useConnectionStore((s) => s.status);

  const prevAp = useRef<AutopilotState | null>(apState);
  const prevConn = useRef(connStatus);
  const initialized = useRef(false);

  useEffect(() => {
    // Skip firing toasts on mount — only react to transitions.
    if (!initialized.current) {
      initialized.current = true;
      prevAp.current = apState;
      prevConn.current = connStatus;
      return;
    }

    // Autopilot state transitions
    if (apState !== prevAp.current) {
      const prev = prevAp.current;
      if (apState === "Active" && prev !== "Active") {
        toast.success("Autopilot engaged");
      } else if (apState === "Off" && (prev === "Active" || prev === "Paused")) {
        toast.info("Autopilot disengaged");
      } else if (apState === "Fault") {
        toast.error("Autopilot fault", {
          description: faultReason ?? "Unknown fault",
          duration: Infinity,
        });
      }
      prevAp.current = apState;
    }
  }, [apState, faultReason]);

  useEffect(() => {
    if (!initialized.current) return;
    const prev = prevConn.current;
    if (connStatus !== prev) {
      if (connStatus === "connected" && prev !== "connected") {
        toast.info("Daemon connected");
      } else if (connStatus === "disconnected" && prev === "connected") {
        toast.warning("Daemon disconnected", { duration: 3000 });
      }
      prevConn.current = connStatus;
    }
  }, [connStatus]);

  return null;
}

import { useEffect, useState } from "react";
import { useAutopilotStore } from "@/stores/autopilot";
import { useConnectionStore } from "@/stores/connection";
import { useBlackboardStore } from "@/stores/blackboard";
import { sendCommand, subscribeBlackboardKeys } from "@/lib/ipc";
import { EngageButton, type EngageState } from "@/components/EngageButton";
import { PreconditionRow, type Precondition } from "@/components/PreconditionPill";
import { EngagementChecklist } from "@/components/EngagementChecklist";
import type { AutopilotState, PreconditionSnapshot } from "@/lib/types";

function toEngageState(
  ap: AutopilotState,
  preconditions: PreconditionSnapshot,
  disconnected: boolean,
  engageReady: boolean | null,
): EngageState {
  if (disconnected) return "off-disabled";
  switch (ap) {
    case "Off": {
      const missing = missingPreconditions(preconditions);
      const blockedByEngage = engageReady === false;
      return (missing.length > 0 || blockedByEngage) ? "off-disabled" : "off-ready";
    }
    case "Engaging":     return "engaging";
    case "Active":       return "engaged";
    case "Paused":       return "engaged";
    case "Fault":        return "fault";
  }
}

function toPreconditions(snap: PreconditionSnapshot): Precondition[] {
  return [
    {
      id: "telemetry",
      label: "Telemetry",
      state: snap.telemetry_ok ? "ok" : "missing",
      hint: snap.telemetry_ok ? undefined : "No telemetry feed from ETS2.",
    },
    {
      id: "engine",
      label: "Engine",
      state: snap.engine_running ? "ok" : "missing",
      hint: snap.engine_running ? undefined : "Start the truck engine.",
    },
    {
      id: "plugins",
      label: "Plugins",
      state: snap.critical_plugins_loaded ? "ok" : "missing",
      hint: snap.critical_plugins_loaded ? undefined : "Critical plugins not loaded.",
    },
    {
      id: "route",
      label: "Route",
      state: snap.router_active ? "ok" : "missing",
      hint: snap.router_active ? undefined : "Set a route goal in the Route card.",
    },
  ];
}

function missingPreconditions(p: PreconditionSnapshot): string[] {
  const missing: string[] = [];
  if (!p.telemetry_ok) missing.push("Telemetry");
  if (!p.engine_running) missing.push("Engine");
  if (!p.critical_plugins_loaded) missing.push("Plugins");
  if (!p.router_active) missing.push("Route");
  return missing;
}

type Command = "engage" | "disengage" | "reset";

const ENGAGE_READY_KEYS = ["state.engage_ready", "state.engage_blocked_by"] as const;

export function AutopilotStatusCard() {
  const connection = useConnectionStore((s) => s.status);
  const state = useAutopilotStore((s) => s.state);
  const faultReason = useAutopilotStore((s) => s.faultReason);
  const preconditions = useAutopilotStore((s) => s.preconditions);
  const tickCount = useAutopilotStore((s) => s.tickCount);
  const lastUpdateMs = useAutopilotStore((s) => s.lastUpdateMs);
  const bbValues = useBlackboardStore((s) => s.values);

  const disconnected = connection !== "connected";
  const [pendingCommand, setPendingCommand] = useState<Command | null>(null);
  const [pendingSince, setPendingSince] = useState(0);
  const [warnNoResponse, setWarnNoResponse] = useState(false);

  useEffect(() => subscribeBlackboardKeys(ENGAGE_READY_KEYS), []);

  useEffect(() => {
    if (!pendingCommand) return;
    const handle = window.setTimeout(() => {
      if (lastUpdateMs <= pendingSince) setWarnNoResponse(true);
      setPendingCommand(null);
    }, 2_000);
    return () => window.clearTimeout(handle);
  }, [pendingCommand, pendingSince, lastUpdateMs]);

  useEffect(() => {
    if (pendingCommand && lastUpdateMs > pendingSince) {
      setPendingCommand(null);
      setWarnNoResponse(false);
    }
  }, [lastUpdateMs, pendingCommand, pendingSince]);

  const dispatch = (kind: Command) => {
    setPendingSince(lastUpdateMs);
    setPendingCommand(kind);
    setWarnNoResponse(false);
    const type =
      kind === "engage" ? "autopilot_engage"
      : kind === "disengage" ? "autopilot_disengage"
      : "autopilot_reset";
    void sendCommand({ type });
  };

  const effectiveState: AutopilotState = state ?? "Off";
  const engageReadyVal = bbValues["state.engage_ready"];
  const engageReady: boolean | null =
    engageReadyVal === "true" ? true : engageReadyVal === "false" ? false : null;
  const engageBlockedBy = bbValues["state.engage_blocked_by"];
  const engageState = toEngageState(effectiveState, preconditions, disconnected || pendingCommand !== null, engageReady);
  const pills = toPreconditions(preconditions);
  const activeSeconds = Math.floor(tickCount / 50);
  const missing = missingPreconditions(preconditions);

  const disabledReason = (() => {
    const reasons: string[] = [];
    if (missing.length > 0) reasons.push(`Missing: ${missing.join(", ")}`);
    if (engageReady === false && engageBlockedBy) reasons.push(`Blocked: ${engageBlockedBy}`);
    return reasons.length > 0 ? reasons.join(" | ") : undefined;
  })();

  const handleEngageClick = () => {
    if (effectiveState === "Off") dispatch("engage");
    else if (effectiveState === "Active" || effectiveState === "Paused") dispatch("disengage");
    else if (effectiveState === "Fault") dispatch("reset");
  };

  return (
    <div className="flex flex-col gap-3 bg-surface-card border border-subtle rounded-md p-4 min-w-[260px]">
      <div className="flex items-center justify-between">
        <span className="text-fg-muted text-xs uppercase tracking-wider font-sans">Autopilot</span>
        {effectiveState === "Active" && (
          <span className="font-mono text-[10px] text-fg-muted">
            {activeSeconds}s active
          </span>
        )}
      </div>

      <div className="flex items-center gap-3 flex-wrap">
        <EngageButton
          state={engageState}
          hotkey={effectiveState === "Off" ? "F5" : "F6"}
          onClick={handleEngageClick}
          disabledReason={disabledReason}
        />
        {effectiveState === "Fault" && faultReason && (
          <p className="text-xs font-mono text-danger">{faultReason}</p>
        )}
      </div>

      <PreconditionRow preconditions={pills} />

      <EngagementChecklist />

      {disconnected && (
        <p className="text-xs text-fg-muted">Daemon offline.</p>
      )}
      {warnNoResponse && (
        <p className="text-xs text-warning">No response from daemon — try again.</p>
      )}
    </div>
  );
}

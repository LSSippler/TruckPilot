import { useEffect, useState } from "react";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { useAutopilotStore } from "@/stores/autopilot";
import { useConnectionStore } from "@/stores/connection";
import { sendCommand } from "@/lib/ipc";
import type { AutopilotState, PreconditionSnapshot } from "@/lib/types";

const PRECONDITION_LABELS: Array<[keyof PreconditionSnapshot, string]> = [
  ["telemetry_ok", "Telemetry feed"],
  ["engine_running", "Engine running"],
  ["cruise_active", "Cruise control"],
  ["critical_plugins_loaded", "Critical plugins"],
  ["router_active", "Route planned"],
];

const STATE_LABELS: Record<AutopilotState, string> = {
  Off: "Offline",
  Engaging: "Engaging…",
  Active: "Active",
  Paused: "Paused",
  Fault: "FAULT",
};

const STATE_CLASSES: Record<AutopilotState, string> = {
  Off: "bg-slate-500 text-slate-50",
  Engaging: "bg-amber-500 text-amber-50 animate-pulse",
  Active: "bg-emerald-500 text-emerald-50",
  Paused: "bg-sky-500 text-sky-50",
  Fault: "bg-red-600 text-red-50",
};

type Command = "engage" | "disengage" | "reset";

export function AutopilotStatusCard() {
  const connection = useConnectionStore((s) => s.status);
  const state = useAutopilotStore((s) => s.state);
  const faultReason = useAutopilotStore((s) => s.faultReason);
  const preconditions = useAutopilotStore((s) => s.preconditions);
  const tickCount = useAutopilotStore((s) => s.tickCount);
  const lastUpdateMs = useAutopilotStore((s) => s.lastUpdateMs);

  const disconnected = connection !== "connected";
  const [pendingCommand, setPendingCommand] = useState<Command | null>(null);
  const [pendingSince, setPendingSince] = useState(0);
  const [warnNoResponse, setWarnNoResponse] = useState(false);

  useEffect(() => {
    if (!pendingCommand) return;
    const handle = window.setTimeout(() => {
      if (lastUpdateMs <= pendingSince) {
        setWarnNoResponse(true);
      }
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
      kind === "engage"
        ? "autopilot_engage"
        : kind === "disengage"
          ? "autopilot_disengage"
          : "autopilot_reset";
    void sendCommand({ type });
  };

  const effectiveState: AutopilotState = state ?? "Off";
  const activeSeconds = Math.floor(tickCount / 50);

  return (
    <Card className="min-w-[300px]">
      <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
        <CardTitle className="text-sm font-medium text-muted-foreground">Autopilot</CardTitle>
        <Badge className={STATE_CLASSES[effectiveState]}>
          {STATE_LABELS[effectiveState]}
        </Badge>
      </CardHeader>
      <CardContent className="space-y-3">
        <SubInfo
          state={effectiveState}
          preconditions={preconditions}
          faultReason={faultReason}
          activeSeconds={activeSeconds}
        />
        <ActionButton
          state={effectiveState}
          disabled={disconnected || pendingCommand !== null}
          onDispatch={dispatch}
        />
        {disconnected ? (
          <p className="text-xs text-muted-foreground">Daemon offline.</p>
        ) : null}
        {warnNoResponse ? (
          <p className="text-xs text-amber-500">No response from daemon — try again.</p>
        ) : null}
      </CardContent>
    </Card>
  );
}

function SubInfo({
  state,
  preconditions,
  faultReason,
  activeSeconds,
}: {
  state: AutopilotState;
  preconditions: PreconditionSnapshot;
  faultReason: string | null;
  activeSeconds: number;
}) {
  if (state === "Engaging") {
    return (
      <ul className="space-y-1 text-xs">
        {PRECONDITION_LABELS.map(([key, label]) => {
          const ok = preconditions[key];
          return (
            <li key={key} className="flex items-center gap-2">
              <span className={ok ? "text-emerald-500" : "text-muted-foreground"}>
                {ok ? "✓" : "✗"}
              </span>
              <span className={ok ? "" : "text-muted-foreground"}>{label}</span>
            </li>
          );
        })}
      </ul>
    );
  }
  if (state === "Active") {
    return (
      <p className="text-xs text-muted-foreground">
        Active for <span className="font-mono text-foreground">{activeSeconds}s</span>
      </p>
    );
  }
  if (state === "Paused") {
    return (
      <p className="text-xs text-muted-foreground">
        Paused for <span className="font-mono text-foreground">{activeSeconds}s</span>
      </p>
    );
  }
  if (state === "Fault") {
    return (
      <p className="text-xs font-mono text-red-500">
        {faultReason ?? "unknown fault"}
      </p>
    );
  }
  return (
    <p className="text-xs text-muted-foreground">Idle. Press enable when ready.</p>
  );
}

function ActionButton({
  state,
  disabled,
  onDispatch,
}: {
  state: AutopilotState;
  disabled: boolean;
  onDispatch: (kind: Command) => void;
}) {
  switch (state) {
    case "Off":
      return (
        <Button
          className="w-full"
          disabled={disabled}
          onClick={() => onDispatch("engage")}
        >
          Enable Autopilot
        </Button>
      );
    case "Engaging":
      return (
        <Button className="w-full" disabled>
          Engaging…
        </Button>
      );
    case "Active":
    case "Paused":
      return (
        <Button
          className="w-full"
          variant="secondary"
          disabled={disabled}
          onClick={() => onDispatch("disengage")}
        >
          Disable Autopilot
        </Button>
      );
    case "Fault":
      return (
        <Button
          className="w-full"
          variant="destructive"
          disabled={disabled}
          onClick={() => onDispatch("reset")}
        >
          Reset
        </Button>
      );
  }
}

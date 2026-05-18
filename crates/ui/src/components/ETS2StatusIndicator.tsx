import { cn } from "@/lib/utils";

export type ETS2State = "connected" | "waiting" | "disconnected";

export interface ETS2StatusIndicatorProps {
  state: ETS2State;
  detail?: string;
  className?: string;
}

const LABEL: Record<ETS2State, string> = {
  connected:    "ETS2 connected",
  waiting:      "Waiting for ETS2",
  disconnected: "ETS2 not detected",
};

const DOT: Record<ETS2State, string> = {
  connected:    "bg-success",
  waiting:      "bg-warning animate-tp-pulse-soft",
  disconnected: "bg-fg-muted",
};

export function ETS2StatusIndicator({ state, detail, className }: ETS2StatusIndicatorProps) {
  return (
    <div className={cn("flex items-center gap-2 px-3 h-[var(--footer-h)]", className)}>
      <span className={cn("w-1.5 h-1.5 rounded-full shrink-0", DOT[state])} aria-hidden />
      <span className="text-fg-secondary text-xs font-sans">{LABEL[state]}</span>
      {detail && (
        <span className="ml-auto font-mono text-[10px] text-fg-muted tabular-nums truncate">
          {detail}
        </span>
      )}
    </div>
  );
}

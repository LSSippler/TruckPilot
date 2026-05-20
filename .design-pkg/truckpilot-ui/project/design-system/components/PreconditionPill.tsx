// crates/ui/src/components/PreconditionPill.tsx
//
// Example:
//   <PreconditionRow
//     preconditions={[
//       { id: "engine",    label: "Engine",     state: "ok" },
//       { id: "cruise",    label: "Cruise",     state: "ok" },
//       { id: "navi",      label: "ETS2 Navi",  state: "missing", hint: "Set a destination in ETS2" },
//       { id: "telemetry", label: "Telemetry",  state: "ok" },
//       { id: "vjoy",      label: "vJoy",       state: "ok" },
//     ]}
//   />

import * as React from "react";
import {
  Tooltip,
  TooltipContent,
  TooltipProvider,
  TooltipTrigger,
} from "@/components/ui/tooltip";
import { cn } from "@/lib/utils";

export type PreconditionState = "ok" | "missing" | "error" | "pending";

export interface Precondition {
  id: string;
  label: string;
  state: PreconditionState;
  hint?: string;
}

const DOT: Record<PreconditionState, string> = {
  ok:      "bg-success",
  missing: "bg-fg-muted",
  error:   "bg-danger",
  pending: "bg-warning animate-tp-pulse-soft",
};

export interface PreconditionPillProps {
  precondition: Precondition;
  className?: string;
}

export function PreconditionPill({ precondition: p, className }: PreconditionPillProps) {
  const pill = (
    <span
      className={cn(
        "inline-flex items-center gap-1.5 h-6 px-2 rounded-sm",
        "border border-subtle bg-surface-card",
        "text-xs font-sans",
        p.state === "ok" ? "text-fg-secondary" : "text-fg-muted",
        className,
      )}
    >
      <span className={cn("w-1.5 h-1.5 rounded-full", DOT[p.state])} aria-hidden />
      {p.label}
    </span>
  );
  if (!p.hint) return pill;
  return (
    <TooltipProvider delayDuration={200}>
      <Tooltip>
        <TooltipTrigger asChild>{pill}</TooltipTrigger>
        <TooltipContent side="bottom" className="font-sans text-xs">
          {p.hint}
        </TooltipContent>
      </Tooltip>
    </TooltipProvider>
  );
}

export interface PreconditionRowProps {
  preconditions: Precondition[];
  className?: string;
}

export function PreconditionRow({ preconditions, className }: PreconditionRowProps) {
  return (
    <div className={cn("flex flex-wrap items-center gap-1.5", className)}>
      {preconditions.map((p) => (
        <PreconditionPill key={p.id} precondition={p} />
      ))}
    </div>
  );
}

export default PreconditionPill;

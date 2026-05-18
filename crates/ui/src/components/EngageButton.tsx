import { Power, Loader2, AlertTriangle } from "lucide-react";
import { cn } from "@/lib/utils";

export type EngageState =
  | "off-ready"
  | "off-disabled"
  | "engaging"
  | "engaged"
  | "disengaging"
  | "fault";

export interface EngageButtonProps {
  state: EngageState;
  hotkey?: string;
  onClick?: () => void;
  disabledReason?: string;
  className?: string;
}

const LABEL: Record<EngageState, string> = {
  "off-ready":    "Engage",
  "off-disabled": "Engage",
  "engaging":     "Engaging…",
  "engaged":      "Engaged",
  "disengaging":  "Disengaging…",
  "fault":        "Fault — tap to clear",
};

export function EngageButton({
  state,
  hotkey = "F5",
  onClick,
  disabledReason,
  className,
}: EngageButtonProps) {
  const isDisabled  = state === "off-disabled";
  const isTransient = state === "engaging" || state === "disengaging";
  const isEngaged   = state === "engaged";
  const isFault     = state === "fault";

  const Icon = isTransient ? Loader2 : isFault ? AlertTriangle : Power;

  return (
    <div className={cn("relative inline-flex", className)}>
      {isEngaged && (
        <span
          aria-hidden
          className="pointer-events-none absolute -inset-px rounded-md animate-tp-pulse-soft"
          style={{
            padding: "1px",
            background: "var(--brand-gradient)",
            WebkitMask:
              "linear-gradient(#000 0 0) content-box, linear-gradient(#000 0 0)",
            WebkitMaskComposite: "xor",
            maskComposite: "exclude",
          }}
        />
      )}

      <button
        type="button"
        disabled={isDisabled || isTransient}
        title={isDisabled ? disabledReason : undefined}
        onClick={onClick}
        className={cn(
          "relative inline-flex items-center gap-2 h-10 px-4 rounded-md",
          "font-sans text-sm font-medium",
          "border transition-colors",
          "outline-none focus-visible:ring-2 focus-visible:ring-[var(--ring)] focus-visible:ring-offset-2 focus-visible:ring-offset-[var(--ring-offset)]",
          !isEngaged && !isFault && [
            "bg-surface-card border-strong text-fg",
            "hover:bg-surface-elevated hover:border-brand-soft",
          ],
          isDisabled && "opacity-50 cursor-not-allowed hover:bg-surface-card hover:border-strong",
          isEngaged && "bg-surface-card border-transparent text-brand",
          isFault && "bg-danger-soft border-danger text-danger hover:bg-danger-soft",
        )}
      >
        <Icon
          size={16}
          className={cn(
            isTransient && "animate-spin",
            state === "disengaging" && "[animation-direction:reverse]",
          )}
          aria-hidden
        />
        <span>{LABEL[state]}</span>

        {hotkey && (
          <kbd
            className={cn(
              "ml-1 inline-flex items-center justify-center min-w-[22px] h-[18px] px-1",
              "font-mono text-[10px] rounded-sm",
              "bg-surface-elevated border border-subtle text-fg-muted",
              isFault && "bg-transparent border-danger/40 text-danger",
            )}
          >
            {hotkey}
          </kbd>
        )}
      </button>
    </div>
  );
}

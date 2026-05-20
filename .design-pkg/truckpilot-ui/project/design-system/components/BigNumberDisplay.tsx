// crates/ui/src/components/BigNumberDisplay.tsx
//
// Example:
//   <BigNumberDisplay label="Speed" unit="km/h" value={speed} target={cruiseTarget} />
//
// Mono numerics, large but not huge. A short color flash fires when the
// rounded integer changes — debounced so it doesn't strobe at high refresh.

import * as React from "react";
import { cn } from "@/lib/utils";

export interface BigNumberDisplayProps {
  label?: string;
  value: number | null | undefined;
  unit?: string;
  target?: number | null;       // dezent darunter angezeigt
  decimals?: number;            // default 0
  flashOnChange?: boolean;      // default true
  className?: string;
}

export function BigNumberDisplay({
  label,
  value,
  unit,
  target,
  decimals = 0,
  flashOnChange = true,
  className,
}: BigNumberDisplayProps) {
  const display = value == null || !Number.isFinite(value)
    ? "—"
    : value.toFixed(decimals);

  // Flash key: changes only when the rounded display changes.
  const flashKey = React.useMemo(() => display, [display]);
  const [k, setK] = React.useState(0);
  React.useEffect(() => {
    if (!flashOnChange) return;
    setK((x) => x + 1);
  }, [flashKey, flashOnChange]);

  return (
    <div className={cn("flex flex-col gap-0.5", className)}>
      {label && (
        <span className="text-fg-muted text-xs uppercase tracking-wider font-sans">
          {label}
        </span>
      )}
      <div className="flex items-baseline gap-1.5">
        <span
          key={flashOnChange ? k : undefined}
          className={cn(
            "font-mono text-3xl leading-tight tabular-nums text-fg",
            flashOnChange && "animate-tp-flash",
          )}
        >
          {display}
        </span>
        {unit && (
          <span className="font-sans text-sm text-fg-muted">{unit}</span>
        )}
      </div>
      {target != null && Number.isFinite(target) && (
        <span className="font-mono text-xs text-fg-muted">
          target {target.toFixed(decimals)}{unit ? ` ${unit}` : ""}
        </span>
      )}
    </div>
  );
}

export default BigNumberDisplay;

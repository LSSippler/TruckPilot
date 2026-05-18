import { cn } from "@/lib/utils";

export interface VJoyBarProps {
  variant: "unipolar" | "bipolar";
  label: string;
  /** unipolar: 0..1   |   bipolar: -1..1 */
  value: number;
  showValue?: boolean;
  accent?: "brand" | "warning" | "danger";
  className?: string;
}

const ACCENT_VAR: Record<NonNullable<VJoyBarProps["accent"]>, string> = {
  brand:   "var(--brand)",
  warning: "var(--warning)",
  danger:  "var(--danger)",
};

export function VJoyBar({
  variant,
  label,
  value,
  showValue = true,
  accent = "brand",
  className,
}: VJoyBarProps) {
  const clamped =
    variant === "unipolar"
      ? Math.max(0, Math.min(1, value))
      : Math.max(-1, Math.min(1, value));

  const fillPct = variant === "unipolar"
    ? clamped * 100
    : Math.abs(clamped) * 50;

  const offsetLeft = variant === "unipolar"
    ? "0%"
    : clamped >= 0 ? "50%" : `${50 - fillPct}%`;

  const color = ACCENT_VAR[accent];
  const valueText = `${(clamped * 100).toFixed(0)}%`;

  return (
    <div className={cn("flex flex-col gap-1.5", className)}>
      <div className="flex items-center justify-between">
        <span className="text-fg-secondary text-xs font-sans">{label}</span>
        {showValue && (
          <span className="font-mono text-xs text-fg-muted tabular-nums">
            {valueText}
          </span>
        )}
      </div>
      <div
        className="relative h-1.5 rounded-sm bg-surface-elevated border border-subtle overflow-hidden"
        role="meter"
        aria-label={label}
        aria-valuenow={clamped}
        aria-valuemin={variant === "unipolar" ? 0 : -1}
        aria-valuemax={1}
      >
        {variant === "bipolar" && (
          <span
            aria-hidden
            className="absolute top-0 bottom-0 left-1/2 w-px bg-border-strong"
          />
        )}
        <span
          aria-hidden
          className="absolute top-0 bottom-0 transition-[left,width] duration-[var(--dur-1)] ease-out"
          style={{
            left: offsetLeft,
            width: `${fillPct}%`,
            background: color,
          }}
        />
      </div>
    </div>
  );
}

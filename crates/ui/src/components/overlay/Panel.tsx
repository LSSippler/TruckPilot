import type { ReactNode } from "react";
import { cn } from "@/lib/utils";

/// HUD panel with a semi-transparent dark backdrop. Positioning is owned by the
/// Overlay container (the panels are placed into corner clusters), so the panel
/// itself is a plain box — pass only sizing via `className`.
export function Panel({
  title,
  className,
  children,
}: {
  title: string;
  className?: string;
  children: ReactNode;
}) {
  return (
    <section
      className={cn(
        "rounded-lg bg-black/55 px-3 py-2 text-white shadow-lg ring-1 ring-white/10 backdrop-blur-sm",
        className
      )}
    >
      <h2 className="mb-1 text-[10px] font-semibold uppercase tracking-widest text-white/45">
        {title}
      </h2>
      <dl className="space-y-0.5">{children}</dl>
    </section>
  );
}

export function Row({
  label,
  value,
  hint,
  warn,
}: {
  label: string;
  value: ReactNode;
  hint?: string;
  warn?: boolean;
}) {
  return (
    <div className="flex items-baseline justify-between gap-6" title={hint}>
      <dt className="text-white/55">{label}</dt>
      <dd className={cn("font-mono tabular-nums", warn ? "text-red-400" : "text-white")}>
        {value}
      </dd>
    </div>
  );
}

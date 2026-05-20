// crates/ui/src/components/DiagnosticPanel.tsx
//
// Example:
//   <DiagnosticPanel
//     latencies={[{ plugin: "steering", p50: 4.2, p95: 9.1, p99: 14.0 }, ...]}
//     watchdog={[{ t: 1730000000000, ok: true }, ...]}
//   />
//
// Skeleton: per-plugin latency rows + a 60-tick watchdog strip. Minimal chrome —
// labels and mono numerics. Bars are single-color (no gradient).

import * as React from "react";
import { cn } from "@/lib/utils";

export interface PluginLatency {
  plugin: string;
  p50: number;       // ms
  p95: number;
  p99: number;
  budgetMs?: number; // default 16ms
}

export interface WatchdogTick { t: number; ok: boolean }

export interface DiagnosticPanelProps {
  latencies: PluginLatency[];
  watchdog?: WatchdogTick[];
  className?: string;
}

function LatencyRow({ row }: { row: PluginLatency }) {
  const budget = row.budgetMs ?? 16;
  const widthPct = (n: number) => Math.min(100, (n / (budget * 1.5)) * 100);
  const isOver = row.p95 > budget;

  return (
    <div className="grid grid-cols-12 gap-2 items-center h-7 px-2 hover:bg-surface-elevated transition-colors duration-[var(--dur-1)] rounded-sm">
      <span className="col-span-3 font-mono text-[11px] text-fg truncate">{row.plugin}</span>
      <div className="col-span-6 relative h-1.5 rounded-sm bg-surface-card border border-subtle overflow-hidden">
        <span
          aria-hidden
          className="absolute top-0 bottom-0 left-0"
          style={{
            width: `${widthPct(row.p95)}%`,
            background: isOver ? "var(--warning)" : "var(--brand)",
          }}
        />
        <span
          aria-hidden
          className="absolute top-0 bottom-0 w-px bg-border-strong"
          style={{ left: `${(budget / (budget * 1.5)) * 100}%` }}
        />
      </div>
      <div className="col-span-3 flex items-center justify-end gap-3 font-mono text-[10px] tabular-nums text-fg-muted">
        <span>p50 <span className="text-fg-secondary">{row.p50.toFixed(1)}</span></span>
        <span>p95 <span className={isOver ? "text-warning" : "text-fg-secondary"}>{row.p95.toFixed(1)}</span></span>
        <span>p99 <span className="text-fg-secondary">{row.p99.toFixed(1)}</span></span>
      </div>
    </div>
  );
}

function WatchdogStrip({ ticks }: { ticks: WatchdogTick[] }) {
  const slice = ticks.slice(-60);
  return (
    <div className="flex items-center gap-[2px] h-6">
      {slice.map((t, i) => (
        <span
          key={i}
          title={new Date(t.t).toLocaleTimeString()}
          className={cn(
            "w-1.5 h-4 rounded-[1px]",
            t.ok ? "bg-success/70" : "bg-danger",
          )}
        />
      ))}
      {slice.length === 0 && (
        <span className="text-fg-muted text-xs font-sans">No samples yet.</span>
      )}
    </div>
  );
}

export function DiagnosticPanel({ latencies, watchdog, className }: DiagnosticPanelProps) {
  return (
    <div className={cn("bg-surface-card border border-subtle rounded-md p-4 flex flex-col gap-4", className)}>
      <header>
        <h3 className="text-fg-muted text-xs uppercase tracking-wider font-sans">
          Plugin latencies
        </h3>
      </header>
      <div className="flex flex-col">
        {latencies.length === 0 ? (
          <p className="text-fg-muted text-xs font-sans px-2">No plugin samples yet.</p>
        ) : latencies.map((row) => <LatencyRow key={row.plugin} row={row} />)}
      </div>

      {watchdog && (
        <div className="mt-2">
          <h3 className="text-fg-muted text-xs uppercase tracking-wider font-sans mb-2">
            Watchdog — last 60 ticks
          </h3>
          <WatchdogStrip ticks={watchdog} />
        </div>
      )}
    </div>
  );
}

export default DiagnosticPanel;

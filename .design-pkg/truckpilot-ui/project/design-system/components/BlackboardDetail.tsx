// crates/ui/src/components/BlackboardDetail.tsx
//
// Example:
//   <BlackboardDetail node={selected} history={selectedHistory} />
//
// Right-hand pane of the Blackboard inspector. Mono value, muted type badge,
// optional sparkline if the value is numeric and a history buffer is supplied.

import * as React from "react";
import { Copy, Check } from "lucide-react";
import type { BlackboardNode, BlackboardType } from "./BlackboardTreeItem";
import { TelemetrySparkline } from "./TelemetrySparkline";
import { cn } from "@/lib/utils";

export interface BlackboardDetailProps {
  node: BlackboardNode | null;
  history?: Array<{ t: number; v: number }>;
  className?: string;
}

const NUMERIC: BlackboardType[] = ["i32", "i64", "f32", "f64"];

export function BlackboardDetail({ node, history, className }: BlackboardDetailProps) {
  const [copied, setCopied] = React.useState(false);

  if (!node) {
    return (
      <div className={cn("h-full flex items-center justify-center text-fg-muted text-sm font-sans", className)}>
        Select a node to inspect.
      </div>
    );
  }

  const isNumeric = NUMERIC.includes(node.type);
  const valueText = node.value == null ? "null"
    : typeof node.value === "object" ? JSON.stringify(node.value, null, 2)
    : String(node.value);

  const copy = async () => {
    try {
      await navigator.clipboard.writeText(valueText);
      setCopied(true);
      setTimeout(() => setCopied(false), 1200);
    } catch {/* no-op */}
  };

  return (
    <div className={cn("h-full flex flex-col gap-4 p-4", className)}>
      <header className="flex items-start gap-2">
        <div className="flex-1 min-w-0">
          <p className="font-mono text-xs text-fg-muted truncate">{node.key}</p>
          <h2 className="font-sans text-md text-fg mt-1 truncate">
            {node.label ?? node.key.split(".").pop()}
          </h2>
        </div>
        <span className="font-mono text-[10px] uppercase tracking-wider text-fg-muted px-1.5 h-5 inline-flex items-center rounded-sm border border-subtle">
          {node.type}
        </span>
        <button
          type="button"
          onClick={copy}
          aria-label="Copy value"
          className="inline-flex items-center justify-center w-7 h-7 rounded-sm text-fg-muted hover:text-fg hover:bg-surface-elevated transition-colors"
        >
          {copied ? <Check size={14} className="text-success" /> : <Copy size={14} />}
        </button>
      </header>

      <div className="font-mono text-2xl tabular-nums text-fg leading-tight">
        {isNumeric && Number.isFinite(Number(node.value))
          ? Number(node.value).toFixed(node.type.startsWith("f") ? 3 : 0)
          : valueText.length > 80 ? `${valueText.slice(0, 79)}…` : valueText}
      </div>

      {isNumeric && history && history.length > 1 && (
        <div className="bg-surface-elevated border border-subtle rounded-sm p-3">
          <p className="text-fg-muted text-xs uppercase tracking-wider font-sans mb-2">
            Recent
          </p>
          <TelemetrySparkline data={history} accent="brand" fill height={80} />
        </div>
      )}

      {!isNumeric && typeof node.value === "object" && node.value !== null && (
        <pre className="font-mono text-xs text-fg-secondary bg-surface-elevated border border-subtle rounded-sm p-3 overflow-auto max-h-[40vh]">
          {JSON.stringify(node.value, null, 2)}
        </pre>
      )}
    </div>
  );
}

export default BlackboardDetail;

// crates/ui/src/components/BlackboardTreeItem.tsx
//
// Example:
//   <BlackboardTreeItem
//     node={{ key: "telemetry.speed", type: "f32", value: 64.3, depth: 1, hasChildren: false }}
//     selected={selected === node.key}
//     onSelect={() => setSelected(node.key)}
//   />

import * as React from "react";
import { ChevronRight } from "lucide-react";
import { cn } from "@/lib/utils";

export type BlackboardType = "bool" | "i32" | "i64" | "f32" | "f64" | "string" | "object" | "array" | "null";

export interface BlackboardNode {
  key: string;            // dot-separated path
  label?: string;         // last segment if not provided
  type: BlackboardType;
  value: unknown;
  depth: number;          // 0-based
  hasChildren?: boolean;
  expanded?: boolean;
}

export interface BlackboardTreeItemProps {
  node: BlackboardNode;
  selected?: boolean;
  onSelect?: () => void;
  onToggleExpand?: () => void;
  style?: React.CSSProperties;
  className?: string;
}

function formatValue(type: BlackboardType, v: unknown): string {
  if (v == null) return "null";
  if (type === "object" || type === "array") return type;
  if (type === "f32" || type === "f64") {
    const n = Number(v);
    return Number.isFinite(n) ? n.toFixed(2) : "—";
  }
  if (typeof v === "string") return `"${v.length > 24 ? v.slice(0, 23) + "…" : v}"`;
  return String(v);
}

export function BlackboardTreeItem({
  node,
  selected,
  onSelect,
  onToggleExpand,
  style,
  className,
}: BlackboardTreeItemProps) {
  const label = node.label ?? node.key.split(".").pop() ?? node.key;
  return (
    <div
      role="treeitem"
      aria-selected={selected}
      aria-expanded={node.hasChildren ? !!node.expanded : undefined}
      onClick={onSelect}
      style={style}
      className={cn(
        "group flex items-center gap-2 h-6 pr-2 cursor-pointer",
        "border-l-2 border-l-transparent",
        "hover:bg-surface-elevated transition-colors duration-[var(--dur-1)]",
        selected && "bg-brand-soft border-l-brand",
        className,
      )}
    >
      <span style={{ width: node.depth * 12 + 4 }} aria-hidden />

      {node.hasChildren ? (
        <button
          type="button"
          onClick={(e) => { e.stopPropagation(); onToggleExpand?.(); }}
          aria-label={node.expanded ? "Collapse" : "Expand"}
          className="w-4 h-4 inline-flex items-center justify-center text-fg-muted hover:text-fg shrink-0"
        >
          <ChevronRight
            size={12}
            className={cn("transition-transform duration-[var(--dur-1)]", node.expanded && "rotate-90")}
          />
        </button>
      ) : (
        <span className="w-4 shrink-0" aria-hidden />
      )}

      <span className="font-mono text-[12px] text-fg truncate flex-1">{label}</span>

      <span className="font-mono text-[10px] uppercase text-fg-muted shrink-0 w-12 text-right">
        {node.type}
      </span>

      <span className="font-mono text-[12px] text-fg-secondary tabular-nums shrink-0 max-w-[160px] truncate">
        {formatValue(node.type, node.value)}
      </span>
    </div>
  );
}

export default BlackboardTreeItem;

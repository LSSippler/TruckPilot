import * as React from "react";
import { cn } from "@/lib/utils";

export type LogLineLevel = "DEBUG" | "INFO" | "WARN" | "ERROR";

export interface LogLineEntry {
  ts: number;
  level: LogLineLevel;
  source: string;
  message: string;
}

export interface LogLineProps {
  entry: LogLineEntry;
  style?: React.CSSProperties;
  className?: string;
}

const BORDER: Record<LogLineLevel, string> = {
  DEBUG: "border-l-border-strong",
  INFO:  "border-l-info",
  WARN:  "border-l-warning",
  ERROR: "border-l-danger",
};

const LEVEL_TEXT: Record<LogLineLevel, string> = {
  DEBUG: "text-fg-muted",
  INFO:  "text-info",
  WARN:  "text-warning",
  ERROR: "text-danger",
};

function fmtTs(ts: number) {
  const d = new Date(ts);
  const h = d.getHours().toString().padStart(2, "0");
  const m = d.getMinutes().toString().padStart(2, "0");
  const s = d.getSeconds().toString().padStart(2, "0");
  const ms = d.getMilliseconds().toString().padStart(3, "0");
  return `${h}:${m}:${s}.${ms}`;
}

export function LogLine({ entry, style, className }: LogLineProps) {
  return (
    <div
      style={style}
      className={cn(
        "flex items-center gap-3 h-6 pl-2 pr-3 border-l-2",
        "hover:bg-surface-elevated transition-colors duration-[var(--dur-1)]",
        BORDER[entry.level],
        className,
      )}
    >
      <span className="font-mono text-[11px] text-fg-muted tabular-nums shrink-0 w-[88px]">
        {fmtTs(entry.ts)}
      </span>
      <span className={cn("font-mono text-[10px] uppercase shrink-0 w-12", LEVEL_TEXT[entry.level])}>
        {entry.level}
      </span>
      <span className="font-mono text-[11px] text-fg-secondary shrink-0 w-24 truncate">
        {entry.source}
      </span>
      <span className="font-mono text-[12px] text-fg truncate flex-1">
        {entry.message}
      </span>
    </div>
  );
}

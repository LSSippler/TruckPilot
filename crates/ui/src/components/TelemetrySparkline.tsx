import * as React from "react";
import {
  ResponsiveContainer,
  LineChart,
  Line,
  YAxis,
  Tooltip,
} from "recharts";
import { cn } from "@/lib/utils";

export interface TelemetrySparklineProps {
  data: Array<{ t: number; v: number }>;
  accent?: "brand" | "chart-1" | "chart-2" | "chart-3" | "danger" | "warning";
  fill?: boolean;
  height?: number;
  unit?: string;
  className?: string;
}

const COLOR_VAR: Record<NonNullable<TelemetrySparklineProps["accent"]>, string> = {
  brand:     "var(--brand)",
  "chart-1": "var(--chart-1)",
  "chart-2": "var(--chart-2)",
  "chart-3": "var(--chart-3)",
  danger:    "var(--danger)",
  warning:   "var(--warning)",
};

export function TelemetrySparkline({
  data,
  accent = "brand",
  fill = false,
  height = 36,
  unit,
  className,
}: TelemetrySparklineProps) {
  const stroke = COLOR_VAR[accent];
  const gradId = React.useId().replace(/:/g, "_");
  const last = data[data.length - 1]?.v;

  return (
    <div className={cn("relative w-full", className)} style={{ height }}>
      <ResponsiveContainer width="100%" height="100%">
        <LineChart data={data} margin={{ top: 4, right: 8, bottom: 0, left: 0 }}>
          <defs>
            {fill && (
              <linearGradient id={gradId} x1="0" y1="0" x2="0" y2="1">
                <stop offset="0%" stopColor={stroke} stopOpacity={0.18} />
                <stop offset="100%" stopColor={stroke} stopOpacity={0} />
              </linearGradient>
            )}
          </defs>
          <YAxis hide domain={["dataMin", "dataMax"]} />
          <Tooltip
            cursor={false}
            isAnimationActive={false}
            content={({ active, payload }) => {
              if (!active || !payload?.length) return null;
              const v = payload[0]?.value as number | undefined;
              return (
                <div className="bg-surface-overlay border border-subtle rounded-sm px-2 py-1 font-mono text-xs text-fg">
                  {v != null && Number.isFinite(v) ? (v as number).toFixed(0) : "—"}
                  {unit && <span className="text-fg-muted ml-1">{unit}</span>}
                </div>
              );
            }}
          />
          <Line
            type="monotone"
            dataKey="v"
            stroke={stroke}
            strokeWidth={1.5}
            dot={false}
            isAnimationActive={false}
            fill={fill ? `url(#${gradId})` : "none"}
          />
        </LineChart>
      </ResponsiveContainer>
      {Number.isFinite(last) && (
        <span
          aria-hidden
          className="absolute right-1 top-1/2 -translate-y-1/2 w-1.5 h-1.5 rounded-full"
          style={{ background: stroke }}
        />
      )}
    </div>
  );
}

// crates/ui/src/components/SpeedLimitDisplay.tsx
//
// Example:
//   <SpeedLimitDisplay limitKmh={80} currentKmh={86} />
//
// Round badge with mono number. Conveys violation via a red ring (not a full red
// fill). Generic visual language — a circle with a number, not a copy of any
// specific country's road-sign design.

import * as React from "react";
import { cn } from "@/lib/utils";

export interface SpeedLimitDisplayProps {
  limitKmh: number | null;
  currentKmh?: number;
  size?: "sm" | "md";
  className?: string;
}

export function SpeedLimitDisplay({
  limitKmh,
  currentKmh,
  size = "md",
  className,
}: SpeedLimitDisplayProps) {
  const dim = size === "sm" ? 36 : 52;
  const overLimit =
    limitKmh != null && currentKmh != null && currentKmh > limitKmh + 2;

  return (
    <div
      role="img"
      aria-label={limitKmh ? `Speed limit ${limitKmh} km/h` : "No speed limit"}
      style={{ width: dim, height: dim }}
      className={cn(
        "relative rounded-full bg-surface-card border-2 flex items-center justify-center select-none",
        overLimit ? "border-danger" : "border-strong",
        className,
      )}
    >
      <span
        className={cn(
          "font-mono tabular-nums leading-none",
          size === "sm" ? "text-sm" : "text-md font-medium",
          overLimit ? "text-danger" : "text-fg",
        )}
      >
        {limitKmh ?? "—"}
      </span>
    </div>
  );
}

export default SpeedLimitDisplay;

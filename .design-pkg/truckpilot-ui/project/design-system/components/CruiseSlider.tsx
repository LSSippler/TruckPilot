// crates/ui/src/components/CruiseSlider.tsx
//
// Example:
//   <CruiseSlider value={cruise} onChange={setCruise} min={0} max={120} markers={[50, 80, 100]} />
//
// shadcn Slider extended: brand track, mono value badge, tick markers
// rendered as 1px lines, hover tooltip.

import * as React from "react";
import * as SliderPrimitive from "@radix-ui/react-slider";
import { cn } from "@/lib/utils";

export interface CruiseSliderProps {
  value: number;
  onChange: (v: number) => void;
  min?: number;
  max?: number;
  step?: number;
  unit?: string;       // default "km/h"
  markers?: number[];  // default [50, 80, 100]
  className?: string;
}

export function CruiseSlider({
  value,
  onChange,
  min = 0,
  max = 120,
  step = 1,
  unit = "km/h",
  markers = [50, 80, 100],
  className,
}: CruiseSliderProps) {
  const pct = (n: number) => ((n - min) / (max - min)) * 100;

  return (
    <div className={cn("flex flex-col gap-2", className)}>
      <div className="flex items-baseline justify-between">
        <span className="text-fg-muted text-xs uppercase tracking-wider font-sans">
          Cruise target
        </span>
        <span className="font-mono text-md tabular-nums text-fg">
          {value}
          <span className="text-fg-muted ml-1 text-xs">{unit}</span>
        </span>
      </div>

      <SliderPrimitive.Root
        value={[value]}
        min={min}
        max={max}
        step={step}
        onValueChange={(v) => onChange(v[0])}
        className="relative flex items-center select-none touch-none w-full h-5 group"
      >
        <SliderPrimitive.Track
          className="relative grow h-1 rounded-sm bg-surface-elevated border border-subtle overflow-hidden"
        >
          <SliderPrimitive.Range className="absolute h-full bg-brand" />
        </SliderPrimitive.Track>

        {/* Tick markers — sit behind the thumb */}
        <div className="absolute inset-x-0 top-1/2 -translate-y-1/2 h-3 pointer-events-none">
          {markers.map((m) => (
            <span
              key={m}
              className="absolute top-1/2 -translate-y-1/2 w-px h-2 bg-border-strong"
              style={{ left: `${pct(m)}%` }}
              aria-hidden
            />
          ))}
        </div>

        <SliderPrimitive.Thumb
          className={cn(
            "block w-3.5 h-3.5 rounded-full bg-fg border-2 border-brand",
            "transition-transform duration-[var(--dur-1)] ease-out",
            "focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-[var(--ring)] focus-visible:ring-offset-2 focus-visible:ring-offset-[var(--ring-offset)]",
            "hover:scale-110",
          )}
        />
      </SliderPrimitive.Root>

      <div className="relative h-3 -mt-1">
        {markers.map((m) => (
          <span
            key={m}
            className="absolute -translate-x-1/2 font-mono text-[10px] text-fg-muted tabular-nums"
            style={{ left: `${pct(m)}%` }}
          >
            {m}
          </span>
        ))}
      </div>
    </div>
  );
}

export default CruiseSlider;

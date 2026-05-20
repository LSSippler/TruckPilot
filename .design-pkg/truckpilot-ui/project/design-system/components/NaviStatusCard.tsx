// crates/ui/src/components/NaviStatusCard.tsx
//
// Example:
//   <NaviStatusCard navi={navi} />
//
// Three states. The no-navi state is calm: no warning color, no FAULT banner —
// just a hint that the user has to set a destination in ETS2.

import * as React from "react";
import {
  ArrowUpRight,
  ArrowUpLeft,
  ArrowUp,
  CornerUpRight,
  CornerUpLeft,
  Flag,
  MapPinOff,
  Clock,
  Route as RouteIcon,
} from "lucide-react";
import { cn } from "@/lib/utils";

export type ManeuverKind =
  | "straight"
  | "turn-left"
  | "turn-right"
  | "slight-left"
  | "slight-right"
  | "uturn"
  | "destination";

export interface NaviState {
  currentRoad?: string;
  nextManeuver?: ManeuverKind;
  nextManeuverHint?: string;       // e.g. "Onto E45"
  distanceToManeuver?: number;     // meters
  totalDistance?: number;          // meters
  etaIso?: string;                 // ISO string, formatted client-side
}

export interface NaviStatusCardProps {
  navi: NaviState | null;
  className?: string;
}

const MANEUVER_ICON: Record<ManeuverKind, React.ComponentType<{ size?: number; className?: string }>> = {
  "straight":      ArrowUp,
  "turn-left":     CornerUpLeft,
  "turn-right":    CornerUpRight,
  "slight-left":   ArrowUpLeft,
  "slight-right":  ArrowUpRight,
  "uturn":         CornerUpLeft,
  "destination":   Flag,
};

function fmtDistance(m?: number) {
  if (m == null || !Number.isFinite(m)) return "—";
  if (m < 1000) return `${Math.round(m / 10) * 10} m`;
  return `${(m / 1000).toFixed(m < 10000 ? 1 : 0)} km`;
}

function fmtEta(iso?: string) {
  if (!iso) return "—";
  const d = new Date(iso);
  if (Number.isNaN(d.getTime())) return "—";
  return d.toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" });
}

export function NaviStatusCard({ navi, className }: NaviStatusCardProps) {
  // ---------- no-navi state ----------
  if (!navi || !navi.nextManeuver) {
    return (
      <div
        className={cn(
          "h-full bg-surface-card border border-subtle rounded-md p-4",
          "flex items-center gap-3",
          className,
        )}
      >
        <div className="w-9 h-9 rounded-sm bg-surface-elevated border border-subtle flex items-center justify-center text-fg-muted">
          <MapPinOff size={16} />
        </div>
        <div className="min-w-0">
          <p className="text-fg text-sm font-sans">No active route</p>
          <p className="text-fg-muted text-xs font-sans mt-0.5">
            Set a destination in the ETS2 in-game navigation to begin.
          </p>
        </div>
      </div>
    );
  }

  // ---------- active state ----------
  const Icon = MANEUVER_ICON[navi.nextManeuver];
  const approaching = navi.distanceToManeuver != null && navi.distanceToManeuver < 250;

  return (
    <div
      className={cn(
        "h-full bg-surface-card border border-subtle rounded-md p-4",
        "grid grid-cols-12 gap-4 items-center",
        className,
      )}
    >
      {/* Maneuver — left, big */}
      <div className="col-span-6 flex items-center gap-3">
        <div
          className={cn(
            "w-12 h-12 rounded-md flex items-center justify-center shrink-0",
            "border",
            approaching
              ? "bg-brand-soft border-transparent text-brand"
              : "bg-surface-elevated border-subtle text-fg",
          )}
        >
          <Icon size={24} />
        </div>
        <div className="min-w-0">
          <div className="flex items-baseline gap-2">
            <span className="font-mono text-2xl tabular-nums text-fg leading-none">
              {fmtDistance(navi.distanceToManeuver)}
            </span>
          </div>
          {navi.nextManeuverHint && (
            <p className="text-fg-secondary text-sm font-sans mt-1 truncate">
              {navi.nextManeuverHint}
            </p>
          )}
        </div>
      </div>

      {/* Current road */}
      <div className="col-span-3 min-w-0">
        <p className="text-fg-muted text-xs uppercase tracking-wider font-sans">On</p>
        <p className="text-fg text-sm font-sans mt-1 truncate" title={navi.currentRoad}>
          {navi.currentRoad ?? "—"}
        </p>
      </div>

      {/* ETA + total distance */}
      <div className="col-span-3 flex flex-col gap-1 text-right">
        <div className="flex items-center justify-end gap-1.5 text-fg-secondary text-sm font-sans">
          <Clock size={12} className="text-fg-muted" />
          <span className="font-mono tabular-nums">{fmtEta(navi.etaIso)}</span>
        </div>
        <div className="flex items-center justify-end gap-1.5 text-fg-muted text-xs font-sans">
          <RouteIcon size={12} />
          <span className="font-mono tabular-nums">{fmtDistance(navi.totalDistance)}</span>
        </div>
      </div>
    </div>
  );
}

export default NaviStatusCard;

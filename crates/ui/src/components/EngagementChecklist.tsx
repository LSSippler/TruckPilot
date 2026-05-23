import { useEffect } from "react";
import { Check, X, Clock } from "lucide-react";
import { cn } from "@/lib/utils";
import { subscribeBlackboardKeys } from "@/lib/ipc";
import { useBlackboardStore } from "@/stores/blackboard";

const ENGAGE_KEYS = [
  "state.engage_precondition_telemetry_fresh",
  "state.engage_precondition_truck_on_road",
  "state.engage_precondition_heading_aligned",
  "state.engage_precondition_route_planned",
  "state.engage_precondition_truck_on_route",
  "state.engage_precondition_speed_ok",
  "state.engage_precondition_heading_ok_for_engage",
  "state.engage_precondition_lane_keeper_engage_allowed",
  "state.engage_detail_snap_dist_m",
  "state.engage_detail_heading_diff_deg",
  "state.engage_detail_speed_kmh",
  "state.engage_detail_telemetry_age_ms",
  "state.engage_advisory",
  "plugin.lane_keeper.mode",
] as const;

interface ChecklistRow {
  key: string;
  label: string;
  detailKey: string;
  detailUnit: string;
  /** If set, this row is only shown in the given mode. undefined = always shown. */
  onlyMode?: "vision" | "route_following";
}

const ROWS: ChecklistRow[] = [
  {
    key: "state.engage_precondition_telemetry_fresh",
    label: "Telemetry fresh",
    detailKey: "state.engage_detail_telemetry_age_ms",
    detailUnit: " ms",
  },
  {
    key: "state.engage_precondition_truck_on_road",
    label: "Truck on road",
    detailKey: "state.engage_detail_snap_dist_m",
    detailUnit: " m",
  },
  {
    key: "state.engage_precondition_heading_aligned",
    label: "Heading aligned",
    detailKey: "state.engage_detail_heading_diff_deg",
    detailUnit: "°",
    onlyMode: "route_following",
  },
  {
    key: "state.engage_precondition_route_planned",
    label: "Route planned",
    detailKey: "",
    detailUnit: "",
    onlyMode: "route_following",
  },
  {
    key: "state.engage_precondition_truck_on_route",
    label: "Truck on route",
    detailKey: "",
    detailUnit: "",
    onlyMode: "route_following",
  },
  {
    key: "state.engage_precondition_speed_ok",
    label: "Speed > 5 km/h",
    detailKey: "state.engage_detail_speed_kmh",
    detailUnit: " km/h",
  },
  {
    key: "state.engage_precondition_lane_keeper_engage_allowed",
    label: "Lane-keeper ready",
    detailKey: "",
    detailUnit: "",
    onlyMode: "vision",
  },
];

function parseState(val: string | undefined): "ok" | "fail" | "pending" {
  if (val === "true") return "ok";
  if (val === "false") return "fail";
  return "pending";
}

export function EngagementChecklist() {
  const values = useBlackboardStore((s) => s.values);

  useEffect(() => subscribeBlackboardKeys(ENGAGE_KEYS), []);

  const mode = values["plugin.lane_keeper.mode"] ?? "route_following";
  const advisory = values["state.engage_advisory"];

  const visibleRows = ROWS.filter(
    (row) => row.onlyMode === undefined || row.onlyMode === mode,
  );

  return (
    <div className="flex flex-col gap-1">
      <span className="text-[10px] font-sans uppercase tracking-wider text-fg-muted mb-0.5">
        Engagement
      </span>
      {visibleRows.map((row) => {
        const state = parseState(values[row.key]);
        const detail = row.detailKey ? values[row.detailKey] : undefined;

        return (
          <div
            key={row.key}
            className={cn(
              "flex items-center justify-between gap-2 h-6 px-1.5 rounded-sm",
              "text-xs font-sans",
              state === "ok"
                ? "text-fg-secondary"
                : state === "fail"
                  ? "text-fg-muted"
                  : "text-fg-muted/50",
            )}
          >
            <span className="inline-flex items-center gap-1.5 min-w-0">
              {state === "ok" && (
                <Check size={12} className="text-success shrink-0" aria-hidden />
              )}
              {state === "fail" && (
                <X size={12} className="text-danger shrink-0" aria-hidden />
              )}
              {state === "pending" && (
                <Clock size={12} className="text-warning shrink-0" aria-hidden />
              )}
              <span className="truncate">{row.label}</span>
            </span>
            {detail !== undefined && (
              <span className="font-mono text-[10px] text-fg-muted shrink-0">
                {detail}
                {row.detailUnit}
              </span>
            )}
          </div>
        );
      })}
      {advisory && (
        <div className="mt-0.5 px-1.5 py-1 rounded-sm bg-danger/10 text-[10px] font-sans text-danger leading-snug">
          {advisory}
        </div>
      )}
    </div>
  );
}

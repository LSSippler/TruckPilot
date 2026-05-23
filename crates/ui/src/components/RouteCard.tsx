import { useEffect, useMemo, useRef, useState } from "react";
import { toast } from "sonner";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { sendCommand, subscribeBlackboardKeys } from "@/lib/ipc";
import { useBlackboardStore } from "@/stores/blackboard";

// Convenience targets from validate-cities output 2026-05-23.
// Cities marked ⚠ have snap_dist > 5 km — reachable but routing may be slow.
const CITY_PRESETS: Array<{ label: string; uid: string }> = [
  { label: "Berlin",      uid: "6919855103841468416" },
  { label: "Hamburg",     uid: "6526933291294064640" },
  { label: "München",     uid: "6972029165941424128" },
  { label: "Wien",        uid: "7164828206179905841" },
  { label: "Prag",        uid: "6054870934110617351" },
  { label: "Warschau",    uid: "6219844824041607977" },
  { label: "Amsterdam",   uid: "511341911874732032" },
  { label: "Brüssel",     uid: "437485961594863618" },
  { label: "Paris",       uid: "358087285684699136" },
  { label: "Mailand",     uid: "11599704834989049" },
  { label: "Madrid ⚠",   uid: "5616765011398492172" },
  { label: "Barcelona",   uid: "340240176744890368" },
  { label: "Sevilla",     uid: "3474532028983869440" },
  { label: "Valencia ⚠", uid: "367699086200471552" },
  { label: "Lissabon ⚠", uid: "3869659156576534528" },
  { label: "Rom",         uid: "5638809273708251869" },
  { label: "Venedig",     uid: "7122087662525098421" },
  { label: "Neapel",      uid: "5638809254120877035" },
  { label: "Genua",       uid: "367702008359157762" },
  { label: "Marseille",   uid: "3384404907402272269" },
  { label: "Lyon",        uid: "425473086242226176" },
  { label: "Stockholm",   uid: "137837484578245493" },
  { label: "Oslo ⚠",     uid: "6445172487082147841" },
  { label: "Göteborg ⚠", uid: "6393693561438601218" },
  { label: "Helsinki",    uid: "236442477454688258" },
  { label: "Bukarest",    uid: "4809609004064950816" },
  { label: "Sofia",       uid: "4809609004245254663" },
  { label: "Istanbul",    uid: "307179060578484224" },
  { label: "Konstanza",   uid: "11599705663390297" },
  { label: "Zagreb",      uid: "4096968979838010780" },
  { label: "Belgrad",     uid: "4328500494390527701" },
  { label: "Ljubljana",   uid: "4229118664441988414" },
  { label: "Riga",        uid: "5657244115898544942" },
  { label: "Vilnius",     uid: "6302930001291771904" },
  { label: "Tallinn",     uid: "6804060420532011009" },
  { label: "Krakau",      uid: "3523795922494426004" },
  { label: "Bratislava",  uid: "5463961135592842905" },
  { label: "Budapest",    uid: "7101282311152336898" },
];

const ROUTE_KEYS = [
  "router.active",
  "router.goal_uid",
  "router.waypoints",
  "router.current_goal_uid",
  "router.last_planning_result",
  "router.last_planning_attempt_at",
  "router.last_planning_duration_ms",
  "router.last_planning_error_detail",
  "router.waypoint_count",
  "router.path_total_distance_m",
] as const;

// How long to wait for router.last_planning_result after Set Goal.
const PLANNING_TIMEOUT_MS = 10_000;

type PlanningState = "idle" | "waiting" | "done";

function parseUid(raw: string): bigint | null {
  const t = raw.trim();
  if (t === "") return null;
  try {
    if (/^0x[0-9a-fA-F]+$/.test(t)) return BigInt(t);
    if (/^[0-9]+$/.test(t)) return BigInt(t);
  } catch {
    return null;
  }
  return null;
}

function waypointCount(json: string | undefined): number | null {
  if (!json) return null;
  try {
    const parsed = JSON.parse(json);
    return Array.isArray(parsed) ? parsed.length : null;
  } catch {
    return null;
  }
}

function distanceKm(raw: string | undefined): string | null {
  if (!raw) return null;
  const v = parseFloat(raw);
  if (isNaN(v) || v <= 0) return null;
  return (v / 1000).toFixed(1);
}

function resultLabel(result: string): string {
  switch (result) {
    case "ok":                 return "Route planned";
    case "uid_not_in_graph":   return "UID not in map";
    case "no_path_found":      return "No path found";
    case "start_node_unknown": return "Start position unknown";
    case "uid_parse_error":    return "UID parse error";
    default:                   return result || "—";
  }
}

export function RouteCard() {
  const [uidInput, setUidInput] = useState("");
  const [planningState, setPlanningState] = useState<PlanningState>("idle");
  // Timestamp (ms) of when Set Goal was last clicked — used to detect a *new*
  // planning result vs. a stale one that was already in the blackboard.
  const clickTimestampRef = useRef<number | null>(null);
  const timeoutRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  const values = useBlackboardStore((s) => s.values);

  useEffect(() => subscribeBlackboardKeys(ROUTE_KEYS), []);

  // Watch for planning result and show toast when waiting.
  const lastAttemptAt = values["router.last_planning_attempt_at"];
  const lastResult = values["router.last_planning_result"];
  const lastDurationMs = values["router.last_planning_duration_ms"];
  const lastDetail = values["router.last_planning_error_detail"];
  const waypointCountBb = values["router.waypoint_count"];
  const distanceRaw = values["router.path_total_distance_m"];

  useEffect(() => {
    if (planningState !== "waiting") return;
    if (!lastAttemptAt || !lastResult) return;

    // Ignore stale results that pre-date our click.
    const attemptMs = Number(lastAttemptAt);
    if (clickTimestampRef.current !== null && attemptMs < clickTimestampRef.current - 2000) {
      return;
    }

    // Clear the timeout since we got a result.
    if (timeoutRef.current !== null) {
      clearTimeout(timeoutRef.current);
      timeoutRef.current = null;
    }
    setPlanningState("done");

    const durationMs = lastDurationMs ? `${lastDurationMs} ms` : "";
    const wps = waypointCountBb ?? "?";
    const km = distanceKm(distanceRaw);

    switch (lastResult) {
      case "ok":
        toast.success("Route geplant", {
          description: `${wps} waypoints · ${km ? km + " km" : ""} · ${durationMs}`,
        });
        break;
      case "uid_not_in_graph":
        toast.error("UID nicht in Map", {
          description: lastDetail || "Goal UID not found in graph.json",
        });
        break;
      case "no_path_found":
        toast.warning("Kein Pfad gefunden", {
          description: lastDetail || "A* returned no route",
        });
        break;
      case "start_node_unknown":
        toast.warning("Start-Position nicht auf der Karte", {
          description: lastDetail || "No graph node near truck position",
        });
        break;
      case "uid_parse_error":
        toast.error("UID Parse Error", {
          description: lastDetail || "Could not parse UID",
        });
        break;
      default:
        toast.info(resultLabel(lastResult), { description: lastDetail });
    }
  }, [lastAttemptAt, lastResult, planningState, lastDurationMs, lastDetail, waypointCountBb, distanceRaw]);

  // Cleanup timeout on unmount.
  useEffect(
    () => () => {
      if (timeoutRef.current !== null) clearTimeout(timeoutRef.current);
    },
    [],
  );

  const parsed = useMemo(() => parseUid(uidInput), [uidInput]);
  const valid = parsed !== null;

  const routerActive = values["router.active"] === "true";
  const goalUid = values["router.goal_uid"];
  const wpRemaining = waypointCount(values["router.waypoints"]);
  const km = distanceKm(distanceRaw);

  const handleApply = () => {
    if (parsed === null) return;
    // Wire transport is a decimal string — see the `u64_string` serde helper
    // in `crates/ipc-protocol/src/lib.rs`. `bigint.toString()` is exact for
    // any u64 (no IEEE-754 path).
    const wire = parsed.toString();

    // Start waiting-for-result timer.
    clickTimestampRef.current = Date.now();
    setPlanningState("waiting");
    if (timeoutRef.current !== null) clearTimeout(timeoutRef.current);
    timeoutRef.current = setTimeout(() => {
      setPlanningState("done");
      toast.info("No response from router", {
        description: "Route planning may take up to 50 s on the next replan tick.",
      });
    }, PLANNING_TIMEOUT_MS);

    void sendCommand({ type: "set_router_goal", uid: wire }).then(
      () => toast.success("Goal sent", { description: wire }),
      (err) => {
        setPlanningState("idle");
        if (timeoutRef.current !== null) clearTimeout(timeoutRef.current);
        toast.error("Goal failed", { description: String(err) });
      },
    );
  };

  // ── Planning status badge ──────────────────────────────────────────────────
  const planningBadge = () => {
    if (planningState === "waiting") {
      return (
        <span className="text-xs text-yellow-500 animate-pulse">Planning…</span>
      );
    }
    if (!lastResult) return null;
    const ok = lastResult === "ok";
    return (
      <span className={`text-xs ${ok ? "text-emerald-500" : "text-red-400"}`}>
        {resultLabel(lastResult)}
        {lastDurationMs ? ` (${lastDurationMs} ms)` : ""}
      </span>
    );
  };

  return (
    <Card>
      <CardHeader>
        <CardTitle className="text-sm font-medium text-muted-foreground">Route</CardTitle>
      </CardHeader>
      <CardContent className="space-y-3">
        <div className="space-y-1.5">
          <Label className="text-xs">City preset</Label>
          <Select
            value=""
            onValueChange={(v) => {
              if (v) setUidInput(v);
            }}
          >
            <SelectTrigger>
              <SelectValue placeholder="Pick a city…" />
            </SelectTrigger>
            <SelectContent>
              {CITY_PRESETS.map((c) => (
                <SelectItem key={c.uid} value={c.uid}>
                  {c.label}
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
        </div>
        <div className="space-y-1.5">
          <Label className="text-xs">Goal UID (hex or decimal)</Label>
          <div className="flex gap-2">
            <Input
              value={uidInput}
              onChange={(e) => setUidInput(e.target.value)}
              placeholder="0x… or decimal u64"
              className="font-mono text-xs"
            />
            <Button onClick={handleApply} disabled={!valid || planningState === "waiting"}>
              {planningState === "waiting" ? "Planning…" : "Set Goal"}
            </Button>
          </div>
          {uidInput && !valid ? (
            <p className="text-xs text-red-500">Invalid UID (expected hex 0x… or decimal).</p>
          ) : null}
        </div>

        {/* Planning result feedback */}
        {(planningState !== "idle" || lastResult) && (
          <div className="rounded border border-dashed bg-muted/20 px-2 py-1.5 text-xs space-y-0.5">
            <div className="flex justify-between items-center">
              <span className="text-muted-foreground">Last attempt</span>
              {planningBadge()}
            </div>
            {lastResult && lastResult !== "ok" && lastDetail && (
              <p className="text-xs text-muted-foreground truncate" title={lastDetail}>
                {lastDetail}
              </p>
            )}
          </div>
        )}

        {/* Current route state */}
        <div className="rounded border bg-muted/40 p-2 text-xs space-y-1">
          <div className="flex justify-between">
            <span className="text-muted-foreground">router.active</span>
            <span className={routerActive ? "font-mono text-emerald-500" : "font-mono text-muted-foreground"}>
              {routerActive ? "true" : "false"}
            </span>
          </div>
          <div className="flex justify-between">
            <span className="text-muted-foreground">current goal</span>
            <span className="font-mono">{goalUid ?? "—"}</span>
          </div>
          <div className="flex justify-between">
            <span className="text-muted-foreground">waypoints</span>
            <span className="font-mono">{wpRemaining ?? "—"}</span>
          </div>
          {km && (
            <div className="flex justify-between">
              <span className="text-muted-foreground">distance</span>
              <span className="font-mono">{km} km</span>
            </div>
          )}
        </div>
      </CardContent>
    </Card>
  );
}

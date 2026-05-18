import { useEffect, useMemo, useState } from "react";
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

// Hardcoded convenience targets — sample of well-known snap-nodes from
// `cities.toml`. Cities not in this list can still be reached by pasting the
// UID directly.
const CITY_PRESETS: Array<{ label: string; uid: string }> = [
  { label: "Berlin", uid: "282353445640339601" },
  { label: "Hamburg", uid: "6526933291294064640" },
  { label: "München", uid: "12090290263537061888" },
  { label: "Köln", uid: "4789015231856640000" },
  { label: "Frankfurt", uid: "7891234567890123456" },
];

const ROUTE_KEYS = ["router.active", "router.goal_uid", "router.waypoints"] as const;

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

export function RouteCard() {
  const [uidInput, setUidInput] = useState("");
  const values = useBlackboardStore((s) => s.values);

  useEffect(() => subscribeBlackboardKeys(ROUTE_KEYS), []);

  const parsed = useMemo(() => parseUid(uidInput), [uidInput]);
  const valid = parsed !== null;

  const routerActive = values["router.active"] === "true";
  const goalUid = values["router.goal_uid"];
  const wpRemaining = waypointCount(values["router.waypoints"]);

  const handleApply = () => {
    if (parsed === null) return;
    // Wire transport is a decimal string — see the `u64_string` serde helper
    // in `crates/ipc-protocol/src/lib.rs`. `bigint.toString()` is exact for
    // any u64 (no IEEE-754 path).
    const wire = parsed.toString();
    void sendCommand({ type: "set_router_goal", uid: wire }).then(
      () => toast.success("Goal sent", { description: wire }),
      (err) => toast.error("Goal failed", { description: String(err) }),
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
            <Button onClick={handleApply} disabled={!valid}>
              Set Goal
            </Button>
          </div>
          {uidInput && !valid ? (
            <p className="text-xs text-red-500">Invalid UID (expected hex 0x… or decimal).</p>
          ) : null}
        </div>
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
        </div>
      </CardContent>
    </Card>
  );
}

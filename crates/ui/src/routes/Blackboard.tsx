import { useEffect, useMemo, useState } from "react";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { Badge } from "@/components/ui/badge";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Sparkline } from "@/components/telemetry/Sparkline";
import { cn } from "@/lib/utils";
import { useBlackboardStore, classifyKeyGroup } from "@/stores/blackboard";
import { subscribeBlackboardKeys } from "@/lib/ipc";

export function Blackboard() {
  const keys = useBlackboardStore((s) => s.keys);
  const values = useBlackboardStore((s) => s.values);
  const updatedAt = useBlackboardStore((s) => s.updatedAt);
  const history = useBlackboardStore((s) => s.history);
  const [filter, setFilter] = useState("");
  const [selected, setSelected] = useState<string | null>(null);

  // Subscribe to every known key. The poller batches BlackboardGet, so this
  // is cheap regardless of fanout. We re-subscribe whenever the key list
  // changes (every 5 s) so newly published keys appear without a refresh.
  useEffect(() => {
    if (keys.length === 0) return;
    return subscribeBlackboardKeys(keys);
  }, [keys]);

  const filteredKeys = useMemo(() => {
    if (!filter) return keys;
    const needle = filter.toLowerCase();
    return keys.filter((k) => k.toLowerCase().includes(needle));
  }, [keys, filter]);

  const grouped = useMemo(() => {
    const groups = new Map<string, string[]>();
    for (const k of filteredKeys) {
      const g = classifyKeyGroup(k);
      const list = groups.get(g) ?? [];
      list.push(k);
      groups.set(g, list);
    }
    return Array.from(groups.entries()).sort(([a], [b]) => a.localeCompare(b));
  }, [filteredKeys]);

  const selectedHistory = selected ? history[selected] ?? [] : [];
  const selectedNumeric = selectedHistory.length > 0;
  const selectedValue = selected ? values[selected] : undefined;
  const selectedType = inferType(selectedValue);
  const selectedAge = selected && updatedAt[selected]
    ? Date.now() - updatedAt[selected]!
    : null;

  return (
    <div className="grid h-full grid-cols-1 gap-4 md:grid-cols-[320px_1fr]">
      <Card className="flex flex-col overflow-hidden">
        <CardHeader className="space-y-2 pb-2">
          <CardTitle className="text-sm font-medium text-muted-foreground">
            Blackboard ({keys.length} keys)
          </CardTitle>
          <Input
            value={filter}
            onChange={(e) => setFilter(e.target.value)}
            placeholder="Filter…"
            className="h-8"
          />
        </CardHeader>
        <CardContent className="flex-1 overflow-hidden p-0">
          <ScrollArea className="h-full">
            <div className="space-y-2 p-3">
              {grouped.map(([group, items]) => (
                <div key={group}>
                  <div className="mb-1 text-[10px] uppercase tracking-wide text-muted-foreground">
                    {group}
                  </div>
                  <ul>
                    {items.map((k) => (
                      <li key={k}>
                        <button
                          onClick={() => setSelected(k)}
                          className={cn(
                            "block w-full rounded px-2 py-1 text-left text-xs font-mono",
                            selected === k
                              ? "bg-accent text-accent-foreground"
                              : "hover:bg-accent/40",
                          )}
                        >
                          {k.slice(group.length + 1) || k}
                        </button>
                      </li>
                    ))}
                  </ul>
                </div>
              ))}
              {grouped.length === 0 && (
                <p className="px-2 py-1 text-xs text-muted-foreground">No keys.</p>
              )}
            </div>
          </ScrollArea>
        </CardContent>
      </Card>

      <Card>
        <CardHeader>
          <CardTitle className="text-sm font-mono">
            {selected ?? "(select a key)"}
          </CardTitle>
        </CardHeader>
        <CardContent className="space-y-3">
          {selected ? (
            <>
              <div className="flex items-center gap-2">
                <Badge variant="secondary">{selectedType}</Badge>
                {selectedAge != null && (
                  <span className="text-xs text-muted-foreground">
                    updated {Math.round(selectedAge / 100) / 10}s ago
                  </span>
                )}
              </div>
              <div className="rounded border bg-muted/30 p-3 font-mono text-sm break-all">
                {selectedValue ?? <span className="text-muted-foreground">absent</span>}
              </div>
              {selectedNumeric && (
                <div>
                  <div className="mb-1 text-xs text-muted-foreground">last 60 s</div>
                  <Sparkline data={selectedHistory.map((p) => ({ t: p.tMs, v: p.v }))} color="#a78bfa" height={60} />
                </div>
              )}
            </>
          ) : (
            <p className="text-sm text-muted-foreground">
              Pick a key from the inventory on the left to inspect its current value and
              recent history.
            </p>
          )}
        </CardContent>
      </Card>
    </div>
  );
}

function inferType(v: string | undefined): string {
  if (v === undefined) return "absent";
  if (v === "true" || v === "false") return "bool";
  if (/^-?\d+$/.test(v)) return "int";
  if (/^-?\d*\.\d+([eE][+-]?\d+)?$/.test(v)) return "f64";
  if (v.startsWith("[") || v.startsWith("{")) return "json";
  return "string";
}

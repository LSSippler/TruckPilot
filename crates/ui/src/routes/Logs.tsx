import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { FixedSizeList, type ListChildComponentProps } from "react-window";
import { save as saveDialog } from "@tauri-apps/plugin-dialog";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { Badge } from "@/components/ui/badge";
import { selectFilteredLogs, useLogsStore, type LogEntry } from "@/stores/logs";
import { usePluginsStore } from "@/stores/plugins";
import { sendCommand } from "@/lib/ipc";
import { useConnectionStore } from "@/stores/connection";
import type { LogLevel } from "@/lib/types";

const LEVELS: LogLevel[] = ["trace", "debug", "info", "warn", "error"];

const LEVEL_BADGE: Record<LogLevel, "secondary" | "default" | "warning" | "destructive"> = {
  trace: "secondary",
  debug: "secondary",
  info: "default",
  warn: "warning",
  error: "destructive",
};

export function Logs() {
  const status = useConnectionStore((s) => s.status);
  const filterLevels = useLogsStore((s) => s.filterLevels);
  const filterPlugin = useLogsStore((s) => s.filterPlugin);
  const search = useLogsStore((s) => s.search);
  const setFilter = useLogsStore((s) => s.setFilter);
  const clear = useLogsStore((s) => s.clear);
  const filtered = useLogsStore(selectFilteredLogs);
  const plugins = usePluginsStore((s) => s.list);
  const containerRef = useRef<HTMLDivElement>(null);
  const dims = useContainerSize(containerRef);

  useEffect(() => {
    if (status === "connected") {
      void sendCommand({
        type: "set_log_subscription",
        levels: filterLevels,
        plugin: filterPlugin,
      });
    }
  }, [status, filterLevels, filterPlugin]);

  const exportLogs = async () => {
    const path = await saveDialog({
      defaultPath: `truckpilot-${new Date().toISOString().slice(0, 10)}.txt`,
      filters: [{ name: "Text", extensions: ["txt"] }],
    });
    if (typeof path !== "string") return;
    const lines = filtered.map(formatLine).join("\n");
    const { writeTextFile } = await import("@tauri-apps/plugin-fs");
    await writeTextFile(path, lines);
  };

  const toggleLevel = (level: LogLevel) => {
    const next = filterLevels.includes(level)
      ? filterLevels.filter((l) => l !== level)
      : [...filterLevels, level];
    setFilter({ filterLevels: next });
  };

  const items = useMemo(() => filtered, [filtered]);

  return (
    <Card className="flex h-full flex-col">
      <CardHeader className="flex flex-row flex-wrap items-end gap-3">
        <div>
          <CardTitle>Logs</CardTitle>
          <p className="text-xs text-muted-foreground">{items.length} entries</p>
        </div>
        <div className="flex flex-1 flex-wrap items-center gap-2">
          <div className="flex items-center gap-1">
            {LEVELS.map((level) => (
              <Button
                key={level}
                size="sm"
                variant={filterLevels.includes(level) ? "default" : "outline"}
                onClick={() => toggleLevel(level)}
                className="capitalize"
              >
                {level}
              </Button>
            ))}
          </div>
          <Select
            value={filterPlugin ?? "__all__"}
            onValueChange={(v) => setFilter({ filterPlugin: v === "__all__" ? null : v })}
          >
            <SelectTrigger className="w-44">
              <SelectValue placeholder="All plugins" />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="__all__">All plugins</SelectItem>
              {plugins.map((p) => (
                <SelectItem key={p.name} value={p.name}>
                  {p.name}
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
          <Input
            value={search}
            onChange={(e) => setFilter({ search: e.target.value })}
            placeholder="Search…"
            className="w-48"
          />
          <Button variant="outline" size="sm" onClick={() => void exportLogs()}>
            Export
          </Button>
          <Button variant="outline" size="sm" onClick={() => clear()}>
            Clear
          </Button>
        </div>
      </CardHeader>
      <CardContent className="flex-1 p-0">
        <div ref={containerRef} className="h-full px-3 pb-3">
          {items.length === 0 ? (
            <p className="p-3 text-sm text-muted-foreground">No log entries match the current filters.</p>
          ) : dims.height === 0 ? null : (
            <FixedSizeList
              height={dims.height}
              width={dims.width}
              itemCount={items.length}
              itemSize={28}
              itemData={items}
              overscanCount={8}
            >
              {LogRow}
            </FixedSizeList>
          )}
        </div>
      </CardContent>
    </Card>
  );
}

const LogRow = ({ index, style, data }: ListChildComponentProps<LogEntry[]>) => {
  const entry = data[index];
  if (!entry) return null;
  return (
    <div style={style} className="flex items-center gap-2 font-mono text-xs">
      <span className="w-20 text-muted-foreground">{new Date(entry.ts).toLocaleTimeString()}</span>
      <Badge variant={LEVEL_BADGE[entry.level]} className="w-12 justify-center capitalize">
        {entry.level}
      </Badge>
      <span className="w-32 truncate text-muted-foreground">{entry.plugin ?? "core"}</span>
      <span className="flex-1 truncate">{entry.message}</span>
    </div>
  );
};

function useContainerSize(ref: React.RefObject<HTMLElement | null>) {
  const dims = useRef({ width: 0, height: 0 });
  const [, force] = useTickState();

  useEffect(() => {
    const node = ref.current;
    if (!node) return;
    const observer = new ResizeObserver((entries) => {
      for (const entry of entries) {
        const { width, height } = entry.contentRect;
        if (width !== dims.current.width || height !== dims.current.height) {
          dims.current = { width, height };
          force();
        }
      }
    });
    observer.observe(node);
    return () => observer.disconnect();
  }, [ref, force]);

  return dims.current;
}

function useTickState(): [number, () => void] {
  const [tick, setTick] = useState(0);
  const force = useCallback(() => setTick((t) => t + 1), []);
  return [tick, force];
}

function formatLine(entry: LogEntry): string {
  return `${new Date(entry.ts).toISOString()} [${entry.level}] (${entry.plugin ?? "core"}) ${entry.message}`;
}

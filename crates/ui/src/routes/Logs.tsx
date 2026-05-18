import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { FixedSizeList, type ListChildComponentProps, type ListOnScrollProps } from "react-window";
import { save as saveDialog } from "@tauri-apps/plugin-dialog";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { filterEntries, useLogsStore, type LogEntry } from "@/stores/logs";
import { usePluginsStore } from "@/stores/plugins";
import { sendCommand } from "@/lib/ipc";
import { useConnectionStore } from "@/stores/connection";
import type { LogLevel } from "@/lib/types";
import { cn } from "@/lib/utils";

const LEVELS: LogLevel[] = ["trace", "debug", "info", "warn", "error"];
const ROW_HEIGHT = 28;
const FOLLOW_THRESHOLD_PX = ROW_HEIGHT * 2;

const LEVEL_ACTIVE_CLASS: Record<LogLevel, string> = {
  trace: "bg-fg-muted text-surface-base",
  debug: "bg-fg-muted text-surface-base",
  info:  "bg-info text-surface-base",
  warn:  "bg-warning text-surface-base",
  error: "bg-danger text-surface-base",
};

const LEVEL_BORDER: Record<LogLevel, string> = {
  trace: "border-l-border-strong",
  debug: "border-l-border-strong",
  info:  "border-l-info",
  warn:  "border-l-warning",
  error: "border-l-danger",
};

const LEVEL_TEXT: Record<LogLevel, string> = {
  trace: "text-fg-muted",
  debug: "text-fg-muted",
  info:  "text-info",
  warn:  "text-warning",
  error: "text-danger",
};

export function Logs() {
  const status = useConnectionStore((s) => s.status);
  // Subscribe to each slice with its own selector. The default Object.is
  // diff is stable for primitives and arrays-as-state-references — we only
  // re-render when the *store* mutates these fields, not on every push.
  const entries = useLogsStore((s) => s.entries);
  const filterLevels = useLogsStore((s) => s.filterLevels);
  const filterPlugin = useLogsStore((s) => s.filterPlugin);
  const search = useLogsStore((s) => s.search);
  const setFilter = useLogsStore((s) => s.setFilter);
  const clear = useLogsStore((s) => s.clear);
  const plugins = usePluginsStore((s) => s.list);
  const containerRef = useRef<HTMLDivElement>(null);
  const listRef = useRef<FixedSizeList<LogEntry[]> | null>(null);
  const dims = useContainerSize(containerRef);
  const [follow, setFollow] = useState(true);
  const followRef = useRef(true);
  followRef.current = follow;

  useEffect(() => {
    if (status === "connected") {
      void sendCommand({
        type: "set_log_subscription",
        levels: filterLevels,
        plugin: filterPlugin,
      });
    }
  }, [status, filterLevels, filterPlugin]);

  // The single O(N) pass per render. With virtualization+memo the only
  // time this re-runs is when entries/filters genuinely change.
  const items = useMemo(
    () => filterEntries(entries, filterLevels, filterPlugin, search),
    [entries, filterLevels, filterPlugin, search],
  );

  // Auto-scroll to bottom on new entries when "following".
  useEffect(() => {
    if (!followRef.current) return;
    if (items.length === 0) return;
    listRef.current?.scrollToItem(items.length - 1, "end");
  }, [items.length]);

  const onScroll = useCallback(
    (props: ListOnScrollProps) => {
      if (props.scrollUpdateWasRequested) return;
      const visible = dims.height;
      const total = items.length * ROW_HEIGHT;
      const atBottom = props.scrollOffset + visible >= total - FOLLOW_THRESHOLD_PX;
      if (atBottom !== followRef.current) {
        setFollow(atBottom);
      }
    },
    [dims.height, items.length],
  );

  const jumpToBottom = () => {
    setFollow(true);
    listRef.current?.scrollToItem(items.length - 1, "end");
  };

  const exportLogs = async () => {
    const path = await saveDialog({
      defaultPath: `truckpilot-${new Date().toISOString().slice(0, 10)}.txt`,
      filters: [{ name: "Text", extensions: ["txt"] }],
    });
    if (typeof path !== "string") return;
    const lines = items.map(formatLine).join("\n");
    const { writeTextFile } = await import("@tauri-apps/plugin-fs");
    await writeTextFile(path, lines);
  };

  const toggleLevel = (level: LogLevel) => {
    const next = filterLevels.includes(level)
      ? filterLevels.filter((l) => l !== level)
      : [...filterLevels, level];
    setFilter({ filterLevels: next });
  };

  return (
    <div className="flex h-full flex-col bg-surface-card border border-subtle rounded-md">
      {/* Filter bar */}
      <div className="flex flex-wrap items-center gap-2 px-4 py-3 border-b border-subtle">
        <div>
          <p className="text-sm font-sans font-medium text-fg">Logs</p>
          <p className="text-[10px] text-fg-muted font-mono">
            {items.length} / {entries.length}{!follow && " · paused"}
          </p>
        </div>
        <div className="flex items-center gap-1 ml-2">
          {LEVELS.map((level) => (
            <button
              key={level}
              type="button"
              onClick={() => toggleLevel(level)}
              className={cn(
                "h-6 px-2 text-[10px] font-mono uppercase rounded-sm border transition-colors",
                filterLevels.includes(level)
                  ? LEVEL_ACTIVE_CLASS[level]
                  : "border-subtle text-fg-muted hover:text-fg hover:bg-surface-elevated",
              )}
            >
              {level}
            </button>
          ))}
        </div>
        <Select
          value={filterPlugin ?? "__all__"}
          onValueChange={(v) => setFilter({ filterPlugin: v === "__all__" ? null : v })}
        >
          <SelectTrigger className="w-40 h-7 text-xs">
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
          className="w-44 h-7 text-xs"
        />
        <div className="flex items-center gap-1 ml-auto">
          {!follow && (
            <Button variant="outline" size="sm" onClick={jumpToBottom} className="h-7 text-xs">
              Tail
            </Button>
          )}
          <Button variant="outline" size="sm" onClick={() => void exportLogs()} className="h-7 text-xs">
            Export
          </Button>
          <Button variant="outline" size="sm" onClick={() => clear()} className="h-7 text-xs">
            Clear
          </Button>
        </div>
      </div>

      {/* Log body */}
      <div className="flex-1 p-0">
        <div ref={containerRef} className="h-full">
          {items.length === 0 ? (
            <p className="p-3 text-xs text-fg-muted font-sans">No log entries match the current filters.</p>
          ) : dims.height === 0 ? null : (
            <FixedSizeList
              ref={listRef}
              height={dims.height}
              width={dims.width}
              itemCount={items.length}
              itemSize={ROW_HEIGHT}
              itemData={items}
              overscanCount={8}
              onScroll={onScroll}
            >
              {LogRow}
            </FixedSizeList>
          )}
        </div>
      </div>
    </div>
  );
}

function fmtLogTs(ts: number) {
  const d = new Date(ts);
  const h = d.getHours().toString().padStart(2, "0");
  const m = d.getMinutes().toString().padStart(2, "0");
  const s = d.getSeconds().toString().padStart(2, "0");
  const ms = d.getMilliseconds().toString().padStart(3, "0");
  return `${h}:${m}:${s}.${ms}`;
}

const LogRow = ({ index, style, data }: ListChildComponentProps<LogEntry[]>) => {
  const entry = data[index];
  if (!entry) return null;
  const lvl = entry.level.toUpperCase() as "DEBUG" | "INFO" | "WARN" | "ERROR";
  const borderClass = LEVEL_BORDER[entry.level] ?? "border-l-border-strong";
  const textClass = LEVEL_TEXT[entry.level] ?? "text-fg-muted";
  return (
    <div
      style={style}
      className={cn(
        "flex items-center gap-3 pl-2 pr-3 border-l-2",
        "hover:bg-surface-elevated transition-colors duration-[var(--dur-1)]",
        borderClass,
      )}
    >
      <span className="font-mono text-[11px] text-fg-muted tabular-nums shrink-0 w-[88px]">
        {fmtLogTs(entry.ts)}
      </span>
      <span className={cn("font-mono text-[10px] uppercase shrink-0 w-12", textClass)}>
        {lvl}
      </span>
      <span className="font-mono text-[11px] text-fg-secondary shrink-0 w-24 truncate">
        {entry.plugin ?? "core"}
      </span>
      <span className="font-mono text-[12px] text-fg truncate flex-1">
        {entry.message}
      </span>
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

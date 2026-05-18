import { Switch } from "@/components/ui/switch";
import { GripVertical } from "lucide-react";
import { cn } from "@/lib/utils";

export type PluginRunState = "running" | "stopped" | "error" | "disabled" | "starting";

export interface PluginInfo {
  id: string;
  name: string;
  version: string;
  state: PluginRunState;
  detail?: string;
}

export interface PluginToggleItemProps {
  plugin: PluginInfo;
  enabled: boolean;
  selected?: boolean;
  draggable?: boolean;
  onToggle?: (next: boolean) => void;
  onSelect?: () => void;
  className?: string;
}

const DOT: Record<PluginRunState, string> = {
  running:  "bg-success",
  stopped:  "bg-fg-muted",
  error:    "bg-danger",
  disabled: "bg-fg-disabled",
  starting: "bg-warning animate-tp-pulse-soft",
};

export function PluginToggleItem({
  plugin,
  enabled,
  selected,
  draggable = true,
  onToggle,
  onSelect,
  className,
}: PluginToggleItemProps) {
  return (
    <div
      role="button"
      tabIndex={0}
      onClick={onSelect}
      onKeyDown={(e) => (e.key === "Enter" || e.key === " ") && onSelect?.()}
      className={cn(
        "group flex items-center gap-3 px-3 h-11 rounded-sm",
        "border border-transparent",
        "transition-colors duration-[var(--dur-1)]",
        "hover:bg-surface-elevated",
        selected && "bg-brand-soft border-subtle",
        className,
      )}
    >
      {draggable && (
        <GripVertical
          size={14}
          className="text-fg-muted opacity-0 group-hover:opacity-100 transition-opacity cursor-grab shrink-0"
          aria-hidden
        />
      )}

      <span
        className={cn("w-1.5 h-1.5 rounded-full shrink-0", DOT[plugin.state])}
        aria-label={`status: ${plugin.state}`}
      />

      <div className="flex-1 min-w-0">
        <div className="flex items-baseline gap-2">
          <span className="text-fg text-sm font-sans truncate">{plugin.name}</span>
          <span className="font-mono text-[10px] text-fg-muted shrink-0">
            v{plugin.version}
          </span>
        </div>
        {plugin.detail && (
          <p className="text-fg-muted text-xs font-sans truncate">{plugin.detail}</p>
        )}
      </div>

      <Switch
        checked={enabled}
        onCheckedChange={onToggle}
        onClick={(e) => e.stopPropagation()}
        aria-label={`enable ${plugin.name}`}
      />
    </div>
  );
}

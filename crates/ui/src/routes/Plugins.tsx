import { useEffect, useMemo } from "react";
import {
  DndContext,
  KeyboardSensor,
  PointerSensor,
  closestCenter,
  useSensor,
  useSensors,
  type DragEndEvent,
} from "@dnd-kit/core";
import { SortableContext, useSortable, verticalListSortingStrategy } from "@dnd-kit/sortable";
import { CSS } from "@dnd-kit/utilities";
import { RefreshCw } from "lucide-react";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Separator } from "@/components/ui/separator";
import { sendCommand } from "@/lib/ipc";
import { usePluginsStore } from "@/stores/plugins";
import { useConnectionStore } from "@/stores/connection";
import { SchemaForm } from "@/components/schema-form/SchemaForm";
import { PluginToggleItem, type PluginInfo, type PluginRunState } from "@/components/PluginToggleItem";

function toPluginInfo(p: { name: string; version: string; enabled: boolean; state?: string }): PluginInfo {
  const stateMap: Record<string, PluginRunState> = {
    running: "running",
    stopped: "stopped",
    error: "error",
    disabled: "disabled",
    starting: "starting",
  };
  return {
    id: p.name,
    name: p.name,
    version: p.version,
    state: p.enabled
      ? (stateMap[p.state ?? ""] ?? "running")
      : "stopped",
  };
}

export function Plugins() {
  const status = useConnectionStore((s) => s.status);
  const list = usePluginsStore((s) => s.list);
  const order = usePluginsStore((s) => s.order);
  const selected = usePluginsStore((s) => s.selected);
  const select = usePluginsStore((s) => s.select);
  const reorder = usePluginsStore((s) => s.reorder);
  const schemas = usePluginsStore((s) => s.schemas);

  useEffect(() => {
    if (status === "connected") void sendCommand({ type: "request_plugin_list" });
  }, [status]);

  useEffect(() => {
    if (selected && !(selected in schemas)) {
      void sendCommand({ type: "request_plugin_schema", plugin: selected });
    }
  }, [selected, schemas]);

  const ordered = useMemo(
    () => order.map((name) => list.find((p) => p.name === name)).filter((p): p is (typeof list)[number] => Boolean(p)),
    [order, list]
  );

  const sensors = useSensors(
    useSensor(PointerSensor, { activationConstraint: { distance: 8 } }),
    useSensor(KeyboardSensor),
  );

  const onDragEnd = (event: DragEndEvent) => {
    const { active, over } = event;
    if (!over || active.id === over.id) return;
    const from = order.indexOf(String(active.id));
    const to = order.indexOf(String(over.id));
    if (from === -1 || to === -1) return;
    reorder(from, to);
  };

  const selectedPlugin = selected ? list.find((p) => p.name === selected) : null;
  const selectedSchema = selected ? schemas[selected] : null;

  return (
    <div className="grid h-full gap-4 lg:grid-cols-[minmax(0,2fr)_minmax(0,3fr)]">
      {/* Plugin list */}
      <div className="bg-surface-card border border-subtle rounded-md flex flex-col">
        <div className="px-4 py-3 border-b border-subtle">
          <h2 className="text-sm font-sans font-medium text-fg">Plugins</h2>
        </div>
        <div className="flex-1 overflow-y-auto p-2">
          {status !== "connected" ? (
            <p className="text-xs text-fg-muted font-sans px-2 py-2">Not connected.</p>
          ) : ordered.length === 0 ? (
            <p className="text-xs text-fg-muted font-sans px-2 py-2">No plugins reported by the core.</p>
          ) : (
            <DndContext sensors={sensors} collisionDetection={closestCenter} onDragEnd={onDragEnd}>
              <SortableContext items={ordered.map((p) => p.name)} strategy={verticalListSortingStrategy}>
                <div className="space-y-0.5">
                  {ordered.map((plugin) => (
                    <SortablePluginItem
                      key={plugin.name}
                      plugin={plugin}
                      selected={selected === plugin.name}
                      onSelect={() => select(plugin.name)}
                    />
                  ))}
                </div>
              </SortableContext>
            </DndContext>
          )}
        </div>
      </div>

      {/* Detail panel */}
      <div className="bg-surface-card border border-subtle rounded-md flex flex-col">
        <div className="px-4 py-3 border-b border-subtle flex items-center gap-2">
          <h2 className="text-sm font-sans font-medium text-fg flex-1">
            {selectedPlugin ? selectedPlugin.name : "Settings"}
          </h2>
          {selectedPlugin && (
            <Button
              size="icon"
              variant="ghost"
              aria-label={`Reload ${selectedPlugin.name}`}
              onClick={() => void sendCommand({ type: "plugin_reload", name: selectedPlugin.name })}
              className="h-7 w-7"
            >
              <RefreshCw size={13} />
            </Button>
          )}
        </div>
        <div className="flex-1 overflow-y-auto p-4">
          {!selectedPlugin ? (
            <p className="text-xs text-fg-muted font-sans">Select a plugin to view its settings.</p>
          ) : !selectedSchema ? (
            <p className="text-xs text-fg-muted font-sans">Loading schema…</p>
          ) : (
            <>
              <div className="mb-3 flex items-center gap-2 text-xs">
                <Badge variant="outline" className="border-subtle text-fg-muted">v{selectedPlugin.version}</Badge>
                <Badge
                  variant="secondary"
                  className={
                    selectedPlugin.enabled
                      ? "bg-success-soft text-success border-0"
                      : "bg-surface-elevated text-fg-muted border-0"
                  }
                >
                  {selectedPlugin.enabled ? "enabled" : "disabled"}
                </Badge>
              </div>
              <Separator className="mb-4 bg-border-subtle" />
              <SchemaForm
                schema={selectedSchema}
                onSubmit={(values) =>
                  sendCommand({
                    type: "plugin_settings_update",
                    plugin: selectedPlugin.name,
                    settings: values,
                  })
                }
              />
            </>
          )}
        </div>
      </div>
    </div>
  );
}

function SortablePluginItem({
  plugin,
  selected,
  onSelect,
}: {
  plugin: { name: string; version: string; enabled: boolean };
  selected: boolean;
  onSelect: () => void;
}) {
  const { attributes, listeners, setNodeRef, transform, transition, isDragging } = useSortable({
    id: plugin.name,
  });
  const style = {
    transform: CSS.Transform.toString(transform),
    transition,
    opacity: isDragging ? 0.6 : 1,
  };

  const onToggle = (enabled: boolean) =>
    sendCommand({ type: "plugin_toggle", name: plugin.name, enabled });

  return (
    <div ref={setNodeRef} style={style} {...attributes} {...listeners}>
      <PluginToggleItem
        plugin={toPluginInfo(plugin)}
        enabled={plugin.enabled}
        selected={selected}
        onToggle={(v) => void onToggle(v)}
        onSelect={onSelect}
        draggable
      />
    </div>
  );
}

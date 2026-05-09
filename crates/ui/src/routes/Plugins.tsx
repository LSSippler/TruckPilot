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
import { GripVertical, RefreshCw } from "lucide-react";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Switch } from "@/components/ui/switch";
import { Badge } from "@/components/ui/badge";
import { Separator } from "@/components/ui/separator";
import { sendCommand } from "@/lib/ipc";
import { usePluginsStore } from "@/stores/plugins";
import { useConnectionStore } from "@/stores/connection";
import { SchemaForm } from "@/components/schema-form/SchemaForm";

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

  const sensors = useSensors(useSensor(PointerSensor), useSensor(KeyboardSensor));

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
      <Card>
        <CardHeader>
          <CardTitle>Plugins</CardTitle>
        </CardHeader>
        <CardContent>
          {status !== "connected" ? (
            <p className="text-sm text-muted-foreground">Not connected.</p>
          ) : ordered.length === 0 ? (
            <p className="text-sm text-muted-foreground">No plugins reported by the core.</p>
          ) : (
            <DndContext sensors={sensors} collisionDetection={closestCenter} onDragEnd={onDragEnd}>
              <SortableContext items={ordered.map((p) => p.name)} strategy={verticalListSortingStrategy}>
                <ul className="space-y-1">
                  {ordered.map((plugin) => (
                    <PluginRow
                      key={plugin.name}
                      plugin={plugin}
                      active={selected === plugin.name}
                      onSelect={() => select(plugin.name)}
                    />
                  ))}
                </ul>
              </SortableContext>
            </DndContext>
          )}
        </CardContent>
      </Card>

      <Card>
        <CardHeader>
          <CardTitle>{selectedPlugin ? selectedPlugin.name : "Settings"}</CardTitle>
        </CardHeader>
        <CardContent>
          {!selectedPlugin ? (
            <p className="text-sm text-muted-foreground">Select a plugin to view its settings.</p>
          ) : !selectedSchema ? (
            <p className="text-sm text-muted-foreground">Loading schema…</p>
          ) : (
            <>
              <div className="mb-3 flex items-center gap-2 text-xs text-muted-foreground">
                <Badge variant="outline">v{selectedPlugin.version}</Badge>
                <Badge variant={selectedPlugin.enabled ? "success" : "secondary"}>
                  {selectedPlugin.enabled ? "enabled" : "disabled"}
                </Badge>
              </div>
              <Separator className="mb-4" />
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
        </CardContent>
      </Card>
    </div>
  );
}

function PluginRow({
  plugin,
  active,
  onSelect,
}: {
  plugin: { name: string; version: string; enabled: boolean };
  active: boolean;
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
  const onReload = () => sendCommand({ type: "plugin_reload", name: plugin.name });

  return (
    <li
      ref={setNodeRef}
      style={style}
      className={`flex items-center gap-2 rounded-md border bg-card p-2 ${active ? "border-primary" : ""}`}
    >
      <button
        type="button"
        className="cursor-grab rounded p-1 text-muted-foreground hover:text-foreground"
        aria-label="Drag to reorder"
        {...attributes}
        {...listeners}
      >
        <GripVertical className="size-4" />
      </button>
      <button type="button" className="flex-1 text-left" onClick={onSelect}>
        <div className="text-sm font-medium">{plugin.name}</div>
        <div className="text-xs text-muted-foreground">v{plugin.version}</div>
      </button>
      <Switch checked={plugin.enabled} onCheckedChange={(v) => void onToggle(Boolean(v))} />
      <Button size="icon" variant="ghost" aria-label={`Reload ${plugin.name}`} onClick={() => void onReload()}>
        <RefreshCw className="size-4" />
      </Button>
    </li>
  );
}

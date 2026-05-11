import { useEffect } from "react";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Switch } from "@/components/ui/switch";
import { Badge } from "@/components/ui/badge";
import { Progress } from "@/components/ui/progress";
import {
  AlertDialog,
  AlertDialogAction,
  AlertDialogCancel,
  AlertDialogContent,
  AlertDialogDescription,
  AlertDialogFooter,
  AlertDialogHeader,
  AlertDialogTitle,
  AlertDialogTrigger,
} from "@/components/ui/alert-dialog";
import { useModsStore } from "@/stores/mods";
import { useConnectionStore } from "@/stores/connection";
import { sendCommand } from "@/lib/ipc";

export function ModManager() {
  const status = useConnectionStore((s) => s.status);
  const list = useModsStore((s) => s.list);
  const active = useModsStore((s) => s.active);
  const progress = useModsStore((s) => s.progress);
  const result = useModsStore((s) => s.result);
  const setActive = useModsStore((s) => s.setActive);

  useEffect(() => {
    if (status === "connected") void sendCommand({ type: "request_mod_list" });
  }, [status]);

  const toggle = (name: string, on: boolean) =>
    setActive(on ? [...active, name] : active.filter((n) => n !== name));

  const apply = () => sendCommand({ type: "mod_apply", active_mods: active });
  const clearCache = () => sendCommand({ type: "cache_clear" });

  return (
    <div className="grid gap-4 lg:grid-cols-[minmax(0,3fr)_minmax(0,2fr)]">
      <Card>
        <CardHeader className="flex flex-row items-center justify-between gap-4">
          <CardTitle>Mods</CardTitle>
          <Button onClick={() => void apply()} disabled={status !== "connected" || progress !== null}>
            Apply
          </Button>
        </CardHeader>
        <CardContent>
          {list.length === 0 ? (
            <p className="text-sm text-muted-foreground">No mods detected.</p>
          ) : (
            <ul className="space-y-1">
              {list.map((mod) => (
                <li key={mod.path} className="flex items-center justify-between rounded-md border p-2">
                  <div>
                    <div className="text-sm font-medium">{mod.name}</div>
                    <div className="text-xs text-muted-foreground">{mod.path}</div>
                  </div>
                  <Switch
                    checked={active.includes(mod.name)}
                    onCheckedChange={(v) => toggle(mod.name, Boolean(v))}
                  />
                </li>
              ))}
            </ul>
          )}
        </CardContent>
      </Card>

      <div className="space-y-4">
        <Card>
          <CardHeader>
            <CardTitle>Build status</CardTitle>
          </CardHeader>
          <CardContent>
            {progress ? (
              <div className="space-y-2">
                <div className="flex items-center justify-between text-sm">
                  <span className="font-medium">{progress.phase}</span>
                  <span className="text-muted-foreground">{progress.percent.toFixed(0)}%</span>
                </div>
                <Progress value={progress.percent} />
                {progress.etaSeconds != null ? (
                  <p className="text-xs text-muted-foreground">~{progress.etaSeconds}s remaining</p>
                ) : null}
              </div>
            ) : result ? (
              <div className="space-y-2 text-sm">
                <Badge variant={result.ok ? "success" : "destructive"}>
                  {result.ok ? "OK" : "Failed"}
                </Badge>
                <p className="text-muted-foreground">{result.message}</p>
                {result.fromCache ? (
                  <p className="text-xs text-muted-foreground">Loaded from cache</p>
                ) : null}
              </div>
            ) : (
              <p className="text-sm text-muted-foreground">No build in progress.</p>
            )}
          </CardContent>
        </Card>

        <Card>
          <CardHeader>
            <CardTitle>Cache</CardTitle>
          </CardHeader>
          <CardContent className="space-y-3">
            <p className="text-sm text-muted-foreground">
              Clearing the cache forces a full rebuild on next apply.
            </p>
            <AlertDialog>
              <AlertDialogTrigger asChild>
                <Button variant="destructive" size="sm">
                  Clear cache
                </Button>
              </AlertDialogTrigger>
              <AlertDialogContent>
                <AlertDialogHeader>
                  <AlertDialogTitle>Clear graph cache?</AlertDialogTitle>
                  <AlertDialogDescription>
                    This deletes the cached routing graph. The next mod-apply will rebuild from scratch (this can take several minutes).
                  </AlertDialogDescription>
                </AlertDialogHeader>
                <AlertDialogFooter>
                  <AlertDialogCancel>Cancel</AlertDialogCancel>
                  <AlertDialogAction onClick={() => void clearCache()}>Clear</AlertDialogAction>
                </AlertDialogFooter>
              </AlertDialogContent>
            </AlertDialog>
          </CardContent>
        </Card>
      </div>
    </div>
  );
}

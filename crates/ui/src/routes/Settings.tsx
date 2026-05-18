import { useEffect, useState } from "react";
import { open as openDialog } from "@tauri-apps/plugin-dialog";
import { toast } from "sonner";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Switch } from "@/components/ui/switch";
import { Label } from "@/components/ui/label";
import { Input } from "@/components/ui/input";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { useSettingsStore } from "@/stores/settings";
import {
  detectEts2Path,
  daemonGetAutoStart,
  daemonSetAutoStart,
  daemonStatus,
  daemonStart,
  daemonStop,
  daemonRestart,
  type DaemonStatus,
} from "@/lib/tauri-bridge";

function DaemonCard() {
  const [status, setStatus] = useState<DaemonStatus | null>(null);
  const [autoStart, setAutoStart] = useState<boolean>(true);
  const [busy, setBusy] = useState(false);

  const refresh = async () => {
    try {
      setStatus(await daemonStatus());
    } catch (err) {
      toast.error("Daemon status failed", { description: String(err) });
    }
  };

  useEffect(() => {
    void daemonGetAutoStart().then(setAutoStart).catch(() => undefined);
    void refresh();
    const id = setInterval(() => void refresh(), 2000);
    return () => clearInterval(id);
  }, []);

  const toggleAuto = async (v: boolean) => {
    setAutoStart(v);
    try {
      await daemonSetAutoStart(v);
    } catch (err) {
      toast.error("Persist failed", { description: String(err) });
    }
  };

  const wrap = (fn: () => Promise<DaemonStatus>, label: string) => async () => {
    setBusy(true);
    try {
      setStatus(await fn());
    } catch (err) {
      toast.error(`${label} failed`, { description: String(err) });
    } finally {
      setBusy(false);
    }
  };

  const dot = (() => {
    switch (status?.state) {
      case "runningmanaged":
        return { color: "bg-success", label: "Running (managed)" };
      case "runningexternal":
        return { color: "bg-info", label: "Running (external)" };
      case "crashed":
        return { color: "bg-danger", label: "Crashed" };
      case "stopped":
      default:
        return { color: "bg-fg-muted", label: "Stopped" };
    }
  })();

  return (
    <Card>
      <CardHeader>
        <CardTitle>Daemon</CardTitle>
      </CardHeader>
      <CardContent className="space-y-3">
        <div className="flex items-center justify-between">
          <div>
            <Label className="text-sm">Auto-start daemon on launch</Label>
            <p className="text-xs text-muted-foreground">
              Spawn truckpilot-core when the UI starts.
            </p>
          </div>
          <Switch checked={autoStart} onCheckedChange={(v) => void toggleAuto(v)} />
        </div>

        <div className="flex items-center gap-2 text-sm">
          <span className={`h-2.5 w-2.5 rounded-full ${dot.color}`} />
          <span>{dot.label}</span>
          {status?.pid != null && (
            <span className="text-xs text-muted-foreground">pid {status.pid}</span>
          )}
        </div>

        {status?.binary_path && (
          <p className="text-xs text-muted-foreground break-all">{status.binary_path}</p>
        )}
        {status?.last_error && (
          <p className="text-xs text-red-500 break-all">{status.last_error}</p>
        )}

        <div className="flex gap-2">
          <Button
            variant="outline"
            size="sm"
            disabled={busy || status?.state === "runningmanaged" || status?.state === "runningexternal"}
            onClick={() => void wrap(daemonStart, "Start")()}
          >
            Start
          </Button>
          <Button
            variant="outline"
            size="sm"
            disabled={busy || status?.state !== "runningmanaged"}
            onClick={() => void wrap(daemonStop, "Stop")()}
          >
            Stop
          </Button>
          <Button
            variant="outline"
            size="sm"
            disabled={busy}
            onClick={() => void wrap(daemonRestart, "Restart")()}
          >
            Restart
          </Button>
        </div>
      </CardContent>
    </Card>
  );
}

export function Settings() {
  const settings = useSettingsStore();
  const [detecting, setDetecting] = useState(false);

  const browseEts2 = async () => {
    const picked = await openDialog({ directory: true, multiple: false });
    if (typeof picked === "string") settings.setEts2Path(picked);
  };

  const browseCache = async () => {
    const picked = await openDialog({ directory: true, multiple: false });
    if (typeof picked === "string") settings.setCacheDir(picked);
  };

  const autoDetect = async () => {
    setDetecting(true);
    try {
      const path = await detectEts2Path();
      if (path) {
        settings.setEts2Path(path);
        toast.success("ETS2 found", { description: path });
      } else {
        toast.warning("ETS2 not found", { description: "No Steam library contains app 227300." });
      }
    } catch (err) {
      toast.error("Detection failed", { description: String(err) });
    } finally {
      setDetecting(false);
    }
  };

  return (
    <div className="grid gap-4 lg:grid-cols-2">
      <Card>
        <CardHeader>
          <CardTitle>Appearance</CardTitle>
        </CardHeader>
        <CardContent className="space-y-3">
          <div className="space-y-1.5">
            <Label className="text-sm">Theme</Label>
            <Select value={settings.theme} onValueChange={(v) => settings.setTheme(v as typeof settings.theme)}>
              <SelectTrigger>
                <SelectValue />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="system">System</SelectItem>
                <SelectItem value="light">Light</SelectItem>
                <SelectItem value="dark">Dark</SelectItem>
              </SelectContent>
            </Select>
          </div>
        </CardContent>
      </Card>

      <DaemonCard />

      <Card>
        <CardHeader>
          <CardTitle>Connection</CardTitle>
        </CardHeader>
        <CardContent className="space-y-3">
          <div className="flex items-center justify-between">
            <div>
              <Label className="text-sm">Auto-connect on startup</Label>
              <p className="text-xs text-muted-foreground">Reconnect to the core daemon automatically.</p>
            </div>
            <Switch checked={settings.autoConnect} onCheckedChange={settings.setAutoConnect} />
          </div>
        </CardContent>
      </Card>

      <Card>
        <CardHeader>
          <CardTitle>Hotkeys</CardTitle>
        </CardHeader>
        <CardContent className="space-y-3">
          <div className="grid grid-cols-2 gap-3">
            <div className="space-y-1.5">
              <Label className="text-sm">Engage autopilot</Label>
              <Input
                value={settings.hotkeyEngage}
                onChange={(e) => settings.setHotkeyEngage(e.target.value)}
                placeholder="F5"
              />
            </div>
            <div className="space-y-1.5">
              <Label className="text-sm">Disengage autopilot</Label>
              <Input
                value={settings.hotkeyDisengage}
                onChange={(e) => settings.setHotkeyDisengage(e.target.value)}
                placeholder="F6"
              />
            </div>
          </div>
        </CardContent>
      </Card>

      <Card>
        <CardHeader>
          <CardTitle>IPC</CardTitle>
        </CardHeader>
        <CardContent className="space-y-2 text-xs">
          <div className="flex justify-between">
            <span className="text-muted-foreground">WebSocket port</span>
            <span className="font-mono">127.0.0.1:8765</span>
          </div>
          <p className="text-muted-foreground">
            Port is hard-coded in the daemon (Phase 6.x). Change in
            <code className="mx-1 font-mono">crates/core/src/ipc.rs</code>
            and recompile.
          </p>
        </CardContent>
      </Card>

      <Card>
        <CardHeader>
          <CardTitle>Paths</CardTitle>
        </CardHeader>
        <CardContent className="space-y-3">
          <div className="space-y-1.5">
            <Label className="text-sm">ETS2 install directory</Label>
            <div className="flex gap-2">
              <Input
                value={settings.ets2Path ?? ""}
                onChange={(e) => settings.setEts2Path(e.target.value || null)}
                placeholder="C:\\…\\Euro Truck Simulator 2"
              />
              <Button variant="outline" onClick={() => void browseEts2()}>
                Browse
              </Button>
              <Button variant="outline" onClick={() => void autoDetect()} disabled={detecting}>
                {detecting ? "Detecting…" : "Auto-detect"}
              </Button>
            </div>
          </div>
          <div className="space-y-1.5">
            <Label className="text-sm">Cache directory</Label>
            <div className="flex gap-2">
              <Input
                value={settings.cacheDir ?? ""}
                onChange={(e) => settings.setCacheDir(e.target.value || null)}
                placeholder="(default: %LOCALAPPDATA%/TruckPilot/cache)"
              />
              <Button variant="outline" onClick={() => void browseCache()}>
                Browse
              </Button>
            </div>
          </div>
        </CardContent>
      </Card>
    </div>
  );
}

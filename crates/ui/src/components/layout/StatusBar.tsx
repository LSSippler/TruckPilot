import { CircleDot } from "lucide-react";
import { useConnectionStore } from "@/stores/connection";
import { reconnect } from "@/lib/tauri-bridge";
import { Button } from "@/components/ui/button";
import { cn } from "@/lib/utils";

const STATUS_COLOR: Record<string, string> = {
  connected: "text-emerald-500",
  reconnecting: "text-amber-500",
  disconnected: "text-rose-500",
};

const STATUS_LABEL: Record<string, string> = {
  connected: "Connected",
  reconnecting: "Reconnecting…",
  disconnected: "Disconnected",
};

export function StatusBar() {
  const status = useConnectionStore((s) => s.status);
  const protocolVersion = useConnectionStore((s) => s.protocolVersion);
  const lastError = useConnectionStore((s) => s.lastError);

  return (
    <footer className="flex h-9 items-center justify-between border-t bg-card px-4 text-xs">
      <div className="flex items-center gap-2">
        <CircleDot className={cn("size-3", STATUS_COLOR[status] ?? "text-muted-foreground")} />
        <span className="font-medium">
          {STATUS_LABEL[status] ?? status}
          {protocolVersion ? ` • core v${protocolVersion}` : ""}
        </span>
        {lastError ? <span className="text-muted-foreground">— {lastError}</span> : null}
      </div>
      {status !== "connected" ? (
        <Button size="sm" variant="ghost" className="h-7 px-2" onClick={() => void reconnect()}>
          Retry now
        </Button>
      ) : (
        <span className="text-muted-foreground">ws://localhost:8765</span>
      )}
    </footer>
  );
}

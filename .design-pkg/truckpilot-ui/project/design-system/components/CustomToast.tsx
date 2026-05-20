// crates/ui/src/components/CustomToast.tsx
//
// Mount once in your root layout:
//   import { Toaster } from "sonner";
//   <Toaster position="bottom-right" theme="dark" toastOptions={{ duration: 1600 }} />
//
// Then use:
//   import { notify } from "@/components/CustomToast";
//   notify.engaged({ atKmh: 0 });
//   notify.failsafe({ reason: "Steering timeout" });

import * as React from "react";
import { toast, type ExternalToast } from "sonner";
import {
  CheckCircle2,
  AlertOctagon,
  AlertTriangle,
  Info,
  Power,
  Navigation,
  type LucideIcon,
} from "lucide-react";
import { cn } from "@/lib/utils";

type Severity = "success" | "info" | "warning" | "danger";

const ICON: Record<Severity, LucideIcon> = {
  success: CheckCircle2,
  info:    Info,
  warning: AlertTriangle,
  danger:  AlertOctagon,
};

const ACCENT: Record<Severity, string> = {
  success: "text-success",
  info:    "text-info",
  warning: "text-warning",
  danger:  "text-danger",
};

const BORDER: Record<Severity, string> = {
  success: "border-l-success",
  info:    "border-l-info",
  warning: "border-l-warning",
  danger:  "border-l-danger",
};

interface BaseProps {
  severity: Severity;
  title: string;
  body?: string;
  icon?: LucideIcon;
}

function ToastBody({ severity, title, body, icon }: BaseProps) {
  const Icon = icon ?? ICON[severity];
  return (
    <div
      className={cn(
        "w-[320px] max-w-[90vw] flex items-start gap-2.5 pl-3 pr-3 py-2.5",
        "bg-surface-overlay border border-subtle rounded-sm",
        "border-l-2", BORDER[severity],
      )}
    >
      <Icon size={14} className={cn("mt-0.5 shrink-0", ACCENT[severity])} />
      <div className="flex-1 min-w-0">
        <p className="text-fg text-sm font-sans truncate">{title}</p>
        {body && <p className="text-fg-muted text-xs font-sans mt-0.5">{body}</p>}
      </div>
    </div>
  );
}

function emit(props: BaseProps, options?: ExternalToast) {
  return toast.custom(() => <ToastBody {...props} />, options);
}

export const notify = {
  engaged: ({ atKmh }: { atKmh: number }) =>
    emit({
      severity: "info",
      icon: Power,
      title: "Engaged",
      body: `at ${Math.round(atKmh)} km/h`,
    }, { duration: 1600 }),

  disengaged: () =>
    emit({ severity: "info", icon: Power, title: "Disengaged" }, { duration: 1200 }),

  failsafe: ({ reason }: { reason: string }) =>
    emit({
      severity: "danger",
      title: "Failsafe",
      body: reason,
    }, { duration: Infinity }),

  pluginError: ({ plugin, message }: { plugin: string; message: string }) =>
    emit({
      severity: "warning",
      title: `${plugin} — error`,
      body: message,
    }, { duration: 4000 }),

  naviAcquired: ({ to }: { to: string }) =>
    emit({
      severity: "info",
      icon: Navigation,
      title: "Following ETS2 route",
      body: `to ${to}`,
    }, { duration: 1600 }),
};

export default notify;

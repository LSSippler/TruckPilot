import * as React from "react";
import { cn } from "@/lib/utils";

export interface NavGroupProps {
  label: string;
  children: React.ReactNode;
  className?: string;
}

export function NavGroup({ label, children, className }: NavGroupProps) {
  return (
    <div className={cn("flex flex-col gap-0.5", className)}>
      <p className="px-2 mb-1 text-xs uppercase tracking-[0.08em] text-fg-muted font-sans font-medium">
        {label}
      </p>
      {children}
    </div>
  );
}

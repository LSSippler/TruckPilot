import { clsx, type ClassValue } from "clsx";
import { twMerge } from "tailwind-merge";

export function cn(...inputs: ClassValue[]) {
  return twMerge(clsx(inputs));
}

/// SINGLE source of truth for the daemon's m/s → km/h conversion.
/// The daemon emits speeds in m/s (TelemetrySnapshot.speed_ms); every km/h
/// value shown in the UI/overlay goes through here. All `*_kmh` blackboard
/// keys are already km/h and must NOT be passed through this.
export function msToKmh(ms: number): number {
  return ms * 3.6;
}

export function formatSpeed(ms: number): string {
  return `${msToKmh(ms).toFixed(1)} km/h`;
}

export function formatHeading(rad: number): string {
  const deg = ((rad * 180) / Math.PI + 360) % 360;
  return `${deg.toFixed(0)}°`;
}

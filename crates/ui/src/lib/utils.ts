import { clsx, type ClassValue } from "clsx";
import { twMerge } from "tailwind-merge";

export function cn(...inputs: ClassValue[]) {
  return twMerge(clsx(inputs));
}

export function formatSpeed(ms: number): string {
  return `${(ms * 3.6).toFixed(1)} km/h`;
}

export function formatHeading(rad: number): string {
  const deg = ((rad * 180) / Math.PI + 360) % 360;
  return `${deg.toFixed(0)}°`;
}

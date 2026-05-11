interface SpeedGaugeProps {
  speedMs: number;
  navLimitKmh: number;
  maxKmh?: number;
}

const ARC_RADIUS = 70;
const ARC_CENTER = 90;
const ARC_START = -210;
const ARC_END = 30;

function describeArc(startDeg: number, endDeg: number) {
  const start = polar(startDeg);
  const end = polar(endDeg);
  const largeArc = endDeg - startDeg > 180 ? 1 : 0;
  return `M ${start.x} ${start.y} A ${ARC_RADIUS} ${ARC_RADIUS} 0 ${largeArc} 1 ${end.x} ${end.y}`;
}

function polar(deg: number) {
  const rad = (deg * Math.PI) / 180;
  return { x: ARC_CENTER + ARC_RADIUS * Math.cos(rad), y: ARC_CENTER + ARC_RADIUS * Math.sin(rad) };
}

export function SpeedGauge({ speedMs, navLimitKmh, maxKmh = 130 }: SpeedGaugeProps) {
  const speedKmh = speedMs * 3.6;
  const ratio = Math.max(0, Math.min(1, speedKmh / maxKmh));
  const arcDeg = ARC_START + ratio * (ARC_END - ARC_START);
  const limitRatio = Math.max(0, Math.min(1, navLimitKmh / maxKmh));
  const limitDeg = ARC_START + limitRatio * (ARC_END - ARC_START);
  const limitPos = polar(limitDeg);

  return (
    <svg viewBox="0 0 180 180" className="h-44 w-44" role="img" aria-label="Speedometer">
      <path d={describeArc(ARC_START, ARC_END)} fill="none" stroke="var(--muted)" strokeWidth={10} strokeLinecap="round" />
      <path d={describeArc(ARC_START, arcDeg)} fill="none" stroke="var(--primary)" strokeWidth={10} strokeLinecap="round" />
      {navLimitKmh > 0 ? (
        <circle cx={limitPos.x} cy={limitPos.y} r={5} fill="var(--chart-4)" />
      ) : null}
      <text x={ARC_CENTER} y={ARC_CENTER + 5} textAnchor="middle" className="fill-foreground" fontSize={28} fontWeight={600}>
        {speedKmh.toFixed(0)}
      </text>
      <text x={ARC_CENTER} y={ARC_CENTER + 28} textAnchor="middle" className="fill-muted-foreground" fontSize={10}>
        km/h
      </text>
    </svg>
  );
}

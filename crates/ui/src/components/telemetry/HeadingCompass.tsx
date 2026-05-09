interface HeadingCompassProps {
  headingRad: number;
}

export function HeadingCompass({ headingRad }: HeadingCompassProps) {
  const deg = ((headingRad * 180) / Math.PI + 360) % 360;
  return (
    <svg viewBox="0 0 180 180" className="h-44 w-44" role="img" aria-label="Heading compass">
      <circle cx={90} cy={90} r={75} fill="none" stroke="var(--border)" strokeWidth={2} />
      <g transform={`rotate(${-deg} 90 90)`}>
        <text x={90} y={25} textAnchor="middle" className="fill-foreground" fontSize={14} fontWeight={600}>
          N
        </text>
        <text x={155} y={95} textAnchor="middle" className="fill-muted-foreground" fontSize={11}>
          E
        </text>
        <text x={90} y={165} textAnchor="middle" className="fill-muted-foreground" fontSize={11}>
          S
        </text>
        <text x={25} y={95} textAnchor="middle" className="fill-muted-foreground" fontSize={11}>
          W
        </text>
      </g>
      <polygon points="90,32 84,90 96,90" fill="var(--primary)" />
      <polygon points="90,148 84,90 96,90" fill="var(--muted-foreground)" />
      <circle cx={90} cy={90} r={6} fill="var(--background)" stroke="var(--border)" strokeWidth={1.5} />
      <text x={90} y={94} textAnchor="middle" className="fill-foreground" fontSize={9} fontWeight={600}>
        {deg.toFixed(0)}°
      </text>
    </svg>
  );
}

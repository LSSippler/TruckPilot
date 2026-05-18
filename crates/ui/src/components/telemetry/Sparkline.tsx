import { LineChart, Line, ResponsiveContainer, YAxis } from "recharts";

export interface SparklinePoint {
  t: number;
  v: number;
}

export function Sparkline({
  data,
  color = "currentColor",
  height = 32,
  domain,
}: {
  data: SparklinePoint[];
  color?: string;
  height?: number;
  domain?: [number, number];
}) {
  if (data.length === 0) {
    return <div style={{ height }} className="text-[10px] text-muted-foreground">no data</div>;
  }
  return (
    <ResponsiveContainer width="100%" height={height}>
      <LineChart data={data} margin={{ top: 2, right: 0, bottom: 2, left: 0 }}>
        <YAxis hide domain={domain ?? ["auto", "auto"]} />
        <Line type="monotone" dataKey="v" stroke={color} strokeWidth={1.5} dot={false} isAnimationActive={false} />
      </LineChart>
    </ResponsiveContainer>
  );
}

import { useEffect, useMemo, useState } from "react";
import { CartesianGrid, Line, LineChart, ResponsiveContainer, Tooltip as ChartTooltip, XAxis, YAxis } from "recharts";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { Button } from "@/components/ui/button";
import { Slider } from "@/components/ui/slider";
import { Label } from "@/components/ui/label";
import { sendCommand } from "@/lib/ipc";
import { usePidStore } from "@/stores/pid";
import { useConnectionStore } from "@/stores/connection";
import type { PidProfile } from "@/lib/types";

export function PidTuning() {
  const status = useConnectionStore((s) => s.status);
  const profiles = usePidStore((s) => s.profiles);
  const samples = usePidStore((s) => s.samples);
  const [active, setActive] = useState<string | null>(null);

  useEffect(() => {
    if (status === "connected") void sendCommand({ type: "request_pid_profiles" });
  }, [status]);

  useEffect(() => {
    if (!active && profiles[0]) setActive(profiles[0].name);
  }, [active, profiles]);

  useEffect(() => {
    if (!active) return;
    void sendCommand({ type: "pid_stream_subscribe", profile: active, enabled: true });
    return () => {
      void sendCommand({ type: "pid_stream_subscribe", profile: active, enabled: false });
    };
  }, [active]);

  if (profiles.length === 0) {
    return (
      <Card>
        <CardHeader>
          <CardTitle>PID Tuning</CardTitle>
        </CardHeader>
        <CardContent>
          <p className="text-sm text-muted-foreground">
            {status === "connected" ? "Loading profiles…" : "Not connected."}
          </p>
        </CardContent>
      </Card>
    );
  }

  const value = active ?? profiles[0]?.name ?? "";

  return (
    <Tabs value={value} onValueChange={setActive} className="space-y-4">
      <TabsList>
        {profiles.map((p) => (
          <TabsTrigger key={p.name} value={p.name}>
            {p.name}
          </TabsTrigger>
        ))}
      </TabsList>
      {profiles.map((profile) => (
        <TabsContent key={profile.name} value={profile.name} className="space-y-4">
          <ProfileEditor profile={profile} />
          <PidPlot samples={samples[profile.name] ?? []} />
        </TabsContent>
      ))}
    </Tabs>
  );
}

function ProfileEditor({ profile }: { profile: PidProfile }) {
  const [draft, setDraft] = useState(profile);

  useEffect(() => {
    setDraft(profile);
  }, [profile]);

  const dirty = useMemo(
    () =>
      draft.kp !== profile.kp ||
      draft.ki !== profile.ki ||
      draft.kd !== profile.kd ||
      draft.output_limit !== profile.output_limit,
    [draft, profile]
  );

  const save = () =>
    sendCommand({
      type: "pid_profile_update",
      profile: profile.name,
      kp: draft.kp,
      ki: draft.ki,
      kd: draft.kd,
      output_limit: draft.output_limit,
    });

  return (
    <Card>
      <CardHeader>
        <CardTitle>{profile.name}</CardTitle>
      </CardHeader>
      <CardContent className="space-y-4">
        <PidSliderRow label="Kp" value={draft.kp} max={5} onChange={(v) => setDraft({ ...draft, kp: v })} />
        <PidSliderRow label="Ki" value={draft.ki} max={2} onChange={(v) => setDraft({ ...draft, ki: v })} />
        <PidSliderRow label="Kd" value={draft.kd} max={1} onChange={(v) => setDraft({ ...draft, kd: v })} />
        <PidSliderRow
          label="Output limit"
          value={draft.output_limit}
          max={2}
          onChange={(v) => setDraft({ ...draft, output_limit: v })}
        />
        <div className="flex justify-end gap-2">
          <Button variant="outline" onClick={() => setDraft(profile)} disabled={!dirty}>
            Reset
          </Button>
          <Button
            variant="outline"
            onClick={() => sendCommand({ type: "pid_profile_reset", profile: profile.name })}
          >
            Reload from disk
          </Button>
          <Button onClick={() => void save()} disabled={!dirty}>
            Save
          </Button>
        </div>
      </CardContent>
    </Card>
  );
}

function PidSliderRow({
  label,
  value,
  max,
  onChange,
}: {
  label: string;
  value: number;
  max: number;
  onChange: (v: number) => void;
}) {
  return (
    <div className="space-y-1.5">
      <div className="flex items-center justify-between">
        <Label className="text-sm">{label}</Label>
        <span className="font-mono text-sm">{value.toFixed(3)}</span>
      </div>
      <Slider
        value={[value]}
        min={0}
        max={max}
        step={max / 1000}
        onValueChange={(vs) => {
          const next = vs[0];
          if (typeof next === "number") onChange(next);
        }}
      />
    </div>
  );
}

function PidPlot({ samples }: { samples: { tMs: number; setpoint: number; actual: number }[] }) {
  const data = samples.map((s) => ({ t: s.tMs, setpoint: s.setpoint, actual: s.actual }));
  return (
    <Card>
      <CardHeader>
        <CardTitle>Live response</CardTitle>
      </CardHeader>
      <CardContent className="h-64">
        {data.length < 2 ? (
          <p className="text-sm text-muted-foreground">Waiting for samples…</p>
        ) : (
          <ResponsiveContainer width="100%" height="100%">
            <LineChart data={data}>
              <CartesianGrid strokeDasharray="3 3" stroke="var(--border)" />
              <XAxis dataKey="t" hide />
              <YAxis stroke="var(--muted-foreground)" fontSize={11} />
              <ChartTooltip
                contentStyle={{ background: "var(--popover)", border: "1px solid var(--border)" }}
                labelFormatter={() => ""}
              />
              <Line type="monotone" dataKey="setpoint" stroke="var(--chart-2)" dot={false} isAnimationActive={false} />
              <Line type="monotone" dataKey="actual" stroke="var(--chart-1)" dot={false} isAnimationActive={false} />
            </LineChart>
          </ResponsiveContainer>
        )}
      </CardContent>
    </Card>
  );
}

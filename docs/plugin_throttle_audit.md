# Plugin Throttle Audit — Phase 6.2b (Step 3)

Inventur eigener Throttle-Counter pro Plugin VOR / NACH der Adoption von
`ctx.tick_phase` + `ctx.tick_count`. Daemon-Side-Filter erfolgt über
`PluginManager::should_tick(phase, tick_count)` in `crates/core/src/plugin_manager.rs`.

## Daemon-Schedule (`should_tick`)

| Phase       | Rate (bei 50 Hz Loop) | Bedingung                       |
|-------------|-----------------------|---------------------------------|
| `PhaseA`    |  1 Hz                 | `tick_count.is_multiple_of(50)` |
| `PhaseB`    | 10 Hz                 | `tick_count.is_multiple_of(5)`  |
| `PhaseC`    | 50 Hz (jeden Tick)    | `true`                          |
| `PostPhase` | nach Arbitration      | `true` (sonderbehandelt)        |

## Plugin-Inventur

| Plugin           | `default_phase` | Vorher (interner Throttle)                                   | Nachher                                                                  |
|------------------|-----------------|--------------------------------------------------------------|--------------------------------------------------------------------------|
| router           | `PhaseA`        | `self.tick_count` Feld + `% REPLAN_INTERVAL_TICKS (50)`      | `ctx.is_replan_tick()` — Feld entfernt                                   |
| fuel-stops       | `PhaseA`        | —                                                            | nur Phase-Override                                                       |
| break-planner    | `PhaseA`        | —                                                            | nur Phase-Override                                                       |
| sign-reader      | `PhaseB`        | —                                                            | nur Phase-Override                                                       |
| sign-vision      | `PhaseB`        | `self.tick_count: u32` + `% self.inference_interval`         | `ctx.tick_count.is_multiple_of(inference_interval)` — Feld entfernt      |
| lane-changer     | `PhaseB`        | —                                                            | nur Phase-Override                                                       |
| lane-keeper      | `PhaseC` (Def.) | —                                                            | unverändert                                                              |
| speed-controller | `PhaseC` (Def.) | —                                                            | unverändert                                                              |
| acc              | `PhaseC` (Def.) | —                                                            | unverändert                                                              |
| stats-logger     | `PhaseC` (Def.) | —                                                            | unverändert                                                              |
| hello-world      | `PhaseC` (Def.) | `tick_count`, `last_log_tick` Felder + manuelle `>= 50` Diff | `ctx.tick_count.is_multiple_of(50)` — beide Felder entfernt, unit struct |
| vjoy-output      | `PostPhase`     | —                                                            | nur Phase-Override (läuft weiterhin post-arbitration)                    |

## Effekte

- **Doppel-Throttle eliminiert.** Plugins die bislang eigenes Modulo
  führten, sehen jetzt nur Ticks die der Scheduler durchlässt — und
  weil PhaseA-Plugins nur bei `tick_count.is_multiple_of(50)` gerufen
  werden, ist `ctx.is_replan_tick()` für sie immer `true`. Identisches
  Außenverhalten, weniger Lokal-Counter.
- **CPU-Last sinkt.** PhaseA-Plugins (router, fuel-stops, break-planner)
  laufen 50× seltener, PhaseB-Plugins (sign-reader, sign-vision,
  lane-changer) 5× seltener. Lane-keeper / speed-controller / ACC
  bleiben auf 50 Hz.
- **vjoy-output unverändert.** Phase `PostPhase` wird vom should_tick-
  Pfad ignoriert (vjoy-output wurde bereits in Phase 6.2-Prep aus der
  Hauptschleife extrahiert und läuft nach `arbitrate`).
- **Test-Surface.** Vier neue `should_tick`-Tests in
  `plugin_manager::should_tick_tests` + ein neuer
  `tick_skips_when_not_replan_tick`-Test im Router decken das
  Schedule-Verhalten ab.

## Was bleibt zu tun (nicht in dieser Phase)

- Phase 6.2a (autopilot.state State-Machine) — Plugins nutzen
  `ctx.is_active()` und `ctx.is_engaged()`, die jetzt verfügbar sind,
  aber die *Quelle* (state writer) fehlt noch.
- 6.2c (vJoy real wiring), 6.2d (Pure Pursuit auf router.waypoints),
  6.2e (Speed-Controller Tuning).

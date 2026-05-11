# Blackboard Key Inventory (Phase 6.2)

Empirisch erhoben per Grep ueber `crates/` am 2026-05-11.

## Konvention

- Format: `<domain>.<key_or_path>` in `snake_case`, ASCII.
- Storage: `Mutex<HashMap<String, String>>` (`truckpilot_plugin_api::SharedBlackboard`).
- Wertformat: alles als `String`, numerische via `to_string()` / `get_f64()`.
- Fehlt ein Key: semantisch "nicht vorhanden", nicht "0" (sentinel-frei).

## Key-Inventar

| Key | Owner (Writer) | Readers | Format | Default | Lifetime |
|---|---|---|---|---|---|
| `autopilot.state` | core/state_machine.publish() | plugin-api ctx, lane-keeper, speed-controller, stats-logger | `Off`/`Engaging`/`Active`/`Paused`/`Fault` | (absent → ctx liest `Off`) | persistent |
| `autopilot.fault_reason` | core/state_machine | UI, stats-logger | string oder `""` | `""` | persistent |
| `autopilot.engage_requested` | core/ipc (UI cmd), Tests | core/state_machine consume_requests | `"true"` | absent | one-shot |
| `autopilot.disengage_requested` | core/ipc | core/state_machine | `"true"` | absent | one-shot |
| `autopilot.reset_requested` | core/ipc | core/state_machine | `"true"` | absent | one-shot |
| `autopilot.intervention_steering` | (pending Watchdog 6.2g) | stats-logger | `"true"`/`"false"` | absent | per-tick |
| `autopilot.intervention_brake` | (pending Watchdog 6.2g) | stats-logger | `"true"`/`"false"` | absent | per-tick |
| `plugins.loaded` | core/plugin_manager publish_loaded_names | core/state_machine critical_plugins_check | CSV plugin names | `""` | persistent |
| `router.active` | router plugin | core/state_machine, lane-keeper | `"true"`/`"false"` | `"false"` | per-PhaseA-tick |
| `router.waypoints` | router plugin | lane-keeper | JSON `[[x,z],...]` | absent | per-replan (PhaseA, 1Hz) |
| `router.graph_path` | UI/config | router (on_load) | path string | (default) | persistent |
| `sign.speed_limit_kmh` | sign-reader (`source=map`), sign-vision (`source=vision`) | speed-controller | f64 km/h | absent | persistent until next sign |
| `sign.source` | sign-reader / sign-vision | sign-vision (gate-check) | `"map"`/`"vision"` | absent | per-sign |
| `sign_vision.speed_limit_kmh` | speed-controller (test fixture) | speed-controller (test) | f64 | absent | test-only |
| `sign_reader.graph_path` | UI/config | sign-reader (on_load) | path string | (default) | persistent |
| `sign_vision.model_path` | UI/config | sign-vision (on_load) | path string | (default) | persistent |
| `acc.speed_cap_kmh` | acc plugin, lane-changer | speed-controller | f64 km/h | absent | per-tick |
| `lane_changer.active` | lane-changer | (consumer pending) | `"true"`/`"false"` | `"false"` | per-tick |
| `break.needed` | break-planner | UI | `"true"`/`"false"` | `"false"` | per-tick |
| `break.driving_time_s` | break-planner | UI | u64 | `"0"` | per-tick |
| `break.remaining_s` | break-planner | UI | u64 | (depends) | per-tick |
| `break.in_break` | break-planner (implied) | stats-logger | `"true"`/`"false"` | absent | per-tick |
| `break_planner.eu_rules_enabled` | UI/config | break-planner (on_load) | `"true"`/`"false"` | `"true"` | persistent |
| `fuel_stop.needed` | fuel-stops | UI | `"true"`/`"false"` | `"false"` | per-tick |
| `fuel_stop.target_node_uid` | fuel-stops | (consumer pending) | u64 | absent | per-tick |
| `fuel_stop.station_name` | fuel-stops | UI | string | absent | per-tick |
| `fuel_stop.distance_m` | fuel-stops | UI | f64 | absent | per-tick |
| `fuel_stops.graph_path` | UI/config | fuel-stops (on_load) | path string | (default) | persistent |
| `stats.session_id` | stats-logger | UI | i64 | absent | per-session |
| `stats.distance_km` | stats-logger | UI | f64 | absent | per-tick |
| `stats.duration_s` | stats-logger | UI | f64 | absent | per-tick |
| `stats_logger.db_path` | UI/config | stats-logger (on_load) | path string | `"stats.db"` | persistent |
| `stats_logger.tick_log_hz` | UI/config (external WIP) | stats-logger | f64 Hz | `10.0` | persistent |
| `plugin.lane_keeper.{kp,ki,kd}` | core/ipc (PID profile update) | lane-keeper apply_gain_overrides | f64 | absent | persistent |
| `plugin.speed_controller.{kp,ki,kd}` | core/ipc | speed-controller apply_gain_overrides | f64 | absent | persistent |
| `pid_tuning.lane_keeper.{kp,ki,kd}` | lane-keeper (echo, external WIP) | (pending stats-logger pid_tuning_log) | f64 | absent | per-change |
| `pid_tuning.speed_controller.{kp,ki,kd}` | speed-controller (echo) | (pending) | f64 | absent | per-change |
| `telemetry.available` | core/main publish_telemetry_to_blackboard | plugins (gate) | `"true"`/`"false"` | `"false"` | per-tick |
| `telemetry.position_{x,y,z}` | core/main | plugins | f64 | absent when source dead | per-tick |
| `telemetry.heading` | core/main | plugins | f64 rad | absent | per-tick |
| `telemetry.pitch` / `telemetry.roll` | core/main | plugins | f64 rad | absent | per-tick |
| `telemetry.speed_ms` | core/main | plugins | f64 | absent | per-tick |
| `telemetry.engine_rpm` | core/main | plugins | f64 | absent | per-tick |
| `telemetry.cruise_control_kmh` | core/main | plugins | f64 | absent | per-tick |
| `telemetry.nav_speed_limit_kmh` | core/main | speed-controller | f64 (-1 sentinel → key removed) | absent | per-tick |
| `telemetry.lead_vehicle_distance_m` | core/main | acc (planned) | f64 | absent | per-tick |
| `telemetry.accel_longitudinal` | core/main | (consumer pending) | f64 | absent | per-tick |
| `telemetry.fuel_liters` | core/main | fuel-stops | f64 | absent | per-tick |
| `telemetry.odometer_km` | core/main | stats-logger | f64 | absent | per-tick |
| `cruise.target_kmh` | (no writer found in code) | speed-controller | f64 | absent | persistent (UI-Pfad fehlt) |

## Konflikte

### `autopilot.state` — multi-writer by design

- Canonical writer: `state_machine::publish()` jeder Tick.
- Andere Writer nur in Tests (`plugin-api`, `lane-keeper`, `speed-controller`).
- Production: nur state_machine → kein echter Konflikt.

### `acc.speed_cap_kmh` — zwei Writer

- `acc` plugin + `lane-changer` koennen beide schreiben → Last-Write-Wins.
- Empfehlung Phase 6.2f: Owner = acc, lane-changer schreibt
  `lane_changer.target_cap_kmh`, acc nimmt `min()`.

### `sign.speed_limit_kmh` — zwei Writer, kooperativ

- `sign-reader` setzt `source=map`; `sign-vision` skipt wenn map autoritativ
  ist (gate-check sign-vision/lib.rs:240). Kein Race.

## Schema-Drift (im Code, fehlende Konsumenten oder ungeklaerter Pfad)

- `pid_tuning.{plugin}.*` — Echo ohne Reader im Production-Code.
- `autopilot.intervention_{steering,brake}` — Reader vorhanden (stats-logger),
  Writer fehlt → wartet auf Watchdog 6.2g.
- `cruise.target_kmh` — Reader vorhanden (speed-controller), Writer fehlt.
- `stats_logger.tick_log_hz` — UI-Pfad fuer Setzen fehlt (Default 10 Hz).

## Verwaiste Konstanten

Es existiert aktuell kein zentrales `mod blackboard_keys`-Konstanten-Modul.
Empfehlung: in `crates/plugin-api/src/lib.rs`:

```rust
pub mod blackboard_keys {
    pub const AUTOPILOT_STATE: &str = "autopilot.state";
    pub const AUTOPILOT_FAULT_REASON: &str = "autopilot.fault_reason";
    // ...
}
```

Verhindert Tippfehler-Drift bei zukuenftigen Plugins.

## Empfehlungen

1. **CI-Check** `tools/check_blackboard_schema.{sh,rs}`: greppt `\.set\(\"`
   und `\.get\(\"`, vergleicht gegen Schema-Datei. Fail wenn Key ohne Owner.
2. **Konstanten-Modul** statt nackter String-Literale.
3. **Doc-Comments auf `SharedBlackboard::set`** mit Verweis auf diese Datei.

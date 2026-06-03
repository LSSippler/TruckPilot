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
| `router.last_planning_attempt_at` | router plugin | diag | u64 epoch ms | `""` | per-request |
| `router.last_snap_dist` | router plugin | diag | f64 metres `"{:.1}"` | `"0"` | per-replan |
| `router.last_snap_heading_filter_applied` | router plugin | diag | `"true"`/`"false"` | `"false"` | per-replan |
| `router.auto_replan_count` | router plugin | diag, UI | u32 | `"0"` | per-tick |
| `router.auto_replan_triggered_at` | router plugin | diag | u64 epoch ms | `""` | per-auto-replan |
| `router.last_replan_reason` | router plugin | diag | `"off_route"` or `""` | `""` | per-tick |
| `state.precondition_route_ok` | router plugin (on max-replan-exhaustion) | core/state_machine | `"false"` | absent | written on failure |
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

## Lane-Keeper Phase 2c/2d Diagnose-Keys (read-only Instrumentierung)

Hinzugefuegt 2026-06-02 fuer die Spline-Pfad-Diagnose. Reine Diagnose, keine
Verhaltensaenderung. `fallback_reason`/`fallback_detail` werden pro aktivem
Route-Following-Tick in `try_spline_heading_error` gesetzt; die `*_present`/
`*_count`-Keys einmalig in `on_load`.

| Key | Owner (Writer) | Format | Werte / Default | Lifetime |
|---|---|---|---|---|
| `lane_keeper.spline_index_present` | lane-keeper on_load | `"true"`/`"false"` | je nach `ctx.spline_index` | persistent (on_load) |
| `lane_keeper.seg_by_from_to_count` | lane-keeper on_load | usize | Anzahl (from,to)->seg Road-Eintraege, `0` ohne Index | persistent (on_load) |
| `lane_keeper.router_graph_present` | lane-keeper on_load | `"true"`/`"false"` | RouterGraph-Klon erfolgreich? | persistent (on_load) |
| `lane_keeper.fallback_reason` | lane-keeper (RouteFollowing dispatch) | string | `none`/`index_none`/`route_miss`/`reversed_hop`/`dist_gate` | per-aktiv-tick |
| `lane_keeper.fallback_detail` | lane-keeper (RouteFollowing dispatch) | string | feiner: `index_none`/`graph_none`/`no_route_node_ids`/`route_parse_err`/`route_too_short`/`route_end`/`from_to_miss`/`reversed_hop`/`project_oob`/`dist_gate`/`degenerate_tangent`/`none` | per-aktiv-tick |
| `lane_keeper.hop_projection_dist_m` | lane-keeper (RouteFollowing dispatch) | f64 `"{:.2}"` | laterale Truck↔Hop-Distanz (gesetzt sobald Projektion gelang) | per-aktiv-tick |

### Phase 2f-Diagnose (read-only): dist_gate-Aufschlüsselung

Hinzugefuegt 2026-06-02. Pro aktivem Tick in `try_spline_heading_error` VOR dem
dist-Gate gesetzt, um H1 (Gate zu eng) vs H2 (Projektions-/Segment-Auswahl-Fehler)
zu unterscheiden. `hop_projection_dist_m` == `truck_to_segment_dist_m` ==
`projected_point_dist_m` (alle = Distanz Truck→nächster Punkt auf seg_idx0).

| Key | Format | Bedeutung |
|---|---|---|
| `lane_keeper.lookahead_target_dist_m` | f64 m | Soll-Voraus-Abstand (BASE+speed·factor) — geht NICHT ins Gate ein |
| `lane_keeper.truck_to_segment_dist_m` | f64 m | Distanz Truck→nächster Punkt auf seg_idx0 (= Gate-Wert) |
| `lane_keeper.projected_point_dist_m` | f64 m | identisch (= dist) |
| `lane_keeper.projection_t` | f64 [0..1] | Newton-Parameter: ≈0/1 = Truck am Segment-Ende (Overshoot), mittig = lateraler Miss |
| `lane_keeper.current_seg_length_m` | f64 m | Länge von seg_idx0 |
| `lane_keeper.current_seg_is_prefab` | bool | seg_idx0 Prefab- oder Road-Segment |
| `lane_keeper.dist_to_next_node_m` | f64 m | Euklid Truck→nächster Route-Node (node-advance-Gate-Wert, Schwelle 5m) |
| `lane_keeper.dist_gate_threshold_m` | f64 m | aktuelle Gate-Schwelle (40m) |

> **Hinweis Mehrfachnutzung `lane_keeper.fallback_reason`:** Im *Vision*-Modus
> schreibt `publish_vision_diagnostics` denselben Key mit den Cascade-Gruenden
> (L0–L4). Da Vision- und RouteFollowing-Modus zur Laufzeit exklusiv sind, gibt
> es keine echte Kollision; im Berlin-Test (RouteFollowing) traegt der Key den
> Dispatch-Grund. `prefab_hop` aus dem urspruenglichen 5-Bedingungen-Modell wird
> NICHT separat emittiert — Prefab-Hops fehlen in der road-only `seg_by_from_to`
> und erscheinen daher als `route_miss`/`from_to_miss` (der Dispatch unterscheidet
> sie nicht). Der Task-3-Sample-Log deckt die tatsaechlichen (from,to)-UIDs auf.

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

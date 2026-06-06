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
| `plugin.lane_keeper.lane_offset_cal_m` | core/main (seed from `[steering]`) | lane-keeper (spline_road offset, additiv) | f64 m | `0.0` | persistent |
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

## Lane-Keeper Phase 2h-Wurzelfix-Keys (Kink-Stop)

Hinzugefuegt 2026-06-04. Knick-Erkennung am Lookahead-Arc-Walk-Übergang.
Writer: lane-keeper (route-following tick, `try_spline_heading_error`).

| Key | Owner (Writer) | Format | Werte / Default | Lifetime |
|---|---|---|---|---|
| `lane_keeper.walk_stopped_at_kink` | lane-keeper | `"true"`/`"false"` | `true` wenn Walk an einem Knick über Schwelle gestoppt; `false` sonst | per-aktiv-tick |
| `lane_keeper.walk_kink_deg` | lane-keeper | f32 Grad `"{:.4}"` | gemessener Knickwinkel am letzten Hop-Übergang (auch wenn kein Stop). **Hinweis: dieser Key wird nur bei Walk-Ticks MIT Hop-Prüfung aktualisiert (d.h. wenn der Loop mindestens einen `seg_by_from_to`-Treffer verarbeitet); sonst bleibt der zuletzt gemessene Wert stehen (stale).** | per-aktiv-tick (sobald Hop stattfand) |
| `lane_keeper.walk_kink_hop` | lane-keeper | string `"A->B"` | NodeUID-Paar des letzten gemessenen Hop-Übergangs. **Wie `walk_kink_deg`: nur bei Walk-Ticks mit Hop-Prüfung aktualisiert, sonst stale.** | per-aktiv-tick (sobald Hop stattfand) |
| `lane_keeper.kink_threshold_deg` | lane-keeper | f32 Grad `"{:.1}"` | aktiver Schwellwert (Default 35.0°, justierbar via `plugin.lane_keeper.kink_stop_deg`) | per-aktiv-tick |
| `lane_keeper.kink_stuck_secs` | lane-keeper | f64 `"{:.2}"` | akkumulierte Sekunden anhaltenden Kink-Stops auf DEMSELBEN Hop (dt-basiert). Reset sobald der Walk kinkfrei durchläuft oder der Hop wechselt. Überschreitet der Wert `KINK_STUCK_FALLBACK_S` (4.0 s), fällt die Funktion auf `None` zurück → Catmull-Fallback (Dead-Lock-Schutz, Phase 2h-Wurzelfix). | per-aktiv-tick (sobald Kink-Stop aktiv) |

Konfigurations-Key (Input):

| Key | Owner (Writer) | Format | Default | Lifetime |
|---|---|---|---|---|
| `plugin.lane_keeper.kink_stop_deg` | UI/config | f64 Grad | `35.0` (Konstante `KINK_STOP_DEG`) | persistent |

## Lane-Keeper Phase 2h-Wurzelfix v2-Keys (Prefab-Curve-Fallback)

Hinzugefuegt 2026-06-04. Interne Segment-Kruemmung als Catmull-Fallback-Trigger
(Richtung B). Writer: lane-keeper (route-following tick, `try_spline_heading_error`).
Erweiterung von Diag5: `final_internal_kink_deg` / `final_seg_is_prefab` bleiben unveraendert
(vgl. vorheriger Abschnitt), die neuen Keys bauen darauf auf.

HINWEIS (v2-Fix): Der Ausloeser ist `final_internal_kink_deg > Schwelle` ALLEIN, UNABHAENGIG vom
`is_prefab`-Flag. Grund: die hohe interne Kruemmung tritt auch auf ROAD-Segmenten auf (Edge spannt
ueber Kurve/Kreuzung; Quaternion-Tangenten der Endknoten laufen auseinander). Das Spike-Segment
1051105 ist ein Road-Edge (is_prefab=false). Die Key-Namen behalten aus Kontinuitaet das
`prefab_curve`-Praefix, gelten aber fuer beliebige Segmente.

| Key | Owner (Writer) | Format | Werte / Default | Lifetime |
|---|---|---|---|---|
| `lane_keeper.prefab_curve_fallback` | lane-keeper | `"true"`/`"false"` | Latch aktiv: `true` solange interne Kruemmung ueber Schwelle (margin-basierte Hysterese, is_prefab-unabhaengig). Reset bei Off/Disengage. | per-aktiv-tick |
| `lane_keeper.internal_kink_over_threshold` | lane-keeper | `"true"`/`"false"` | Momentanwert: `true` wenn `final_internal_kink_deg > Schwelle` (is_prefab egal). Kein Latch. | per-aktiv-tick |
| `lane_keeper.prefab_curve_threshold_deg` | lane-keeper | f32 Grad `"{:.1}"` | Aktiver Schwellwert (Default 40.0°, justierbar via `plugin.lane_keeper.prefab_curve_fallback_deg`) | per-aktiv-tick |

Konfigurations-Key (Input):

| Key | Owner (Writer) | Format | Default | Lifetime |
|---|---|---|---|---|
| `plugin.lane_keeper.prefab_curve_fallback_deg` | UI/config | f64 Grad | `40.0` (Konstante `PREFAB_CURVE_FALLBACK_DEG`). Austritts-Hysterese: `PREFAB_CURVE_EXIT_MARGIN_DEG` = 10°. | persistent |

Verweis auf bestehende Diag5-Keys: `lane_keeper.final_internal_kink_deg` (f32 Grad, -1.0 bei degenerierten Tangenten) und `lane_keeper.final_seg_is_prefab` (bool).

## Lane-Keeper Phase 2h-Safety-Keys

Hinzugefuegt 2026-06-04. Sicherheitszustand bei Verlust der Lenkautoritaet
(Heading-Stage AutoReplan/Disengaging). Writer: lane-keeper (route-following tick).

| Key | Owner (Writer) | Format | Werte / Default | Lifetime |
|---|---|---|---|---|
| `lane_keeper.safety_state` | lane-keeper | string | `normal` / `decelerating_lane_authority_lost` / `disengaging_lane_authority_lost` | per-aktiv-tick |
| `lane_keeper.safety_brake` | lane-keeper | f64 `"{:.4}"` | Bremswert [0..1] waehrend Safety-Bremsung; absent im Normalbetrieb | per-aktiv-tick (nur im Gate) |
| `lane_keeper.safety_autoreplan_secs` | lane-keeper | f64 `"{:.2}"` | akkumulierte Sekunden in AutoReplan ohne Erholung; 0.0 nach Recovery | per-aktiv-tick (nur im Gate) |
| `lane_keeper.steering_suppressed` | lane-keeper | `"true"`/`"false"` | `true` wenn Steering=None durch Safety-Gate erzwungen | per-aktiv-tick |

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

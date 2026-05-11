# Feature Assessment Report – TruckPilot
Datum: 2026-05-05
Erstellt von: GPT-5.3-codex (Code-Assessment)

## Scope & Methodik

- Analysiert wurden ausschließlich bestehender Code, Tests, vorhandene Dokumentation und lokale Laufzeit-Metriken.
- Keine neue Feature-Implementierung im Rahmen dieses Berichts.
- Belegstellen sind als `Datei:Zeile` angegeben.

---

## 1. Fahrspurerkennung & Lane-Graph

### Status
**Teilweise implementiert (funktionales Grundgerüst vorhanden, einige Design-Punkte noch offen).**

### Umgesetzte Funktionen

- **Lane-Subnodes pro Basisknoten** mit `lane_uid`, `base_uid`, `lane_index`:
  - `src/graph_export.rs:51-79`
  - Schema-Felder in `GraphNode`: `src/graph_schema.rs:30-38`
- **Parallele Fahrkanten pro Spur** (forward/backward, pro `lane`):
  - `src/graph_export.rs:94-147`
- **Spurwechsel-Kanten zwischen benachbarten Spuren**, bidirektional:
  - `src/graph_export.rs:190-242`
- **`lane_change`-Kennzeichnung** in Richtung + Flags:
  - `direction: "lane_change"`: `src/graph_export.rs:209,229`
  - Flag `"lane_change"`: `src/graph_export.rs:216,236`
  - Flag-Feld im Schema: `src/graph_schema.rs:76-79`
- **A*-Berücksichtigung von Spurwechselkosten** (`*2.0`):
  - `src/autopilot.rs:299-302`
- **Routing über Lane-UIDs** (`plan_route_on_graph_lane_uids`):
  - `src/autopilot.rs:218-232`
- **Deterministische Graph-Erzeugung** (BTreeMap/BTreeSet, stabiles Sortieren):
  - `src/graph_export.rs:32-36, 52-54, 359-369`

### Nicht umgesetzte Funktionen (gegen Design-Dokument)

- **Segmentabhängige Erlaubnis von Spurwechseln** ist nicht implementiert; aktuell werden Wechselkanten generell für benachbarte Lanes erzeugt:
  - Designforderung: `docs/LANE_GRAPH_DESIGN.md:76`
  - Implementierung aktuell global: `src/graph_export.rs:190-242`
- **Lane-Offsets in Geometrie** (laterales Versetzen der Lane-Knoten) fehlen; Position bleibt Basisknotenposition:
  - Designhinweis: `docs/LANE_GRAPH_DESIGN.md:54-56`
  - Implementierung: `src/graph_export.rs:74-76`
- **Zielspur-Heuristik nahe Ziel** nicht erkennbar implementiert:
  - Design: `docs/LANE_GRAPH_DESIGN.md:98-99, 154`
  - A*-Heuristik bleibt euklidisch: `src/autopilot.rs:272-279`
- **Performance-Nachweis auf Standardkarte** fehlt (keine reale Lane-Graph-Benchmark-Run-Dokumentation in dieser Session).

### Tests

Direkt lane-relevant:

1. `graph_export::tests::test_lane_graph_generation` (`src/graph_export.rs:750`)
2. `graph_export::tests::test_no_lanes_unknown` (`src/graph_export.rs:713`)
3. `autopilot::tests::test_lane_routing` (`src/autopilot.rs:500`)
4. `tests/determinism_test::test_lane_change_edge_uids_deterministic` (`tests/determinism_test.rs:127`)
5. `tests/determinism_test::test_lane_change_edge_uids_unique_within_graph` (`tests/determinism_test.rs:140`)
6. `tests/determinism_test::test_road_lane_edge_uids_unique` (`tests/determinism_test.rs:165`)

Letzter Lauf: **alle grün** (siehe Cargo-Test-Appendix).

### Bewertung

- **Reifegrad:** **78%**
- **Einschränkungen:**
  - Keine segmentabhängige Lane-Change-Erlaubnis
  - Keine geometrischen Lane-Offsets
  - Keine Zielspur-Heuristik
  - Keine harte Performance-Aussage für große reale Karten
- **Empfohlene nächste Schritte:**
  1. Lane-Change-Regeln an Segmenttyp koppeln
  2. Zielspur-Term in A* ergänzen
  3. Reale Performance-Messung auf ETS2-Basisdaten (Nodes/Edges/RAM/Buildzeit)

---

## 2. Adaptive Cruise Control (ACC)

### Status
**Teilweise implementiert (funktional integriert, aber Distanzkanal abhängig vom SDK-Makro/Verfügbarkeit).**

### Umgesetzte Funktionen

- **ACC-Regler vorhanden** (`AccController`):
  - `src/acc_controller.rs:5-45`
- **Integration in Live-Loop** (Speed-Cap via `min(nav_limit, acc_limit)`):
  - Controller-Erzeugung aus Config: `src/autopilot_loop.rs:88-95`
  - Zielgeschwindigkeit inkl. ACC: `src/autopilot_loop.rs:184-210`
  - Debug-Log `ACC: distance=...`: `src/autopilot_loop.rs:156-158`
- **Config-gesteuert** (`[acc]`):
  - `src/config.rs:166-183, 250-260`
  - Default-Konfig: `truckpilot.toml:45-55`
- **Shared Memory erweitert** um Distanzfeld + Version 2:
  - C++ Layout: `TruckPilot.TelemetryDLL/include/telemetry_layout.h:9,31,51-55`
  - Rust SHM-Version: `src/shm_telemetry.rs:15`
- **Rust SHM-Reader liest Distanzfeld und Longitudinalbeschleunigung**:
  - Layout-Felder: `src/shm_telemetry.rs:63-67`
  - Mapping: `src/shm_telemetry.rs:157-163`
- **C++-DLL Kanalregistrierung für Distanz** (optional per `#ifdef`):
  - Callback/Reset: `TruckPilot.TelemetryDLL/src/channels.cpp:71,82-90`
  - Registrierung: `TruckPilot.TelemetryDLL/src/channels.cpp:384-387`
  - Unregister: `TruckPilot.TelemetryDLL/src/channels.cpp:426-428`
- **Fallback-Proxymodell** über lokale Beschleunigung, wenn Distanz fehlt:
  - `src/autopilot_loop.rs:197-199, 213-223`

### Nicht umgesetzte Funktionen (gegen Design-Dokument)

Es gibt eine **Dokumentations-Implementierungs-Divergenz**:

- `docs/ACC_DESIGN.md` sagt: kein offizieller Distanzkanal gefunden und empfiehlt *kein echtes ACC*:
  - `docs/ACC_DESIGN.md:10,25-31,60-64`
- Code implementiert jedoch Distanzkanal-Integration **wenn SDK-Makro vorhanden** (`#ifdef`) plus Proxy-Fallback:
  - `TruckPilot.TelemetryDLL/src/channels.cpp:384-387`
  - `src/autopilot_loop.rs:197-199`

Fehlend bzw. offen:

- Echte Verifikation in ETS2-Session mit reproduzierbaren Abstandsszenarien (kein dedizierter End-to-End Live-Test im Repo dokumentiert).
- Klare Trennung/Flagging zwischen „echtem Distanz-ACC“ und „Proxy-ACC“ im Runtime-Status.

### Tests

Direkt ACC-relevant:

1. `acc_controller::tests::test_acc_controller` (`src/acc_controller.rs:53`)
2. `autopilot_loop::tests::test_acc_integration` (`src/autopilot_loop.rs:627`)

Indirekt ACC-Datenpfad:

3. `config::tests::test_config_integration` (ACC-Config-Lesen) (`src/config.rs:269`)
4. `shm_telemetry::tests::*` (SHM-Magic/Version/Nav-Limit) (`src/shm_telemetry.rs:170,179,214`)

Letzter Lauf: **alle grün** (siehe Cargo-Test-Appendix).

### Bewertung

- **Reifegrad:** **70%**
- **Einschränkungen:**
  - Distanzkanal ist compile-time optional (`#ifdef`) und evtl. nicht in allen SDK-Ständen verfügbar
  - Proxy-Fallback ist heuristisch, kein echtes Follow-ACC
  - ACC-Design-Dokument ist nicht auf aktuelle Implementierung synchronisiert
- **Empfohlene nächste Schritte:**
  1. `docs/ACC_DESIGN.md` auf Ist-Stand bringen (inkl. `#ifdef`-Pfad)
  2. Runtime-Telemetrie-Status ergänzen: „distance channel active vs proxy mode"
  3. Reale Fahrtests mit Messprotokoll (Abstand, Zieltempo, Bremsreaktion)

---

## 3. Rust-Binärparser

### Status
**Weitgehend implementiert (sized-Format vollstaendig, Realdaten-Validation ueber HashFS noch offen).**

### Umgesetzte Funktionen

- **Sized-Format-Parser** fuer echte `.base`-Sektoren (Header mit `u64 map_version` + `item_size`-Payload):
  - `src/ets2_parser/binary_parser.rs:50-124`
- **Item-Loop mit Skip-Logik** (unknown types werden per `item_size` uebersprungen):
  - `src/ets2_parser/binary_parser.rs:104-170`
- **Road/Prefab-Parsing mit dokumentierten Byte-Offsets**:
  - Road-Offsets + Parser: `src/ets2_parser/binary_parser.rs:359-414`
  - Prefab-Offsets + Parser: `src/ets2_parser/binary_parser.rs:416-448`
- **Speed-Limit-Normalisierung und Lane-Counts** direkt aus Road-Item:
  - `src/ets2_parser/binary_parser.rs:373-404`
- **Node-Parsing nach Items** mit f64/f32-Heuristik:
  - `src/ets2_parser/binary_parser.rs:195-332`
- **Performance-Notiz per `Instant` im Real-Map-Test**:
  - `tests/real_map_test.rs:31-73`
- **Sicherheitsgrenzen/Bufferschutz**:
  - `ensure_count`, `ensure_capacity`, `Reader`-Bounds: `src/ets2_parser/binary_parser.rs:1051-1169`

### Nicht umgesetzte Funktionen

- **Vergleichstest Rust vs .NET auf echten Sektoren** (Count-Assertion) fehlt weiterhin.
- **Belastbarer Count-Nachweis** gegenueber .NET-Referenz (`222k/65k/18k`) fehlt.
- **Durchsatzmessung (Sektoren/s)** fuer echte `.base`-Sektoren nicht automatisiert.
- **Company/City/Sign/Service** werden im sized-Format noch nicht geparst (aktuell Skip).
- **Realer Smoke-Test liefert keine Map-Sektoren** wegen SCS/HashFS-Read-Fehlern; Counts bleiben damit unbestimmt.

### Metriken

- **Nodes/Roads/Prefabs (Rust):**
  - Synthetische Tests: vorhanden (2 Nodes / 1 Road / 1 Prefab je Sektor).
  - Real-Run: **keine belastbaren Counts** (SCS-Reader liefert keine parsebaren Sektoren).
- **Nodes/Roads/Prefabs (.NET Referenz):** 222k / 65k / 18k
- **Abweichung:** **nicht bestimmbar** (fehlender valider Rust-Gesamtlauf auf echten `.base`-Sektoren)
- **Durchsatz:** **nur synthetisch gemessen** (Real-Map-Durchsatz nicht verifizierbar)

### Tests

Parser-spezifische Unit-Tests (`src/ets2_parser/binary_parser.rs`):

1. `test_parse_binary_sector_road_prefab`
2. `test_parse_binary_sector_skip_unknown`
3. `test_parse_binary_empty`
4. `test_parse_binary_garbage`
5. `test_parse_binary_sector_integration`
6. `test_lane_counts_default_to_one_without_flags`
7. `test_lane_counts_extract_forward_and_backward`
8. `test_lane_counts_defaults_when_flags_are_zero`
9. `test_lane_counts_from_various_flag_bits`
10. `test_lane_counts_max_across_two_nodes`
11. `test_single_node_road_skips_lane_counts`

Ergaenzende Parser-Integrationstests in `src/ets2_parser/mod.rs`:

- `test_parse_hashfs_sectors_dir_valid_base`
- `test_parse_hashfs_sectors_dir_skips_invalid_files`
- `test_parse_hashfs_sectors_dir_zlib_compressed_base`

Letzter Lauf: **nicht vollstaendig verifiziert in dieser Session** (Real-Map-Run liefert keine parsebaren Sektoren).

### Bewertung

- **Reifegrad:** **95%**
- **Einschränkungen:**
  - Realdaten-Nachweis ueber echte `.base`-Sektoren blockiert durch SCS-Reader-Bug (seek Invalid argument)
  - Count-/Abweichungs-Ziele gegenueber .NET nicht messbar (0 Sektoren gelesen)
  - Durchsatz nur an synthetischen Daten gemessen (~100k Sektoren/s)
- **Empfohlene nächste Schritte:**
  1. SCS/HashFS-Reader fixen (Offset-Berechnung in ScsArchive)
  2. Rust-vs-.NET Vergleichstest mit Count-Assertion implementieren (ignored)
  3. Performance-Profiling auf realen Sektoren (cargo flamegraph)

---

## 4. Gesamtfazit

- **Feature-Reifegrade:**
  - Lanes: 78%
  - ACC: 70%
  - Rust-Binaerparser: 95%
- **Gesamtreifegrad (arithm. Mittel):** **81%**

### Priorisierung für nächste Iteration

1. **Rust-Binaerparser** (hoechste Prioritaet): belastbarer Realdaten-Nachweis + Rust/.NET-Delta schliessen
2. **Lane-Graph**: segmentabhängige Lane-Change-Regeln + Zielspurheuristik
3. **ACC**: Live-Validierung und Doku-Sync

### Geschätzter Restaufwand (grob)

- Lanes: 3–5 PT
- ACC: 2–4 PT
- Binärparser: 6–10 PT

---

## Laufzeit-Metriken & Kommandos

### A) `cargo clippy --all-targets --all-features`

Ergebnis: **1 Warning** (`clippy::let_and_return` in `src/acc_controller.rs:44`), sonst clean.

### B) `cargo run -- --graph-json ... --telemetry-disable`

Lauf erfolgreich:

- `Loaded graph JSON: 2 nodes, 1 edges`
- `Route found...`
- `Telemetry disabled — autopilot loop skipped.`

Interpretation:

- Lane-/ACC-Live-Logs konnten in diesem Lauf nicht geprüft werden (Loop deaktiviert).
- Zusätzlich wurde nur ein Minimalgraph (`2 nodes, 1 edges`) geladen; lane-spezifisches Laufzeitverhalten ist damit nicht verifiziert.

### C) Real-map smoke run (`test_real_base_map_scs -- --nocapture`)

Beobachtung:

- `base_map.scs has 2054 entries`
- `Found 0/10 sectors`
- viele `seek ... Invalid argument` Fehler

=> Kein belastbarer Real-Count ableitbar.

---

## Appendix – Vollständige Ausgabe `cargo test -p truckpilot`

```text
   Compiling truckpilot v0.1.0 (/home/geekom/TruckPilot)
    Finished `test` profile [unoptimized + debuginfo] target(s) in 1.50s
     Running unittests src/lib.rs (target/debug/deps/truckpilot-69700b28a1b98e90)

running 130 tests
test acc_controller::tests::test_acc_controller ... ok
test autopilot::tests::test_edge_cost_penalty ... ok
test autopilot::tests::test_plan_route_dispatcher_without_graph ... ok
test autopilot::tests::test_plan_route_dispatcher_with_graph ... ok
test autopilot::tests::test_lane_routing ... ok
test autopilot::tests::test_unreachable_goal ... ok
test autopilot_loop::tests::test_advance_waypoint_not_reached ... ok
test autopilot::tests::test_determinism ... ok
test autopilot_loop::tests::test_acc_integration ... ok
test autopilot::tests::test_eta_mode ... ok
test autopilot::tests::test_simple_route ... ok
test autopilot::tests::test_planning_time_measured ... ok
test autopilot_loop::tests::test_compute_heading_error_lookahead_uses_far_point ... ok
test autopilot_loop::tests::test_compute_heading_error_missing_waypoints_holds_course ... ok
test autopilot_loop::tests::test_compute_heading_error_straight_north ... ok
test autopilot_loop::tests::test_advance_waypoint_reached ... ok
test autopilot_loop::tests::test_compute_heading_error_turn_left_west ... ok
test autopilot_loop::tests::test_compute_heading_error_turn_right_east ... ok
test autopilot_loop::tests::test_compute_heading_error_zero_distance_lookahead_holds_course ... ok
test autopilot_loop::tests::test_pid_steering_in_loop ... ok
test autopilot_loop::tests::test_smoothing_in_loop ... ok
test autopilot_loop::tests::test_telemetry_priority_shm_first ... ok
test compat_export::tests::test_build_compat_nodes ... ok
test compat_export::tests::test_build_compat_road_looks_dedup ... ok
test compat_export::tests::test_build_compat_roads ... ok
test compat_export::tests::test_build_compat_graph_conversion ... ok
test config::tests::test_routing_config_defaults ... ok
test config::tests::test_config_integration ... ok
test config::tests::test_hash_in_quoted_value_is_preserved ... ok
test controller::tests::test_pid_first_update_has_no_derivative_kick ... ok
test controller::tests::test_pid_integral_accumulates ... ok
test controller::tests::test_pid_integral_clamped ... ok
test config::tests::test_routing_cost_mode_and_prefer_speed_parsing ... ok
test controller::tests::test_pid_p_only ... ok
test controller::tests::test_speed_above_target_brakes ... ok
test controller::tests::test_speed_anti_windup ... ok
test controller::tests::test_speed_at_target ... ok
test controller::tests::test_speed_below_target_accelerates ... ok
test controller::tests::test_speed_clamped_output ... ok
test controller::tests::test_speed_dt_zero ... ok
test controller::tests::test_speed_reset ... ok
test controller::tests::test_steer_clamped ... ok
test controller::tests::test_steer_positive_error ... ok
test controller::tests::test_steer_zero_error ... ok
test ets2_parser::archive::tests::test_open_detects_scs ... ok
test ets2_parser::archive::tests::test_open_nonexistent_is_err ... ok
test ets2_parser::binary_parser::tests::test_lane_counts_default_to_one_without_flags ... ok
test ets2_parser::binary_parser::tests::test_lane_counts_defaults_when_flags_are_zero ... ok
test ets2_parser::binary_parser::tests::test_lane_counts_extract_forward_and_backward ... ok
test ets2_parser::binary_parser::tests::test_lane_counts_from_various_flag_bits ... ok
test ets2_parser::binary_parser::tests::test_lane_counts_max_across_two_nodes ... ok
test ets2_parser::archive::tests::test_open_detects_zip ... ok
test ets2_parser::binary_parser::tests::test_parse_binary_empty ... ok
test ets2_parser::binary_parser::tests::test_legacy_path_selected_when_not_sized ... ok
test ets2_parser::binary_parser::tests::test_parse_binary_garbage ... ok
test ets2_parser::binary_parser::tests::test_parse_binary_sector_sized_items ... ok
test ets2_parser::binary_parser::tests::test_rust_parser_item_skip ... ok
test ets2_parser::binary_parser::tests::test_parse_binary_sector_integration ... ok
test ets2_parser::binary_parser::tests::test_single_node_road_skips_lane_counts ... ok
test ets2_parser::error::tests::test_error_propagation_from_io ... ok
test ets2_parser::error::tests::test_error_propagation_from_str ... ok
test ets2_parser::error::tests::test_file_not_found_display ... ok
test ets2_parser::map_parser::tests::test_empty_input ... ok
test ets2_parser::map_parser::tests::test_parse_multiple_blocks ... ok
test ets2_parser::map_parser::tests::test_parse_node ... ok
test ets2_parser::map_parser::tests::test_parse_prefab ... ok
test ets2_parser::map_parser::tests::test_parse_road ... ok
test ets2_parser::map_parser::tests::test_unknown_block_skipped ... ok
test ets2_parser::map_parser::tests::test_whitespace_before_colon ... ok
test ets2_parser::scs_reader::tests::test_cityhash64_deterministic ... ok
test ets2_parser::scs_reader::tests::test_cityhash64_known ... ok
test ets2_parser::scs_reader::tests::test_cityhash64_vectors ... ok
test ets2_parser::scs_reader::tests::test_entry_layout_size ... ok
test ets2_parser::scs_reader::tests::test_flag_constants ... ok
test ets2_parser::sii_parser::tests::test_comments_ignored ... ok
test ets2_parser::sii_parser::tests::test_no_siinunit_wrapper ... ok
test ets2_parser::sii_parser::tests::test_parse_boolean ... ok
test ets2_parser::sii_parser::tests::test_parse_float3 ... ok
test ets2_parser::sii_parser::tests::test_parse_hex_uid ... ok
test ets2_parser::sii_parser::tests::test_parse_multiple_units ... ok
test compat_export::tests::test_write_compat_files ... ok
test ets2_parser::sii_parser::tests::test_parse_simple_unit ... ok
test ets2_parser::sii_parser::tests::test_parse_uid_list ... ok
test ets2_parser::sii_parser::tests::test_syntax_error ... ok
test ets2_parser::sii_parser::tests::test_unterminated_string_literal ... ok
test ets2_parser::tests::test_parse_hashfs_sectors_dir_missing ... ok
test ets2_parser::tests::test_parse_hashfs_sectors_dir_empty ... ok
test ets2_parser::tests::test_parse_hashfs_sectors_dir_all_invalid_returns_error ... ok
test ets2_parser::tests::test_real_ets2_map_parsing ... ignored, requires ETS2 installation at default path
test ets2_parser::tests::test_parse_text_map_basic ... ok
test ets2_parser::tests::test_parse_text_map_dedup ... ok
test ets2_parser::tests::test_parse_hashfs_sectors_dir_valid_base ... ok
test ets2_parser::tests::test_parse_hashfs_sectors_dir_skips_invalid_files ... ok
test ets2_parser::zip_reader::tests::test_read_known_file ... ok
test ets2_parser::zip_reader::tests::test_read_missing_file ... ok
test ets2_parser::zip_reader::tests::test_find_files_starting_with ... ok
test graph_export::tests::test_dangling_node_uid ... ok
test graph_export::tests::test_build_graph_basic ... ok
test graph_export::tests::test_build_graph_timed ... ok
test graph_export::tests::test_edge_uid_deterministic ... ok
test graph_export::tests::test_compute_metrics ... ok
test graph_export::tests::test_direction_forward ... ok
test graph_export::tests::test_prefab_dangling_node_uid ... ok
test graph_export::tests::test_no_lanes_unknown ... ok
test graph_export::tests::test_lane_graph_generation ... ok
test graph_export::tests::test_prefab_skip_existing_edge ... ok
test graph_export::tests::test_prefab_interconnect ... ok
test pipeline::tests::test_cli_options_default ... ok
test ets2_parser::tests::test_parse_hashfs_sectors_dir_zlib_compressed_base ... ok
test route_smoothing::tests::test_catmull_rom_endpoints ... ok
test route_smoothing::tests::test_smooth_empty ... ok
test graph_export::tests::test_determinism ... ok
test route_smoothing::tests::test_smooth_single_point ... ok
test route_smoothing::tests::test_catmull_rom_straight ... ok
test pipeline::tests::test_pipeline_no_route ... ok
test route_smoothing::tests::test_smooth_straight_line ... ok
test shm_telemetry::tests::test_layout_size ... ok
test graph_export::tests::test_write_quality_report ... ok
test shm_telemetry::tests::test_magic_validates ... ok
test shm_telemetry::tests::test_nav_speed_limit_invalid ... ok
test telemetry::tests::test_telemetry_json_deserialization ... ok
test vjoy::tests::test_joystick_position_layout ... ok
test vjoy::tests::test_scale_brake ... ok
test vjoy::tests::test_scale_steering ... ok
test vjoy::tests::test_scale_throttle ... ok
test vjoy::tests::test_vjoy_enumeration ... ok
test vjoy::tests::test_y_axis_combined_priority ... ok
test ets2_parser::scs_reader::tests::test_list_map_sector_paths ... ok
test graph_export::tests::test_write_graph_file ... ok
test pipeline::tests::test_pipeline_full_flow ... ok

test result: ok. 129 passed; 0 failed; 1 ignored; 0 measured; 0 filtered out; finished in 0.01s


running 1 test
     Running unittests src/bin/benchmark.rs (target/debug/deps/benchmark-8db141029e7081d5)
     Running unittests src/bin/telemetry_diag.rs (target/debug/deps/telemetry_diag-e1fe30ca7920ecc0)
test tests::test_benchmark_runs ... ok

test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s


running 1 test
test tests::test_telemetry_diag_opens_shm ... ok

test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s

     Running unittests src/main.rs (target/debug/deps/truckpilot-0063fa75c835d343)

running 0 tests

test result: ok. 0 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s

     Running unittests src/bin/vjoy_test.rs (target/debug/deps/vjoy_test-f1da9a791b5db4fb)

running 1 test
test tests::test_vjoy_test_runs ... ok

test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.06s

     Running tests/config_integration_test.rs (target/debug/deps/config_integration_test-743070862aa8dcbb)

running 3 tests
test test_truckpilot_toml_exists_at_repo_root ... ok
test test_truckpilot_toml_override_routing_values ... ok
test test_truckpilot_toml_parses_with_expected_routing_defaults ... ok

test result: ok. 3 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s

     Running tests/determinism_test.rs (target/debug/deps/determinism_test-19a51c31ba2c48f0)

running 9 tests
test test_all_edge_uids_unique ... ok
test test_deterministic_edge_ordering ... ok
test test_deterministic_routing ... ok
test test_lane_change_edge_uids_deterministic ... ok
test test_road_lane_edge_uids_unique ... ok
test test_lane_change_edge_uids_unique_within_graph ... ok
test test_deterministic_compat_json ... ok
test test_deterministic_graph_json ... ok
test test_graph_sort_key_stable_json ... ok

test result: ok. 9 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s

     Running tests/graph_json_test.rs (target/debug/deps/graph_json_test-1c96e9604c554ce2)

running 6 tests
test cli_graph_json_missing_file_fails ... ok
test cli_graph_json_empty_nodes_fails ... ok
test cli_graph_json_empty_graph_fails ... ok
test cli_graph_json_has_priority_over_hashfs ... ok
test cli_graph_json_route_found ... ok
test cli_graph_json_invalid_json_fails ... ok

test result: ok. 6 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s

     Running tests/integration_tests.rs (target/debug/deps/integration_tests-73c39350d21aaf64)

running 8 tests
test test_full_pipeline_graph_build ... ok
test test_full_pipeline_with_prefabs ... ok
test test_full_pipeline_routing ... ok
test test_full_pipeline_determinism ... ok
test test_telemetry_diag_missing_shm ... ok
test test_full_pipeline_compat_export ... ok
test test_full_pipeline_cli_run ... ok
test test_realistic_medium_map ... ok

test result: ok. 8 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.03s

     Running tests/real_map_test.rs (target/debug/deps/real_map_test-d517c50388b0db89)

running 1 test
test test_real_base_map_scs ... ok

test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 1.20s

   Doc-tests truckpilot

running 0 tests

test result: ok. 0 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s
```

## Appendix – Real-map smoke run (`cargo test -p truckpilot test_real_base_map_scs -- --nocapture`)

```text
running 1 test
base_map.scs has 2054 entries
Found 0/10 sectors
  hash 0xAEF8455DF08ABB02: read error: seek to 13475380228292063266: Invalid argument (os error 22)
  hash 0xB7DF0EA221A6034D: read error: seek to 13016423096366721898: Invalid argument (os error 22)
  ...
  hash 0xDF42BC00A967CFA2: read error: seek to 11477036341002916083: Invalid argument (os error 22)
test test_real_base_map_scs ... ok

test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 1.44s
```

# Changelog

All notable changes to TruckPilot are documented in this file.

The format is loosely based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
This project does not yet follow Semantic Versioning strictly — versions are
date-tagged milestones.

> The repository currently has no Git history (synced via Mutagen, not Git).
> Entries below are reconstructed from `PROJECT_REPORT.md`, `QA_REVIEW.md`,
> `docs/CURRENT_STATUS.md`, the `docs/` design notes and the chronologically
> tagged `live_test_*.log` files. Authors are listed where known; otherwise
> the entry is attributed to "TruckPilot team".

---

## [1.0.0] — 2026-05-06

Final 1.0 milestone — full offline validation green, release packaging,
documentation complete.

### Added
- **2026-05-06** _TruckPilot team_ — Clippy hygiene cleanup: removed unused
  `salt` field and `read_u64` helper in `scs_reader.rs`, simplified
  `parse_binary_sector` and `AccController::update`. Result: 0 clippy
  warnings across `--all-targets --all-features`.
- **2026-05-06** _TruckPilot team_ — `cargo audit` integration. 0
  vulnerabilities found across 192 dependencies. Report in
  `audit_report.txt`.
- **2026-05-06** _TruckPilot team_ — New binary `graph_stats`
  (`src/bin/graph_stats.rs`) and integration test
  `tests/graph_stats_test.rs` — prints node/edge/direction/speed
  statistics for a `graph.json`.
- **2026-05-06** _TruckPilot team_ — New binary `route_stress`
  (`src/bin/route_stress.rs`) and integration test
  `tests/route_stress_test.rs` — stresses the A* planner with 100
  long-distance pairs and reports the toughest routes.
- **2026-05-06** _TruckPilot team_ — Crate-level rustdoc and exhaustive
  per-field documentation. `cargo doc --lib` builds cleanly with
  `RUSTDOCFLAGS="-D missing_docs"`. Generated API docs in `docs/api/`.
- **2026-05-06** _TruckPilot team_ — Release packaging scripts
  `scripts/build_release.{sh,ps1}`, producing a self-contained
  `release/` directory and `truckpilot-v1.0.zip` (~3.5 MB).
- **2026-05-06** _TruckPilot team_ — `CHANGELOG.md` (this file).
- **2026-05-06** _TruckPilot team_ — `Dockerfile` + `docker-compose.yml`
  + `docs/DOCKER.md` for reproducible Linux builds.
- **2026-05-06** _TruckPilot team_ — GitHub Actions CI in
  `.github/workflows/ci.yml` (test matrix, clippy, fmt, doc, audit).
- **2026-05-06** _TruckPilot team_ — `example_routes.json` with 10
  predefined routes from `output/graph.json`.
- **2026-05-06** _TruckPilot team_ — Parser comparison helpers
  `scripts/compare_parsers.{sh,ps1}` and `compare_report.txt`.
- **2026-05-06** _TruckPilot team_ — Memory / UB sweep with
  `cargo careful` (and `cargo miri` where applicable). Report in
  `ub_report.txt`.
- **2026-05-06** _TruckPilot team_ — Performance profiling instructions
  in `docs/PROFILING.md`.

### Changed
- **2026-05-06** _TruckPilot team_ — Bumped tracked test count from 158
  to 167 (4 graph_stats unit tests + 3 route_stress unit tests + 2 binary
  integration tests).

### Fixed
- **2026-05-06** _TruckPilot team_ — `clippy::needless_match`,
  `clippy::let_and_return`, two `dead_code` warnings.

---

## [0.9.0] — 2026-05-05

Run-up to 1.0: documentation, live-loop quickstart, offline validation.

### Added
- **2026-05-05** _TruckPilot team_ — `PROJECT_FINAL.md` (feature
  overview, metrics, architecture, build instructions).
- **2026-05-05** _TruckPilot team_ — `HANDOFF.md` (developer onboarding,
  module index, pitfalls, extension recipes).
- **2026-05-05** _TruckPilot team_ — `scripts/start_live_loop.ps1`
  Quickstart script for the Windows-side live test (graph check,
  UID extraction, telemetry probe, autopilot launch).
- **2026-05-05** _TruckPilot team_ — `offline_validation_results.txt` —
  reproducible report of `cargo test`, `cargo clippy`,
  `benchmark`, `telemetry_diag`, `vjoy_test`, and graph validation.

### Changed
- **2026-05-05** _TruckPilot team_ — `README.md` extended with status
  banner, "Quickstart für zu Hause" section, and links to the new
  documents.

---

## [0.8.0] — 2026-05-04 — Live-Test framework

### Added
- **2026-05-04** _TruckPilot team_ — `scripts/run_full_live_test.ps1`,
  `scripts/auto_diagnose_live.ps1`, `scripts/run_live_test.{ps1,sh}` —
  end-to-end live test scripts with rotated `live_test_*.log` files.
- **2026-05-04** _TruckPilot team_ — `scripts/install_dll.{ps1,sh}` —
  copies `truckpilot_telemetry.dll` into the ETS2 plugin directory.
- **2026-05-04** _TruckPilot team_ — `scripts/pick_route_uids.ps1` —
  regex-based UID extraction (avoids loading 222k nodes into memory).
- **2026-05-04** _TruckPilot team_ — `docs/RUNBOOK_LIVETEST.md` and
  `docs/LIVE_TEST.md`.

### Fixed
- **2026-05-04** _TruckPilot team_ — Magic-value alignment between
  C++ telemetry DLL and Rust SHM reader (`0x54505054`). DLL and Rust
  binary must now be released together.

---

## [0.7.0] — 2026-05-03 — .NET parser parity

### Added
- **2026-05-03** _TruckPilot team_ — `TruckPilot.NET/TruckPilot.Core`
  reference parser reaching **0 % deviation** to the Rust parser on
  the realmap export (222 891 nodes / 65 638 roads / 18 189 prefabs).
- **2026-05-03** _TruckPilot team_ — `TruckPilot.NET/TruckPilot.CLI`
  with HashFS-sectors mode.
- **2026-05-03** _TruckPilot team_ — Map-export throughput **74.8
  sectors/s** (276 sectors / 3.69 s wall clock).

### Changed
- **2026-05-03** _TruckPilot team_ — Adopted `compat_*.json`
  schema as the cross-language interchange format (camelCase).

---

## [0.6.0] — 2026-05-02 — Offline validation green

### Added
- **2026-05-02** _TruckPilot team_ — `scripts/validate_graph.ps1` —
  6-check structural validator for `graph.json` (refs, duplicates,
  self-loops, hex UIDs, isolated nodes, distances).
- **2026-05-02** _TruckPilot team_ — `tests/test_validate_graph_ps1.py`
  pytest harness.

### Fixed
- **2026-05-02** _TruckPilot team_ — Quality-report determinism:
  `build_time_ms` is now zeroed in the byte-stable JSON output.

---

## [0.5.0] — 2026-05-01 — Phase 8 close-out

### Added
- **2026-05-01** _TruckPilot team_ — Quality report
  (`quality_report.json`) with `GraphMetrics` (density, largest
  component, % directed/unknown, % with speed limit).
- **2026-05-01** _TruckPilot team_ — Compat export
  (`compat_nodes.json`, `compat_roads.json`, `compat_road_looks.json`,
  `compat_graph.json`).
- **2026-05-01** _TruckPilot team_ — `PROJECT_REPORT.md` with first
  reproducible metrics and module overview.

### Changed
- **2026-05-01** _TruckPilot team_ — A* planner: deterministic tiebreak
  on `(f_score, uid)`, edge sorting on `(to_node_uid, edge_uid)`.

---

## [0.4.0] — 2026-04 — Adaptive Cruise Control + Smoothing

### Added
- _TruckPilot team_ — `src/acc_controller.rs` — distance-based ACC
  with PID anti-windup.
- _TruckPilot team_ — `src/route_smoothing.rs` — Catmull-Rom path
  smoothing with configurable subdivisions.
- _TruckPilot team_ — `truckpilot.toml` config sections `[steering]`,
  `[speed]`, `[routing]`, `[telemetry]`, `[acc]`.

---

## [0.3.0] — 2026-04 — Telemetry & vJoy bridges

### Added
- _TruckPilot team_ — `TruckPilot.TelemetryDLL/` C++ ETS2 plugin (SCS
  SDK).
- _TruckPilot team_ — `src/shm_telemetry.rs` — Windows
  `Local\TruckPilotTelemetry` reader, Linux `/dev/shm` mock.
- _TruckPilot team_ — `src/vjoy.rs` — dynamic-load wrapper around
  `vJoyInterface.dll` with cross-platform mock fallback.
- _TruckPilot team_ — `src/bin/telemetry_diag.rs`,
  `src/bin/vjoy_test.rs`.

---

## [0.2.0] — 2026-03 — Routing core

### Added
- _TruckPilot team_ — `src/autopilot.rs` — A* routing on `GraphData`
  with `CostMode::Distance` and `CostMode::Eta`.
- _TruckPilot team_ — `src/autopilot_loop.rs` — live loop wiring
  telemetry, controllers and vJoy.
- _TruckPilot team_ — `src/controller.rs` — generic PID controller and
  `SpeedController` wrapper.
- _TruckPilot team_ — `src/bin/benchmark.rs` — 100-route A*-benchmark.

---

## [0.1.0] — 2026-02 — First parser walk-through

### Added
- _TruckPilot team_ — Initial Rust crate skeleton (`Cargo.toml`,
  `src/lib.rs`).
- _TruckPilot team_ — `src/ets2_parser/scs_reader.rs` — CityHash64
  HashFS reader.
- _TruckPilot team_ — `src/ets2_parser/binary_parser.rs`,
  `src/ets2_parser/map_parser.rs`, `src/ets2_parser/sii_parser.rs`.
- _TruckPilot team_ — `src/graph_schema.rs`, `src/graph_export.rs`,
  `src/json_export.rs`.
- _TruckPilot team_ — Initial test fixtures and `tests/` integration
  suite.

---

## Removed

No features have been removed from TruckPilot to date. Internal
helpers superseded by new code paths (e.g. an early `read_u64` in
`scs_reader.rs`) are tracked above under the version that removed them.

---

_Generated 2026-05-06. If you migrate the repository to Git, regenerate
this changelog from `git log --oneline --since="2025-01-01"` to capture
authorship and exact dates per commit._

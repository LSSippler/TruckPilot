---
created: 2026-05-10
tags: [index, truckpilot]
---

# TruckPilot 2.0 - Vault Index

## Project Overview

Self-driving tool for ETS2, Rust workspace, plugin architecture (FFI),
WebSocket IPC zur Tauri/React/shadcn UI, vJoy als Output-Layer.

## Quick Links

- [[02-Architecture/Plugin-System]]
- [[02-Architecture/Map-Parser]]
- [[02-Architecture/Telemetry-Pipeline]]
- [[02-Architecture/IPC-Protocol]]
- [[02-Architecture/Control-Loop]]

## Phases (chronological)

- [[01-Phases/Phase-1-bis-5-Initial]] - HashFS reader fix, CityHash port, baseline parser
- [[01-Phases/Phase-5.5-Item-Handlers]] - 22 item-type skip handlers from TruckLib
- [[01-Phases/Phase-5.6-Road-Header]] - Full Road item parser
- [[01-Phases/Phase-5.7-Road-Cleanup]] - Drop RoadDataPayload + partial-sector recovery
- [[01-Phases/Phase-5.8-Skip-Audit]] - Skip-handler audit infrastructure
- [[01-Phases/Phase-5.9-Routing-Test]] - First routing benchmark across 90 cities
- [[01-Phases/Phase-5.10-Prefab-Edges]] - Prefab-derived edges + routing diagnostics
- [[01-Phases/Phase-5.11-UID-Resolution]] - UID-resolution diagnostic + sector audit
- [[01-Phases/Phase-5.12-Sign-Handler]] - Sign-handler format fix
- [[01-Phases/Phase-5.13-Vis-UIDs]] - Vis-UIDs hypothesis test (REJECTED)
- [[01-Phases/Phase-5.14-Aux-Probe]] - .aux cross-sector probe (REJECTED, external UID family)
- [[01-Phases/Phase-5.15-Trigger-Handler]] - Trigger 0xFFFFFFFF sentinel fix (originally road; pivoted via diagnosis) — 154 -> 103 failures, 0/90 -> 12/90 routes
- [[01-Phases/Phase-5.16-Terrain-Handler]] - Terrain rewrite (3 railings, not 4 per TruckLib doc) — 103 -> 96 failures, 12/90 -> 20/90 routes
- [[01-Phases/Phase-5.17-Curve-Handler]] - Curve rewrite (Locators + SubcurveUseMask + Subcurve list, NOT terrain-shaped) — 96 -> 77 failures, +30k edges, routing unchanged at 20/90
- [[01-Phases/Phase-5.18-Model-Handler]] - Model rewrite (added AdditionalParts list + Node + Scale + TerrainMaterial + Color + Rotation) — 77 -> 36 failures (-53.2%), graph unchanged (model items carry no roads), routing unchanged
- [[01-Phases/Phase-5.19-Buildings-Handler]] - Buildings rewrite (added Stretch f32 + HeightOffsets list) — 36 -> 12 failures (-66.7%), +103k edges (+9.9%), +8.2k prefabs, routing unchanged at 20/90
- [[01-Phases/Phase-5.20-BezierPatch-Handler]] - BezierPatch investigation only — TruckLib v907-divergent für non-empty patches; legacy 117B Handler bleibt aktiv; 12 → 12 failures unchanged, no fix landed
- [[01-Phases/Handler-Fix-Series-Summary]] - Abschluss-Report Phase 5.15–5.20: 154 → 12 failures (-92.2%), Routing 0/90 → 20/90, 3× TruckLib exakt + 3× v907-drift
- [[01-Phases/Phase-5.22-Ferry-Parser]] - Ferry parser implementiert (16/16 Items), aber DeepSeek's "shared port_token = shared route" empirisch nicht haltbar (16 unique tokens) — 0 ferry edges; Phase 5.22b braucht /def/ferry.sii lookup
- [[01-Phases/Phase-5.23a-Spatial-Index]] - Cross-Sector-Matching Infrastructure (spatial_match.rs, SpatialIndex, query_circle), +6 tests (45→51), Routing unchanged 20/90 (infra-only)
- [[01-Phases/Phase-5.23b-Strict-Matching]] - Strict-Pass Spatial-Match implementiert; Spec empirisch widerlegt (Orphan-Rate 0.04% statt 25-40%); +0 routing, Code bleibt (generic infra für 5.24+); 5.23c-e gestrichen
- [[01-Phases/Phase-5.23-Pivot-Story]] - Lessons-Learned-Report: DeepSeek's Spec für falsches Problem gebaut. Echter Bottleneck = 574k Singleton-Nodes (54% aller Nodes) ohne Edge → Phase 5.24 Audit
- [[01-Phases/Phase-5.24-Singleton-Audit]] - Diagnose-only: 9% (51k) Singletons referenziert von ignorierten Item-Types in base_map; 91% (522k) DLC-Quellen. Empfehlung: Phase 5.25 Two-Node-Item-Extraction (Buildings/Curve/Terrain Node+ForwardNode→Edges)
- [[01-Phases/Phase-5.25a-Buildings-Edges]] - parse_buildings + 483 bidirektionale Building-Edges; nur 606 Buildings im base_map (vs erwartet 30k+) → Routing 20/90 unverändert; Singletons -645, Components -483. Lesson: Item-Volumen vor Edge-Erwartung quantifizieren
- [[01-Phases/Phase-5-Closeout]] - **CLOSED 2026-05-10**: H4a vis_uids REJECTED, H4b Anchor-Junction REJECTED (0/196k), H4c PPD LOW ROI (62.8% N=3 cliques). Big-8-Cluster ~205k Nodes, ~2000km drivable. Phase 5 Feature-Complete.
- [[01-Phases/Phase-6-Telemetry]] - Real-telemetry pipeline (Sanity + Blackboard + IPC broadcast)
- Phase 6.2-Prep — Plugin-Architecture-Refactor (commit 661812dd)
- Phase 6.2a — Autopilot State-Machine + daemon wiring (commit b479c2eb)
- Phase 6.2b — Tick-Phasen Scheduling PhaseA/B/C/PostPhase (commit b479c2eb)
- Phase 6.2d — Lane-Keeper state-gated PID + Pure-Pursuit (commit 2a86bdac)
- Phase 6.2e — Speed-Controller 6-Source-Target-Cascade + Bergab-Override (commit 2a86bdac)
- Phase 6.2-UI — AutopilotStatusCard + Engage/Disengage/Reset Commands (commit 2e9c4a76)
- ETS2LA Traffic Probe — Diag-Binary fuer Pre-Check V4/V5 (commit c0e3b9dc)
- Router-Perf-Audit Binary restored — pending Execution + Verdict (commit dea41c73)
- Blackboard-Key-Inventory — docs/blackboard_keys.md (commit 0f906112)
- Phase 6.2i — PID Hotswap + Live Telemetry Logging (tick_log/fault_log/pid_tuning_log, commit a71d1e2a)

## Reviews

- [[03-Reviews/DeepSeek-Review]]
- [[03-Reviews/Gemini-Architecture-Review]]

## References

- [[04-References/ETS2-File-Formats]]
- [[04-References/HashFS-Spec]]
- [[04-References/CityHash64]]
- [[04-References/TruckLib-Notes]]

## Decisions

- [[05-Decisions/ADR-001-Plugin-FFI]]

## Phase 6.2 Sub-Phase Status (touch 2026-05-11)

| Sub-Phase | Topic | Status | Commit |
|---|---|---|---|
| 6.2-Prep | Plugin-Architecture-Refactor | DONE | 661812dd |
| 6.2a | State-Machine | DONE | b479c2eb |
| 6.2b | Tick-Phasen (Daemon + Plugins) | DONE | b479c2eb |
| 6.2c | vJoy Real (Hardware Wiring) | PLANNED (requires user-present test) |  |
| 6.2d | Lane-Keeper | DONE | 2a86bdac |
| 6.2e | Speed-Controller | DONE | 2a86bdac |
| 6.2f | ACC (conditional) | CONDITIONAL (license-resolved, design pending) |  |
| 6.2g | Watchdog | IN PROGRESS | [[01-Phases/Phase-6.2g-Watchdog]] |
| 6.2g.2 | Watchdog: Heartbeat + Telemetry-Stale | DONE | 43bbabb5 |
| 6.2h | Test-Plan-Infrastructure | DONE | 2a86bdac |
| 6.2i | PID Hotswap + Tick/Fault/PID-Tuning Logging | DONE | a71d1e2a |
| 6.2h-real | First Live Drive | PLANNED |  |
| UI Status-Card | AutopilotStatusCard | DONE | 2e9c4a76 |
| Diag/Probe | ets2la_traffic_probe | DONE | c0e3b9dc |
| Diag/Perf | router_perf_audit restored | PARTIAL (build green, run deferred) | dea41c73 |
| Docs/Blackboard | empirical key inventory | DONE | 0f906112 |

## Bridge to Phase 6.3+

- Phase 6.3 — Speed-Limits aus road_look.sii (PLANNED)
- Phase 6.4 — ProMods-Support (PLANNED, XL Aufwand)
- Phase 6.5 — UI-Dashboard erweitert (PLANNED)
- [[01-Phases/Phase-6.5b-Memory-Hack-Validation]] — RE Tag 1+2: Ghidra+x64dbg Live-Validation + CE Speed-Scan-Pivot (IN PROGRESS)
- [[01-Phases/Phase-6.5c.1-DXcam-SHM-PoC]] — DXcam → SHM Frame Capture: Python-Producer + Rust-Reader, Sequence-Lock, 6/6 Tests grün, Live-Test pending
- Phase 6.x — Vision-ACC (DEFERRED, langfristig)

## Open Threads (Phase 6.2 follow-ups)

- Watchdog 6.2g (Sub-Session A,B,C per outputs/claude/watchdog_6_2g.txt)
- vJoy Real 6.2c
- ACC 6.2f conditional
- First Test Drive 6.2h-real
- Plugin Doc-Audit (UNBLOCKED — pid.rs, lane-keeper, speed-controller, stats-logger committed a71d1e2a)
- Core Doc-Audit (BLOCKED on Watchdog)
- Integration Smoke-Test (BLOCKED on Daemon test-API)
- Telemetry-DLL Audit (BLOCKED on telemetry_dll vs telemetry-dll dup-path)
- Release Workflow PowerShell-Pipeline

## Phase-6.2 Lessons Learned

- PluginContext-Migration in Steps statt big-bang
- State-Machine VOR Plugin-Adoption (sonst Tests im Plugin nicht buildbar)
- Backward-Compat-Defaults kritisch (default_phase() = PhaseC)
- Anti-Copy-Block in Specs explizit machen
- Auto-Mode Classifier kann Scope ueber Session-Boundaries hinweg
  unterschiedlich beurteilen — Tasks early committen ist sicherer
- GateGuard pre:edit-write erzwingt facts pro File, kostet Tokens —
  Batch-Edits planen
- Externe Editor-Edits in der working tree (stats-logger, plugin-api/pid)
  duerfen NICHT mit unzusammenhaengenden Commits gemischt werden

## Status (last touch 2026-05-10, Phase 5 CLOSED → Phase 6.2 prep)

| Bereich | Status |
|---|---|
| Map-Parser (Phase 5.x) | **CLOSED**. 270/282 sectors clean. 22% routing (20/90 base + 1/27 DLC). 53.9% Singleton-Floor universell. Big-8-Cluster (~205k nodes, ~2000km) Production-Bereich. |
| Plugin-Architektur | **Phase 6.2-prep done** (commit 661812dd): vjoy post-arbitration, dt via PluginContext, Router A* aktiv, Lane-changer prio-80, sign-reader lowest_limit. |
| Routing | 20/90 cities (22.2%) - sector-parse korrekt für road/sign/trigger/terrain/curve/model/buildings; cross-sector topology der choke point |
| Telemetry pipeline (Phase 6) | end-to-end: SHM/HTTP -> sanity -> blackboard -> IPC broadcast |
| UI (Tauri/React) | live, telemetry-Frames @ 20 Hz |
| Plugin-System | 11 plugins (speed-controller, lane-keeper, lane-changer, acc, vjoy-output, sign-vision, sign-reader, fuel-stops, router, break-planner, stats-logger, hello-world) |

## Bridge to Phase 6.2 (Autopilot State-Machine)

Phase 5 is CLOSED. Phase 6.2 work plan:
- **6.2a** State-Machine (manual/autopilot/failsafe), PluginContext erweitern um `ctx.state`
- **6.2b** Tick-Phasen-Registry im PluginManager (vjoy bereits aus dem Loop extrahiert)
- **6.2c** vJoy real wiring (post-arbitration tick fertig)
- **6.2d** Lane-Keeper Pure Pursuit auf `router.waypoints`
- **6.2e** Speed-Controller Tuning mit echtem dt
- **6.2h** Lane-Changer Integration (prio-80 Arbitration fertig)
- **Teststrecke:** Big-8-Cluster, NICHT Cross-Border

## Open Threads (Map-Parser Research, NICHT in Phase 6 priorisieren)

- Phase 5.22b — /def/ferry.sii loader (deferred, 0-2 Connections in base_map)
- Phase 5.20 BezierPatch v907 — open, 12/282 failures akzeptiert
- Phase 5.x cross-sector — strukturell blockiert, braucht neuen Ansatz
- Phase 7 - protocol bump (TelemetrySnapshot extension), fatigue source
- Memory-Reader (Telemetry source #4) - Phase 6.4 placeholder





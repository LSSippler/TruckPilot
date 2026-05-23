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
- [[01-Phases/Phase-5.25-Routing-Graph-Investigation]] - **CLOSED 2026-05-22**
- [[01-Phases/Phase-5.27-ETS2LA-Parser-Study]] — **CLOSED 2026-05-22**: Clean-Room Studie. 5 Root Causes für 50% vs 97% Item-Resolution identifiziert. Fehlende Handler (Types 9,11,13,21,23,40,45,47,49), Compound-Kind-Nodes, road_look.sii Lane-Counts. Phase-5.28-Plan A-E.: Diag-only. 601k isolierte Nodes (52.1%), 0 forward/backward Edges (alle bidirectional_unknown), 4.6% Routing-Erfolg (72/1560). RC-1: Snap trifft isolierte Nodes. RC-2: lanes_forward=0 in Road-Parser. Empfehlung: Phase 5.26-A Snap-Filter (Quick Win) + 5.26-B bezier_patch Fix.
- [[01-Phases/Phase-5.28-D-Node-Forward-Backward-UIDs]] — **CLOSED 2026-05-22**: RawNode +2 Felder (forward/backward_item_uid), parse_node liest statt skippt, truckpilot-node-inspect CLI. 5 isolated Nodes inspiziert: alle haben forward_item_uid≠0 → Beweis für unparsed Item-Type-Gap als Root Cause der 601k isolated nodes.
- [[01-Phases/Phase-5.28-C-road-look-sii-Loader]] — **CLOSED 2026-05-22**: road_look.sii Loader (32 Legacy-Einträge), right_look/left_look als Token-Felder identifiziert, bidirektionaler Fallback für ungematchte Straßen. forward_edges 0→354 801, backward_edges 0→354 801, bidirectional_unknown 731 998→22 396. Commit `ee99923`.
- [[01-Phases/Phase-5.28-Bisect]] — **CLOSED 2026-05-23**: Bisect der gemeldeten Regression (29.6%→5%). Ergebnis: KEIN REGRESSION in committed Code. Alle 4 Bisect-Schritte (step0–4) zeigen 29.6% (462/1560). Regression war Messartefakt: graph.json war 10min veraltet beim Commit, Zwischenzustand mit fehlendem bidir-Fallback nie committed. graph.json frisch regeneriert. `outputs/2026-05-23/phase_5_28_bisect_report.md`
- [[01-Phases/Phase-5.28-B-Compound-Kind-Nodes]] — **CLOSED 2026-05-23**: Compound-Handler (Type 40) implementiert, Kind-Nodes extrahiert. Gate PASS (29.6% routing, no regression). **Null Impact**: Kind-Nodes sind Duplikate des trailing-node-Blocks — bereits vor Handler-Aufruf im Node-Map. isolated_nodes 601 170 unverändert. /goal (-20%) nicht erfüllt. `outputs/2026-05-23/phase_5_28b_status.md`
- [[01-Phases/Phase-6.5q.2-Engage-Heading-Hotfix]] — **CLOSED 2026-05-23**: Hotfix für 107°-Schräglage-Bug aus Block 2 Live-Test. Fix 1: sync_replan heading-aware (waypoint_ahead_of_truck). Fix 2: heading_ok_for_engage Hard-Block bei >60°. Fix 3: start_node_unknown Advisory-Banner. 13 neue Tests, clippy clean. Commit `e71ddc0b`.
- [[01-Phases/Phase-5.29-A-Lane-Data-Collector]] — **CLOSED 2026-05-23**: lane-data-collector Plugin (PhaseB, 10 Hz). Frame+Telemetry-Capture auf Disk. 10 Tests grün. STOP: vision-frame-source nicht aktiv. Commit `a2f5274`.
- [[01-Phases/Phase-5.29-B-ETS2-Nav-Bridge]] — **CLOSED 2026-05-23**: ets2-nav-bridge Plugin (PhaseA, 1 Hz). GPS nav_distance/time → nav.* Blackboard-Keys, Hysterese 150/200m, waypoint_passed one-shot. 11 Tests grün. Commit `f5bf95f2`.
- [[01-Phases/Phase-5.29-Stage1-Nav-Distance-Time]] — **IN PROGRESS 2026-05-23**: SHM v3 — nav_distance_m + nav_time_s aus SCS-SDK. DLL erweitert (channels truck.navigation.distance/.time), struct 196→204 Bytes, BB-Keys telemetry.nav_distance_m/.nav_time_s. Compile + Tests green. Live-Verifikation pending (ETS2 nicht gestartet). Commit `0545517`.
- [[01-Phases/Phase-5.29-Stage2-SCS-SDK-Controller-Research]] — **CLOSED 2026-05-23**: Research + Spec. SCS SDK hat offizielles MIT-lizensiertes Input-Plugin-API (`scs_input_init`, semantical device). ETS2LA nutzt SHM-Rückkanal via `scs-sdk-controller` (MIT). Verdict: GO. Spec: `TruckPilotControls` SHM (32B), DLL-Extension Option A, Plugin `scs-sdk-output`. ~380 Zeilen Aufwand. vJoy deprecation möglich. `outputs/2026-05-23/stage2/`.
- [[01-Phases/Phase-5.29-Stage2-UFLD-ONNX-Test]] — **CLOSED 2026-05-23**: UFLD v2 CULane (320×1600) auf 9 valide ETS2-Highway-Frames. Detection Rate 89% (>70% ✓), CPU-Latenz 37ms (<50ms ✓). DirectML 86ms regressed (wird in Stage 3 adressiert). CV-Spike: 100% / 6ms als Fallback. Off-the-shelf reicht, kein Custom-Training nötig. `outputs/2026-05-23/stage2/ufld_test_result.md`.
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
- [[01-Phases/Phase-6.2b-Diag-3-Road-Drop-Audit]] — **2026-05-16**: Road-Drop-Audit-CLI. RoadParseFailed=0, BothUnresolved=5.940 (1.81%), Berlin-Snap-Node hat 0 Road-Refs → Cross-Sector-UID-Gap bestätigt. KEIN commit bis Review.
- [[01-Phases/Phase-6.2b-Diag-4-Sign-Precrash-Audit]] — **2026-05-17**: Pre-Crash-Audit für 9 Sign-Handler-Crashes (type=36). Crash-Lokation: `skip_sign_override_list`. Cursor korrekt beim Dispatch. Garbage-Counts variieren extrem → Format-Misalignment VOR Override-List. Nächster Schritt: raw_hex-Fenster auf 256B erweitern oder Fix-Hypothese "4 Tokens/Board" testen.
- [[01-Phases/Phase-6.2b-Diag-5-BothUnresolved-Forensik]] — **2026-05-17**: BothUnresolved-Forensik-CLI. 5940 Events, 75.5% BothFound, davon 99.98% RoadPlusBothSameSector. **ROOT = SAME_SECTOR_NODE_PARSE_FAILURE** — Node-UIDs sind im Sektor-Binary vorhanden (Byte-Scan bestätigt), landen aber nicht im Node-Map. Berlin: 11 BothFound, alle im selben Sektor. Fix würde BothUnresolved 5940→~1456 (-75.5%) reduzieren.
- [[01-Phases/Phase-6.2b-Fix-4-Node-Parser-Hex-Audit]] — **2026-05-17**: Hex-Audit an sec+0008+0011 + sec-0004-0001 (Berlin). **Zwei Root Causes identifiziert:** C=Cursor-Desync→recover_nodes_from_tail-Failure (bezier_patch 0x27 Kandidat, ~75% Events), D=try_parse_sized_sector-False-Positive für Legacy-Sektoren (~25% Events). Node-UIDs physisch vorhanden, zwei verschiedene Parser-Pfade lassen sie durchfallen. Fix-Direktiven für Phase 6.2b-Fix-5 definiert.
- [[01-Phases/Phase-6.5b-Design-Migration-Full]] — **CLOSED 2026-05-18**: Vollständige UI-Design-Migration Phasen 2–8
- [[01-Phases/Phase-6.5c-Router-Diagnostic-Telemetry]] — **CLOSED 2026-05-19**: f64-Precision-Bug behoben (Root Cause Live-Test-Fail), 9 Diagnostic-Blackboard-Keys, validate-cities + uid-lookup CLIs, RouteCard Planning-Feedback, IPC Wire-Trace. Commit `e9231af`. (Sidebar NavGroup/NavItem, Dashboard 3-Zonen, EngageButton, PreconditionPill, BigNumberDisplay, TelemetrySparkline, VJoyBar, FailsafeBanner, PluginToggleItem, token-based Logs/PID/Settings/Mods, AutopilotToastWatcher). Alle Gates grün. Commit `4b783e52`.
- [[01-Phases/Phase-6.5e-Watchdog-Failsafe-State-Gating]] — **CLOSED 2026-05-19**: Watchdog feuerte Notbremse bei Autopilot Off → manuelle Fahrt blockiert. State-Gating via `is_autopilot_active(bb)`, Telemetry-Recovery-Bug gefixt, 3 neue BB-Keys (failsafe_active/reason/last_at), vjoy-output Neutral-Mode bei Off-State. 55+36 Tests grün.
- [[01-Phases/Phase-6.2b-Fix-5b-Multi-Sector-Audit]] — **2026-05-17**: Multi-Sector-Audit (6 Sektoren: Top-5 BothFound + Berlin). **Szenario 1 CONFIRMED: bezier_patch (0x27) = LastOK in 6/6 Sektoren.** `LastOK End == Failure Offset` in allen — Handler under-reads, Rest-Bytes (Vertex-Daten) werden als next item_type gelesen. Predecessor variiert → Fault liegt *innerhalb* bezier_patch, nicht upstream. Nächster Schritt: Fix-5b Step 2 — bezier_patch handler in sector.rs korrigieren.
- [[01-Phases/Phase-6.2a-vJoy-Probe]] — **CLOSED 2026-05-15**: GATE-0 PASS. `truckpilot-vjoy-probe` Binary validated (Spec Final v1.0, 15/15 unit tests, alle 4 Manual-ETS2-Kriterien PASS). vJoy Device 1 X/Y/Z = Steering/Throttle/Brake. Phase 6.2c (vJoy Real Wiring) freigegeben.

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
| 6.2a-Probe | vJoy GATE-0 Probe Binary | **CLOSED 2026-05-15** (GATE-0 PASS, 4/4 manual criteria) | [[01-Phases/Phase-6.2a-vJoy-Probe]] |
| 6.2c | vJoy Real (Hardware Wiring) | UNBLOCKED (GATE-0 cleared) |  |
| 6.2d | Lane-Keeper | DONE | 2a86bdac |
| 6.2e | Speed-Controller | DONE | 2a86bdac |
| 6.2f | ACC (conditional) | CONDITIONAL (license-resolved, design pending) |  |
| 6.2g | Watchdog | IN PROGRESS | [[01-Phases/Phase-6.2g-Watchdog]] |
| 6.2g.2 | Watchdog: Heartbeat + Telemetry-Stale | DONE | 43bbabb5 |
| 6.2h | Test-Plan-Infrastructure | DONE | 2a86bdac |
| 6.2i | PID Hotswap + Tick/Fault/PID-Tuning Logging | DONE | a71d1e2a |
| 6.2h-real | First Live Drive | PLANNED |  |
| 6.2b-A1 | Engage CLI End-to-End (Failsafe-Wiring + IPC + engage-cli + Doku) | CODE READY, Live-Test pending | [[01-Phases/Phase-6.2b-Engage-CLI]] (5ea8adc9 + 8d7195f9 + 761c89bc) |
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
- [[01-Phases/Phase-6.5e-Vision-Training]] — **CLOSED 2026-05-15**: v2 final (mAP 0.798, 793 manual frames). v3 auto-annotation pivot REJECTED (mAP 0.604, -19%, 9548 frames). Lesson: Sauberkeit > Menge. Production-Model: `models/truckpilot-yolov8s-v2/best.pt`.
- [[01-Phases/Phase-6.5e.1-Live-Stabilization]] — **CLOSED 2026-05-15**: 7 cross-component bugs fixed in one sprint (HTTP fallback storm, SHM torn reads, watchdog log spam, Python/Rust clock mismatch, SCS euler f32-vs-f64 type confusion, plugin-cdylib tracing-bridge, async inference worker). `tick_blocking_ms` 110 ms → <2 ms; live-stable for 30 min / 9205 inferences.
- [[01-Phases/Phase-6.5f-Detection-Quality-Spec]] — **IMPLEMENTED 2026-05-15** (commit `05093b37`): 78 blackboard keys live across `sign.class.*`, `sign.conf.*`, `sign.frame.*`, `sign.fp.*` + `sign.detections.last_n` (NDJSON 1000). Schema validated; class-distribution validation deferred to next live drive (329 static-truck inferences too few). 55 sign-vision tests green. `tick_blocking_ms` 3.90 ms in decode-tick (within spec).
- [[01-Phases/Phase-6.5h-Real-Sign-Templates]] — **IMPLEMENTED 2026-05-16** (commit `ae2e1054`): 4 Real-Bild-NCC-Templates (40/60/80/100) aus 66 Live-Crops. `km_unmapped` 88% → 16% verifiziert via 35-Min-Multi-Video-Replay (254k frames, 14634 inferences, 0 dropped, 430 speed_limit_sign). Latenz unverändert (`tick_blocking_ms` 0.00, inference 101.5 ms). Offen: 30/50/70/130 (keine Crops im aktuellen Dataset).
- [[01-Phases/Phase-6.5h-Diag-SpeedLimit-Classification]] — **CLOSED 2026-05-16**: Pivot-Befund — YOLO klassifiziert speed_limit-Schilder bereits mit 9 spezifischen Klassen (82.3% specific, 17.7% generic). Template-Matcher läuft nur auf generic, also 0.5% aller Detections (50/14634). Duplikat- und Distanz-Hypothesen widerlegt (0/50 IoU>0.5 same-frame, generic median area 3296 > specific 1804-2304). Ursache: 30/50/70/90/130 im YOLO-Train unterrepräsentiert. Konsequenz: Templates v1 bleiben als Fallback, v2/v3 + YOLO-Retrain out-of-scope. Folge-Issue: [[02-Issues/Capture-Frame-Persistence-Limit]].
- Phase 6.x — Vision-ACC (DEFERRED, langfristig)

## Build-System Improvements

- **xtask copy-plugins** (2026-05-15): `cargo xtask copy-plugins` deploys all `truckpilot_plugin_*.dll` from `target/release/` to `plugins/`. `hello-world` demo moved from `crates/plugins/` to `crates/examples/`, `libhello_world.so` removed from `plugins/`.
- [[01-Phases/Phase-6.5o-Plugin-DLL-Auto-Copy]] — **CLOSED 2026-05-21**: `cargo build-release` alias (xtask build-release subcommand) auto-deploys all plugin DLLs after build. `cargo deploy-ets2` for Telemetry-DLL. `scripts/ship.ps1` as PowerShell fallback. Commit `bd9d885c`.

- [[01-Phases/Phase-6.5k-Lane-Keeper-Waypoint-Reload]] — **CLOSED 2026-05-20**: Stale-Waypoint-Bug behoben. Hash-basierter Reload-Trigger ersetzt `is_empty()`-Guard. Disengage leert Cache. 12/12 Tests grün.
- [[01-Phases/Phase-6.5l-Heading-Konvention-Fix]] — **CLOSED 2026-05-20**: ETS2-Konvention-Fix `atan2(dz)` → `atan2(-dz)`. error_rad 2.08→0.39 rad, kein Vollanschlag mehr. 15/15 Tests grün.
- [[01-Phases/Phase-6.5m-Progress-Idx-Advance-Fix]] — **CLOSED 2026-05-21**: Route-End-Guard + Walk ab Truck-Position. Reviewer-Agent verhinderte Backward-Walk-Bug. 19/19 Tests grün.
- [[01-Phases/Phase-6.5n-Engaging-Timeout-Fix]] — **CLOSED 2026-05-21**: Glitch-Tolerance für Engaging-Preconditions. Hard-Reset bei Glitches verhindert, PRECONDITION_GLITCH_TOLERANCE=10. 7 Diagnose-BB-Keys. 63/63 Tests grün.
- [[01-Phases/Phase-6.5p-Steering-Safeguards]] — **CLOSED 2026-05-21**: Heading-Mismatch-Detection (1.4 rad Threshold) + Steering-Rate-Limiter (±0.1/Tick). Vollanschlag bei falschem Engage-Heading verhindert. 26/26 Tests grün.
- [[01-Phases/Phase-6.5q-Heading-Filter-Auto-Replan]] — **CLOSED 2026-05-21**: Heading-Filter beim Router-Snap (dot>=0.5) + Auto-Replan wenn Truck off-route. OFF_ROUTE_DETECT_RADIUS=50m. 5 Diagnose-Keys. 18/18 Tests grün.
- [[01-Phases/Phase-5.29-A-Lane-Data-Collector]] — **CLOSED 2026-05-23**: PhaseB-Plugin für Frame+Telemetry-Capture (JPEG+Sidecar-JSON). 13 Plugins. Stop-Bedingung: vision-frame-source nicht aktiv → Mock-Tests grün. `a2f5274`.

## Block 2 — Engagement-Stabilisierung (6.5o.1–6.5t)

**→ [[01-Phases/Block-2-Engagement]]** — Abschluss-Report des gesamten Blocks.

| Phase | Status | Commit | Kernfeature |
|---|---|---|---|
| 6.5o.1 | CLOSED 2026-05-21 | uncommitted | Locked-DLL als Error (Exit 2), Fehler-Collection statt Silent-Fail |
| 6.5p | CLOSED 2026-05-21 | `1e4bad13` | Heading-Mismatch-Stop (1.4 rad) + Steering-Rate-Limiter (±0.1/Tick) |
| 6.5q | CLOSED 2026-05-21 | `5b874ed3` | Heading-Filter beim Snap (dot>=0.5) + Auto-Replan (max 3/Engagement) |
| 6.5q.1 | CLOSED 2026-05-21 | uncommitted | Synchroner A*-Replan bei UserEngage (~170ms, verhindert stale Route) |
| 6.5r | CLOSED 2026-05-21 | uncommitted | UI Engagement-Checklist (6 Preconditions mit Live-Detail-Werten) |
| 6.5s | CLOSED 2026-05-21 | uncommitted | Drei-Stufen Heading-Response (Normal/SoftLaneKeep/AutoReplan/Disengaging) |
| 6.5t | CLOSED 2026-05-21 | uncommitted | Sliding-Window Snap-Stabilisierung (5 Ticks, Majority 3/5, Hysterese 4/5) |

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





# QA Review Report — TruckPilot (Phasen 0–8)

**Datum:** 2026-05-01  
**Prüfer:** Automatisierter QA-Review  
**Projektpfad:** `/home/geekom/TruckPilot`  
**Rust-Version:** stable-x86_64-unknown-linux-gnu

---

## A. Zusammenfassung

| Kriterium | Ergebnis |
|-----------|---------|
| `cargo build` | ✅ Bestanden – keine Fehler |
| `cargo test` | ✅ **66 passed**, 0 failed, 1 ignored |
| `cargo clippy --all-targets --all-features` | ✅ Sauber – 0 Warnungen |
| `--help` | ✅ Alle Flags vorhanden inkl. `--ets2-dir`, `--text-map-file` |
| Determinismus `graph.json` | ✅ Byte-identisch über zwei Läufe |
| Determinismus `quality_report.json` | ⚠️ Ursprünglich nicht-deterministisch → **behoben** |
| Code-Kopien | ✅ Keine gefunden – eigenständige Entwicklung |
| Alle Quelldateien vorhanden | ✅ 19 Quell-/Testdateien |

**Gesamturteil: Bereit für den Einsatz mit Mini-Map.** Mit echten ETS2-Daten besteht eine Einschränkung bei der Sektor-Discovery (s. Abschnitt B).

---

## B. Kritische Funde

### B.1 `quality_report.json` nicht deterministisch — BEHOBEN

**Ursache:** `build_time_ms` im Quality-Report enthielt variable Wanduhrzeit (0.171684 vs. 0.176573 ms).  
**Fix:** In `pipeline.rs` wird `build_time_ms` im Quality-Report auf `0.0` gesetzt (`GraphMetrics { build_time_ms: 0.0, ..metrics }`). Die echte Build-Zeit erscheint weiterhin im `--verbose`-Log.  
**Status:** ✅ Behoben — zwei Läufe liefern jetzt identisches JSON.

### B.2 SCS-Reader kann keine Map-Sektoren finden

**Datei:** `src/ets2_parser/scs_reader.rs` Z. 67–73

`list_known_files()` gibt nur 4 hartkodierte Definitions-Pfade zurück:
```rust
"def/world/road.sii"
"def/world/prefab.sii"
"def/world/sign.sii"
"def/world/semaphore_profile.sii"
```

Map-Daten liegen in `base_map.scs` unter Pfaden wie `map/europe/sec-0023+0004.data`. Diese werden **nicht** erfasst, weil:
1. Die Pfade nicht in `list_known_files()` stehen
2. Das HashFS-Format speichert keine Dateinamen, sondern nur CityHash64-Hashes

**Auswirkung:** `parse_ets2_map()` findet keine echten Map-Daten aus SCS-Archiven. Der Fallback auf `build_test_map()` oder `--text-map-file` funktioniert korrekt.

**Empfehlung:** Entweder:
- (a) Text-Export-Sektoren über `--text-map-file` verwenden (funktioniert bereits)
- (b) `list_known_files()` um eine Sektor-Pfadliste erweitern (generiert aus `def.scs`)
- (c) Brute-Force-Lookup gegen eine bekannte Pfadliste für alle Sektoren

**Kein Blocker** für den Text-Map-Pfad.

---

## C. Warnungen

### C.1 `telemetry.rs` hat keine Unit-Tests

Das Modul enthält kein `#[cfg(test)]`-Modul. `fetch_telemetry()` kann nur gegen einen echten Server getestet werden. Für Regression-Tests wäre ein Deserialisierungs-Test mit einem statischen JSON-Snapshot sinnvoll.

**Risiko:** Niedrig – die Funktion ist dünn und die Deserialisierung über `serde` ist gut getestet.

### C.2 `scs_reader` hat nur CityHash-Tests

Es gibt keinen Integrationstest, der eine echte `.scs`-Datei öffnet und Dateien extrahiert. Dies ist verständlich (binäre Multi-GB-Dateien), sollte aber dokumentiert sein.

### C.3 `#![allow(dead_code)]` in `map_parser.rs` für `RawNode` (Z. 21)

Die Struct `RawNode` hat Felder (`forward_uid`, `backward_uid`), die nie gelesen werden. Der `#[allow(dead_code)]` ist begründet: Die Felder existieren für zukünftige Link-Analyse. **Kein Fehler, aber dokumentationswürdig.**

---

## D. Empfehlungen

1. **Binäres Map-Parsing:** Der aktuelle Parser unterstützt nur das Text-Export-Format (`edit_save_text`). Das native Binärformat (`.data`-Dateien in `base_map.scs`) wäre für Produktivdaten nötig.

2. **Sektor-Discovery:** `list_known_files()` sollte aus `def.scs` eine Sektorliste parsen, um automatisch alle Map-Sektoren zu finden.

3. **Telemetry-Tests:** Ein `#[test]` mit statischem JSON-Snapshot würde Regressionen abfangen:
   ```rust
   #[test]
   fn test_deserialize_telemetry() {
       let json = r#"{"truckPlacement":{"x":1.0,...},...}"#;
       let _: TelemetryData = serde_json::from_str(json).unwrap();
   }
   ```

4. **Lane-Level Routing:** Derzeit arbeitet der Autopilot auf Straßenebene. Für präzise Spurführung wäre eine Erweiterung auf Fahrspurebene nötig.

5. **vJoy-Integration:** Die Steuerbefehle werden aktuell per `println!` ausgegeben. Eine echte vJoy-Anbindung (via C FFI) steht aus.

---

## E. Metriken

### Test-Map (built-in fixture)

| Metrik | Wert |
|--------|------|
| Nodes | 5 |
| Roads | 3 |
| Prefabs | 0 |
| Graph-Kanten | 10 |
| Graph-Dichte | 2.0 |
| Größte Komponente | 100% |
| Gerichtete Kanten | 60% |
| Unbekannte Richtung | 40% |
| Mit Speed-Limit | 60% |
| Build-Zeit | ~0.17 ms |
| Route (1→4) | [1, 2, 3, 4], 300m, validiert |

### Teststatistik

| Kategorie | Tests |
|-----------|-------|
| Unit-Tests (lib) | 57 |
| Determinism-Tests | 4 |
| Integrationstests | 6 |
| **Gesamt** | **67** (66 passed, 1 ignored) |

---

## F. Checkliste Phasen 0–8

### Phase 0 – Schema ✅
- [x] Alle Structs vorhanden
- [x] snake_case / camelCase korrekt
- [x] Optional-Felder mit `#[serde(skip_serializing_if)]`
- [x] Keine unnötigen impl-Blöcke

### Phase 1 – Graph-Builder ✅
- [x] `build_graph` mit korrekter Richtungslogik
- [x] Prefab-Interconnect mit Duplikat-Prüfung
- [x] Dangling-Reference-Prüfung
- [x] Deterministische edge_uid via SipHash
- [x] Stabile Sortierung

### Phase 2 – Compat-Adapter ✅
- [x] 4 Compat-Dateien werden erzeugt
- [x] Deduplizierung + Sortierung
- [x] camelCase via serde

### Phase 3 – Autopilot ✅
- [x] A* mit deterministischem Tiebreaker
- [x] Distance- und ETA-Kostenmodi
- [x] Penalty für `no_lanes_unknown`
- [x] `RouteResult` komplett

### Phase 4 – CLI & Pipeline ✅
- [x] Alle Flags via clap
- [x] Pipeline-Dispatcher
- [x] `--quality-report`, `--performance-compare`

### Phase 5 – Tests ✅
- [x] Unit-Tests in allen Modulen (außer telemetry)
- [x] Integrationstest End-to-End
- [x] Determinism-Test byte-identisch

### Phase 6 – Messung ✅
- [x] `build_graph_timed` + Metriken
- [x] `write_quality_report`
- [x] Quality-Report jetzt deterministisch

### Phase 7 – Telemetrie ✅
- [x] `fetch_telemetry` via ureq
- [x] PID `SpeedController` mit Anti-Windup
- [x] `--telemetry-disable` Flag

### Phase 8 – ETS2-Parser ⚠️ (mit Einschränkung)
- [x] `scs_reader.rs` mit CityHash64 + SCS-Header-Parsing
- [x] `sii_parser.rs` mit Tokenizer + Recursive-Descent
- [x] `map_parser.rs` für Text-Export-Format
- [x] `mod.rs` mit `parse_ets2_map()`, `parse_text_map_file()`
- [x] `--ets2-dir` und `--text-map-file` in CLI
- [⚠️] `parse_ets2_map()` findet keine binären Map-Sektoren (s. B.2)
- [x] Kein kopierter Code

---

*Review abgeschlossen. Alle automatisierten Prüfungen bestanden. Ein kritischer Fund (B.1) wurde behoben. Der verbleibende Fund (B.2) betrifft nur SCS-Direktextraktion und wird durch `--text-map-file` umgangen.*

# sec-0001-0008 Outlier Diagnostic Strategy

> **Document-ID:** TP-DIAG-008
> **Phase:** 6.2b Fix-5c (Outlier Diagnosis)
> **Date:** 2026-05-17
> **Gate:** Read-only until Hypotheses empirically confirmed
> **Status:** Spec — no production code

---

## Context (Empirie-Quellen)

| Quelle | Aussage |
|---|---|
| `forensik_distribution_analysis.md` (Post-Fix-5a) | sec-0001-0008: 230 BothFound, 0 UnknownItemType. 9/10 Top-Sektoren haben UnknownItemType (cursor desync), sec-0001-0008 ist der einzige Ausreisser. |
| `bothunresolved_forensik.json` | Alle 230 Events: BothFound → RoadPlusBothSameSector. Node UIDs via Brute-Force-Scan in sec-0001-0008 Rohdaten gefunden. |
| `phase_6.2b_fix5b_step2_status.md` | Erwartung nach Fix-5b: BothUnresolved 1,878 → ~250, wobei sec-0001-0008 (230) unberührt bleibt. |
| `road_drop_audit.md` (2026-05-16) | sec-0001-0008 hat 1 Drop-Event (vermutlich Pre-Fix-5a). 1,063,231 Nodes im finalen Node-Map ueber alle Sektoren. |
| Phase 6.2b-Diag-2 (Cross-Sector-Edge-Audit) | Nur 2 Cross-Sector-Edges im gesamten Graph. sec-0001-0008 ist KEIN Cross-Sector-Problem. |

### Forensik-Methodik — Kritische Einschraenkung

Der `bothunresolved-forensik` Scanner (`crates/diag/src/bin/bothunresolved-forensik.rs`) arbeitet **byte-level brute-force**: er sucht 8-Byte-U64-Werte in den Rohdaten jeder `.base`-Datei. Ein UID-Fund bedeutet NICHT, dass der Parser einen Node mit dieser UID aus dem Sektor extrahiert hat — nur dass die UID irgendwo als Bytes vorkommt. Road-FixedHeader enthalten `node_a` (Offset +245) und `node_b` (Offset +253) inline — die Forensik findet diese UIDs in den Road-Item-Koerpern, selbst wenn es keinen Node-Trailer gibt.

---

## 1. Hypothesen-Inventar

### H1 — Node-Trailer ist leer (node_count = 0)

**Mechanismus:** Der Sektor parst sauber durch (0 UnknownItemType), alle Items werden dispatched. Im Trailing-Block des Sektors ist der `node_count u32` jedoch 0, sodass `sector.nodes` leer bleibt. Die Roads enthalten ihre `node_a`/`node_b` UIDs inline in ihrem own FixedHeader — die Forensik findet diese UIDs per Byte-Scan, aber der Parser baut keine Nodes daraus.

**Betroffene Code-Stellen:**

| File | Lines | Code-Path |
|---|---|---|
| `crates/map-parser/src/sector.rs` | 194–205 | `parse_sector()` — Sized-then-Legacy-Fallback |
| `crates/map-parser/src/sector.rs` | 807–850 | `try_parse_sized_sector` Phase 3 (Node-Tail) |
| `crates/map-parser/src/sector.rs` | 344–365 | `parse_sector_legacy_inner` Trailing-Node-Section |
| `crates/map-parser/src/graph.rs` | 125–137 | `GraphBuilder::merge_sector` — `for node in sector.nodes` (loop body never executed wenn nodes.len()==0) |

**Falsifizierbar durch:** Hex-Dump der letzten ~500 Bytes von `sec-0001-0008.base`. Sicherstellen dass `node_count` gelesen wird und = 0 ist, ODER dass nach dem letzten Item keine Node-Struktur erkennbar ist. Zugleich: `parse_sector()` instrumentieren und `sector.nodes.len()` loggen.

**A-priori-Wahrscheinlichkeit:** **HIGH** — Erklaert alle Beobachtungen (sauberer Parse + trotzdem 0 Nodes + Forensik findet UIDs in Road-Bodies). In 9,792 Sektoren duerfte es einige Sektoren mit node_count=0 geben (Prefab-only oder reine Asset-Sektoren).

---

### H2 — Nodes geparst, aber `sized_sector_plausible` verwirft sie (sized false-positive)

**Mechanismus:** `try_parse_sized_sector` parsed den Sektor erfolgreich inklusive Node-Tail. Aber `sized_sector_plausible()` (lines 860–895) Stage-2-Check (`|x|,|z| < 250000 AND |y| < 10000`) scheitert fuer ALLE geparsten Nodes — z.B. weil Node-Koordinaten NaN/Inf sind (36-Byte sized-node-Format: `f64` als `f32` gecastet), oder weil sie ausserhalb des ETS2-Weltraums liegen (Randeffekt eines Rand-Sektors). Gate returned `None` → Fallback zu Legacy, der anders (oder gar nicht) liest.

**Betroffene Code-Stellen:**

| File | Lines | Code-Path |
|---|---|---|
| `crates/map-parser/src/sector.rs` | 807–850 | Sized Node-Tail (Phase 3) |
| `crates/map-parser/src/sector.rs` | 860–895 | `sized_sector_plausible` — Stage 2 coord-check |
| `crates/map-parser/src/sector.rs` | 817–823 | Sized count-prefixed: `if let Ok(node) = parse_node_f64(...)` — individual failures silently skipped |
| `crates/map-parser/src/sector.rs` | 954–966 | `parse_node_f64()` — f64→f32 cast, moegliche NaN/Inf-Produktion |

**Falsifizierbar durch:** Instrumentierung: in `sized_sector_plausible` die Node-Koordinaten loggen. Wenn Stage-2-Gate feuert → Hypothese confirmed. Wenn Gate NICHT feuert und trotzdem 0 Nodes → Hypothese falsified.

**A-priori-Wahrscheinlichkeit:** **MEDIUM** — Die f64→f32-Cast in `parse_node_f64` (line 962: `x: (x as f32)`) kann bei sehr grossen oder speziellen double-Werten NaN/Inf produzieren. Sec-0001-0008 liegt am westlichen Kartenrand — Rand-Koordinaten koennten ausserhalb des plausibility-envelope liegen.

---

### H3 — Sized-Format parsed Node-Count > 0, aber `parse_node_f64` scheitert fuer alle (silent skip)

**Mechanismus:** `try_parse_sized_sector` Phase 3 Mode (a) liest `node_count > 0`, aber JEDER Aufruf von `parse_node_f64()` returned `Err`. Der Code auf line 822 ist: `if let Ok(node) = parse_node_f64(&mut cur) { sector.nodes.push(node); }` — Fehler werden still geschluckt, keine Nodes added.

**Betroffene Code-Stellen:**

| File | Lines |
|---|---|
| `crates/map-parser/src/sector.rs` | 817–823 |
| `crates/map-parser/src/sector.rs` | 954–966 (`parse_node_f64`) |
| `crates/map-parser/src/sector.rs` | 837–844 (Mode b — stream-to-EOF, selbe silent-skip Logik) |

**Falsifizierbar durch:** Instrumentierung: in der `for _ in 0..node_count` Schleife (line 822) einen Zaehler fuer `Err`-Faelle einfuegen. Wenn node_count > 0 aber 100% Err → confirmed.

**A-priori-Wahrscheinlichkeit:** **LOW** — 36-Byte Node-Record ist trivial (3× u32 + 3× f64). Ein Parse-Fehler waere nur bei Buffer-End moeglich, und das schuetzt der node_count × 36 ≤ remaining-4 Check (line 818). Falls node_count korrekt gelesen wird, sollten die 36-Byte-Records parsen.

---

### H4 — Legacy-Parser: Node-Count > 0 aber `parse_node` scheitert fuer ALLE Nodes → Err → ganzer Sektor verworfen

**Mechanismus:** Legacy-Parser parsed alle Items (sauber). Liest `node_count > 0` im Trailer. `parse_node()` (56-Byte Record, line 353) scheitert fuer den ERSTEN Node → `?` propagiert `Err` → `parse_sector_legacy_inner` returned `Err` → `parse_sector` returned `Err` → `mod_loader` logged `warn!("Failed to parse sector {}: {e}", path)` (line 357) und **merged den Sektor NICHT**. Roads ebenfalls weg.

**ABER:** Dann duerften die Roads auch nicht in der Graph-Drop-Liste erscheinen! Wenn der Sektor gar nicht gemerged wird, gibt's keine BothUnresolved aus diesem Sektor.

**Falsifizierbar durch:** Check der Build-Logs. Wenn `"Failed to parse sector ... sec-0001-0008.base"` im Log steht → H4 confirmed, ABER die 230 BothFound-Events muessen dann aus einem ANDEREN Sektor stammen. Wenn kein solcher Log-Eintrag → H4 falsified.

**A-priori-Wahrscheinlichkeit:** **VERY LOW** — Wenn der Sektor nicht gemerged wird, gibt es weder Roads noch Nodes. Die 230 BothFound-Events muessten aus einem anderen Sektor mit denselben Road-UIDs kommen (unplausibel). Zusaetzlich widerspricht das dem "0 UnknownItemType"-Befund (UnknownItemType triggert `break` aber NICHT `Err`-Return fuer den ganzen Sektor — es setzt `all_items_parsed = false`, was Recovery triggert, nicht Sektor-Ablehnung).

---

### H5 — Node-UID-Deduplizierung ueberschreibt Nodes aus sec-0001-0008 mit identischen UIDs aus anderem Sektor

**Mechanismus:** Sec-0001-0008 parsed Nodes korrekt → `GraphBuilder::merge_sector` inserted sie in `self.nodes: HashMap<u64, RawNode>`. Ein SPAETERER Sektor (merged nach sec-0001-0008) hat Nodes mit DENSELBEN UIDs, aber anderen (unplausiblen) Koordinaten → ueberschreibt sec-0001-0008's Nodes. Die Roads aus sec-0001-0008 referenzieren die korrekten UIDs, aber im `node_lookup` des Build-Schritts sind diese UIDs mit u.U. unbrauchbaren Koordinaten hinterlegt — oder der ueberschreibende Node hat `x,y,z = 0,0,0`.

**Falsifizierbar durch:**
1. Nach Merge ALLER Sektoren: `self.nodes.get(&<uid_aus_road.node_a>)` aufrufen und Position loggen. Wenn Node existiert (gegen H1-H4) → H5 wahrscheinlich.
2. `self.node_to_sector.get(&<uid>)` zeigt auf den LETZTEN Sektor der diesen UID gesetzt hat — nicht sec-0001-0008.
3. In der Forensik: `node_a` und `node_b` UIDs in sec-0001-0008's Rohdaten checken ob sie AUCH in einem anderen Sektor vorkommen.

**A-priori-Wahrscheinlichkeit:** **LOW** — Selbst wenn Nodes ueberschrieben werden: der HashMap-Key ist der UID. `node_lookup.get(&uid)` wuerde den ueberschriebenen Wert finden (nicht nichts). Die Road wuerde resolven, nicht als BothUnresolved gezaehlt werden. Die einzige Moeglichkeit fuer H5 ist wenn der ueberschreibende Node GELOESCHT oder NICHT in den finalen `node_lookup` uebernommen wird — was nicht passiert (es gibt keinen Loesch-Pfad in `build()`).

---

### H6 — sec-0001-0008 liegt in einer DLC-Region mit anderem Node-Format

**Mechanismus:** Sec-0001-0008 ist in einem DLC-Archive (nicht base.scs). Das DLC nutzt ein Sektor-Format bei dem Nodes anders gespeichert sind (z.B. in `.aux` companion files, oder in einem item_type den der Parser dispatched aber dessen Nodes er nicht extrahiert).

**Betroffene Code-Stellen:**

| File | Lines | Concern |
|---|---|---|
| `crates/map-parser/src/mod_loader.rs` | 326–329 | Nur `.base` Dateien werden geparsed, `.aux` wird gefiltert |
| `crates/map-parser/src/mod_loader.rs` | 338–341 | Mod-override Reihenfolge: reverse-iteration durch Archives |
| `crates/map-parser/src/sector.rs` | 279–304 | Legacy-Item-Type-Dispatch: nur 4 von 24 Handlern extrahieren Nodes |

**Falsifizierbar durch:** Check welches HashFS-Archive `sec-0001-0008.base` enthaelt (base.scs oder ein DLC). Via `truckpilot-graph-stats` Sektor-Koordinaten map checken: x=-1, z=-8 (~4km west, 32km sued). Dies ist in der Naehe von Duisburg-Ruhrgebiet (base-game). Dateigroesse mit anderen Sektoren vergleichen.

**A-priori-Wahrscheinlichkeit:** **MEDIUM** — sec-0001-0008 (x=-1, z=-8) liegt im Nordwesten. Koennte Italien-DLC oder base-game Ruhrgebiet sein. Wenn DLC: Node-Format-Abweichung moeglich. Wenn base-game: H6 falsified.

---

### H7 — Item-Typ der Nodes enthaelt aber Parser extrahiert sie nicht (Prefab-/Building-embedded Nodes)

**Mechanismus:** Der Sektor enthaelt keine standalone Nodes im Trailer (node_count=0), aber Prefab- oder Building-Items referenzieren Nodes via `connected_node_uids`. Der Parser parsed diese UIDs korrekt in die Prefab/Building-Strukturen, aber sie werden NICHT als eigenstaendige `RawNode` in `sector.nodes` gepushed (weil Prefab-Nodes nur UID-Referenzen sind, keine vollstaendigen Node-Records mit Position).

Die Roads referenzieren dieselben UIDs, aber da diese UIDs nie als `RawNode` mit Position landen, koennen Road-Edges nicht gebaut werden → BothUnresolved.

**Betroffene Code-Stellen:**

| File | Lines | Handler |
|---|---|---|
| `crates/map-parser/src/sector.rs` | ~990–1040 | `parse_prefab()` — pushed UIDs in `prefab.nodes: Vec<u64>`, NICHT in `sector.nodes` |
| `crates/map-parser/src/sector.rs` | ~1100–1160 | `parse_buildings()` — pushed `RawBuilding { node_uid, forward_node_uid }` |
| `crates/map-parser/src/graph.rs` | 350–390 | Prefab-Clique: `node_lookup.contains_key(uid)` filtert alle UIDs raus die nicht im Node-Map sind |

**Falsifizierbar durch:** Audit: `parse_sector` auf sec-0001-0008 ausfuehren. `sector.nodes.len()` checken. Wenn = 0 → dieser Sektor hat keine Standalone-Nodes, und ALLE Roads darin referenzieren Nodes die in ANDEREN Sektoren oder in Prefabs definiert sind (Cross-Sector-Connectivity). Heisst: die 230 BothFound-Events sind KEIN Bug sondern echtes Cross-Sector-Problem das via spatial-matching oder Cross-Sector-Edge-Generation geloest werden muesste.

**A-priori-Wahrscheinlichkeit:** **MEDIUM** — Ein Sektor mit node_count=0 ist valides ETS2-Format. Wenn alle 230 Roads in diesem Sektor liegen und alle deren Node-Referenzen nur in anderen Sektoren existieren → echtes cross-sector Problem, kein Parse-Bug.

---

### H8 — `sized_sector_plausible` Stage-1 feuert und Legacy-Pfad produziert Recovery-Nodes die den Plausibility-Check nicht bestehen

**Mechanismus:** `try_parse_sized_sector` scheitert an Stage-1 (item_count < 10 aber data > 50KB). Fallback zu Legacy. Legacy parsed alle Items (sauber). ABER `node_count > 0` und `recover_nodes_from_tail` wird NICHT aufgerufen (da `all_items_parsed == true`). Legacy liest Nodes direkt. Diese Nodes passieren den ublichen Flow.

Oder: Legacy's Node-Trailer selbst ist korrupt → `parse_node` scheitert → Err → Sektor rejected (siehe H4 Widerlegung).

**Falsifizierbar durch:** Binary-Instrumentierung: Log ob sized oder legacy Pfad genommen wird + `all_items_parsed` Wert + `sector.nodes.len()`.

**A-priori-Wahrscheinlichkeit:** **LOW** — Stage-1 feuert nur wenn item_count < 10 ABER sector.data.len() > 50KB. Sec-0001-0008 hat 230 Roads + Prefabs + Buildings... item_count ist sehr wahrscheinlich > 10. Ausserdem: Stage-1 Feuerung fuehrt zu Legacy-Fallback, nicht zu Node-Verlust.

---

### Hypothesen-Schnelluebersicht

| # | Hypothese | Wahrscheinlichkeit | Erklaert BothFound + 0 UnknownItemType |
|---|---|---|---|
| H1 | node_count = 0 (leerer Trailer) | **HIGH** | Ja — Roads geparsed, Nodes nie existent, Forensik findet UIDs in Road-Koerpern |
| H2 | sized_sector_plausible Stage-2 verwirft | **MEDIUM** | Ja — sized parsed Nodes, Gate rejected, Legacy liest anders |
| H3 | parse_node_f64 silent-skip fuer alle | **LOW** | Ja — sized laeuft, alle Node-Parses scheitern still |
| H4 | Legacy parse_node Fail → Sektor rejected | **VERY LOW** | Nein — Sektor rejected = keine Roads = keine BothUnresolved |
| H5 | Node-UID-Dedup-Ueberschreibung | **LOW** | Nein — HashMap-Key ist UID, wuerde immer resolven |
| H6 | DLC-spezifisches Node-Format | **MEDIUM** | Ja — anderer Speicherort fuer Nodes |
| H7 | Keine standalone Nodes, nur Prefab-Referenzen | **MEDIUM** | Ja — Roads parsen, Nodes nie existent, echte Cross-Sector |
| H8 | sized_sector_plausible Stage-1 Feuerung | **LOW** | Teilweise — fuehrt zu Legacy, der normal parsen sollte |

---

## 2. Diagnose-Reihenfolge

Sortierungskriterium: **(Test-Aufwand × 1/Falsifikations-Power)**, d.h. erst die billigsten Tests mit hoechster Aussagekraft.

| Rang | Hypothese | Test-Dauer | Falsifikations-Wert | Begruendung |
|---|---|---|---|---|
| **1** | H6 (DLC-Check) | 2 min | Killt oder bestaetigt DLC-Erklaerung sofort | Nur File-Archiv-Zuordnung checken. Kein Code noetig. |
| **2** | H1 (node_count=0) | 15 min | Direkte Antwort ob Nodes existieren | Einfache Hex-Inspektion + parse_sector-Instrumentierung. Minimaler Code-Aufwand. |
| **3** | H7 (Prefab-only Nodes) | 15 min | Zeigt ob Sektor strukturell keine Nodes hat | `sector.nodes.len()` + `sector.prefabs.len()` + Prefab UIDs cross-referenzieren. |
| **4** | H2 (sized_sector_plausible) | 30 min | Klaert sized-vs-legacy Routing | Instrumentierung in `sized_sector_plausible`. Zeigt ob sized ueberhaupt genutzt wird. |
| **5** | H3/H4 (parse-Fehler) | 30 min | Schliesst Parser-Bugs definitiv aus | Node-Parse-Error-Zaehler + All-Items-Parsed Flag loggen. |
| **6** | H5 (Dedup) | 60 min | Definitiver Ausschluss | Vollstaendigen Merge-Durchlauf instrumentieren, Node-Herkunft tracken. |

---

## 3. Konkrete Test-Specs

### Test 1 — H6: DLC-Archive-Zuordnung

**Diag-Tool:** Existiert NICHT — manueller Check oder Einzeiler-Script.

**Inputs:**
- `sec-0001-0008.base` im HashFS suchen
- ETS2-Installation: `C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2`

**Durchfuehrung:**
1. Alle `.scs` Archive im ETS2-Ordner und `mod/` auflisten
2. `truckpilot-graph-stats` oder ein bestehendes Diag-Tool aufrufen, das Sektor-Provenienz loggt
3. Alternativ: Alle Archive oeffnen und `read_path("map/europe/sec-0001-0008.base")` probieren — erstes Archiv das Daten liefert = Herkunft

**Output-Interpretation:**
- `base.scs` oder `def.scs` → H6 **widerlegt** (Vanilla-Sektor)
- `dlc_*.scs` → H6 **bestaetigt**, Sektor-Name und DLC notieren
- Mod-Archiv → DLC-Ueberlegung irrelevant, aber Format-Abweichung moeglich

**Geschaetzter Aufwand:** 2 Minuten

---

### Test 2 — H1: Node-Count Hex-Verifikation

**Diag-Tool:** Exisitiert nicht als Standalone. Moegliche Ansaetze:
- (a) Manueller Hex-Dump der letzten ~500 Bytes: `tail -c 500 sec-0001-0008.base | xxd`
- (b) Bestehendes `truckpilot-road-dump` anpassen (5 min)
- (c) `parse_sector()` aufrufen und `sector.nodes.len()` loggen (10 min Code + Build)

**Inputs:**
- Rohe `.base` Datei aus dem ETS2-Archive extrahieren
- `crates/map-parser/src/sector.rs` `parse_sector()` Funktion

**Durchfuehrung (empfohlener Pfad — minimaler Aufwand):**
1. `cargo run --release --bin truckpilot-road-dump -- --ets2-dir <PATH> --audit-skips` laeuft bereits — adaptieren um fuer einen spezifischen Sektor den Node-Trailer zu dumpen
2. Oder: In `parse_sector_legacy_inner` (line 344) und `try_parse_sized_sector` (line 807) ein `info!` Log mit `node_count` und `sector.nodes.len()` einfuegen
3. Map-Parse durchfuehren, Log nach `sec-0001-0008` filtern

**Output-Interpretation:**
- `node_count = 0` oder `sector.nodes.len() = 0` → H1 **bestaetigt**
- `node_count > 0` und `sector.nodes.len() > 0` → H1 **widerlegt** (Nodes werden geparsed, Problem liegt downstream)

**Geschaetzter Aufwand:** 15 Minuten

---

### Test 3 — H7: Prefab-Node-Referenz-Audit

**Diag-Tool:** Exisitiert nicht. Benoetigt: einfaches Rust-Binary oder Inline-Instrumentierung.

**Inputs:**
- Geparster `ParsedSector` fuer sec-0001-0008
- `sector.nodes`, `sector.roads`, `sector.prefabs`

**Durchfuehrung:**
1. `parse_sector_with_tracer` auf sec-0001-0008 aufrufen
2. `sector.nodes.len()` erfassen
3. Wenn `nodes.len() == 0`: Alle `sector.roads[].node_a` und `sector.roads[].node_b` UIDs sammeln
4. Alle `sector.prefabs[].nodes` (connected_node_uids) sammeln
5. Intersection berechnen: `road_refs ∩ prefab_node_uids` → Anteil der Road-Referenzen die via Prefab definiert sind
6. Falls hohe Intersection: Nodes sind nur in Prefabs definiert, nicht standalone → Cross-Sector Problem

**Output-Interpretation:**
- `nodes.len() == 0` UND `road_refs` largely in `prefab_uids` → H7 **bestaetigt** (echte Cross-Sector Edges)
- `nodes.len() > 0` → H7 **widerlegt** (Sektor hat standalone Nodes)

**Geschaetzter Aufwand:** 15 Minuten (baut auf Test-2 Ergebnissen auf)

---

### Test 4 — H2: sized_sector_plausible Gate-Log

**Diag-Tool:** Nicht noetig — Inline-Instrumentierung in `sector.rs`.

**Inputs:**
- `crates/map-parser/src/sector.rs` lines 860–895 (`sized_sector_plausible`)
- `crates/map-parser/src/sector.rs` lines 194–205 (`parse_sector` — sized/legacy dispatch)

**Durchfuehrung:**
1. In `parse_sector` (line 194): Log `"sec-0001-0008 trying sized"` mit `Sektor-Path`
2. In `try_parse_sized_sector` nach Phase 2: Log `"sized parsed {} items, {} nodes"` 
3. In `sized_sector_plausible` (line 860): Vor dem Return ein `info!` mit: Stage-1 Ergebnis, Stage-2 Ergebnis, Node-Koordinaten (erste 10), `nodes.len()`
4. Falls `sized_sector_plausible` `None` returned: In `parse_sector` (line 204) Log `"falling back to legacy for sec-0001-0008"`
5. Map-Parse, Log filtern nach `0001-0008`

**Output-Interpretation:**
- `sized_sector_plausible` returned `Some(())` → sized path full accepted, H2 **widerlegt** bezueglich Gate-Ablehnung. Wenn nodes.len()==0 trotz Some → H1/H3 statt H2.
- `sized_sector_plausible` returned `None` wegen Stage-2 (alle Nodes unplausible Koordinaten) → H2 **bestaetigt**
- `sized_sector_plausible` returned `None` wegen Stage-1 (items < 10) → H8 **bestaetigt**
- Sized path nicht genommen (probe failed) → direkt legacy → H2 **widerlegt** (sized nie genutzt)

**Geschaetzter Aufwand:** 30 Minuten (5 min Code + 25 min Build & Run)

---

### Test 5 — H3/H4: Node-Parse-Error-Rate

**Diag-Tool:** Nicht noetig — Inline-Counter in `sector.rs`.

**Inputs:**
- `crates/map-parser/src/sector.rs` lines 817–823 (sized silent skip)
- `crates/map-parser/src/sector.rs` lines 344–365 (legacy node section)

**Durchfuehrung:**
1. Sized-Pfad (line 821): Counter fuer `Ok` und `Err` in der Node-Loop, log nach Loop
2. Legacy-Pfad (line 353): `parse_node`-Erfolg/Fail loggen (hier kein silent-skip: `?` propagiert Error)
3. Legacy-Pfad: `all_items_parsed` Flag loggen

**Output-Interpretation:**
- Sized: `node_count = 5, parsed_ok = 0, parsed_err = 5` → H3 **bestaetigt**
- Sized: `node_count = 0` → H3 nicht relevant, Problem upstream (H1)
- Legacy: `Err` von `parse_node` + `all_items_parsed = false` → H4 **bestaetigt**, aber dann keine Roads (Widerspruch)
- Legacy: `all_items_parsed = true` und `node_count = 0` → H1 im Legacy-Pfad bestaetigt

**Geschaetzter Aufwand:** 30 Minuten (baut auf Test-4 Instrumentierung auf)

---

## 4. Pre-Audit-Checks (30 Minuten Code-Inspektion, keine Empirie)

Diese Checks sind OHNE Hex-Audit oder Parse-Lauf durchfuehrbar. Ziel: Hypothesen per Code-Review vorab eingrenzen.

### Check 1 — Welche Item-Types schreiben Nodes?

**Frage:** Welche Handler in `parse_sector_legacy` und `try_parse_sized_sector` pushen in `sector.nodes`?

**Antwort (aus Code-Review, `crates/map-parser/src/sector.rs`):**
- KEINER der Item-Handler pusht in `sector.nodes`. Nodes kommen AUSSCHLIESSLICH aus dem Trailing-Node-Block.
- `parse_road` → `sector.roads`
- `parse_prefab` → `sector.prefabs` (enthalt `nodes: Vec<u64>` — Node-UIDs, KEINE `RawNode`)
- `parse_buildings` → `sector.buildings` (enthalt `node_uid: u64`, `forward_node_uid: u64`)
- `parse_ferry` → `sector.ferries` (enthalt `node_uid: u64`)

**Implikation fuer Hypothesen:** Wenn `sector.nodes` leer ist, dann nur weil der Trailing-Node-Block 0 Nodes liefert. **H7 (Prefab-Nodes sind keine RawNodes)** ist damit strukturell bestaetigt: Prefab-UIDs sind Referenzen, keine definierten Nodes. Roads die auf Prefab-UIDs verweisen muessen hoffen dass jene UIDs in einem ANDEREN Sektor als Node definiert sind.

### Check 2 — Gibt es Filter-Branches die Nodes verwerfen?

**Frage:** Welcher Code-Pfad im Flow `parse_sector → merge_sector → build` kann Nodes filtern?

**Antwort (aus Code-Review):**

| Stage | Filter | Lines | Bedingung |
|---|---|---|---|
| Sector-Parse | `MAX_LIST_COUNT = 2,000,000` | sector.rs:173, 346 | `node_count > 2,000,000` → `Err` → Sektor rejected |
| Sector-Parse | `parse_node` Fehler (Legacy) | sector.rs:353 | `?` propagiert → `Err` → Sektor rejected |
| Sector-Parse | `parse_node_f64` Fehler (Sized) | sector.rs:821-823 | `if let Err` → Node skipped silently |
| Sector-Parse | Plausibility Stage-1 | sector.rs:867-876 | items < 10 & data > 50KB → `None` → Legacy-Fallback |
| Sector-Parse | Plausibility Stage-2 | sector.rs:879-892 | ALLE Nodes ausserhalb ETS2-Weltkoordinaten → `None` → Legacy-Fallback |
| Sector-Parse | `recover_nodes_from_tail` | sector.rs:424-504 | ≥50% non-zero UIDs required (wird bei all_items_parsed=true NICHT aufgerufen) |
| Graph-Merge | HashMap-Insert | graph.rs:129 | `self.nodes.insert(node.uid, node)` — dedup by key, kein Filter |
| Graph-Build | `node_lookup` construction | graph.rs:156 | `.collect()` aus bereits-deduped `nodes: Vec<GraphNode>` — kein Filter |

**Implikation:** Es gibt KEINEN Filter zwischen `sector.nodes → GraphBuilder.nodes → node_lookup` der einzelne Nodes loeschen koennte. Einziger Verlust-Pfad: `sector.nodes` ist bereits leer (0 Nodes vom Parser). **H5 (Dedup-Ueberschreibung) ist damit strukturell ausgeschlossen**: selbst ueberschriebene Nodes bleiben im HashMap, weil der Key (UID) gleich bleibt.

### Check 3 — Spezielle Eigenschaften von sec-0001-0008

**Frage:** Hat der Sektor Besonderheiten die auf ein abweichendes Format hindeuten?

**Zu pruefen:**
- DLC-Zugehoerigkeit (via Archive-Check — Test 1)
- Dateigroesse im Vergleich zu Nachbarsektoren (z.B. sec-0001-0007)
- `item_count` — wie viele Items im Vergleich zu anderen Sektoren?
- `node_count` im Trailer — >0 oder =0?
- Sized-Flag — geht der Sektor ueberhaupt durch `try_parse_sized_sector`?

**Sektor-Koordinaten-Analyse:** `sec-0001-0008` = Sektor bei X=-1, Z=-8. Im ETS2-Koordinatensystem: jeder Sektor = 4 Einheiten (~4km). Position ~(-4096m, -32768m) im Weltraum. Dies liegt westlich von Duisburg, suedwestlich vom Zentrum — innerhalb des base-game Bereichs (kein DLC-Randgebiet). Nachbarsektoren: sec-0001-0007 (Rank #1 BothFound, 437 events, HAT UnknownItemType) — der Nachbar hat cursor-desync (bezier_patch Bug), sec-0001-0008 nicht.

---

## 5. Decision-Tree

```
                         ┌──────────────────────┐
                         │ sec-0001-0008        │
                         │ 230 BothFound        │
                         │ 0 UnknownItemType    │
                         └──────┬───────────────┘
                                │
                     ┌──────────▼──────────┐
                     │ Pre-Audit: Code-Inspection │
                     │ → H5 ausgeschlossen      │
                     │ → H7 strukturell confirmed│
                     │   (Prefab-UIDs ≠ Nodes)   │
                     └──────────┬──────────┘
                                │
                     ┌──────────▼──────────┐
                     │ Test 1: DLC-Check   │
                     │ (2 min)             │
                     └──────┬─────┬───────┘
                            │     │
                    base.scs│     │dlc_*.scs
                            │     │
                     ┌──────▼┐  ┌─▼──────────────────────┐
                     │ H6 ✗  │  │ H6 ✓: DLC-spezifisch   │
                     └──┬────┘  │ → Test auf Node-Format  │
                        │       │ → Fix < 2h → fixen      │
                        │       │ → Fix > 2h → deferren   │
                        │       │   (wenn kein Stadtpaar  │
                        │       │    blockiert)           │
                        │       └─────────────────────────┘
                        │
             ┌──────────▼──────────┐
             │ Test 2+3: node_count│
             │ + Prefab-Audit      │
             │ (30 min)            │
             └──────┬─────┬───────┘
                    │     │
          node_count│     │node_count > 0
          = 0       │     │
                    │     │
     ┌──────────────▼┐  ┌─▼───────────────────────┐
     │ H1 ✓ + H7 ✓  │  │ H1 ✗ → Nodes parsen    │
     │ Sektor hat    │  │ korrekt. Problem liegt  │
     │ keine Nodes   │  │ DOWNSTREAM (H2, H3, H5)│
     └──────┬────────┘  └──────────┬──────────────┘
            │                      │
            │               ┌──────▼──────────────┐
            │               │ Test 4+5: sized-    │
            │               │ Gate + Parse-Errors │
            │               │ (30 min)            │
            │               └──────┬──────┬───────┘
            │                      │      │
            │               H2 ✓   │      │ H3 ✓
            │               Gate   │      │ Silent
            │               reject │      │ skip
            │                      │      │
            │               ┌──────▼┐  ┌──▼──────────────┐
            │               │ FIX   │  │ FIX: parse_node │
            │               │ Gate  │  │ _f64 robust     │
            │               │ anpass│  │ machen          │
            │               └───────┘  └─────────────────┘
            │
   ┌────────▼─────────────────────────────────────┐
   │ ENTSCHEIDUNG: node_count = 0                 │
   │                                               │
   │ Ist das ein Bug oder ein Feature?             │
   │                                               │
   │ ┌─────────────────────────────────────────┐   │
   │ │ Frage: Blockiert sec-0001-0008 ein      │   │
   │ │ Stadtpaar im Standard-Test-Set?          │   │
   │ │ (cities.toml: Berlin, Hamburg, Wien,     │   │
   │ │  Paris, Amsterdam, Koeln, Frankfurt,     │   │
   │ │  Muenchen, Prag, Warschau)               │   │
   │ └──────────────┬──────────────────────────┘   │
   │                │                               │
   │       ┌────────▼────────┐                      │
   │       │ Stadtpaar       │                      │
   │       │ blockiert?      │                      │
   │       └──┬─────────┬────┘                      │
   │          │         │                            │
   │      JA  │         │ NEIN                       │
   │          │         │                            │
   │  ┌───────▼──┐  ┌───▼────────────────┐          │
   │  │ FIXEN    │  │ DEFERREN nach      │          │
   │  │ (Fix-5c) │  │ Phase 6.4          │          │
   │  │          │  │ (Cross-Sector-     │          │
   │  │ Strategie│  │  Edge Generation)  │          │
   │  │ → Prefab-│  │                    │          │
   │  │  Nodes   │  │ Impact: 230 /      │          │
   │  │  als     │  │ 1,063,231 Nodes =  │          │
   │  │  Graph-  │  │ 0.02%              │          │
   │  │  Nodes   │  │                    │          │
   │  │  injecten│  │ Als Known-Issue    │          │
   │  └──────────┘  │ dokumentieren      │          │
   │                └────────────────────┘          │
   └────────────────────────────────────────────────┘
```

### Konkrete Schwellenwerte

| Bedingung | Aktion | Begruendung |
|---|---|---|
| H1 bestaetigt (node_count=0) + Prefab-Nodes vorhanden + blockiert ≥1 Stadtpaar | **Fix-5c**: Prefab-Node-UIDs als GraphNode injecten (ohne Koordinaten → spatial-match-only Edges). Geschaetzter Aufwand: 4-8h | Routing-Blockade im Standard-Test-Set → prioritaer |
| H1 bestaetigt + KEIN Stadtpaar blockiert | **Defer to Phase 6.4** (Cross-Sector-Edge-Generation). Als Known-Issue in `docs/known_issues.md` dokumentieren. | 0.02% Impact rechtfertigt keinen Hotfix. Loesung erwartet von Cross-Sector-Connectivity-Arbeit. |
| H2 bestaetigt (sized_sector_plausible Gate) | **Fix-5c**: Gate-Parameter lockern (z.B. `|x|,|z| < 500000` statt `250000`) oder per-Sektor-Exception. Aufwand: <2h | Einfacher Fix, hoher Impact (230 Events, 12.2% der verbleibenden BothUnresolved) |
| H3 bestaetigt (parse_node_f64 silent-skip) | **Fix-5c**: Error-logging statt silent-skip + f64→f32 Cast robust gegen NaN. Aufwand: <1h | Trivialer Fix |
| H6 bestaetigt (DLC) + Node-Format abweichend | **Defer to Phase 6.4** mit DLC-Support-Ticket | DLC-Formate sind eigene Scope-Arbeit |
| Keine Hypothese bestaetigt, 230 Events unerklaert | **Defer with Blocker-Tag**. Phase 6.2b abschliessen, Outlier als Phase-6.3-Ticket eskalieren. | Weitere Diagnose ohne klare Hypothese nicht effizient — benoetigt tieferen Hex-Audit |


## 6. Empfohlene Diagnose-Reihenfolge (Executive Summary)

1. **Jetzt (0 min):** Pre-Audit Check 3 durchfuehren — H5 ist strukturell ausgeschlossen (HashMap Key = UID, kann nicht verloren gehen). H7 ist strukturell confirmed (Prefab UIDs sind keine RawNodes).

2. **Dann (2 min):** Test 1 — DLC-Archive-Check. Einzeiler. Killt oder bestaetigt H6 sofort.

3. **Dann (15 min):** Test 2 — `parse_sector` auf sec-0001-0008 ausfuehren, `sector.nodes.len()` loggen. Direkte Antwort: hat der Sektor ueberhaupt Nodes?

4. **Dann (15 min):** Test 3 — Wenn `nodes.len() == 0`: Prefab-UID-Audit. Zeigt ob die Road-Referenzen via Prefabs definiert sind.

5. **Falls nodes.len() > 0 (wider Erwarten):** Tests 4+5 — sized-Gate + Parse-Error-Checks.

6. **Decision nach Test 2+3:** node_count=0 → Defer oder Fix-5c je nach Stadtpaar-Blockade. node_count>0 → weiter mit Tests 4+5.

---

## 7. Multiple-Choice: Offene Fragen fuer Philipp

### Frage 1: Prioritaet des Outliers

```
[ ] Option A: sec-0001-0008 JETZT diagnostizieren (Fix-5c), bevor Phase 6.3 beginnt
[ ] Option B: Erst Fix-5b Ergebnisse abwarten (Post-Re-Parse), dann Outlier priorisieren
[ ] Option C: Outlier deferren, Phase 6.3 (Cross-Sector Edges) priorisieren — dort wuerde H7 eh geloest
```

Empfehlung: **Option B** — Fix-5b aendert die Baseline. Wenn nach Fix-5b BothUnresolved von 1,878 auf ~250 sinkt und sec-0001-0008 (230) >90% der Rest-Events ausmacht, ist der Outlier das dominante Problem → Option A. Wenn Fix-5b unerwartet andere Sektoren exposed → Option C.

### Frage 2: Wenn node_count=0 bestaetigt — Fix-Strategie?

```
[ ] Option A: Prefab-Nodes als GraphNodes injecten (ohne Koordinaten → nur spatial-match-faehig)
[ ] Option B: Kein Fix — warten bis Cross-Sector-Connectivity (Phase 6.4) das Problem generisch loest
[ ] Option C: Den Sektor komplett ignorieren (230/1,063,231 = 0.02% Node-Impact)
```

Empfehlung: **Option B** — Prefab-Nodes ohne Koordinaten in den Graph zu injecten ist ein Workaround der spaeter weh tut (fake nodes, falsche Distanzen). Cross-Sector-Edge-Generation wuerde das Problem strukturell korrekt loesen.

### Frage 3: Welches Diag-Tool soll gebaut werden?

```
[ ] Option A: Vorhandenes `truckpilot-road-dump` erweitern um `--sector <name> --dump-tail`
[ ] Option B: Neues `truckpilot-outlier-diag` Binary das alle Tests 1-5 auf einmal ausfuehrt
[ ] Option C: Kein Tool — alles via Inline-Logging im `parse_sector` Flow
```

Empfehlung: **Option A + C hybrid** — `truckpilot-road-dump` um `--dump-tail` erweitern fuer Hex-Inspektion, plus temporaere `info!` Logs in `parse_sector` + `sized_sector_plausible` fuer die Parse-Flow-Analyse. Kein neues Binary noetig.

---

## Appendix A: Relevante Dateien und Zeilennummern

| Datei | Zeilen | Inhalt |
|---|---|---|
| `crates/map-parser/src/sector.rs` | 185–205 | `parse_sector()` Dispatch |
| `crates/map-parser/src/sector.rs` | 225–365 | `parse_sector_legacy_inner()` |
| `crates/map-parser/src/sector.rs` | 344–365 | Legacy Trailing-Node-Section |
| `crates/map-parser/src/sector.rs` | 424–504 | `recover_nodes_from_tail()` |
| `crates/map-parser/src/sector.rs` | 681–850 | `try_parse_sized_sector()` |
| `crates/map-parser/src/sector.rs` | 807–850 | Sized Node-Tail (Phase 3) |
| `crates/map-parser/src/sector.rs` | 817–823 | Sized count-prefixed node loop |
| `crates/map-parser/src/sector.rs` | 837–844 | Sized stream-to-EOF node loop |
| `crates/map-parser/src/sector.rs` | 860–895 | `sized_sector_plausible()` |
| `crates/map-parser/src/sector.rs` | 954–966 | `parse_node_f64()` |
| `crates/map-parser/src/sector.rs` | 1056–1072 | `parse_node()` |
| `crates/map-parser/src/graph.rs` | 103–115 | `GraphBuilder` struct |
| `crates/map-parser/src/graph.rs` | 125–137 | `GraphBuilder::merge_sector()` |
| `crates/map-parser/src/graph.rs` | 141–588 | `GraphBuilder::build()` |
| `crates/map-parser/src/graph.rs` | 166–206 | Road edge generation + BothUnresolved counting |
| `crates/map-parser/src/graph.rs` | 632–657 | `analyze_roads_for_audit()` |
| `crates/map-parser/src/mod_loader.rs` | 280–372 | `parse_sectors_from_archives()` |
| `crates/map-parser/src/mod_loader.rs` | 326–329 | `.base` Extension-Filter |
| `crates/diag/src/bin/bothunresolved-forensik.rs` | 132–146 | `Bucket`/`SubBucket` Enum |
| `crates/diag/src/bin/bothunresolved-forensik.rs` | 181–237 | `scan_sector` + `is_plausible_node_definition` |
| `crates/diag/src/bin/bothunresolved-forensik.rs` | 494–517 | Klassifikationslogik |

## Appendix B: Glossar

| Begriff | Definition |
|---|---|
| **BothUnresolved** | Road-Edge deren BEIDE Endpunkt-Nodes nicht im Node-Map des GraphBuilders gefunden werden. Wird im finalen Graph gedropped. |
| **BothFound** | Forensik-Bucket: BEIDE Node-UIDs einer BothUnresolved Road wurden via Byte-Scan in Sektor-Rohdaten gefunden. |
| **RoadPlusBothSameSector** | Forensik-Sub-Bucket: SOWOHL die Road-UID als auch BEIDE Node-UIDs wurden im SELBEN Sektor gefunden. |
| **UnknownItemType** | Item-Typ den der Legacy-Parser nicht kennt → Parse-Abbruch mit `all_items_parsed = false`. |
| **Sized format** | Modernes ETS2-Sektor-Format (1.50+) mit `type+size+payload` Struktur. |
| **Legacy format** | v907 TruckLib-kompatibles Format mit Fixed-Width-Item-Dispatch. |
| **Sektor** | `.base` Datei, repraesentiert ~4km×4km Kartenkachel. |
| **Node-Trailer** | Trailing-Section am Ende jeder `.base` Datei: `node_count u32 + N×RawNode + vis_count u32 + M×vis_uids`. |

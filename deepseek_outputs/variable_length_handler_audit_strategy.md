# Variable-Length-Handler Audit-Strategie

> **Phase:** 6.2b-Fix-5b Follow-up  
> **Erstellt:** 2026-05-17  
> **Source:** `crates/map-parser/src/sector.rs` (2158 lines)  
> **Kontext:** BezierPatch-Handler (0x27) Under-Read in Fix-5b gefixt — andere variable-length Handler koennten aehnliche Bugs haben.

---

## 1. Handler-Inventory

Jeder Handler aus `parse_sector_legacy_inner` (Z.249-341) katalogisiert. Item-Type-Nummern aus den `ITEM_TYPE_*` Konstanten (Z.145-170). Die Hex-Werte des User-Inputs weichen ab — TruckPilot-Code ist massgeblich.

### Scoring-Regeln

| Faktor | Gewicht | Begruendung |
|--------|---------|-------------|
| Pro top-level variabler Section (count + count×elem) | +2 | Jede count-gefuehrte Liste ist eine potenzielle Under-Read-Quelle |
| Pro verschachtelter variabler Section | +3 | Innere Counts werden leichter uebersehen, besonders bei tiefen Verschachtelungen |
| Pro optionalem / bedingtem Feld | +1 | Flags, Sentinel-Werte, `count>0`-Guards sind Fehlerquellen |
| Handler ist haeufig (Top-5 nach Frequenz) | +2 | Haeufige Handler verursachen mehr Sektor-Failures bei Bug |
| Handler verwendet u8/u16 fuer Counts | +3 | Kleinere Integer-Typen werden leichter vergessen (z.B. u8 board_count, u16 mat_count) |

**Skala 0–10, implicit gecappt.** Scores >10 werden im Cap 10 notiert, die Roh-Summe in Klammern.

### Inventory-Tabelle

| # | Hex | Dec | Name | Fix-Bytes | Var | Nested | Opt | Freq | u8/u16 | Score | Status |
|---|-----|-----|------|-----------|-----|--------|-----|------|--------|-------|--------|
| 1 | 0x01 | 1 | Terrain | 155 | 3 | 14 | 0 | H | Ja | 10 (14) | **Rewrite Ph5.16** |
| 2 | 0x02 | 2 | Buildings | 97 | 1 | 0 | 0 | H | — | 4 | **Rewrite Ph5.19** |
| 3 | 0x03 | 3 | Road | 265 | 0 | 0 | 0 | H | — | 0 | Parse, nicht skip |
| 4 | 0x04 | 4 | Prefab | 69 | 3 | 1\* | 0 | H | — | 8 | Original+erweitert |
| 5 | 0x05 | 5 | Model | 113 | 1 | 0 | 0 | H | — | 4 | **Rewrite Ph5.18** |
| 6 | 0x06 | 6 | Company | 85 | 2 | 0 | 0 | M | — | 5 | Original-Port |
| 7 | 0x07 | 7 | Service | 69 | 1 | 0 | 0 | M | — | 3 | Original-Port |
| 8 | 0x08 | 8 | CutPlane | 53 | 1 | 0 | 0 | N | — | 2 | Original-Port |
| 9 | 0x0C | 12 | City | 77 | 0 | 0 | 0 | M | — | 0 | Original-Port |
| 10 | 0x12 | 18 | MapOverlay | 69 | 0 | 0 | 0 | N | — | 0 | Original-Port |
| 11 | 0x13 | 19 | Ferry | 89 | 0 | 0 | 0 | N | — | 0 | Ph5.22 Capture |
| 12 | 0x16 | 22 | Garage | 81 | 1 | 0 | 0 | N | — | 2 | Original-Port |
| 13 | 0x22 | 34 | **Trigger** | 53 | 3 | 3 | 1 | **H** | — | **10 (15)** | Ph5.15 Sentinel-Fix |
| 14 | 0x23 | 35 | FuelPump | 69 | 2 | 0 | 0 | N | — | 3 | Original-Port |
| 15 | 0x24 | 36 | Sign | 86 | 4 | 2 | 1 | H | Ja | **10 (16)** | Rewrite Ph5.12 |
| 16 | 0x25 | 37 | BusStop | 77 | 0 | 0 | 0 | N | — | 0 | Original-Port |
| 17 | 0x26 | 38 | TrafficArea | 53 | 2 | 0 | 0 | M | — | 4 | Original-Port |
| 18 | 0x27 | 39 | BezierPatch | 305 | 6 | 7 | 0 | H | Ja | **10 (17)** | **Gefixt Fix-5b** |
| 19 | 0x29 | 41 | **Trajectory** | 53 | 4 | 1 | 0 | M | — | **9 (12)** | Original-Port |
| 20 | 0x2A | 42 | MapArea | 53 | 1 | 0 | 0 | N | — | 2 | Original-Port |
| 21 | 0x2B | 43 | FarModel | 85 | 0 | 0 | 0 | N | — | 0 | Original-Port |
| 22 | 0x2C | 44 | **Curve** | 93 | 1 | 1 | 0 | H | — | 7 | Rewrite Ph5.17 |
| 23 | 0x2E | 46 | **Cutscene** | 53 | 2 | 3 | 0 | N | — | **8** | Original-Port |
| 24 | 0x30 | 48 | VisibilityArea | 77 | 1 | 0 | 0 | N | — | 2 | Original-Port |

\* Prefab corner-terrain: `skip(cur, node_count * 12)` — indirekt variabel, +1 statt +2.

### Detaillierte Score-Begruendungen (Top-8)

#### Trigger (Score 10, Roh 15)
```
kdop(53) → skip_token_list(+2) → skip_node_ref_list(+2) → skip_action_list(+2, include_name=true) → optional f32(+1)
  └─ action_base: num_params(+3 nested) → string_params(+3 nested) → target_tags(+3 nested) → 8 fixed bytes
```
- 3 top-level variable Sections = +6
- 3 nested Sections in action_base = +9
- 1 optional f32 (`node_count==1`) = +1
- High freq = +2 (Trigger ist Typ #1 oder #2 nach Frequenz in base_map.scs)
- Keine u8/u16 counts = +0
- **Roh: 18, gecappt: 10**

Trigger verwendet `skip_action_list(cur, true)`, denselben Helper den Cutscene mit `include_name=false` aufruft. Ph5.15 hat den 0xFFFFFFFF-Sentinel in `skip_action_base` gefixt — der Trigger-Handler selbst wurde **nicht isoliert auditiert**. Die `skip_token_list` und `skip_node_ref_list` sind eigenstaendige variable Sections vor dem action_list-Aufruf. Ein Offset-Fehler in einer dieser fruehen Sections wuerde den ganzen action_list-Block desyncen.

#### Sign (Score 10, Roh 16)
```
kdop(53) + 4 tokens(32) = 86 fix
  → board_count u8(+3) + board_count×24 bytes = var (+2)
  → PascalString u64 len + len bytes = var (+2)
  → IF template_len>0 (+1 opt): board_override_list(+2 var) → sign_override_list(+2 var)
       └─ board_override: flags-conditional(+3 nested)
       └─ sign_override: attr_type dispatch(+3 nested)
```
- 4 top-level variable = +8
- 2 nested Sections = +6
- 1 optional = +1
- High freq = +2
- u8 board_count = +3
- **Roh: 20, gecappt: 10**

Trotz Ph5.12-Rewrite: Der Handler hat die hoechste Anzahl count-gefuehrter Listen aller Handler. Die Verschachtelung von flags-gesteuerten Conditionals und attr_type-Dispatch in sign_override macht ihn zum strukturell komplexesten Handler im Parser.

#### BezierPatch (Score 10, Roh 17) — GEFIXT
```
kdop(53) + 16×vec3(192) + tess(4) + node(8) + seed(4) + veg(44) = 305 fix
  → sphere_count×20 = var (+2)
  → TDQ: mat_count×10(+2) + col_count×4(+2) + rows/cols(4) + quad_count×4(+2)
          + off_count×16(+2) + norm_count×16(+2)
     └─ alle 7 sub-lists in TDQ sind nested = +3 aggregiert
```
- 6 top-level variable = +12
- 1 nested Block (TDQ mit 7 sub-lists) = +3
- High freq = +2
- u16 counts (mat_count, col_count) = +3
- **Roh: 20, gecappt: 10**

In Fix-5b gefixt: Vegetation war `skip(55)` statt korrekt `skip(4*11 → 44)`, TDQ war flat-skip statt 7 verschachtelter Listen. Seit Fix keine bekannten Failures.

#### Trajectory (Score 9, Roh 12)
```
kdop(53) → skip_node_ref_list(+2) → skip_token(8 fix) →
  skip_trajectory_rule_list(+2) → checkpoint_count loop(+2) → skip_token_list(+2)
  └─ trajectory_rule_list: pro Rule: params u32 count + count×f32 = nested (+3)
```
- 4 top-level variable = +8
- 1 nested Section = +3
- Medium freq = +1
- Keine u8/u16 = +0
- **Roh: 12, Score: 9**

Vier variable Sections vor dem Ende des Handlers — der Handler mit den meisten top-level variablen Abschnitten nach BezierPatch. `skip_trajectory_rule_list` enthaelt zudem eine verschachtelte `param_count`-Schleife, die uebersehen werden kann wenn der aussere Count inkorrekt gelesen wird.

#### Cutscene (Score 8)
```
kdop(53) → skip_token_list(+2) → read_u64(8 fix) → skip_action_list(+2, include_name=false)
  └─ action_base: num_params(+3) → string_params(+3) → target_tags(+3) → 8 fix
```
- 2 top-level variable = +4
- 3 nested in action_base = +9 (but action_base is shared, already fixed)
- Low freq = +0
- **Roh: 13, Score 8 (reduziert weil action_base via Ph5.15 verifiziert)**

Cutscene verwendet denselben action_base-Helper wie Trigger, der via Ph5.15 Sentinel-Fix korrigiert wurde. Der Handler selbst hat nur 2 variable Sections ausserhalb von action_base (token_list, action_list). Da action_base der Haupt-Risikotraeger ist und bereits gefixt, ist der Residual-Risiko geringer als der Roh-Score suggeriert.

#### Prefab (Score 8)
```
kdop(53) + model_token(8) + variant_token(8) = 69 fix
  → additionalParts: count u32 + count×8 = var (+2)
  → nodes: count u32 + count×8 = var (+2) — wird geparsed, nicht geskipped
  → slaveItems: count u32 + count×8 = var (+2)
  → ferryLinkUid(8) + origin(2) = 10 fix
  → corner terrain: node_count×12 = indirekt var (+1)
  → semaphoreProfile(8) = 8 fix
```
- 3 top-level variable = +6
- corner terrain indirekt variabel = +1
- High freq = +2
- Keine u8/u16/optional = +0
- **Roh: 9, Score: 8**

Die `nodes`-Liste wird aktiv geparsed (Z.1028-1032), die anderen beiden nur geskipped. Corner terrain haengt von `node_count` ab — wenn node_count korrekt gelesen wurde, ist `node_count*12` trivial. Der Handler ist weniger risky als Trigger/Sign weil keine Flags oder Sentinel-Werte vorkommen.

#### Curve (Score 7)
```
kdop(53) + node(8) + fwd_node(8) + 2×locator(16) + length(4) + mask(4) = 93 fix
  → popcount(mask) × subcurve = var (+2)
     └─ subcurve: height_offsets count + count×4 = nested (+3)
```
- 1 top-level variable = +2
- 1 nested Section = +3
- High freq = +2
- **Score: 7**
- Rewrite Ph5.17, audit showed 0 curve failures post-rewrite.

#### Company (Score 5)
```
kdop(53) + 4×u64(32) = 85 fix
  → spawn_nodes: spawn_count×8 = var (+2)
  → spawn_counters: spawn_count×4 = var (+2)
```
- 2 top-level variable (teilen sich spawn_count) = +4
- Medium freq = +1
- **Score: 5**
- Original-Port von `binary_parser.rs`, nie gegen TruckLib verifiziert.

---

## 2. Priorisierte Audit-Kandidaten

Nach absteigendem Risiko-Score. Handler mit "Rewrite"-Status werden niedriger priorisiert, auch wenn ihr Roh-Score hoch ist, weil sie gegen TruckLib verifiziert wurden und im Multi-Sector-Audit 0 Failures als letzter Handler vor Failure zeigen.

| Rang | Handler | Score | Status | Begruendung |
|------|---------|-------|--------|-------------|
| **1** | **Trigger (0x22)** | 10 | Original-Port | Hoechste strukturelle Komplexitaet ausser BezierPatch. action_base zwar via Ph5.15 gefixt, aber `skip_token_list`/`skip_node_ref_list` vor action_base sind ungetestet. Trigger sind die haeufigste oder zweithaeufigste Item-Type. |
| **2** | **Trajectory (0x29)** | 9 | Original-Port | 4 variable Sections, verschachtelte param_count in trajectory_rule_list. Niemals gegen TruckLib referenziert. |
| **3** | **Cutscene (0x2E)** | 8 | Original-Port | action_base via Trigger-Ph5.15 mitgefixt, aber Cutscene-spezifischer Code (`include_name=false`, read_u64 vor action_list) nie isoliert auditiert. |
| **4** | **Prefab (0x04)** | 8 | Erweitert | 3 count-gefuehrte Listen, aktiv geparsed. Corner-terrain haengt von node_count ab. Am haeufigsten nach Road. |
| **5** | **Sign (0x24)** | 10 | Rewrite Ph5.12 | Trotz Rewrite hoechste strukturelle Komplexitaet. u8 board_count, flags-gesteuerte Conditionals, attr_type-Dispatch. Aber: nach Ph5.12 rewrite keine Failures als LastHandler beobachtet → hier niedriger priorisiert. |

**Warum diese Top-5 und nicht die anderen Score-10-Handler?**

- **BezierPatch (Score 10):** Eben in Fix-5b gefixt. Nachgewiesen 0 Failures nach Fix. Kein Audit-Investment noetig.
- **Terrain (Score 10):** Ph5.16-Rewrite gegen TruckLib. Multi-Sector-Audit: 0 terrain-last Failures. Verifiziert.
- **Sign (Score 10):** Ph5.12-Rewrite. Seitdem nie als Failure-Vorgaenger aufgetaucht. Wird auf Rang 5 gesetzt wegen Strukturkomplexitaet — aber investiere nur wenn Trigger+Trajectory+Cutscene sauber sind und BothUnresolved > 0 bleibt.

**Warum Trigger vor Trajectory?**

Trigger hat (a) hoehere Frequenz — mehr Sektoren betroffen bei Bug, (b) 3 variable Sections VOR dem komplexen action_base, jede davon koennte Offset verschieben, (c) optionales `node_count==1`-Feld das leicht vergessen werden kann.

---

## 3. Audit-Methodik

Generische Methodik pro Handler, reproduzierbar. Jeder Schritt hat definierte "Handler OK" vs "Handler verdaechtig"-Kriterien.

### Schritt A: Code-Inspektion (Aufwand: 15-30 min pro Handler)

1. **read_xxx-Count-Audit:** Fuer jeden Handler: manuell alle `read_*`, `skip_*`, `skip(cur, N)` aufsummieren. Mit TruckLib-Referenz (Konzept, kein Code) Feldanzahl und Groesse vergleichen.
2. **Count-Typ-Check:** Jeden `read_u8/u16/u32` der als count verwendet wird markieren. Pruefen ob der Rueckgabewert als `usize` gecastet wird (Z.1498: `let checkpoint_count = read_u32(cur)?` — richtig).
3. **Nested-Loop-Check:** Verschachtelte `for _ in 0..count` auf korrekte `element_size` pruefen (z.B. `skip(cur, count * 20)` — 20 korrekt?).

**Handler OK:** Alle read/skip-Calls stimmen mit erwarteten Feldgroessen ueberein.  
**Handler verdaechtig:** Mindestens ein read/skip-Offset unklar oder widerspricht TruckLib-Referenz.

### Schritt B: Multi-Sector Hex-Audit (Aufwand: 30-60 min pro Handler)

Fuer jeden Handler: 5-10 Sample-Items aus verschiedenen Sektoren per `audit_sector` identifizieren.

1. **Sample-Suche:** Sektor-Audit laufen lassen, Sektoren mit Target-Handler als letztem erfolgreichem Item ODER als Item vor Failure identifizieren.
2. **Hex-Dump:** Pro Sample 256-512 bytes ab Item-Body-Start dumpen.
3. **End-Offset-Verifikation:** `end_offset` des Handlers mit manuell berechnetem expected-End vergleichen.
4. **Next-u32-Check:** Die 4 Bytes bei `end_offset` muessen ein gueltiger `item_type` (1-48) oder `node_count` (plausible Ganzzahl < 4096) sein.

**Handler OK:** Alle 10 Samples zeigen validen next-item-type oder node_count bei `end_offset`.  
**Handler verdaechtig:** ≥1 Sample mit `end_offset` das auf Float-Garbage (`0x3F800000` = 1.0), Null-Sequenz, oder implausible item_type > 100 zeigt.

### Schritt C: Cross-Reference (Aufwand: 10 min pro Handler)

1. **items_parsed vs item_count:** Nach jedem Handler-Dispatch `items_parsed` inkrementieren. Am Ende des Sector-Audits: `report.items.len()` muss gleich `report.item_count` sein wenn der letzte Handler korrekt war (d.h. wenn kein failure).
2. **Failure-Signatur-Matching:** Wenn der Target-Handler fehlschlaegt: was ist der `raw_type` (garbage-u32)? Pattern aus Ph5.19 Buildings-Audit: `0x3F800000` (IEEE 1.0f) ist ein starkes Signal fuer Under-Read — der Cursor steht auf Float-Daten statt auf dem naechsten item_type.

**Handler OK:** `items_parsed == item_count` fuer ≥95% der Sektoren die diesen Handler enthalten.  
**Handler verdaechtig:** >5% Diskrepanz.

### Schritt D: Failure-Signatur-Erkennung

Bekannte Signaturen aus der Empirie (Phase 5.x):

| Garbage-u32 bei end_offset | Interpretation |
|---------------------------|----------------|
| `0x00000000` | Cursor in Null-gefuellten Padding-Bereich — Over-Read |
| `0x3F800000` | Cursor auf Float 1.0 (Stretch, Scale, Default-Koeffizient) — Under-Read |
| `0xFFxxxxxx` | Cursor auf Token- oder UID-high-bytes — Under-Read |
| 1–48 (valid item_type) | Handler korrekt — kein Bug |
| >48, <1000 | Wahrscheinlich Mitten-in-Float oder Token — Under-Read |
| >1000000 | Cursor in Node-Section oder komplett desynct — schwerer Bug |

**Entscheidungsmatrix:**
- `raw_type ∈ {1..48}` → Handler OK (naechstes Item beginnt korrekt)
- `raw_type == 0` ODER `raw_type ∈ {0x3F800000, 0x3F000000, 0x3E800000}` → Under-Read (Cursor auf Float-Daten)
- `raw_type > 1000` → Over-Read oder komplett desynct

### Schritt E: Reuse multi-sector-audit Infrastructure

Das existierende `audit_sector` (Z.562-673) und `sector_audit` Binary (`crates/map-parser/src/bin/sector_audit.rs`) sind direkt wiederverwendbar. Pro Handler ein spezialisiertes Audit-Binary analog zu `bezier_audit.rs` / `curve_audit.rs` / `model_audit.rs`.

Pattern:
```rust
// Reuse from existing audit binaries:
use truckpilot_map_parser::sector::audit_sector;
let report = audit_sector(data);
if let Some(failure) = &report.failure {
    if failure.raw_type == TARGET_ITEM_TYPE {
        // Handler crashed inside itself — item might be malformed
    }
}
// Or check: last successful item before failure
if let Some(last) = report.items.last() {
    if last.item_type == TARGET_ITEM_TYPE {
        // This handler was the last successful one — check end_offset alignment
    }
}
```

---

## 4. Diag-Tool Specs

Neue Tools, die ueber die existierenden Per-Handler-Audit-Binaries hinausgehen.

### 4.1 handler-size-validator

**Input:** Pfad zu `base_map.scs` (oder extrahiertem `map/`-Verzeichnis)  
**Output:** CSV mit `sector_path, item_type, item_index, start_offset, end_offset, claimed_strid, actual_strid, next_u32, verdict`

**Was es testet:** Liest jeden Sektor, dispatched alle Handler, vergleicht fuer JEDEN Handler den `end_offset - start_offset` (= actual stride) mit der erwarteten Groesse. Wenn die erwartete Groesse nicht bekannt ist (variable-length), prueft es ob `next_u32` ein gueltiger item_type ist.

**Aufwand:** ~200 LOC. Kann `audit_sector` wrappen und die `AuditedItem`-Daten aggregieren. Existierende `sector_audit.rs` ist das Template.

**"Handler OK":** `next_u32 ∈ valid_item_types ∪ {0}` (0 = node_count=0 am Sektor-Ende)  
**"Handler verdaechtig":** `next_u32 ∉ valid_item_types` UND `next_u32` ist Float-aehnlich oder Token-high-bytes

### 4.2 per-handler-byte-distribution

**Input:** Pfad zu `base_map.scs`, Target-Item-Type  
**Output:** Histogramm-Textdatei: Bytes pro Item des Target-Typs, Verteilung ueber alle Sektoren.

**Was es testet:** Sammelt `end_offset - start_offset` (= stride) fuer jeden Handler-Aufruf. Plottet Verteilung. Variable-length Handler sollten eine breite, multimodale Verteilung haben. Wenn die Verteilung eine scharfe Spitze bei einem unerwarteten Wert hat (z.B. 117 statt 329 fuer BezierPatch vor Fix-5b), ist das ein Signal.

**Aufwand:** ~150 LOC. Neues Binary unter `crates/map-parser/src/bin/handler_size_distribution.rs`.

**"Handler OK":** Verteilung ist breit/multimodal (konsistent mit variablen Counts).  
**"Handler verdaechtig":** Unimodale Spitze deutlich unter der erwarteten Minimum-Groesse (Under-Read), oder Spitzen bei exakt gleichen Werten ueber viele Sektoren (weil z.B. eine count-Liste immer 0 ist und der Handler den Skip dafuer falsch dimensioniert).

### 4.3 inter-item-gap-detector

**Input:** Pfad zu `base_map.scs`  
**Output:** Liste von Sektoren und Byte-Offsets wo die Gap zwischen `end_offset` von Item N und `start_offset` von Item N+1 > 4 bytes ist (d.h. wo der Handler Bytes ueberspringt oder zu wenige liest).

**Was es testet:** `gap = next_item.start_offset - current_item.end_offset`. Sollte exakt 4 sein (4 bytes = naechster item_type u32). Wenn gap < 4: Over-Read. Wenn gap > 4: Under-Read (Handler hat zu wenig gelesen, die uebrigen Bytes werden als naechster item_type interpretiert → Garbage).

**Aufwand:** ~120 LOC. Erweiterung von `sector_audit.rs`.

**"Handler OK":** gap == 4 fuer alle Items.  
**"Handler verdaechtig":** gap != 4 fuer Items eines bestimmten item_type.

### 4.4 stride-validator

**Input:** Pfad zu `base_map.scs`, erwartete Stride-Tabelle (fixe Handler: exakte Byte-Zahl; variable Handler: minimum + count-abhaengige Formel)  
**Output:** Pro Item-Type: Anzahl Matches, Anzahl Misses, Miss-Beispiele mit sector-path und offset.

**Was es testet:** Fuer fixe Handler: `actual_stride == expected_stride`. Fuer variable Handler: `actual_stride >= min_fixed_stride` UND `(actual_stride - min_fixed_stride) % element_size == 0` (d.h. die variablen Bytes sind ein Vielfaches der Elementgroesse).

**Aufwand:** ~200 LOC. Separat von handler-size-validator wegen Fokus auf strukturelle Konsistenz statt Item-Type-Gueltigkeit.

### Aufwand-Summary

| Tool | LOC | Zeit |
|------|-----|------|
| handler-size-validator | ~200 | 2-3h |
| per-handler-byte-distribution | ~150 | 1.5-2h |
| inter-item-gap-detector | ~120 | 1-1.5h |
| stride-validator | ~200 | 2-3h |
| **Gesamt** | **~670** | **6.5-9.5h** |

---

## 5. Trigger-Bedingungen

### Szenario A: Fix-5b erfolgreich (BothUnresolved < 300)

**Annahme:** Nach Fix-5b (BezierPatch-Handler korrigiert) fallen BothUnresolved von aktuell ~X auf unter 300.

**Empfehlung: Proaktiver Audit der Top-3 (Trigger, Trajectory, Cutscene).**

- Aufwand: ~4-6 Stunden fuer 3 Handler (Code-Inspektion + Hex-Audit + Cross-Reference)
- Erwarteter Nutzen: Wenn einer der drei einen Under-Read hat, werden weitere Sektor-Failures eliminiert bevor Phase 6.3+ startet (Speed-Limits, ProMods-Kompatibilitaet)
- 300 BothUnresolved koennen auch an legitimen Cross-Sector-Edges liegen (Road referenziert Node aus anderem Sektor, aber DLC trennt sie). Das ist nicht durch Handler-Fixes loesbar.

### Szenario B: Fix-5b laesst viele BothUnresolved (> 500)

**Empfehlung: Akuter Audit, Top-1 (Trigger) sofort starten.**

- Indiz: BezierPatch war nicht der einzige Schuldige. Wenn nach Fix-5b immer noch >500 Sektoren unresolvable Nodes haben, ist wahrscheinlich ein anderer Handler de-synct.
- **Starte direkt mit Trigger-Handler Hex-Audit** (Schritt A+B). Wenn Trigger sauber → Trajectory. Wenn Trajectory sauber → Cutscene.
- Erfolgsmessung: BothUnresolved sollte nach jedem gefixten Handler messbar sinken.

### Szenario C: ProMods / DLC-Phase startet

**Empfehlung: Audit-Prioritaet ueber DLC-Item-Typen.**

- DLC-Sektoren enthalten moeglicherweise Item-Types die in base_map.scs selten oder nicht vorkommen (z.B. 0x0D Mover, 0x14 Hinge, 0x18 AnimatedModel, 0x23 Sound, 0x26 CameraPoint, 0x30 BezierPatchControl, 0x31 Compound, 0x33 MapArea).
- Diese Handler existieren **nicht** in `sector.rs` — sie wuerden als `UnknownItemType` abbrechen. Das ist ein anderes Problem (fehlende Handler, nicht falsche Handler).
- Vor DLC-Start: `grep "unsupported item type"` im Audit-Log → Liste aller Typen die in DLC-Sektoren vorkommen aber keinen Handler haben.
- Fuer existierende Handler in DLC-Kontext: gleiche Methodik aber Sample-Sektoren aus DLC-Archiven.

---

## 6. Decision-Tree

```
Fix-5b Ergebnis aus Failures-Log lesen
│
├─ BothUnresolved < 300
│  └─ Proaktiv: Top-3 Handler auditieren (Trigger → Trajectory → Cutscene)
│     Geschaetzter Aufwand: 4-6h
│     Erwarteter Ertrag: Restfehler eliminiert vor Phase 6.3
│
├─ BothUnresolved 300-500
│  └─ Hybrid: Top-1 (Trigger) sofort auditieren.
│     Wenn Trigger-Bug gefunden → fixen → BothUnresolved neu messen.
│     Wenn < 300 nach Fix → Stop, Rest reaktiv.
│     Wenn > 300 nach Fix → Trajectory auditieren.
│
├─ BothUnresolved > 500
│  └─ Akut: Top-1 (Trigger) audit starten. Wahrscheinlich nicht nur BezierPatch.
│     Sequentiell Trigger → Trajectory → Cutscene → Prefab bis < 300.
│
└─ ProMods/DLC-Phase startet
   └─ Zuerst: Fehlende Handler identifizieren (grep "unsupported item type")
      Dann: Existierende variable Handler in DLC-Sektoren testen (gleiche Methodik)
```

### Konkrete Schwellen

| Schwelle | Aktion | Begruendung |
|----------|--------|-------------|
| BothUnresolved ≥ 500 | Akuter Audit | BezierPatch + mindestens 1 weiterer Handler broken |
| BothUnresolved 300–499 | Proaktiv Top-1 | Ein weiterer Bug wahrscheinlich aber nicht sicher |
| BothUnresolved 100–299 | Proaktiv Top-3 | Restfehler koennten legitime Cross-Sector-Edges sein. Niedriger Verdacht auf Handler-Bugs |
| BothUnresolved < 100 | Reaktiv | Wahrscheinlich alle Handler korrekt. Restfehler sind Datenprobleme (fehlende DLC-Nodes) |
| Beim ersten DLC-Sektor mit `unsupported item type` | Fehlende Handler bauen | Kein Audit-Problem sondern Implementations-Luecke |

---

## 7. Multiple-Choice: Offene Fragen

### Q1: Sollen bereits via TruckLib verifizierte Handler (Terrain, Buildings, Model, Curve, Sign) re-auditiert werden?

- **A)** Nein — Vertrauen in Ph5.x Rewrites. Audit-Fokus auf unverifizierte Handler. _(Empfohlen)_
- **B)** Ja — Score-10-Handler sollten unabhaengig vom Status auditiert werden. Paranoia ist billiger als Fehlersuche spaeter.
- **C)** Nur Sign (Score 10 + hoechste Komplexitaet). Terrain/Curve/Model/Buildings sind simpler und durch Rewrite abgedeckt.

### Q2: Trajectory ist nur Score 9, aber Cutscene ist Score 8. Warum Trajectory vor Cutscene?

- **A)** Trajectory hat 4 top-level variable Sections vs Cutscene 2. Mehr Oberflaeche fuer Bugs. _(Empfohlen)_
- **B)** Cutscene verwendet action_base das schon via Ph5.15 gefixt ist — residual risk niedriger.
- **C)** Beide gleichzeitig auditieren — unabhaengige Handler, parallelisierbar.

### Q3: Soll der Audit ein separates Diag-Tool-Binary bekommen oder in sector_audit.rs integriert werden?

- **A)** Neues Binary `handler_audit.rs` das alle Handler auf einmal testet. _(Empfohlen)_
- **B)** Pro Handler ein Binary (trigger_audit.rs, trajectory_audit.rs, ...) — mehr Fokus, mehr Code-Duplizierung.
- **C)** In `sector_audit.rs` erweitern — alles in einem Tool, waechst aber schnell.

### Q4: Soll der Audit auf ETS2 v907 base_map.scs beschraenkt bleiben oder auch DLC-Archive (dlc_east.scs, dlc_north.scs, ...) einschliessen?

- **A)** Nur base_map.scs — DLCs verwenden dieselben Handler. Zusaetzliche Item-Types sind das Problem, nicht falsche Handler. _(Empfohlen)_
- **B)** Alle verfuegbaren .scs-Archive — koennte Randfaelle in DLC-spezifischen Daten aufdecken.
- **C)** base_map + dlc_east (naechstes Ziel fuer Phase 6.4 ProMods East) — fokussiert.

---

## Appendix A: Handler-Index (sector.rs Zeilennummern)

| Handler-Funktion | Zeilen | Item-Type | Typ |
|-----------------|--------|-----------|-----|
| `skip_terrain` | 1108-1141 | 1 | Skip |
| `parse_buildings` | 1161-1182 | 2 | Parse |
| `skip_buildings` | 1187-1203 | 2 | Skip (Audit) |
| `parse_road` | 979-1013 | 3 | Parse |
| `parse_prefab` | 1015-1054 | 4 | Parse |
| `skip_model` | 1227-1244 | 5 | Skip |
| `skip_company` | 1247-1264 | 6 | Skip |
| `skip_service` | 1267-1273 | 7 | Skip |
| `skip_cut_plane` | 1276-1280 | 8 | Skip |
| `skip_city` | 1283-1290 | 12 | Skip |
| `skip_map_overlay` | 1293-1298 | 18 | Skip |
| `parse_ferry` | 1306-1321 | 19 | Parse |
| `skip_ferry` | 1325-1334 | 19 | Skip (Audit) |
| `skip_garage` | 1337-1345 | 22 | Skip |
| `skip_trigger` | 1348-1357 | 34 | Skip |
| `skip_fuel_pump` | 1360-1369 | 35 | Skip |
| `skip_sign` | 1392-1410 | 36 | Skip |
| `skip_bus_stop` | 1432-1438 | 37 | Skip |
| `skip_traffic_area` | 1441-1448 | 38 | Skip |
| `skip_bezier_patch` | 1456-1495 | 39 | Skip |
| `skip_trajectory` | 1498-1510 | 41 | Skip |
| `skip_map_area` | 1513-1518 | 42 | Skip |
| `skip_far_model` | 1521-1528 | 43 | Skip |
| `skip_curve` | 1554-1567 | 44 | Skip |
| `skip_cutscene` | 1606-1611 | 46 | Skip |
| `skip_visibility_area` | 1614-1620 | 48 | Skip |

Helper-Funktionen: `skip_subcurve` (1577-1603), `skip_action_base` (1744-1771), `skip_sign_board_override_list` (1774-1789), `skip_sign_override_list` (1792-1831), `skip_trajectory_rule_list` (1706-1719).

## Appendix B: Variable-Section-Count Matrix

Detaillierte Aufschluesselung jeder variablen Section pro Handler.

| Handler | Var-Section | Count-Typ | Element-Groesse | Nested? |
|---------|------------|-----------|-----------------|---------|
| **Terrain** | vegetation_spheres | u32 | 16 | Nein |
| | terrain_quad_data ×2 | u16/u32 mixed | 10/4/4/16/16 | Ja (7 sub-lists) |
| **Buildings** | height_offsets | u32 | 4 | Nein |
| **Prefab** | additionalParts | u32 | 8 | Nein |
| | nodes | u32 | 8 | Nein |
| | slaveItems | u32 | 8 | Nein |
| | corner_terrain | (node_count) | 12 | Indirekt |
| **Model** | additionalParts | u32 | 8 | Nein |
| **Company** | spawn_nodes | u32 | 8 | Nein |
| | spawn_counters | u32 | 4 | Nein |
| **Service** | node_refs | u32 | 8 | Nein |
| **CutPlane** | node_refs | u32 | 8 | Nein |
| **Garage** | node_refs | u32 | 8 | Nein |
| **Trigger** | token_list | u32 | 8 | Nein |
| | node_ref_list | u32 | 8 | Nein |
| | action_list | u32 | var (26+arith) | Ja (3 sub) |
| **FuelPump** | node_ref_list | u32 | 8 | Nein |
| | expansion_nodes | u32 | 8 | Nein |
| **Sign** | board_list | u8 | 24 | Nein |
| | pascal_string | u64 | 1 | Nein |
| | board_override_list | u32 | var (10-26) | Ja |
| | sign_override_list | u32 | var (attr-dispatch) | Ja |
| **TrafficArea** | token_list | u32 | 8 | Nein |
| | node_ref_list | u32 | 8 | Nein |
| **BezierPatch** | sphere_list | u32 | 20 | Nein |
| | materials | u16 | 10 | Ja (in TDQ) |
| | colors | u16 | 4 | Ja (in TDQ) |
| | quads | u32 | 4 | Ja (in TDQ) |
| | offsets | u32 | 16 | Ja (in TDQ) |
| | normals | u32 | 16 | Ja (in TDQ) |
| **Trajectory** | node_ref_list | u32 | 8 | Nein |
| | trajectory_rules | u32 | 12+arith | Ja |
| | checkpoints | u32 | 16 | Nein |
| | token_list | u32 | 8 | Nein |
| **MapArea** | node_ref_list | u32 | 8 | Nein |
| **Curve** | subcurves | popcount(u32) | 100+arith | Ja (height_offsets) |
| **Cutscene** | token_list | u32 | 8 | Nein |
| | action_list | u32 | var (26+arith) | Ja (3 sub) |
| **VisibilityArea** | item_ref_list | u32 | 8 | Nein |

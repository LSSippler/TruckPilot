# Spec: Route-aware Catmull-Rom Fallback an Kreuzungen

**Projekt:** TruckPilot — Lane-Keeper Plugin  
**Status:** Entwurf  
**Datum:** 2026-06-05  
**Autor:** Strategie/Spec (DeepSeek V4)

---

## 1. Problemzusammenfassung

An einer Kreuzung mit zwei Segmenten, die vom selben Graph-Node ausgehen,
wählt der Catmull-Rom-Fallback-Pfad einen Weg, der den Truck quer in die
Kreuzungsmitte zieht. Empirisch belegt an Kreuzung 1051105/1051103:

| Grösse | Wert | Bedeutung |
|---|---|---|
| near_seg | 1051105 | 640259→920642 (Abbieger) |
| final_seg | 1051103 | 640259→036609 (Route geradeaus) |
| intK | 57° | Interne Krümmung des Lookahead-Zielsegments |
| c_lat | 5.7→20.5 | Cross-Track des Catmull-Ziels springt |  
| heading_error | 1.20 rad | Folge: Stage=AutoReplan, Safety-Brake |

Ein adaptiver Lookahead-Fix (inverse-square, Floor 3.0m) ist implementiert
(`cf=0.513`, `look_m=3.0`) aber reaktiv: er misst intK auf final_seg, der
Lookahead kommt einen Frame zu spät.

---

## 2. Datenfluss-Analyse

### 2.1 Wie der Catmull-Rom-Pfad aktuell konstruiert wird

```
router.waypoints (A*-Knotenpositionen)
    │
    ▼
load_waypoints_from_blackboard (lane-keeper::tick, Zeile 1934-1943)
    │  Ruft smooth_catmull_rom(&pts, self.subdivisions) auf
    ▼
self.waypoints: Vec<[f64; 2]>  (geglättete Catmull-Rom-Punkte)
    │
    ▼  (im Fallback, compute_heading_error Zeile 1155-1227)
1. progress_idx-Advance (Zeile 1159-1167):
   • Nächster Waypoint < WAYPOINT_REACH_M (5m)? → advance
   • Rein euklidisch, kein Route-Bezug

2. Lookahead-Walk (Zeile 1213-1227):
   • Start: (tx, tz) = Truck-Position
   • Walk: alle remaining waypoints ab progress_idx+1
   • Akkumuliert euklidische Distanz bis look_ahead
   • KEIN Hop-Limit, KEINE Kink-Erkennung, KEIN Route-Constraint

3. Lane-Offset (Zeile 1277-1282):
   • Fester LANE_OFFSET_RIGHT_M = 1.875 m
   • Keine Lane-Count-Abhängigkeit

4. Heading-Error (Zeile 1319-1331):
   • Standard: dx.atan2(-dz) → wrap → PID
```

### 2.2 Wo near_seg statt final_seg die Pfad-Basis wird

**ANNAHME-ZU-VERIFIZIEREN:** Der Catmull-Pfad selbst verwendet `self.waypoints`
(aus `router.waypoints`), NICHT das nearest-Segment. Die Log-Meldung
"Der Catmull-Pfad wird auf near_seg (1051105) gebaut" ist eine *Fehlinterpretation
der Diagnostik*. Tatsächlich zeigt der Diag-Log:

- `lane_keeper.nearest_segment_id = 1051105` → **immer gesetzt** bei Eintritt
  in `try_spline_heading_error` (Zeile 487-488), noch bevor der Route-Check.
- `lane_keeper.lookahead_final_seg_id = 1051103` → gesetzt (Zeile 975-976)
  wenn der Arc-Length-Walk durchlief (vorheriger Tick, Spline war noch aktiv).

**Verifikation durch Claude Code:**  
`grep -n "nearest_segment_id" crates/plugins/lane-keeper/src/lib.rs` → Zeile 488  
`grep -n "lookahead_final_seg_id" crates/plugins/lane-keeper/src/lib.rs` → Zeile 976  
Beide Keys werden in `try_spline_heading_error` gesetzt. Bei fallback (None)
bleiben sie im Blackboard bis zum nächsten spline-success-Tick erhalten.

**Diese Werte stammen also aus VERSCHIEDENEN Ticks.** Der near_seg=1051105
aus dem aktuellen Tick (wo der nearest-Query den Abbieger fand), der
final_seg=1051103 aus dem letzten erfolgreichen Spline-Tick.

### 2.3 Wirkmechanismus des Problems

Die eigentliche Ursache liegt NICHT in einer Segment-Wahl im Catmull-Pfad,
sondern:

1. Der Spline-Pfad ist aktiv und läuft auf der Route (final_seg=1051103).
2. Am Node 640259 misst `final_internal_kink_deg` = 57° auf final_seg
   (der Kurvenradius des Route-Segments 1051103 durch die Kreuzung).
3. `prefab_curve_latched` = true → `try_spline_heading_error` gibt None.
4. Catmull-Fallback übernimmt.
5. **resync_progress_idx** (Zeile 1107-1121) scannt ALLE waypoints und findet
   den nächsten Punkt zum Truck. An der Kreuzung ist das ein Waypoint nahe 640259.
6. Der **Euclidean-Lookahead-Walk** (Zeile 1213-1227) läuft von (tx, tz) durch
   **alle remaining waypoints** bis `look_ahead` erreicht ist.
7. Da keine Hop-Begrenzung existiert, wrappt der Walk um **alle Route-Kurven**
   die innerhalb des Lookahead-Radius liegen. Dadurch springt der Zielpunkt
   lateral (c_lat 5.7→20.5).
8. Der heading_error springt → Stage=AutoReplan → Safety-Brake.

**Kernproblem:** Der Catmull-Walk hat kein Hop-Limit. Während der Spline-Pfad
per arc-length Walk exakt entlang der Route-Hops mit Kink-Detektion läuft
(Zeile 777-835), läuft der Catmull-Walk euklidisch durch alle Waypoints ohne
Hop-Begrenzung. Bei ausreichendem Lookahead (> ~15m) erreicht er Waypoints
jenseits des nächsten Route-Hops und damit auch Kurven in der Route.

### 2.4 Ist final_seg/Route-Info verfügbar?

Ja. Der Diag-Log zeigt `final_seg=1051103` als publizierten Key. Die Route-Info
liegt vor:

- `router.route_node_ids` → `self.cached_route_node_ids` im lane-keeper (Zeile 450)
- `self.node_progress_idx` → zeigt auf aktuellen Route-Node (Zeile 452)
- `self.seg_by_from_to` → HashMap<(from_uid, to_uid), segment_idx> (Zeile 1884-1886)
- `self.subdivisions` → 4 (Default, Zeile 198)

**Verifikation durch Claude Code:**  
`grep -n "cached_route_node_ids" crates/plugins/lane-keeper/src/lib.rs` bestätigt
dass die Route-Node-IDs als `Vec<u64>` gecached werden.  
`grep -n "seg_by_from_to" crates/plugins/lane-keeper/src/lib.rs` bestätigt die
HashMap auf Segment-Index.

---

## 3. Fix-Spezifikation

### 3.1 Kern-Idee

Der Catmull-Rom-Lookahead-Walk bekommt ein **Route-Hop-Limit**: er darf nur
Waypoints innerhalb der nächsten `N` Route-Hops (ab aktueller Position) walken,
nicht alle remaining Waypoints.

Der normale Fall (`near_seg == final_seg`, d.h. Spline-Pfad ist aktiv) ändert
sich nicht: der Catmull-Pfad wird nur im Fallback aktiv, und dort bekommt er
die Route-Begrenzung.

### 3.2 Mapping Route-Index → Waypoint-Index

`smooth_catmull_rom` erzeugt aus `n` A*-Nodes:

```
waypoints[j * subdivisions] = A* node j   (für j = 0..n-1)
```

Bei `subdivisions=4`:
- Node 0 → waypoints[0]
- Node 1 → waypoints[4]
- Node 2 → waypoints[8]
- Allgemein: Node j → waypoints[j * 4]

Waypoints für den Hop von Node k → Node k+1 liegen im Intervall:
`waypoints[k*4 ..= (k+1)*4]`

**ANNAHME-ZU-VERIFIZIEREN:** Die `smooth_catmull_rom`-Funktion (Zeile 1969-2003)
erzeugt tatsächlich diese Indexierung. Verifikation durch Claude Code:
Die `for i in 1..n`-Schleife fügt pro Segment `subdivisions` Punkte (ausser
erstes Segment: `subdivisions+1`), so dass `waypoints[j*4] = pts[j]` gilt.
Bei anderen `subdivisions`-Werten entsprechend anpassen.

### 3.3 Die Änderung im Detail

**Wo:** In `compute_heading_error`, Zeile 1218, die Schleife
`for &[px, pz] in &self.waypoints[(self.progress_idx + 1)..]`.

**Was ändern:**

```rust
// NEU: maximal erlaubte Route-Hops für den Catmull-Walk
let catmull_max_hops = 2; // Konfigurierbar per Blackboard

// NEU: berechne maximalen Waypoint-Index basierend auf Route-Hop-Limit
let max_waypoint_idx = if self.cached_route_node_ids.len() >= 2 {
    // Map progress_idx zurück auf Route-Node-Index
    let current_route_idx = self.node_progress_idx; // bereits gesetzt vom Spline-Code
    let max_route_idx = (current_route_idx + catmull_max_hops)
        .min(self.cached_route_node_ids.len().saturating_sub(1));
    // Der letzte Waypoint des max_route_idx-ten Nodes
    (max_route_idx * self.subdivisions)
        .min(self.waypoints.len().saturating_sub(1))
} else {
    self.waypoints.len() - 1 // Fallback: alle (keine Route-Info)
};

// Der Walk iteriert nur bis max_waypoint_idx
let end_idx = (self.progress_idx + 1 + max_waypoint_idx)
    .min(self.waypoints.len());
for &[px, pz] in &self.waypoints[(self.progress_idx + 1)..end_idx] {
    // ... unverändert ...
}
```

**Wichtig:** `self.node_progress_idx` wird bereits im `try_spline_heading_error`
gesetzt (Zeile 547), auch wenn die Funktion später None zurückgibt.
Der Wert bleibt dann im Struct erhalten.

**ANNAHME-ZU-VERIFIZIEREN:** `self.node_progress_idx` überlebt auch nachdem
`try_spline_heading_error` None zurückgegeben hat, weil der Struct nicht
zurückgesetzt wird. Verifikation: `grep -n "node_progress_idx"` zeigt dass
es ein Feldspeicher ist, der nur in `try_spline_heading_error` geschrieben wird.

### 3.4 Verhalten an der Übergangs-Stelle

- **near_seg == final_seg (Spline aktiv):** Catmull wird gar nicht betreten.
  Keine Änderung.
- **near_seg != final_seg ODER prefab_curve_latched:** Catmull-Fallback.
  `node_progress_idx` enthält den letzten gültigen Route-Index.
  Der Walk wird auf `catmull_max_hops` Hops begrenzt.
- **erster Catmull-Tick nach Spline:** `resync_progress_idx` läuft
  unverändert (Zeile 1107-1121). Der Lookahead-Walk wird aber route-begrenzt.

### 3.5 Anchor-Punkt (Pfad-Start)

Der Anchor ist die Truck-Position `(tx, tz)`, unverändert. Der Walk startet
von dort und läuft waypoints-weise. Durch die Route-Begrenzung kann der
Zielpunkt nicht mehr um ferne Kurven springen.

### 3.6 Auswirkung auf existierende Fixes

**Adaptiver Lookahead (inverse-square, Floor 3.0m):** Bleibt drin. Er ist
redundant wenn der Route-Hop-Limit den Walk korrekt begrenzt, aber er schadet
nicht. Seine Hauptwirkung (kürzerer Lookahead bei hohem intK) wird durch das
Hop-Limit ergänzt — der Lookahead kann jetzt noch kurz sein (3m) UND der Walk
bleibt auf dem korrekten Route-Hop.

**AutoReplan-Latch:** Bleibt drin. Er ist der Stage-Wechsel-Mechanismus,
nicht die Ursache. Durch den Fix springt heading_error nicht mehr,
also wird AutoReplan gar nicht erst aktiviert.

---

## 4. Edge-Cases und Risiken

### 4.1 `route_node_ids` fehlt (nicht publiziert)

Wenn `self.cached_route_node_ids.len() < 2` (weil der Router keine Route hat),
dann entfällt die Route-Begrenzung. Der Code fällt auf den aktuellen Walk
zurück (max_waypoint_idx = end of vector). Dies ist korrekt: wenn keine Route
vorliegt, gibt es keine Route-Hops zu begrenzen.

**Risk Level:** Keines — sauberer Fallback.

### 4.2 `node_progress_idx` ist stale

Wenn der Spline-Pfad seit >10s nicht mehr aktiv war (z.B. dauerhafter
Catmull-Betrieb), ist `self.node_progress_idx` möglicherweise nicht mehr
aktuell. Der Wert wurde zuletzt im Spline-Code gesetzt.

**ANNAHME-ZU-VERIFIZIEREN:** `node_progress_idx` wird in `try_spline_heading_error`
Zeile 547 gesetzt, BEVOR die Funktion möglicherweise None retourniert.
Bei jeder erfolglosen Route-Prüfung wird sie trotzdem gesetzt
(Zeile 517-546), da ja `node_progress_idx` vor allen early-returns zugewiesen
wird. Also ist der Wert nie stale solange der Spline-Pfad erreichbar ist
(SplineIndex vorhanden, Router-Graph vorhanden, Route gepublished).

**Verifikation:** Der grep-Befund zeigt dass `node_progress_idx` IMMER gesetzt
wird, sobald der Route-Check erreicht wird — selbst bei `off_route` Fallback.
Also ist er spätestens 1 Tick nach Route-Lade verfügbar.

### 4.3 `node_progress_idx` ausserhalb des Route-Bereichs

Wenn `current_route_idx + catmull_max_hops` den letzten Route-Index
überschreitet, wird `max_route_idx` einfach gecappt:
`.min(self.cached_route_node_ids.len().saturating_sub(1))`.

**Risk Level:** Keines.

### 4.4 `subdivisions != 4`

Die Mapping-Formel `waypoints[route_idx * subdivisions]` funktioniert für jeden
`subdivisions`-Wert → `subdivisions` kommt direkt aus dem Struct.

**Risk Level:** Keines.

### 4.5 Mehrfach-Abzweige (Node mit >2 Kanten)

Die Route-Node-IDs enthalten exakt den A*-Pfad. `node_progress_idx` zeigt auf
den aktuellen Index in diesem Pfad. Der nächste Hop ist `(route[j], route[j+1])`.
Da der A*-Pfad genau eine Kante pro Schritt wählt, ist der korrekte Hop
eindeutig.

**Risk Level:** Keines — die Route ist immer eindeutig.

### 4.6 Regressions-Risiko an bisher funktionierenden Kreuzungen

Der Fix beschränkt den Catmull-Waypoint-Walk. Bisher funktionierende Kreuzungen
hatten entweder:
- Keinen Catmull-Fallback (Spline war aktiv) → keine Änderung
- Einen Catmull-Fallback der trotz des unbegrenzten Walks funktionierte

Im zweiten Fall: Wenn der Walk bisher funktionierte obwohl er unbegrenzt war,
dann wird er mit Begrenzung auf 2 Hops entweder gleich gut oder besser
funktionieren (weil das Ziel nah bleibt).

**ANNAHME-ZU-VERIFIZIEREN:** Der Wert `catmull_max_hops = 2` ist ausreichend
für Geradeausfahrten. Der Lookahead beträgt bei langsamer Fahrt (z.B. 20 km/h)
ca. 15 m. Ein Route-Hop (A*-Knotenabstand) ist im Durchschnitt ~50-200 m lang
auf Landstrassen/Autobahnen. 2 Hops decken also 100-400 m ab → mehr als genug.
In der Stadt/auf Kreuzungen sind Hops kürzer (10-30 m), dort reicht Lookahead
von 15 m für 1 Hop. Der zweite Hop dient als Reserve.

Das minimale look_ahead = 3.0 m (selbst bei höchstem intK + Catmull min).
3 m sind deutlich weniger als ein durchschnittlicher A*-Hop → der Walk
erreicht selbst bei `catmull_max_hops=1` kaum den zweiten Wegpunkt.

**Risk Level:** Niedrig. Empirisch prüfbar durch Testfahrt.

### 4.7 `progress_idx` hinkt hinterher

Wenn der Catmull-Pfad über längere Zeit aktiv ist (weil `prefab_curve_latched`
über viele Ticks hält), und `progress_idx` durch den `WAYPOINT_REACH_M`-Advance
vorrückt (Zeile 1159-1167), dann kann `node_progress_idx` hinter der
tatsächlichen Position des Trucks liegen.

Der Unterschied: `progress_idx` ist der index IN den waypoints (Catmull).
`node_progress_idx` ist der index IN den route-node-ids (A*).
Beide laufen nach dem Spline→Catmull-Übergang auseinander.

**Lösung:** Der Fix muss im Catmull-Code from Grund auf den aktuellen
Route-Index bestimmen, nicht den gestohlenen `node_progress_idx`.
Dafür gibt es zwei Optionen:

**Option 3A (empfohlen):** Aus `progress_idx` den Route-Index zurückrechnen:
```rust
let current_route_idx = (self.progress_idx + self.subdivisions / 2) / self.subdivisions;
let max_route_idx = (current_route_idx + catmull_max_hops)
    .min(self.cached_route_node_ids.len().saturating_sub(1));
let max_waypoint_idx = max_route_idx * self.subdivisions;
```

Dies funktioniert unabhängig vom Spline-Pfad.

**Option 3B (Alternative):** `node_progress_idx` auch in `compute_heading_error`
aktualisieren. Aber das erfordert eine Neu-Berechnung des Route-Index aus den
Waypoints, was Option 3A bereits implizit macht.

**ANNAHME-ZU-VERIFIZIEREN:** Option 3A funktioniert weil `waypoints[j*subdivisions]`
der j-te A*-Knoten ist. Integer-Division von progress_idx / subdivisions gibt
den korrekten Index. Der half-step (`+ subdivisions/2`) gleicht Rundung aus
für den Fall dass progress_idx zwischen zwei Knoten liegt.

---

## 5. Verifikations-Plan

### 5.1 Erfolgskriterien (relevant für Testfahrt 1051105/1051103)

| Metrik | Erwartung | Diag-Key |
|---|---|---|
| `catmull_target_lateral_m` | < 6.0 bei c_along > 0 | `lane_keeper.catmull_target_lateral_m` |
| `catmull_target_lateral_m` Sprung | < 3.0 pro Frame | Differenz über 2 Ticks |
| `heading_stage` | "Normal" (nie "AutoReplan") | `state.heading_stage` |
| `lateral_source` | "spline_road" oder "catmullrom_fallback" | `lane_keeper.lateral_source` |
| `safety_autoreplan_secs` | 0.0 | `lane_keeper.safety_autoreplan_secs` |
| `heading_error` | stabil < 0.3 rad | `lane_keeper.error_rad` |
| `lookahead_final_seg_id` | 1051103 (Route) | `lane_keeper.lookahead_final_seg_id` |

### 5.2 Testfahrt-Protokoll

1. ETS2 mit forciertem Save auf Kreuzung 1051105/1051103 starten
2. `RUST_LOG=truckpilot_plugins=debug cargo run -p truckpilot-core -- daemon`
3. Diag-Keys 60s lang loggen (ab 10s vor Kreuzung bis 20s danach)
4. Prüfen: c_lat bleibt < 6.0, c_along steigt monoton (keine lateralen Sprünge)

### 5.3 Erwartete Diag-Key-Werte

```
# Vor Kreuzung (Normalbetrieb, Spline aktiv)
lane_keeper.lateral_source = spline_road
lane_keeper.error_rad = ~0.02..0.08
lane_keeper.catmull_target_lateral_m = [nicht gesetzt]

# An Kreuzung (Catmull-Fallback aktiv, route-constrained)
lane_keeper.lateral_source = catmullrom_fallback
lane_keeper.catmull_target_lateral_m = < 6.0
lane_keeper.catmull_target_along_m = > 0 (monoton steigend)
lane_keeper.error_rad = ~0.05..0.15 (KEIN Sprung auf 1.2)
lane_keeper.heading_stage = Normal
lane_keeper.fallback_reason = prefab_curve
lane_keeper.fallback_detail = internal_curve_XXdeg

# Nach Kreuzung (zurück zu Spline)
lane_keeper.lateral_source = spline_road
lane_keeper.error_rad = ~0.02..0.08

# NIE:
#   lane_keeper.heading_stage = AutoReplan
#   lane_keeper.fallback_reason = kink_stuck
#   lane_keeper.safety_autoreplan_secs > 0
```

### 5.4 Regressionstest-Kandidaten

- Andere bekannte funktionierende Kreuzungen (z.B. Autobahn-Abfahrten)
- Gerade Strecken mit Catmull-Dauerbetrieb (falls SplineIndex fehlt)
- Mehrfach-Kreuzungen hintereinander (komplexe Stadt-Knoten)

---

## 6. Implementierungs-Reihenfolge

### Schritt 1: Grundlage prüfen (Gate 1)

**Aufgabe:** Lese `crates/plugins/lane-keeper/src/lib.rs` konzentriert:
- `smooth_catmull_rom` (Zeile 1969-2003) → Mapping-Formel BESTÄTIGEN
- `compute_heading_error` (Zeile 1123-1336) → Walk-Schleife LOCATE
- `try_spline_heading_error` (Zeile 395-1105) → `node_progress_idx`-Set POINT
- `tick` (Zeile 1925-1945) → waypoints-Reload BESTÄTIGEN

**Gate 1 Check:** Die ANNAHME-ZU-VERIFIZIEREN-Punkte aus Sektion 3 sind
alle durch Code-Lektüre bestätigt. Wenn nicht, Spec anpassen.

### Schritt 2: Route-Hop-Limit-Konstante einführen (1 Änderung)

**Datei:** `crates/plugins/lane-keeper/src/lib.rs`  
**Änderung:** Füge Konstante `CATMULL_MAX_ROUTE_HOPS: usize = 2` im Konstanten-Block
(neben `CATMULL_CURVE_MIN_LOOK_AHEAD`, ca. Zeile 80).

**Kein Verhaltenseffekt** — noch kein Konsument.

### Schritt 3: Waypoint-Walk route-begrenzen (2 Änderungen)

**Datei:** `crates/plugins/lane-keeper/src/lib.rs`

**Änderung 1:** Nach `self.prefab_lateral_source = "catmullrom_fallback".to_string();`
(Zeile 1153) — berechne `max_waypoint_idx`:

```rust
// Phase 6-Spec: Route-begrenzter Catmull-Walk
// Verhindert dass der Lookahead um Route-Kurven springt.
let catmull_max_hops = CATMULL_MAX_ROUTE_HOPS;
let max_waypoint_idx = if self.cached_route_node_ids.len() >= 2 {
    let current_route_idx = (self.progress_idx + self.subdivisions / 2)
        .saturating_div(self.subdivisions)
        .min(self.cached_route_node_ids.len().saturating_sub(2));
    let max_route_idx = (current_route_idx + catmull_max_hops)
        .min(self.cached_route_node_ids.len().saturating_sub(1));
    (max_route_idx * self.subdivisions)
        .min(self.waypoints.len().saturating_sub(1))
} else {
    self.waypoints.len().saturating_sub(1)
};
```

**ANNAHME-ZU-VERIFIZIEREN:** `saturating_div` existiert. Falls nicht
(MSRV < 1.76), `checked_div().unwrap_or(0)` verwenden.

**Änderung 2:** Ersetze die Walk-Schleife (Zeile 1218):
```rust
// ALT:
for &[px, pz] in &self.waypoints[(self.progress_idx + 1)..] {
```
```rust
// NEU:
let walk_end = (self.progress_idx + 1 + max_waypoint_idx)
    .min(self.waypoints.len());
for &[px, pz] in &self.waypoints[(self.progress_idx + 1)..walk_end] {
```

**Gate 3 Check:** `cargo build` läuft durch. Keine Warnungen.
`cargo test -p truckpilot-plugins` (wenn existiert) oder `cargo test` gibt
keine neuen Fehler.

### Schritt 4: Catmull-Diagnostik erweitern (optional, empfohlen)

**Datei:** `crates/plugins/lane-keeper/src/lib.rs`

Füge Diag-Keys direkt nach max_waypoint_idx-Berechnung:

```rust
ctx.blackboard.set(
    "lane_keeper.catmull_max_hops",
    catmull_max_hops.to_string(),
);
ctx.blackboard.set(
    "lane_keeper.catmull_current_route_idx",
    current_route_idx.to_string(),
);
ctx.blackboard.set(
    "lane_keeper.catmull_max_waypoint_idx",
    max_waypoint_idx.to_string(),
);
ctx.blackboard.set(
    "lane_keeper.catmull_walk_end",
    walk_end.to_string(),
);
ctx.blackboard.set(
    "lane_keeper.catmull_total_waypoints",
    self.waypoints.len().to_string(),
);
```

**Gate 4 Check:** Testfahrt durch Kreuzung 1051105/1051103.  
`lane_keeper.catmull_target_lateral_m` < 6.0 (vorher 20.5).  
`lane_keeper.error_rad` bleibt < 0.3 rad (vorher 1.2).  
Stage bleibt "Normal".

### Schritt 5: Regression (Gate 5)

**Aufgabe:** Fahre eine bekannte funktionierende Route (z.B. Autobahn ohne
Kreuzungen) und prüfe dass die Lenkung nicht oszilliert.

**Gate 5 Check:** Keine neuen AutoReplan-Events auf der Teststrecke.
`lane_keeper.error_rad` bleibt im Normalbereich.

### Schritt 6: `catmull_max_hops` Konfiguration per Blackboard (Post-Fix)

Optional nach erfolgreicher Verifikation: Mach `CATMULL_MAX_ROUTE_HOPS`
überschreibbar via `plugin.lane_keeper.catmull_max_hops` (analog zu
`kink_stop_deg` etc.). Dann im UI justierbar.

---

## 7. Zusammenfassung der offenen Annahmen

| Annahme | Check | Risiko |
|---|---|---|
| `smooth_catmull_rom` erzeugt Mapping `waypoints[j*subdiv] = node j` | Code lesen Zeile 1969-2003 | Mittel: falsches Mapping → falsche Begrenzung |
| `node_progress_idx` ist im Catmull-Fallback verfügbar | Code lesen Zeile 517-546 (wird vor allen early-returns gesetzt) | Niedrig |
| `catmull_max_hops=2` ist in allen Fahrsituationen ausreichend | Empirisch prüfen | Mittel: bei sehr kurzen Hops (Stadt) evtl. zu wenig |
| `saturating_div` existiert auf MSRV | Build-Probe | Sehr niedrig |
| Der Weg `progress_idx` → Route-Index via Integer-Division funktioniert | Rechnen mit `subdivisions=4`: progress_idx 0..3 → route_idx=0, 4..7→1, etc. | Niedrig |

---

## 8. Referenzen

- **lane-keeper:** `crates/plugins/lane-keeper/src/lib.rs`
- **Catmull-Rom Smoothing:** `smooth_catmull_rom()` Zeile 1969-2003
- **Spline Arc-Length Walk (Referenzimplementierung):** Zeile 762-842
- **Catmull Walk (zu ändernd):** Zeile 1213-1227
- **progress_idx Advance:** Zeile 1159-1167
- **resync_progress_idx:** Zeile 1107-1121
- **route/waypoints Load:** `load_waypoints_from_blackboard` Zeile 367-393
- **prefab_curve_latched Logik:** Zeile 914-962
- **Konstanten:** Zeile 39-79

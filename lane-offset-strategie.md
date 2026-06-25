# Lane-Offset-Strategie: Woher kommt die korrekte laterale Soll-Position?

## Status-quo (Code-Inventar, Stand heute)

**Spline-Pfad (Road-Segmente):** Der Offset ist **bereits lane-count-abhaengig**.
`SegmentMetadata.lane_offset_right_m` wird in `spline.rs:375-382` pro Edge
berechnet als:

```
target_lane = (lanes - 1) - TARGET_LANE_FROM_RIGHT  // TARGET_LANE_FROM_RIGHT = 0
lane_offset_right_m = (target_lane + 0.5) * lane_width_m + road_offset_m
```

- 1-spurig: `0.5 * 3.75 + 0 = 1.875 m`
- 2-spurig: `1.5 * 3.75 + 0 = 5.625 m`
- Mit `road_offset_m` (z.B. `ger7` → 1.0 m Median-Shift) additiv

Die Datenquellen (`lane_width_m`, `lanes`, `road_offset_m`) kommen aus
`road_look.sii`, sind vollstaendig durch die Pipeline gepumpt:
`RawRoad.road_type_token` → `GraphEdge.lane_width_m` `GraphEdge.lanes`
`GraphEdge.road_offset_m` → `SegmentMetadata` → Lane-Keeper.

**Spline-Pfad (Prefab/NavCurve-Segmente):** Offset = **0.0** (NavCurve IST
bereits die Spur-Mitte). `lane-keeper/lib.rs:1298`:
```rust
Some(m) if m.is_prefab => (0.0, "spline_prefab"),
```

**Catmull-Rom-Fallback:** Nutzt HARTVERDRAHTET `LANE_OFFSET_RIGHT_M = 1.875`
(lane-keeper/lib.rs:1673), egal ob 1-/2-/3-spurig. Kommentar in
lane-keeper/lib.rs:1679-1684 dokumentiert das bewusst als Befund (1).

**Schwaeche 1 (Kurven-Bias):** Der Offset wird via Tangenten-Normale
angewendet:
```rust
n = (-tan.z, tan.x) / |tan|
lookahead + n * lane_offset
```
Dies ist geometrisch die *Parallelkurve* (offset curve) zum Lookahead-Punkt.
Auf einem Kreisbogen mit Radius R liegt der offsetierte Punkt auf Radius
(R + d) (Aussenkurve) bzw. (R - d) (Innenkurve) — das ist **korrekt**.
Der beobachtete 5.5deg-herr-Sockel auf Connector-Segmenten kommt
hoeherwahrscheinlich NICHT aus der Geometrie des Offsets selbst, sondern
daraus dass der Connector UEBER Catmull laeuft (irgendein Fallback-Grund)
und der 1.875m-Festoffset auf einer schmalen/prefab-internen Spur
ueberdimensioniert ist.

---

## Option A: NavCurve kodiert die Spur bereits

**Status: BESTAETIGT.** NavCurve-Segmente (is_prefab=true) bekommen bereits
Offset=0. Die existierende Code-Stelle in lane-keeper/lib.rs:1298 ist korrekt.

**Hypothese fuer Road-Segmente:** NavCurves/AI-Lane-Geometrie auf Road-Segmenten
reprasentiert die **Strassen-Centerline**, NICHT die einzelne Spur. Der Offset
von der Centerline zur rechten Spur ist notwendig. Die existierende Berechnung
in spline.rs ist geometrisch korrekt.

**Empirische Verifikation:** Vergleich NavCurve-Punkte gegen bekannte
Lane-Mitten auf einem geraden 2-spurigen Segment:
1. Graph-Export: HermiteSegment[(p0,p1)] auf einer geraden 2-spurigen Autobahn.
2. Die NavCurve-Mitte liegt (bei p0 und p1) zwischen den beiden Spuren
   (Centerline). Der Abstand NavCurve→rechter Fahrbahnrand sollte ca.
   `lane_width_m * 2` = 7.5m betragen (bei 3.75m Spurbreite).
3. Wenn die NavCurve schon die rechte Spur waere, waere der Abstand zum Rand
   nur ~1.875m.
4. Einfachster Audit: `road_look_probe.rs` Token-Match auf einem konkreten
   Segment checken, dann Weltkoordinaten der NavCurve-Spline-Endpunkte gegen
   Google-Maps/SCS-Coords visualisieren.

**Entscheidung:** Option A ist bereits umgesetzt (NavCurve=Spur bei Prefabs,
Centerline bei Roads). Keine Aenderung noetig. Die Kernfrage ist
empirisch beantwortet.

---

## Option B: Lane-Count + road_look-Breite

**Status: BEREITS IMPLEMENTIERT — fuer den Spline-Pfad.**
Die `lane_offset_right_m` in `SegmentMetadata` IST lane-count-abhaengig mit
echter Lane-Breite aus road_look. Der spline-Pfad in `try_spline_heading_error`
liest dies korrekt.

**Luecke:** Der **Catmull-Rom-Fallback** nutzt den festen 1.875m-Wert.
Auf 2-spurigen Strassen im Catmull-Fallback ist der Offset somit um
Faktor ~3 zu klein (1.875m statt ~5.625m).

**Aufwand:** S (fuer den Catmull-Backport).

**Aenderungsumfang:**
1. `SegmentMetadata` ist im Lane-Keeper bereits verfuegbar (via
   `ctx.blackboard`-Publizierung aus lane-follower).
2. Catmull-Fallback braucht Zugriff auf den lane_offset des aktuellen Segments.
   Einfachste Loesung: vor dem Fallback den `lane_offset_right_m` aus den
   Metadaten des nearest_segments oder des Truck-Segments lesen, statt
   `LANE_OFFSET_RIGHT_M` zu verwenden.
3. Alternativ: `SegmentMetadata` im Lane-Keeper lokal cachen (keyed by
   nearest_segment_id), so dass auch der Fallback per-segment-Offset nutzen
   kann.

**Risiko:** Gering. Fallback-Fall wird auf korrekten Offset umgestellt.
Regression: keine — Spline-Pfad aendert sich nicht.

**Synergie Phase 6.3:** Geringe direkte Synergie (road_look-Lane-Daten sind
schon erschlossen). Indirekt: wenn Phase 6.3 `SegmentMetadata` um
`speed_limit_kmh` erweitert, arbeitet man an derselben Datenstruktur.

---

## Option C: Kurven-korrigierter Offset

**Analyse:** Der vermutete Fehler (Normal-Vektor zeigt auf Kurven nach aussen)
ist geometrisch nur dann relevant, wenn die Lookahead-Parallelkurve nicht mit
der Lane-Mitte uebereinstimmt. Fuer einen Kreisbogen IST die Tangenten-Normale
identisch zur Frenet-Normale; die Parallelkurve liegt exakt auf Radius
(R + d). Eine echte Frenet-Korrektur (zweite Ableitung / Krummung) ist in
diesem Fall nicht notwendig.

**Wann trotzdem relevant:** Wenn das Hermite-Spline die Road-Geometrie nicht
exakt abbildet (z.B. weil p0/p1 weit auseinander liegen und die
Hermite-Interpolation zwischen den Quaternion-Tangenten die wahre
Strassenkruemmung nur approximiert). Dann kann der Tangenten-Normal-Offset
einen systematischen Fehler erzeugen, weil die Spline-Tangente nicht die
tatsaechliche lokale Kruemmungsrichtung repraesentiert.

**Aufwand:** M. Erfordert Kruemmungsberechnung (zweite Ableitung des
Hermite-Splines) und Frenet-Normalen-Richtung pro Lookahead-Punkt. Nur im
Spline-Pfad anwendbar; Catmull-Fallback hat noch grobere Geometrie.

**Nutzen:** Marginal, solange Catmull-Fallback das groessere Problem (1.875m
fuer alle) hat. Erst lohrend NACHDEM Option B (Catmull-Fallback-Lane-Offset)
gefixt ist, wenn dann immer noch ein messbarer Kurven-Bias uebrig bleibt.

**Entscheidung:** Pflaster. Zurueckgestellt bis Option B live ist.

---

## Option D: Vision-gemessener Offset

**Aufwand:** L (UFLD v2 Integration + Kamera/Welt-Transform + Heading-Kalibrierung).

**Richtige Rolle:** Mess-Werkzeug zur Verifikation von A/B/C. Die Kamera
kann unabhaengig validieren ob der Truck in der Spur-Mitte liegt — das ist
der ultimative Ground Truth.

**Nicht geeignet als:** Primaere Offset-Quelle. Die
Kamera-zu-Welt-Transformation (insb. das Heading-Problem) macht es
fehleranfaelliger als die Graph-basierte Loesung. Und die Kamera kann
ausfallen (Nacht, Regen, verdeckte Lane-Markings).

**Empfehlung:** Vision als **Offline-Validierungstool** aufbauen (loggleiche
Fahrt mit Kamera-Aufzeichnung + Telemetrie → Offline-Vergleich Soll/1st-Lane-
Position), nicht als Online-Feedback. Erspart Echtzeit-Anforderungen und das
Heading-Problem.

---

## Empfehlung: Gestufte Strategie

### Stufe 1 — Zero/Minimal (Aufwand: S)
**Catmull-Fallback auf per-Segment-Lane-Offset umstellen.**

Das ist der groesste Hebel: behebt Schwaeche 2 und 3 fuer ALLE Catmull-
gesteuerten Strecken (incl. Connector-Fallbacks). Der Spline-Pfad ist
bereits korrekt; die Catmull-Luecke ist der dominante Restfehler.

Aenderung in lane-keeper/lib.rs `compute_heading_error` (Z. 1669-1674):
Statt `LANE_OFFSET_RIGHT_M` den `lane_offset_right_m` aus den Metadaten
des aktuellen Segments verwenden. Die Metadaten sind ueber die Blackboard-
Publizierung des lane-follower oder einen direkten Cache im Lane-Keeper
erreichbar.

**Erwartete Wirkung:**
- 2-spurige Autobahn im Catmull-Fallback: Offset 1.875 → ~5.625 m (korrekt)
- Connector-Fallbacks: Offset 1.875 → 0.0 oder kleiner (NavCurve liegt auf
  Spur-Mitte) → Kurven-Bias (Schwaeche 1) verschwindet ggf. automatisch,
  weil der Ueber-Offset auf Connectors entfaellt.

### Stufe 2 — Wenn Stufe 1 nicht reicht (Aufwand: M)
**Kruemmungskorrigierter Offset im Spline-Pfad.**

Falls nach Stufe 1 auf Spline-gesteuerten Kurven immer noch ein messbarer
herr-Sockel bleibt (z.B. >1deg bei 18-28deg Kurvenkruemmung), dann:

1. Zweite Hermite-Ableitung berechnen → Frenet-Normale.
2. Offset nicht entlang der Tangenten-Normalen, sondern entlang der
   Frenet-Normalen (oder alternativ: den Lookahead-Punkt auf der
   *Parallelkurve* durch numerische Integration bestimmen statt einmaliger
   Normalen-Verschiebung).

### Stufe 3 — Validierung (Aufwand: M)
**Vision-basierte Offline-Validierung.**

UFLD v2 aufzeichnen, Kamera-Bilder mit Telemetrie-Log synchronisieren,
Offline die Lane-Position in Weltkoordinaten rechnen und mit Graph-Soll-
Position vergleichen. Liefert harte Evidenz ob Stufe 1+2 ausreichen oder
ob es einen systematischen Restfehler gibt.

---

## Naechster konkreter Schritt

**Spike (Aufwand: 1-2h): Catmull-Fallback-Lane-Offset reparieren.**

1. In `lane-keeper/src/lib.rs` die Funktion `compute_heading_error`
   identifizieren.
2. Zugriff auf SegmentMetadaten im Lane-Keeper herstellen:
   - `SplineIndex.metadata` ist bereits im Lane-Keeper verfuegbar (via
     `self.spline_index`).
   - Im Catmull-Fallback gibt es aber keinen `final_seg`-Index, weil die
     Catmull-Geometrie aus Route-Waypoints lebt, nicht aus Spline-Segmenten.
   - Loesung: Den `lane_offset_right_m` des *naechstgelegenen Spline-Segments*
     (oder des letzten bekannten `cur_seg`) als Offset im Catmull-Fallback
     nutzen. Oder den offset aus dem lane-follower-Blackboard lesen.
3. Vorher/Nachher mit derselben Test-Strecke (2-spurige Autobahn mit
   Connector-Kreuzung) aufzeichnen und herr vergleichen.

**Empfohlener Audit davor (30min):**
- `road_look_probe.rs` auf dem Connector-Segment laufen lassen, das den
  5.5deg-herr-Sockel zeigt. Token-Match checken, Lane-Count/Breite ausgeben.
- Bestimmen ob dieser Connector im Spline- oder Catmull-Pfad war (via
  Blackboard `lane_keeper.fallback_reason` waerend der Fahrt).
- Das beantwortet sofort: ist das Problem (a) Catmull-1.875m-Ueberoffset
  oder (b) echter Spline-Kurven-Bias?

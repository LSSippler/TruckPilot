# TruckPilot Lane Graph Design

## Ziel

Der aktuelle Graph modelliert Straßen als einzelne Kanten zwischen Knoten. Für
fahrspurgenaue Navigation wird ein **Lane-Graph** benötigt:

1. Parallele Vorwärtskanten pro Fahrspur (Lane 0..N-1)
2. Explizite Spurwechsel-Kanten zwischen benachbarten Spuren
3. Penalty-basierte Kosten für Spurwechsel, damit A* unnötige Wechsel vermeidet

Damit können wir später robust umsetzen:

- „rechts halten“ / „links halten“
- korrekte Spurwahl vor Abfahrten
- weniger Zick-Zack in dichter Verkehrsumgebung

---

## Datenquellen

Primäre Felder aus den Road-Daten:

- `lane_count_forward`
- `lane_count_backward`

Falls ein Segment bidirektional ist, werden Vorwärts- und Rückwärtsrichtung
jeweils separat in lane-spezifische Kanten aufgeteilt.

Fallback-Regel:

- Wenn Lane-Information fehlt oder ungültig ist: `1` Lane pro Richtung

---

## Graph-Knoten (Lane Sub-Nodes)

### Bestehender Knoten

`Node(uid, x, z, ...)`

### Erweiterung

Für jeden Richtungsübergang und jede Lane wird ein Sub-Knoten erzeugt:

`LaneNode(base_uid, direction, lane_index)`

Beispiel:

- `LaneNode(123, Forward, 0)`
- `LaneNode(123, Forward, 1)`
- `LaneNode(123, Backward, 0)`

Die geometrische Position bleibt zunächst die Basisknoten-Position. Eine spätere
Version kann Lane-Offsets entlang der Straßenachse einführen.

---

## Graph-Kanten

## 1) Vorwärtskanten pro Lane

Für jede Lane wird eine normale Fahrkante erzeugt:

`LaneEdge(from_lane_node, to_lane_node, base_cost)`

Kosten-Basis wie bisher (Distanz/ETA).

## 2) Spurwechsel-Kanten

Zwischen benachbarten Lanes derselben Richtung:

- `Lane i -> Lane i+1`
- `Lane i+1 -> Lane i`

Nur wenn Wechsel laut Segmenttyp erlaubt ist.

Kosten:

`lane_change_cost = lane_change_penalty + local_distance_component`

Empfohlene Startwerte:

- `lane_change_penalty = 10m` (Distance mode)
- ETA-Mode: ~1.0–1.5s Zusatzkosten

---

## A*-Anpassung

Aktuell: Zustand = `base_node_uid`

Neu: Zustand = `(base_node_uid, direction, lane_index)`

Heuristik:

- weiter euklidische Distanz auf Basisknoten
- optionaler Lane-Term nahe Ziel (bevorzugte Zielspur)

Kostenfunktion:

- normale Fahrkosten auf Lane-Kanten
- Zusatzkosten auf Lane-Change-Kanten

Dadurch bleibt A* kompatibel und determiniert, aber lane-aware.

---

## Speicherverbrauch (grobe Schätzung)

Ausgangswerte (realistische ETS2-Mitte):

- ~65k Roads
- durchschnittlich 2–3 Lanes je Richtung (hier konservativ 2.2)

Grob:

- Knotenfaktor: ~2.2x bis ~3.0x gegenüber Base-Graph
- Kantenfaktor: ~2.5x bis ~4.0x (inkl. Lane-Change)

Wenn Base-Graph ~230k Edges hat:

- Lane-Graph: ~575k bis ~920k Edges

Bei ~40–56 Bytes je Edge (inkl. Overhead, grob):

- Edge-Speicher ~22 MB bis ~52 MB

Mit Knoten + Indizes + A*-Strukturen insgesamt:

- realistisch ~80 MB bis ~180 MB RAM

=> Für Desktop-Betrieb akzeptabel, für sehr große Mods evtl. optional schaltbar.

---

## Implementierungsplan (2–3 Wochen)

## Woche 1

- Datenmodell (`LaneNode`, `LaneEdge`) hinzufügen
- Builder: Base-Roads -> Lane-Kanten
- Grundtests (Determinismus, Kantenanzahl, Fallback auf 1 Lane)

## Woche 2

- Lane-Change-Kanten + Penalties
- A*-State auf Lane erweitern
- Integrationstests auf realen Graphen

## Woche 3 (Puffer/Feinschliff)

- Performance-Optimierung (Nachbarschaftslisten, Pruning)
- Zielspur-Heuristik vor Ausfahrten
- Debug-Export (Lane-Graph JSON / Visualisierung)

---

## Offene Fragen

1. Lane-Offsets geometrisch sofort oder erst später?
2. Lane-Change überall erlauben oder segmentabhängig?
3. Interop mit Prefab-Logik (Kreuzungen) für sauberen Lane-Merge?

Diese Fragen beeinflussen primär Qualität, nicht das Grundgerüst.

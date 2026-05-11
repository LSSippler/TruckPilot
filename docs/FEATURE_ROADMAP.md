# FEATURE_ROADMAP.md

## Aktueller Stand
- Spatial-Graph mit Nodes und Roads
- Kein Lane-Graph, keine Verkehrsregeln

## Ziel 1 – Fahrspurerkennung
- Nutzung der geparsten `lane_count_forward` und `lane_count_backward`
- GraphBuilder erweitert um parallele Lane-Kanten (wie im Rust-Original)
- Lane-Change-Kanten zwischen benachbarten Fahrspuren (Lane-Mapping notwendig)
- Geschaetzte zusaetzliche Nodes: Faktor 4–6 -> ~1 Mio Nodes

## Ziel 2 – Ampeln und Vorfahrtsregeln
- Nutzung der Sign-Items (Typ 36) zur Identifikation von Ampeln und Vorfahrtsschildern
- Nutzung der TrafficRule-Items (Typ 38), falls parsebar
- Integration in A* als temporaere Kanten-Penalties (keine harten Barrieren)

## Ziel 3 – Adaptive Cruise Control (ACC)
- Nutzung des Telemetrie-Channels `SCS_TELEMETRY_TRUCK_CHANNEL_distance_to_lead_vehicle` (falls verfuegbar)
- Erweiterung des PID-Reglers um einen Abstands-Regelkreis

## Ziel 4 – Dynamische Stauumfahrung
- Nutzung von ETS2-Events (Unfall, Stau) – aktuell nicht im SDK dokumentiert
- Fallback: kamerabasierte Stauerkennung (Zukunftsmusik)

## Zeitplan (Schaetzung)
- Fahrspuren: 2–3 Wochen
- Ampeln: 1–2 Wochen
- ACC: 1 Woche
- Stauumfahrung: offen

## Abhaengigkeiten
- Keine. Alle benoetigten Daten sind bereits im Map-Parser extrahiert.

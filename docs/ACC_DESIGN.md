# TruckPilot ACC Design Notes

## Fragestellung

Existiert im offiziellen SCS Telemetry SDK ein Kanal
`SCS_TELEMETRY_TRUCK_CHANNEL_distance_to_lead_vehicle`?

## Ergebnis

**Nein (nicht gefunden).**

Prüfung erfolgte gegen:

1. Offizielle SDK-Header im Projekt (`TruckPilot.TelemetryDLL/scs_sdk/include/...`)
2. Modding-Wiki-Seite Telemetry SDK (`modding.scssoft.com`)

In den verfügbaren Channel-Headern ist kein Symbol mit
`distance_to_lead_vehicle` bzw. ähnlicher „lead vehicle distance“-Semantik
enthalten.

---

## Konsequenz

Ein klassisches Adaptive Cruise Control (ACC) mit direkter
Vordermann-Abstandsregelung ist über reine SCS-Telemetrie **nicht direkt**
möglich, solange dieser Kanal nicht im SDK bereitgestellt wird.

Es gibt daher **keinen sinnvollen SHM-Offset** für einen solchen Kanal in der
aktuellen TruckPilot-Layout-Definition.

---

## Mögliche Alternativen

## 1) Vision-/Sensor-basierte Abstandsschätzung (extern)

- Bildschirm-/Kamerabild analysieren (Object Detection + Depth Approximation)
- Führungsfahrzeug erkennen und Pixel-zu-Meter kalibrieren
- Hohe Komplexität, zusätzliches ML-/CV-Subsystem nötig

## 2) Proxy-Ansatz über Dynamiksignale

- Kanäle wie `local_acceleration`, `speed`, `effective_brake`, `effective_throttle`
  können nur indirekte Hinweise geben
- Kein echter Abstand, nur Reaktions-/Dynamikschätzung
- Für echtes ACC unzuverlässig

## 3) Karten-/Verkehrsmodell (Offline-Heuristik)

- Geschwindigkeitsprofile je Straßensegment und konservative Zielgeschwindigkeit
- Kein Fahrzeug-following, eher intelligenter Tempomat

---

## Empfehlung für TruckPilot

Kurzfristig:

- Kein echtes ACC implementieren, solange kein Distanzkanal verfügbar ist
- Stattdessen „Adaptive Speed Profile“:
  - Kurvenradius-basierte Zielgeschwindigkeit
  - Prefab-/Kreuzungs-/Abfahrts-Penalties
  - optionales Safety-Braking bei starker longitudinaler Beschleunigungsänderung

Mittelfristig:

- Optionales Vision-Modul als separates, klar entkoppeltes Subsystem
- API-Schnittstelle im Controller-Layer vorbereiten (`optional lead_distance_m: Option<f32>`)

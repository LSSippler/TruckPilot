---
created: 2026-05-16
updated: 2026-05-16
tags: [phase, truckpilot, vision, sign-vision, templates, ncc, closeout]
status: implemented
phase: 6.5h
outcome: km_unmapped 88% → 16% mit 4 real-bild NCC-Templates (40/60/80/100); Latenz unverändert
commit: ae2e1054
---

# Phase 6.5h — Real-Sign-Templates v1

**Status:** Implementiert, Live-verifiziert via Multi-Video-Replay
**Datum:** 2026-05-16
**Commits:** `8e509fb7` (Task 1) → `a07eeb64` (Tasks 3-4) → `ae2e1054` (Templates final)
**Vorgänger:** [[Phase-6.5f-Detection-Quality-Spec]]

## Story

### Befund (Phase 6.5f Daten, 30-Min-Real-Fahrt)

Nach dem Detection-Quality-Rollout zeigte das erste Live-Telemetrie-Sample
ein klares Lokalisierungs-Problem auf der Klassifizier-Stufe, nicht beim
Detektor:

- 56 `speed_limit_sign` von YOLO detektiert
- 49 davon (88 %) landeten in `sign.class.unmapped`
- nur 3 (5 %) wurden als `km_100` gemappt
- keine Tempo-40/60/80 Treffer trotz dichtbesiedeltem Streckenmix

Der Detektor selbst arbeitet sauber, die NCC-Klassifizier-Stufe versagt.

### Hypothese

Die in Phase 6.5e geshippten Templates waren synthetische 5×7-Bitmap-Render
aus einer Font-Fallback-Routine. Auf realen ETS2-Schildern unterscheidet
sich der Render-Stil deutlich:

| Aspekt | Synthetisch 5×7 | Real-Schild ETS2 |
|---|---|---|
| Auflösung | 5×7 Glyphe | 30–60 px Höhe pro Ziffer |
| Anti-Aliasing | keines | sub-pixel gefiltert |
| Strichstärke | uniform | proportional skaliert |
| Hintergrund | weiß flat | leichte Tönung / Kompression |

NCC reagiert empfindlich auf Strichstärke und AA-Profil; ein Mismatch hier
trifft die Korrelation direkt.

### Workflow (Tasks 1–4)

1. **Frame-Capture erweitert** (`8e509fb7`): `vision-pipeline-capture` bekam
   ein `--save-frames-dir` Flag, sodass jeder publizierte JPEG parallel zur
   SHM auf Disk landet. Mirror, nicht Hauptpfad, damit Latenz unverändert
   bleibt.
2. **Crop-Extraktion** (`a07eeb64`): `tools/model-tools/extract_sign_crops.py`
   liest das NDJSON-Eventfile + den Frame-Ordner und schneidet Bounding-Boxen
   für alle `speed_limit_sign`-Detections aus.
   - 60 unmapped + 8 mapped Crops aus der ersten Capture-Session
3. **Hand-Label-Pass:** im Crops-Set sichtbar nur 40 / 60 / 80 / 100.
   Tempo 30 / 50 / 70 / 130 fehlten komplett — keine geeigneten
   Frames im aktuellen Recording.
4. **Template-Build** (`ae2e1054`): `build_sign_templates.py` mittelt pro
   Klasse die normalisierten Crops zu einem Master-Template und embedded das
   Ergebnis als statisches Array in `speed_templates_real.rs`. Insgesamt
   66 Crops in 4 Klassen-Templates verbacken.
5. **Tests:** 56 sign-vision Tests grün, kein Regression in
   Decode/Inference-Pfaden.

### Verifikation (Task 5, Multi-Video-Replay)

Statt einer zweiten Live-Fahrt: Drei OBS-Recordings aus Phase 6.5e
hintereinander durch den neuen `vision-pipeline-capture replay`-Command in
die SHM gespielt, FPS=120 mit Worker-Drop-Modus, sodass die Inference-Stage
ihre native Kadenz fährt.

| Metrik | Wert |
|---|---|
| Replay-Dauer | 35.4 min |
| Video-Frames gespielt | 254 933 |
| `inference_calls` | 14 634 |
| Dropped Inferences | 0 |
| `speed_limit_sign` detected | 430 |
| `tick_blocking_ms` | 0.00 |
| `last_inference_ms` | 101.5 |

## Vorher / Nachher

Klassen-Verteilung der `speed_limit_sign`-Treffer:

| Klasse | Vorher (30 min real) | Anteil | Nachher (35.4 min replay) | Anteil |
|---|---:|---:|---:|---:|
| km_40 | 0 | 0 % | 112 | 26 % |
| km_60 | 0 | 0 % | 161 | 37 % |
| km_80 | 0 | 0 % | 18 | 4 % |
| km_100 | 3 | 5 % | 52 | 12 % |
| km_unmapped | 49 | 88 % | 69 | 16 % |
| **Gesamt** | **56** | 100 % | **430** | 100 % |

Latenz-Budget unverändert:

| Metrik | Vorher | Nachher |
|---|---:|---:|
| `tick_blocking_ms` | 0.00 | 0.00 |
| `last_inference_ms` | ~100 | 101.5 |

## Was funktioniert

- 4 Real-Templates feuern stabil: 40 / 60 / 80 / 100
- Klassen-Mix passt zur Strecken-Demografie (Landstraße + Autobahn-Mix)
- Inference-Latenz unverändert; Real-NCC-Templates haben dieselbe Form wie
  die Synthetik-Templates, kein neuer Hot-Path
- Replay-Pipeline als reproduzierbarer Verifikations-Pfad etabliert

## Was offen ist

- **Tempo 30 / 50 / 70 / 130 fehlen** als Templates, weil keine Crops im
  aktuellen Recording verfügbar waren. Nächste Live-Fahrt durch Ortschaften
  und niedrig limitierte Bereiche schließt die Lücke.
- **16 % Restquote** in `km_unmapped`. Mischung aus echten Out-of-Set-Schildern
  (30/50/70/130 sichtbar im Replay aber nicht template-gestützt), Crop-Border-
  Cases (zu klein, halb verdeckt) und vermutlich einigen False-Positives auf
  schilderartigen Texturen.
- Templates sind aktuell Mittelwerte über sehr wenige Crops (≤ 20 pro Klasse).
  Robustheit gegen Beleuchtung / Wetter unklar.

## Lessons Learned

- **Synthetische Bitmap-Templates sind keine valide Approximation für
  Real-Sign-NCC.** Render-Stil, Auflösung und Anti-Aliasing weichen so weit
  ab, dass NCC unter Threshold fällt. Die Synthetik-Templates aus 6.5e
  waren ein Bootstrap, keine produktionsfertige Klassifizier-Basis.
- **Real-NCC-Templates aus Live-Crops liefern direkt brauchbare Match-Rates**,
  selbst mit kleiner Crop-Anzahl (8–20 pro Klasse). Der größte Hebel liegt
  bei der Stilanpassung an den realen Renderer, nicht bei der Sample-Menge.
- **Multi-Video-Replay bei FPS=120 + Worker-Drop ist die richtige Größe**
  für Verifikations-Sweeps. Reproduzierbare Counter ohne den Aufwand einer
  zweiten Live-Fahrt; gleicher Code-Pfad bis auf die Frame-Quelle.
- **Crop-Inventory frühzeitig prüfen.** Hand-Label hat erst nach Crop-Extrakt
  gezeigt, dass 30/50/70/130 fehlen. Künftig: vor dem Template-Build die
  Klassen-Coverage gegen Ziel-Set vergleichen, damit Lücken bewusst sind.

## Nächste Schritte

- Live-Fahrt durch Ortschaften für 30 / 50 / 70 Crops
- Phase 6.5i: Template-Robustheit (mehr Crops pro Klasse, Beleuchtungs-Mix)
- Threshold-Tuning auf Real-Verteilung statt Synthetik-Bias

---
created: 2026-05-16
updated: 2026-05-16
tags: [phase, truckpilot, vision, sign-vision, diagnosis, closeout]
status: closed
phase: 6.5h-diag
outcome: Template-Matcher als Fallback bestaetigt; v2/v3 + YOLO-Retrain out-of-scope
---

# Phase 6.5h-Diag — SpeedLimit Generic vs Specific Classification

**Status:** Closed
**Datum:** 2026-05-16
**Vorgaenger:** [[Phase-6.5h-Real-Sign-Templates]]
**Artefakte:** `tools/model-tools/diag_speed_limit_classification.py`, `outputs/diag/speed_limit_classification_diagnosis.md`

## Story

Nach dem 6.5h v1 Closeout (4 Real-Templates fuer 40 / 60 / 80 / 100,
km_unmapped 88 % auf 16 % gesenkt) war der naechste Schritt geplant als
6.5h v2: weitere Crops sammeln, Lueckenfueller-Templates fuer 30 / 50 / 70 / 130.

Der erste Verifikations-Lauf zu v2 war ein 21.6 Min Multi-Video-Replay v2
bei FPS=240 mit Stream-Persistenz nach NDJSON. Dabei kam ein Pivot-Befund:

YOLO klassifiziert speed_limit-Schilder bereits mit neun spezifischen Klassen
direkt aus dem Detektor: `speed_limit_30 / 40 / 60 / 70 / 80 / 90 / 100 / 110`
plus den generic `SpeedLimitSign`-Knoten. Der Template-Matcher der Phase 6.5h
v1 laeuft **nur** auf generic SpeedLimitSign. Die spezifischen Klassen gehen
direkt durch.

Im Replay (nach Dedup):

| Subset | Anzahl | Anteil an speed_limit |
|---|---:|---:|
| spezifische speed_limit_* | 233 | 82.3 % |
| generic SpeedLimitSign | 50 | 17.7 % |
| **Total** | **283** | 100 % |

Bezogen auf alle Detections im Replay (14634 inference_calls) sind die
50 generic-Hits etwa 0.5 %. Genau die Teilmenge, auf die Templates v1 wirkt.

Das warf zwei Fragen auf: woher kommen die 17.7 % generic, und lohnen
weitere Templates ueberhaupt?

## Diagnose

Skript: `tools/model-tools/diag_speed_limit_classification.py` (stdlib + Pillow).
Liest NDJSON, dedupliziert auf `(frame, class, name, bbox)`, baut Statistiken.

### Drei a-priori-Hypothesen, drei Befunde

| Hypothese | Befund |
|---|---|
| Duplikat: generic = spaeterer specific Hit desselben Schilds (IoU>0.5 same frame) | 0 / 50 (0.0 %). Widerlegt. |
| Duplikat im Zeitfenster +/- 20 Frames | 0 / 50 (0.0 %). Auch widerlegt. |
| Distanz: generic = kleiner / weit weg | Widerlegt. Generic median area 3296 px2 > specific median 1804 - 2304 px2. Generic-Hits sind im Schnitt **groesser**, nicht kleiner. |

### Bestaetigter Faktor

| Metrik | generic | specific |
|---|---:|---:|
| Confidence median | 0.685 | 0.764 - 0.792 |
| Confidence p95 | 0.854 | 0.870 - 0.903 |
| Bbox area median | 3296 px2 | 1462 - 2449 px2 |

Generic-Hits clustern bei niedrigerer Confidence (median 0.685) und liegen
in groesseren Bboxes. Das passt zu naehen Schildern, die das Modell sieht,
aber nicht spezifisch zuordnen kann.

### Wahrscheinliche Ursache

`speed_limit_50` taucht im gesamten Replay-Stream nicht auf, `speed_limit_90`
hat genau einen Treffer, `speed_limit_30 / 70 / 110` zusammen neun Treffer.
Wenn YOLO ein 50er-Schild sieht, faellt es auf den breiten SpeedLimitSign-Knoten
zurueck, statt auf eine duenne Klasse zu commit-ten. Ergo: das Trainingsset
der genutzten v2-final mAP-0.798-Weights deckt 30 / 50 / 70 / 90 / 130 nicht
ausreichend ab, und die generic-Hits sind echte Out-of-Distribution-Schilder
fuer die spezifischen Koepfe.

### Visuelle Verifikation: nicht moeglich

Sample-Crop-Pass aus Task 5 lieferte 0 / 10 Bilder. Grund: das parallele
Capture-Tool persistierte nur Frames bis seq=6456 (frame ~3228), waehrend
die 50 generic-Hits ab frame 15664 aufwaerts liegen. Bug separat protokolliert
in [[02-Issues/Capture-Frame-Persistence-Limit]].

## Konsequenz

- **Template-Matcher bleibt** als Fallback fuer die 17.7 % generic-Detections.
  Auch wenn er nur 0.5 % aller Detections sieht, ist das die einzige Schicht,
  die diese Schilder noch mappen kann. Same-Frame-Overlap zeigt klar: ohne
  Template-Matcher gehen diese Hits ungelabelt durch.
- **Phase 6.5h v2 / v3 ist out-of-scope.** Mehr Crops fuer 30 / 50 / 70 / 130
  loesen das Problem nicht im Gesamtbild, weil YOLO 82.3 % schon direkt mappt.
  Der zusaetzliche Aufwand schlaegt nicht auf die Endmetrik durch.
- **YOLO-Retrain ist out-of-scope.** Kein Trainings-Setup im Repo, kein
  ausreichend grosses gelabeltes Dataset fuer die unterrepraesentierten
  Klassen, keine Validierungs-Pipeline.

## Lessons Learned

- **Pipeline-Architektur-Annahmen empirisch verifizieren, bevor man Skalen
  baut.** Wir haben zwei Tage in Templates investiert ohne zu wissen, dass
  YOLO 82.3 % der speed_limit-Detections schon direkt mappt. Ein 30-Minuten-
  NDJSON-Audit haette das gezeigt.
- **Diagnose-Skript mit NDJSON-Dedupe + Same-Frame-Overlap-Check sollte
  Standard-Schritt vor neuen Strategy-Phasen werden.** Stream-Polling
  produziert massiv Duplikate (15917 raw -> 8598 dedup, Faktor 1.85),
  und Klassen-Ambiguitaeten fallen nur in der dedup-ten Sicht auf.
- **Frame-Persistenz separat absichern.** Wenn das Capture-Tool nur 1 %
  der Replay-Frames speichert, ist visuelle Verifikation tot. Bug ist
  bekannt, blockiert aber Folge-Diagnosen.
- **Bbox-Area + Confidence als Diskriminator funktionieren als
  First-Pass-Test fuer 'OOD vs Distanz'.** Wenn area-Median oben statt
  unten liegt, ist Distanz raus.

## Folge-Issues

- [[02-Issues/Capture-Frame-Persistence-Limit]] — blockiert kuenftige
  frame-basierte Audits (z.B. 6.5g Eval-Tooling).

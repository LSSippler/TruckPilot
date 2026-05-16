---
created: 2026-05-16
tags: [issue, truckpilot, vision, capture, blocker-medium]
status: open
priority: medium
discovered_in: Phase-6.5h-Diag
---

# Capture-Frame-Persistence-Limit

## Beobachtung

`tools/vision-pipeline-capture` mit `--save-frames-dir` persistiert nur die
ersten ~3000 Frames eines laufenden Capture-Mirrors, danach landet nichts mehr
auf Disk. Im 21.6 Min Replay v2 (FPS=240, Inference-Worker ~10 Hz) waren
14634 Inferences erwartet aber nur 3228 JPEGs im Mirror.

Konkrete Zahlen aus dem Replay v2:

| Metrik | Wert |
|---|---:|
| Erwartete Frames (Inference-Rate) | ~14600 |
| Persistierte JPEGs | 3228 |
| Maximale Frame-Seq auf Disk | 6456 (~frame 3228) |
| Frame-Range der Detections im NDJSON | 1119 - 253595 |
| Frame-Range generic SpeedLimitSign | 15664 - 238492 |

Die spaeten Frames existieren in der SHM und werden vom Inference-Worker
verarbeitet, aber der Save-Frames-Pfad faellt aus.

## Symptom

Visuelle Verifikation einer Detection ist nur fuer frame_id < 3228 moeglich.
Diagnose-Skripte die `<seq>.jpg` zu einer Detection-Box laden, bekommen
`MISSING` fuer alle spaeteren Records.

## Auswirkung

- **Phase 6.5h-Diag Task 5**: 0 / 10 Sample-Crops konnten generiert werden.
  Alle 50 generic-Detections lagen ab frame 15664. Visuelle Bestaetigung der
  OOD-Hypothese musste auf Statistik beschraenkt bleiben.
- **Phase 6.5g (Eval-Tooling)**: blockiert, sobald Frame-basierte Ground-Truth-
  Annotation gefordert ist.
- **Phase 6.5h v2/v3**: nicht mehr relevant (out-of-scope nach Diag).

## Hypothesen zur Ursache

1. **Throttling im Capture-Thread**: Die `--save-frames-dir`-Schreiber-Schleife
   ist an die DXcam-Rate gebunden (typ. 30 Hz). Bei FPS=240 Replay schreibt
   der Replay-Producer 8x schneller in die SHM, der Capture liest aber mit
   30 Hz und ueberspringt entsprechend. Ergibt aber nur einen Faktor 8, nicht
   die beobachteten ~14600/3228 = 4.5x. Naehe an der Erklaerung, nicht exakt.
2. **Disk-IO-Backpressure**: Bei 240 FPS Replay landet jeder Frame als
   ~200 KB JPEG; bei 30 Hz Save-Rate sind das ~6 MB/s. Sollte kein
   Bottleneck sein.
3. **Race / Early-Stop**: Der Save-Pfad bricht still ab, ohne Log. Unklar
   ob ein Exception-Swallow oder eine Shutdown-Pfadueberlappung. Stack-Trace
   nicht erfasst.

## Reproduktion

```
python -m vision_pipeline_capture start --save-frames-dir capture_frames_replay_v2 &
python -m vision_pipeline_capture replay --video <playlist> --fps 240
# nach ~30s pruefen: ls capture_frames_replay_v2 | wc -l steigt nicht mehr
```

## Naechster Schritt

- Capture-Tool-Logs auf WARN/ERROR ab Frame 3000 pruefen
- Save-Pfad ggf. in eigenen Thread mit Bounded-Queue + Drop-Policy ziehen
- Telemetrie-Counter `capture.frames_saved_total` + `capture.frames_drop_total`
  ergaenzen, damit der Drop sichtbar ist

## Priority

Medium. Kein Showstopper fuer aktuelle Phase, aber Voraussetzung fuer
Phase 6.5g (Eval-Tooling).

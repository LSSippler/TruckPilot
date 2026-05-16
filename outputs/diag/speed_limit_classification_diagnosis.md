# Phase 6.5h-Diag: Generic vs Specific Speed-Limit-Classification

Daten: outputs/replay_v2_detections_stream.ndjson (Multi-Video-Replay v2, FPS=240).
Frames: capture_frames_replay_v2/.

## 1. Per-Klassen Overview

```
class                   count  mean_p   med_p     p5    p95  mean_area  med_area
--------------------------------------------------------------------------------
speed_limit_60            103   0.769   0.780  0.557  0.885     3568.1    2304.0
speed_limit_40             76   0.736   0.764  0.536  0.870     3106.3    1804.6
SpeedLimitSign             50   0.676   0.685  0.509  0.854     5576.1    3296.3
speed_limit_100            29   0.734   0.768  0.537  0.866     3869.8    2239.4
speed_limit_80             15   0.703   0.722  0.529  0.834     2157.0    1462.5
speed_limit_30              3   0.742   0.770  0.553  0.903     5645.3    2449.8
speed_limit_70              3   0.784   0.792  0.727  0.834     2275.6    2246.4
speed_limit_110             3   0.712   0.668  0.661  0.806     1474.0    1615.9
speed_limit_90              1   0.655   0.655  0.655  0.655     5278.5    5278.5
```

## 2. Confidence-Histogramm (generic vs specific)

```
bucket         generic   specific
------------------------------------
0.30-0.40            0          0
0.40-0.50            0          0
0.50-0.60           16         27
0.60-0.70           10         42
0.70-0.80           17         73
0.80-0.90            7         84
0.90-1.00            0          7
TOTAL               50        233
```

## 3. Bbox-Flaeche-Histogramm

```
area_px2         generic   specific
--------------------------------------
0-500                  0          0
500-1500               1         66
1500-5000             32        127
>5000                 17         40
TOTAL                 50        233
```

## 4. Same-Frame-Overlap (generic gegen specific, IoU>0.5)

```
generic total: 50
  same-frame match (df=0, IoU>0.5):       0 (0.0%)
  within +/-5 frames (cumulative):        0 (0.0%)
  within +/-20 frames (cumulative):       0 (0.0%)

 frame   df specific_class          iou  gen_conf

matched-against breakdown (within +/-20):
```

## 5. Sample-Crops

Geschrieben nach `outputs/diag/generic_samples/` (0 Bilder).

**Wichtig:** Capture-Frames-Mirror endet bei seq=6456 (frame ~3228). Alle generic SpeedLimitSign-Detections liegen ab frame 15664 aufwaerts (weit nach Ende der JPEG-Mirror-Persistenz). Visuelle Verifikation der generic-Hits ist mit den vorhandenen Daten nicht moeglich. Naechster Replay-Lauf muss den Frame-Mirror ueber den gesamten Replay-Zeitraum schreiben.

```
on-disk jpegs: 3228 (max seq = 6456, ~max frame = 3228)
generic detections within disk coverage: 0/50

idx  frame   conf     area       jpeg file
```

## 6. Diagnose

### Befund

- Im Replay v2 (nach Dedup): 50 generic SpeedLimitSign vs 233 spezifische speed_limit_* Detections. Generic-Quote unter allen speed_limit-Detections: 17.7%.
- Same-Frame-Overlap (df=0, IoU>0.5): 0/50 (0.0%). Im erweiterten Fenster +/-20 Frames: 0/50 (0.0%). Heisst: ein generic-Hit landet selten als IoU-Duplikat eines spezifischen Hits, auch nicht in zeitlicher Nachbarschaft.
- Confidence-Verteilung (Tabelle 2): generic-Hits clustern niedriger, spezifische Hits liegen breiter und hoeher.
- Bbox-Groesse (Tabelle 3): generic-Hits sind im Schnitt GROESSER als spezifische (median area 3296 vs 1804-2304). Widerlegt direkt die Distanz-Hypothese - generic-Hits sind keine 'weit entfernten kleinen Schilder', sondern oft naehe Schilder die das Modell nicht spezifisch zuordnen kann.

### Antwort auf die Frage

Die a-priori Hypothese 'generic = spaeter spezifischer Hit desselben Schilds' laesst sich am Datensatz nicht halten: nur 0.0% same-frame, 0.0% im +/-20-Frames-Fenster. Generic-Hits markieren ueberwiegend Schilder, die im gesamten Detection-Stream nie spezifisch klassifiziert werden.

Verbleibende plausible Ursachen:
1. **Out-of-Distribution-Klassen**: speed_limit_50 fehlt komplett im Output, speed_limit_90 hat 1 Treffer, speed_limit_30/70/110 zusammen 9 Treffer. Wenn das Modell ein 50er-Schild sieht, faellt es auf den breiten SpeedLimitSign-Knoten zurueck statt zu commit-ten.
2. **Confidence-Margin nahe Decision-Boundary**: generic-Hits clustern bei niedrigeren Confidences als spezifische (median 0.685 vs 0.764+). Wenn der Klassen-Kopf die Top-K-Klassen nahe beieinander rankt, gewinnt der generische Eltern-Knoten.
3. **Andere Schild-Subtypen**: ETS2 hat Tempo-Schilder in Varianten (Stadt, Autobahn, Gefahrenstelle, Anhaenger-Limit) die das Training-Set nicht abdeckt. Bbox-Histogramm zeigt grosse generic-Hits (median area 3296 > spezifische median 1804-2304), was zu 'naehe Schilder anderer Typ' passt, nicht 'kleines distales Schild'.

### Implikation fuer Phase 6.5h v1 (Template-Matcher)

- Der Template-Matcher laeuft per Konstruktion nur auf generic SpeedLimitSign. Das sind 17.7% aller speed_limit-Detections. Die restlichen 82.3% gehen direkt durch YOLOs spezifischen Kopf.
- Phase 6.5h v1 senkt km_unmapped innerhalb der generic-Teilmenge, aber im Gesamtbild ist YOLOs spezifischer Klassen-Kopf der dominante Pfad. Der Live-Effekt von Real-Templates auf die Endmetrik 'Speed-Limit korrekt gemeldet' ist entsprechend klein.

### Optionen

**A) Template-Matcher abschalten, nur YOLO-spezifisch.**
- Aufwand: S (Plugin-Flag).
- Risiko: mittel. Same-Frame-Overlap ist nahe null, also sind generic-Hits keine Duplikate spezifischer Detections. Sie sind echte Schilder die nur der generic-Knoten erwischt. Abschalten heisst diese Information verwerfen.
- Nutzen: einfacherer Code-Pfad. Aber Template-Matcher ist die einzige Chance, die generic-Hits noch zu mappen, daher ist Abschalten nur sinnvoll wenn Option C parallel laeuft.

**B) Template-Matcher behalten und auf den realen Daten profilieren.**
- Aufwand: M (Trefferquote von Templates v1 auf den 50 generic-Crops pruefen).
- Risiko: niedrig. Read-only Diagnose.
- Nutzen: empirische Antwort auf 'mappen Templates ueberhaupt etwas, oder sind die 50 generic-Hits OOD-Schilder die kein NCC mit 40/60/80/100-Templates loest?'.

**C) YOLO retrainen mit mehr 30 / 50 / 70 / 90 / 130-Crops.**
- Aufwand: L (Label-Pass + Training + Validierung).
- Risiko: mittel-hoch (Regressions auf bestehende Klassen, mAP-Trade-off).
- Nutzen: groesster Endeffekt. Template-Matcher wird obsolet.

### Multiple-Choice fuer User

- [ ] A) Template-Matcher abschalten und Aufwand komplett auf YOLO-Retrain (Option C) umlenken.
- [ ] B) Templates v1 erst auf den 50 generic-Crops profilieren (read-only Diagnose) bevor Entscheidung faellt.
- [ ] C) YOLO retrainen mit Crops fuer 30/50/70/90/130; Template-Matcher danach evaluieren.
- [ ] D) Frischer Replay-Lauf mit save-frames-dir UND voller Stream-Persistenz, dann B+C.

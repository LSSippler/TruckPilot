# Cross-Track-Term im Lane-Keeper — Control-Law Spec

## 1. Status quo (Inventar)

**Regler-Struktur heute:** Heading-only, PID auf herr:

```
lookahead(Centerline(t)) + Lane_Offset → offset_lookahead
herr = atan2(dx, -dz) - heading_rad           // Winkel Truck → Offset-Punkt
raw = PID(herr, dt)                           // Kp=0.8, Ki=0.1, Kd=0.3, integral_limit=2.0, output_limit=1.0
steering = rate_limit(raw, prev, 0.1)         // Δ/tick ≤ 0.1
```

**Bereits vorhanden (nur Diagnose):**
- `truck_lat_vs_centerline_m` (lib.rs:767-773): signierter Normalabstand Truck → Centerline
- `truck_lat_vs_offsetline_m` (lib.rs:1385-1388): `truck_lat_vs_centerline - lane_offset`
  → **Das ist e_lat, der Cross-Track-Fehler. Wird geloggt, aber nicht zurueckgefuehrt.**

**Beobachteter Effekt:** Ohne Cross-Track-Term stellt sich ein stationaeres
herr-Plateau von ~0.05 rad ein. Der PID-Integrator kompensiert einen Teil
(0.5 Integral * 0.1 Ki = 0.05 i_term), aber der Restfehler bleibt: der Truck
faehrt konstant leicht versetzt zur Soll-Linie.

---

## 2. Architektur-Entscheidung

### Optionen-Vergleich

| Kriterium | a) Stanley-artig (herr + atan) | b) Kaskadiert (PI_lat → PID_heading) | c) Additiv in PID-Eingang (e_lat direkt) |
|---|---|---|---|
| Aenderungsoertlichkeit | pre-PID (err vor PID) | neuer PI-Controller + Kopplung | pre-PID (err + k*e_lat) |
| Einheitenkonflikt | geloest (atan) | geloest (PI liefert rad) | krass (m + rad) → Skalierung fragil |
| Stabilitat | gut (atan saettigt bei pi/2) | mittel (doppelte I-Anteile) | schlecht (ungebremst, kein Sollwert) |
| Geschw.-Normierung | inharent (1/v in atan) | manuell (v-abhaengige gains) | manuell |
| Anti-Windup | einfach (e_lat-Spike → atan capped) | komplex (zwei I-Glieder) | aufwaendig |
| Aufwand | S (10-15 Zeilen) | XL (neuer PI, Zustand, Tuning) | M |

### Empfehlung: Option a) — Stanley-artige Erweiterung

Der Cross-Track-Term wird als zusaetzlicher Summand in den heading error
eingespeist, VOR dem PID:

```
herr' = herr + atan(k_ct * e_lat / max(v, v_min))
```

**Begruendung:**
1. Der bestehende PID bleibt unveraendert (Kp/Ki/Kd-Struktur bleibt).
2. `atan()` liefert von Natur aus rad-Einheit und saturiert bei +/-pi/2 —
   ein e_lat-Spike an Segmentnaehten erzeugt maximal ~1.57 rad zusaetzlichen
   Error, was unter HEADING_MISMATCH_THRESHOLD (1.4 rad) liegt **nur wenn**
   `atan(...) < 1.4 - herr_max`. In der Praxis: bei k_ct=0.5, e_lat=2m,
   v=4m/s: `0.5*2/4 = 0.25`, `atan(0.25)=0.245 rad` → voellig unkritisch.
3. Geschwindigkeitsnormierung ist eingebaut: bei niedrigem Tempo wirkt der
   Cross-Track-Term staerker (Lateral-Fehler soll schnell abgebaut werden),
   bei hohem Tempo sanfter (kein Pendeln).
4. Der Integrator im PID wirkt weiterhin auf die *Summe* aus herr und
   atan-Term; wenn e_lat → 0 geht, faellt der atan-Term weg und der
   Integrator kann sich auf den (dann kleineren) herr einschwingen.

**Signalfluss:**

```
truck_position →┐
                ├─ e_lat → atan(k_ct * e_lat / max(v, v_min)) ─┐
                │                                                 │
lookahead_geom →┼─ herr ─────────────────────────────────────────┼─ herr' → PID → rate_limit → steering
                │                                                 │
               (truck_lat_vs_offsetline_m)              (Summe, rad)
```

---

## 3. Formel und Parameter

### Control Law (vollstaendig)

```
v_safe   = max(t.speed_ms, V_MIN)               [V_MIN = 1.0 m/s]
e_lat    = truck_lat_vs_centerline - lane_offset [m, +rechts / -links]
xtrack   = atan(K_CT * e_lat / v_safe)          [rad]
herr     = atan2(dx, -dz) - heading_rad          [rad, bestehende Berechnung]
herr'    = herr + xtrack                         [rad, kombinierter Error]
raw      = PID(herr', dt)                        [bestehender PID]
steering = rate_limit(raw, prev, 0.1)            [bestehender Rate-Limiter]
```

### Startwerte (konservativ)

| Parameter | Startwert | Begruendung |
|---|---|---|
| K_CT | 0.5 | Bei 15 km/h, e_lat=0.3m: `atan(0.5*0.3/4.2)=0.036 rad` — bewusst kleiner als der beobachtete 0.05 rad Bias. Halbe Wirkung, um Oszillation auszuschliessen. |
| V_MIN | 1.0 m/s | Verhindert Division durch Null. 1 m/s = 3.6 km/h — unterhalb jeder normalen Fahrt (Kriechgang liegt bei ~5 km/h). |

### Anti-Windup: Segment-Transition Guard

An Segmentuebergaengen (Source-Wechsel oder offset_delta_m > 0.3 m)
kann e_lat springen. Drei Schutzmechanismen:

1. **ATAN-Saettigung:** `|atan(x)| < pi/2` — der Cross-Track-Term allein
   kann niemals einen Heading-Mismatch ausloesen.
2. **Integrator-Clamp bei e_lat-Spike:**
   Wenn `|e_lat| > E_LAT_SPIKE_M` (Default 0.5 m), dann den
   PID-Integrator-Einzugsschritt (`error * dt`) NULLEN fuer diesen Tick.
   Der I-Term wird also nicht aufgeblasen, waerrend der Cross-Track-Term
   den Fehler wegarbeitet.
   *Erkennung:* `(source_changed == 1) || (|offset_delta| > 0.3)`.
3. **PID-Reset bei source_changed:** Optional (Stufe 2, falls Stufe 1 nicht
   reicht): bei source_changed (=1) den PID resetten. Verhindert dass der
   I-Term aus dem alten Segment in das neue getragen wird.
   **Nicht als Default** — ein PID-Reset verursacht einen Lenk-Ruck.
   Nur zuschalten wenn Oszillation an Transitionen messbar ist.

---

## 4. Stabilitaet und Interaktion mit bestehendem PID

### Heading-Term vs. Cross-Track-Term — Koexistenz

Die beiden Terme arbeiten orthogonal:
- **Heading-Term (herr):** Korrigiert die *Richtung*. Wenn der Truck parallel
  zur Soll-Linie, aber versetzt faehrt, ist herr ≈ 0 (der Lookahead-Punkt
  liegt voraus, Winkel stimmt), aber e_lat ≠ 0. Der Cross-Track-Term
  korrigiert jetzt die Position, indem er einen scheinbaren Heading-Error
  erzeugt, der den Truck in die Soll-Linie lenkt.
- **Cross-Track-Term (e_lat):** Korrigiert die *Position*. Wenn der Truck
  auf der Soll-Linie, aber schraeg faehrt, ist e_lat ≈ 0, aber herr ≠ 0.
  Der Heading-Term korrigiert die Richtung.

Die beiden wirken NICHT gegeneinander — hoechstens nacheinander. Ein
typischer Einschwingvorgang: Truck ist versetzt und schraeg →
Cross-Track-Term will zurueck in die Spur, Heading-Term will parallel
stellen → das atan limitiert den Cross-Track-Einfluss, sodass der Truck
einen sanften Bogen in die Spur zieht, statt sie aggressiv zu schneiden.

### Doppelter Integrator? Nein.

Der Cross-Track-Term ist REIN PROPORTIONAL (kein I-Anteil). Es gibt
keinen zweiten Integrator. Der bestehende `Ki=0.1` integriert die
Summe `herr' = herr + xtrack`. Wenn e_lat → 0 und xtrack → 0, laeuft
der Integrator auf den dann residualen herr ein — das ist dasselbe
Verhalten wie heute, nur mit kleinerem residualen herr.

### Einschwingzeit und Daempfung

Erwartetes Verhalten (Pi-Controller-Klassiker, f = 20 Hz):
- Bei K_CT=0.5, v=15 km/h → Cross-Track-Bandbreite ~0.2-0.3 Hz
- Einschwingzeit nach e_lat-Sprung (0.3m) ~3-5 Sekunden
- Ueberschwingen: nicht erwartet (atan-Glaettung und rate_limit 0.1 wirken
  als Tiefpass). Falls messbar: K_CT reduzieren.

---

## 5. Tuning-Plan

### Phase 1: Gerade Strecke (15-30 km/h)

1. K_CT = 0.5 starten
2. `truck_lat_vs_offsetline_m` beobachten → geht es gegen 0?
3. Wenn nach 10s cruise `|e_lat|` noch > 0.1m: K_CT verdoppeln (1.0)
4. Wenn Pendeln in `truck_lat_vs_offsetline_m` (Amplitude > 0.15m):
   K_CT halbieren (0.25)
5. Ziel: `truck_lat_vs_offsetline_m` im Bereich [-0.05, +0.05] m im
   eingeschwungenen Zustand auf Gerade

### Phase 2: Gerade Strecke (60-80 km/h)

1. Mit dem K_CT aus Phase 1 testen
2. Erwartet: der Cross-Track-Term ist bei hoher Geschw. nahe 0
   (weil `k_ct * e_lat / v` klein). Wenn hier ein Rest-e_lat bleibt,
   K_CT um Faktor 1.5-2 erhoehen
3. Wenn es bei 80 km/h pendelt: V_MIN erhoehen (z.B. 2.0 m/s) oder
   K_CT reduzieren

### Phase 3: Kurven (30-50 km/h, Radius > 50m)

1. `herr'` beobachten: der Cross-Track-Term darf den Heading-Error in
   der Kurve nicht ueberstimmen
2. Wenn der Truck in der Kurve zur Innenseite wandert (e_lat negativ):
   Cross-Track-Term lenkt dagegen → gut, aber wenn er ueberschiessend
   wirkt (e_lat schwingt nach aussen), K_CT reduzieren
3. In Kurven wird herr' > 0.1 rad typisch sein. Der atan-Term liefert
   bei e_lat=0.1m, v=14m/s, K_CT=0.5 nur ~0.004 rad zusaetzlich →
   Kurvenverhalten aendert sich fast nicht (gut).

### Phase 4: Segmenttransitionen (Kreuzungen)

1. `source_changed` und `|e_lat|` beobachten  
2. Wenn Lenk-Stoss an Transition: Anti-Windup-Mechanismus 2 schaerfer
   stellen (E_LAT_SPIKE_M reduzieren auf 0.3)
3. Wenn der Truck nach der Transition nicht in die neue Spur findet:
   E_LAT_SPIKE_M erhoehen (laengerer I-Einzug erlaubt)

---

## 6. Diagnose-Keys (Blackboard)

| Key | Typ | Beschreibung |
|---|---|---|
| `lane_keeper.crosstrack_error_m` | float | e_lat = truck_lat_vs_centerline - lane_offset [+rechts] |
| `lane_keeper.crosstrack_term_rad` | float | atan(K_CT * e_lat / v) — Beitrag zum heading error |
| `lane_keeper.crosstrack_gain_kct` | float | K_CT (live, via apply_gain_overrides) |
| `lane_keeper.heading_error_orig_rad` | float | herr VOR Cross-Track-Addition (reiner Geom.-Fehler) |
| `lane_keeper.effective_err_rad` | float | herr' = herr + xtrack (PID-Eingang — bereits vorhanden, Wert aendert sich) |
| `lane_keeper.e_lat_spike` | u8 | 1 wenn |e_lat| > E_LAT_SPIKE_M (Integrator-Clamp aktiv) |
| `lane_keeper.crosstrack_source_changed` | u8 | 1 wenn source_changed in diesem Tick |

Bestehende Keys, die weiter relevant sind:
- `lane_keeper.truck_lat_vs_offsetline_m` — bereits vorhanden, Missbrauch.
  Kann durch `crosstrack_error_m` ersetzt werden (gleicher Wert, klarere Benennung).
- `lane_keeper.steer_p_term`, `_i_term`, `_d_term` — zeigen wie der PID
  auf herr' reagiert. Wenn der I-Term nach Cross-Track-Einfuehrung
  zurueckgeht, arbeitet der Term korrekt.
- `lane_keeper.source_changed` — bereits vorhanden.

---

## 7. Risiko-Einschaetzung

**Eingriff: S** (Small).
- ~15 Zeilen Aenderung in `compute_heading_error` und/oder
  `try_spline_heading_error` (Summationspunkt herr' vor dem tail).
- Blackboard-Diagnose-Keys hinzufuegen.
- `apply_gain_overrides` erweitern (K_CT ueber Blackboard justierbar).
- Keine neuen Abhaengigkeiten.
- Keine Struktur-Aenderung am PID, keine zusaetzlichen Zustands-Variablen
  (ausser optional einer Spike-Detection-Schranke).

**Worst Case bei falscher Verstaerkung:**
- K_CT zu gross (z.B. 5.0 statt 0.5): Der Truck schlingert (oszielliert
  um die Soll-Linie). Die Amplitude waechst pro Periode → `|e_lat|` steigt
  → Cross-Track-Term wird groesser → noch mehr Lenkung → Divergenz.
- **Erkennung sofort:** `truck_lat_vs_offsetline_m` zeigt wachsende
  Amplitude statt gegen 0 zu gehen. `effective_err_rad` oszilliert
  mit wachsender Amplitude.
- **Abbruch:** Vor der ersten Fahrt K_CT auf 0.5 setzen und nur live
  (Blackboard) erhoehen. Wenn die Fahrt oszilliert: K_CT per Blackboard
  auf 0.1 reduzieren. Wenn das nicht reicht: K_CT = 0 (Term deaktiviert)
  → Verhalten wie heute.
- **Der Rate-Limiter (0.1/tick) ist die letzte Sicherheitsbarriere.**
  Selbst bei divergierendem PID begrenzt er die Aenderungsgeschwindigkeit
  des Lenkwinkels. Kein Lenk-Stoss moeglich.

**Erkennung in der ersten Fahrt (rote Flaggen):**
1. `truck_lat_vs_offsetline_m` geht NICHT gegen 0, sondern waechst
   oder oszilliert → K_CT zu hoch.
2. `heading_error_orig_rad` steigt trotz fallendem e_lat → Cross-Track
   und Heading kaempfen (selten bei atan-Law, aber messbar).
3. `steer_p_term` springt pro Tick um >0.3 bei ruhiger Fahrt → V_MIN
   zu niedrig (Division durch fast-0 bei sehr langsamer Fahrt).

---

## 8. Implementierungs-Hinweise (kein Code)

### Aenderungsoert

Der Cross-Track-Term muss in BEIDE Pfade eingebracht werden:

1. **Spline-Pfad** (`try_spline_heading_error`): die Funktion gibt
   `Some(herr)` am Tail zurueck (Z. 1451). Vor dem `Some(err)` wird
   `herr = err` in `herr' = herr + xtrack` erweitert.
   `e_lat` ist zu diesem Zeitpunkt bereits auf dem Blackboard als
   `truck_lat_vs_offsetline_m` (Z. 1385-1387). **Achtung:** Das Blackboard
   wird aber im Spline-Tail (Z. 1381-1394) gesetzt, *bevor* der derzeitige
   herr auf dem Blackboard ist (Z. 1449-1451). Reihenfolge: zuerst den
   Cross-Track-Term berechnen (e_lat ist schon da), dann in den herr
   einrechnen, dann loggen.

2. **Catmull-Pfad** (`compute_heading_error`): der herr wird am Tail
   (Z. 1765) zurueckgegeben. Auch hier `herr' = herr + xtrack` setzen.
   `e_lat` muss im Catmull-Pfad **separat berechnet** werden, da der
   Spline-Pfad-Wert `truck_lat_vs_offsetline_m` nur gesetzt wird wenn
   der Spline-Pfad aktiv war. Im Catmull-Pfad gibt es kein Segment-
   Metadata fuer die Centerline-Projektion. Die e_lat-Berechnung im
   Catmull-Pfad kann ueber den nearest-Segment-Mechanismus
   (`self.last_nearest_seg`) und dessen Projektion (`evaluate`)
   erfolgen, oder — einfacher — den bereits berechneten
   `truck_lat_vs_centerline_m` aus dem SPLINE-Durchlauf des
   vorherigen Ticks wiederverwenden. **Bequemste Loesung:**
   `e_lat` im Catmull-Pfad ueber `truck_lat_vs_centerline_m` aus
   dem Blackboard lesen (wurde im letzten Spline-Durchlauf gesetzt).

### Blackboard-Konfiguration (apply_gain_overrides erweitern)

Analog zu den bestehenden `plugin.lane_keeper.kp/ki/kd`:
- `plugin.lane_keeper.crosstrack_gain` → K_CT (f64, Default 0.5)
- `plugin.lane_keeper.crosstrack_v_min` → V_MIN (f64, Default 1.0)
- `plugin.lane_keeper.crosstrack_spike_m` → E_LAT_SPIKE_M (f64,
  Default 0.5)
- `plugin.lane_keeper.crosstrack_enabled` → bool (Default true, bei
  false xtrack = 0.0 fuer das Control Law, ohne den Term zu deaktivieren)

---

## Zusammenfassung

| Aspekt | Wert |
|---|---|
| Architektur | Stanley-artig: herr' = herr + atan(K_CT * e_lat / v) |
| Aenderung | S (~15 Zeilen, 2 Pfade + Gain-Override) |
| Cross-Track-Quelle | `truck_lat_vs_offsetline_m` (bereits berechnet, Zeile 1387) |
| Start K_CT | 0.5 |
| Anti-Windup | e_lat-Spike-Detection (|e_lat| > 0.5m) → I-Einzug pausieren |
| Risiko | Gering. Rate-Limiter 0.1/tick faengt Divergenz. K_CT=0 deaktiviert. |
| Tuning first | `truck_lat_vs_offsetline_m → 0` auf Gerade bei 30 km/h |

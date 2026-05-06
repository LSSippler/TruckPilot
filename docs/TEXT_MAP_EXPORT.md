# Text Map Export — ETS2 Sektor-Daten ohne SCS-Parsing

Wenn das binäre SCS-Parsing nicht funktioniert (z.B. bei neueren ETS2-Versionen
mit abweichendem Hash-Verfahren), kannst du Map-Daten als **Text exportieren**
und direkt in TruckPilot laden.

## Schritt-für-Schritt

### 1. ETS2 Map-Editor öffnen

In ETS2 die Konsole öffnen (²-Taste oder `~` auf US-Tastatur) und den
Map-Editor starten:

```
edit
```

### 2. Zum gewünschten Sektor navigieren

Mit der Kamera (`WASD` + Maus) zum gewünschten Kartenbereich fliegen.
Der aktuelle Sektor wird in der Statusleiste angezeigt.

### 3. Sektor als Text exportieren

In der Konsole:
```
edit_save_text
```

Die Datei wird gespeichert unter:
```
Dokumente/Euro Truck Simulator 2/editor/<sector_name>.txt
```

### 4. Datei in TruckPilot laden

```bash
# Linux / WSL
cargo run --release -- --text-map-file "/pfad/zum/sector.txt" --start 0xUID1 --goal 0xUID2 -v
```
```powershell
# Windows
cargo run --release -- --text-map-file "C:\pfad\sector.txt" --start 0xUID1 --goal 0xUID2 -v
```

## Mehrere Sektoren kombinieren

Wenn du mehrere Sektoren exportiert hast, kannst du sie mit `cat` (Linux) oder
`type` (Windows) zu einer Datei zusammenführen:

```bash
cat sector1.txt sector2.txt > combined.txt
cargo run --release -- --text-map-file combined.txt --start 0xUID --goal 0xUID2 -v
```

## UIDs finden

Die Node-UIDs stehen in der exportierten Textdatei. Suche nach `uid:`:

```bash
grep "uid:" sector.txt | head -5
```

Oder baue den Graphen und lies die UIDs aus `graph.json`:
```bash
cargo run --release -- --text-map-file sector.txt --write-graph --telemetry-disable
# Dann in graph.json nach node_uid suchen
```

## Automatischer Fallback

Das Skript `scripts/run_live_test.sh` prüft automatisch, ob `--ets2-dir`
funktioniert. Falls nicht, wird nach `--text-map-file` gefragt.

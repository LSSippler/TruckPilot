# vision-training-collector

TruckPilot Phase 6.5d — Datensammler für das Custom-Fine-Tune des YOLOv5-Vision-Models auf eigene ETS2-Daten.

## Pipeline-Übersicht

```
YouTube/Live-Capture  ->  data/raw/        (Videos + Live-PNGs)
       |
       v
   extract-frames     ->  data/frames/     (1 Frame / 2s, skip black/menu)
       |
       v
       dedupe         ->  data/deduped/    (pHash hamming <= 5)
       |
       v
       export         ->  data/final/      (train/val/test split + manifest.csv + data.yaml)
```

Labels werden später extern (Roboflow / CVAT) auf den Train/Val/Test-Bildern erzeugt; `data.yaml` enthält bereits die 15 Ziel-Klassen für Phase 1 MVP.

## Ziel-Klassen (15)

`Car, Truck, TruckTrailer, Bus, BrakeLightOn, TurnSignalLeft, TurnSignalRight, TrafficLightRed, TrafficLightYellow, TrafficLightGreen, StopSign, SpeedLimitSign, LaneSolid, LaneDashed, RoadEdge`

## Setup

```powershell
cd tools/vision-training-collector
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -e .[dev]
```

Python >= 3.11 erforderlich. `dxcam` und `keyboard` sind Windows-only und werden nur für Live-Capture geladen.

## CLI

```powershell
python -m vision_training_collector scrape-youtube           # YouTube-Download (yt-dlp)
python -m vision_training_collector scrape-youtube --dry-run # nur probe, kein Download
python -m vision_training_collector capture-live --auto      # Live-Capture (F8=save, F9=quit)
python -m vision_training_collector extract-frames           # Videos -> Frames
python -m vision_training_collector dedupe                   # pHash-Filterung
python -m vision_training_collector export                   # Train/Val/Test-Split
python -m vision_training_collector pipeline                 # alles ausser Live-Capture
python -m vision_training_collector stats                    # Datensatz-Statistik
```

Alle Pfade liegen unter `tools/vision-training-collector/data/` und sind nicht versioniert.

## Pre-Labeling (ETS2LA YOLOv5s)

Phase 6.5d nutzt das vortrainierte ETS2LA YOLOv5s-Model (22 Klassen), um TruckPilot-Frames automatisch vorzubeschriften und so Roboflow/CVAT-Annotation zu beschleunigen. 9 der 15 TruckPilot-Klassen sind direkt mappbar (siehe `class_mapping.yaml`):

| ETS2LA              | -> | TruckPilot                |
|---------------------|----|---------------------------|
| car (0)             | -> | Car (0)                   |
| truck (1)           | -> | Truck (1)                 |
| bus (3)             | -> | Bus (3)                   |
| stop_sign (4)       | -> | StopSign (10)             |
| speedlimit_sign (6) | -> | SpeedLimitSign (11)       |
| green_light (15)    | -> | TrafficLightGreen (9)     |
| yellow_light (16)   | -> | TrafficLightYellow (8)    |
| red_light (17)      | -> | TrafficLightRed (7)       |
| lane_separator (21) | -> | LaneSolid (12) *default*  |

Confidence-Tiers:
- `>= 0.85` -> `labels/auto/` (direkt verwendbar)
- `0.30 - 0.85` -> `labels/review/` (manuell bestaetigen / korrigieren)
- `< 0.30` -> verworfen

Komplett manuell zu annotieren (kein ETS2LA-Counterpart): **TruckTrailer, BrakeLightOn, TurnSignalLeft, TurnSignalRight, LaneDashed, RoadEdge**. Fuer Bilder ohne automatische Detektion legt die Pipeline ein leeres `labels/manual/<stem>.txt` an als To-do-Slot.

### CLI

```powershell
python -m vision_training_collector pre-label `
    --input  data/final/images/train `
    --output data/pre_labeled/train `
    --model  ../../models/ets2la-object-detection/YOLOv5s/YOLOv5s-1-Active.pt
```

Optional: `--conf-auto 0.85`, `--conf-review 0.30`, `--dry-run` (Statistik ohne Files), `--mapping <path-to-yaml>`.

### Output-Struktur

```
data/pre_labeled/train/
├── images/                 (alle Frames - Symlink wenn moeglich)
├── labels/
│   ├── auto/               (>=0.85, ready)
│   ├── review/             (0.30-0.85, pruefen)
│   └── manual/             (leer-Slots fuer Frames ohne Detection)
├── class_mapping.yaml      (Kopie zur Reproduzierbarkeit)
└── pre_label_report.json   (Statistik: per Tier, per Klasse, Inferenzzeit)
```

### CVAT / Roboflow Import

Das Standard-YOLO-Format (`class cx cy w h`, normalisiert) wird sowohl von CVAT als auch Roboflow direkt akzeptiert. `data.yaml` aus dem `export`-Schritt liefert die 15-Klassen-Names, die in Roboflow als Label-Set importiert werden koennen. `labels/auto/` und `labels/review/` koennen entweder zusammengefuehrt oder als separate Projekte angelegt werden — letzteres erleichtert den Review-Workflow.

### Windows-Hinweis

`pre_label.py` setzt vor dem Load `pathlib.PosixPath = pathlib.WindowsPath`, weil ETS2LAs `.pt`-Checkpoint POSIX-Pfade gepickelt enthaelt. Ohne diesen Workaround crasht `torch.load` auf Windows mit `NotImplementedError: cannot instantiate 'PosixPath' on your system`.

## Resume

`data/state.json` trackt bereits heruntergeladene YouTube-IDs, extrahierte Videos und das tägliche Download-Volumen. Wird der Prozess abgebrochen, übernimmt der nächste Lauf nahtlos.

## Tests

```powershell
pytest tools/vision-training-collector/tests
```

`test_dedupe.py` prüft pHash-Verhalten mit synthetischen Bildern, `test_frame_extractor.py` baut ein 10-Frame-MP4 und ruft den Extraktor auf. Codecs müssen auf dem Host verfügbar sein (`mp4v`).

## Disclaimer & Lizenz

- Dieses Tool dient **ausschließlich** der Erstellung eines persönlichen, lokalen Trainings-Datensatzes für TruckPilot. Es ist **nicht** für Re-Upload, Re-Distribution oder kommerzielle Nutzung von YouTube-Inhalten gedacht.
- Beachte die YouTube ToS und die Urheberrechte der jeweiligen Channel-Betreiber. Wenn ein Rechteinhaber Widerspruch einlegt, lösche die betroffenen Videos in `data/raw/` und die abgeleiteten Frames.
- Lizenz-Hinweise der Dependencies:
  - `yt-dlp` — Unlicense / public domain
  - `opencv-python` — Apache 2.0 (Wheel) / BSD-3 (OpenCV)
  - `imagehash` — BSD-2
  - `Pillow` — HPND
  - `dxcam` — MIT
  - `click`, `rich`, `tqdm`, `numpy` — BSD/MIT

Tool-Lizenz: MIT (siehe `pyproject.toml`).

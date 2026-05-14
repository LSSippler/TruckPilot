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

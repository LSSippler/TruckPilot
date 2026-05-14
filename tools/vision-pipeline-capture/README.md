# vision-pipeline-capture

TruckPilot Phase 6.5c.1 — DXcam-Capture eines ETS2-Fensters, JPEG-Encode auf CPU, Publish in einen Windows-Named-Shared-Memory-Buffer (`Local\TruckPilotFrame`), den ein Rust-Plugin (`truckpilot-shm-frame-reader`) lesen kann.

## Architektur

```
+---------------------+        SHM (Local\TruckPilotFrame, 2 MiB+64 B)        +-------------------+
| Python Producer     |  ----> [ header(64) | JPEG-payload (variable) ] ----> | Rust Consumer     |
| DXcam.grab @10 FPS  |                                                       | image-rs decode   |
| cv2.imencode q=85   |                                                       | gap-detection     |
+---------------------+                                                       +-------------------+
```

Single Writer / Many Readers via Sequence-Lock:

| Schritt | Aktion                                | seq      |
|---------|---------------------------------------|----------|
| 0       | initial committed                     | even N   |
| 1       | mark in-progress vor dem Write        | N + 1 (odd)  |
| 2       | Header + Payload schreiben            | N + 1    |
| 3       | commit                                | N + 2 (even) |

Reader liest seq vor und nach dem Payload-Read. Bei `odd` oder bei Differenz → retry.

## Buffer-Layout (64-byte Header, little-endian, packed)

| Offset | Feld         | Typ    | Bytes |
|--------|--------------|--------|-------|
| 0      | magic        | char[4]| 4     |
| 4      | version      | u32    | 4     |
| 8      | frame_id     | u64    | 8     |
| 16     | timestamp_us | u64    | 8     |
| 24     | width        | u32    | 4     |
| 28     | height       | u32    | 4     |
| 32     | jpeg_size    | u32    | 4     |
| 36     | reserved     | u8[28] | 28    |
| 64     | jpeg_bytes   | u8[]   | variable |

`magic == "TPF1"` (0x31464654 little-endian), `version == 1`. `frame_id` ist der Sequence-Counter (odd ⇒ Write in progress, even ⇒ committed). Echte Frame-Nummer = `frame_id / 2`.

## Setup

```powershell
cd tools/vision-pipeline-capture
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -e .[dev]
```

Python ≥ 3.11. `dxcam`, `pywin32`, `keyboard` sind Windows-only.

## CLI

```powershell
# Producer starten (10 FPS, Quality 85, Window-Title-Match)
python -m vision_pipeline_capture start --fps 10 --quality 85

# Tail-Stats vom SHM lesen (5 s)
python -m vision_pipeline_capture stats --seconds 5
```

Hotkeys während `start`:
- **F8** — Pause/Resume (Frames werden nicht in SHM geschrieben)
- **F9** — Quit

## Rust-Reader

`crates/diag/src/bin/shm_frame_reader.rs` öffnet `Local\TruckPilotFrame`, decodet pro Frame das JPEG mit `image` und gibt nach 30 s eine Statistik aus:

```powershell
cargo run -p truckpilot-diag --bin shm-frame-reader --release -- --duration 30
```

## Tests

```powershell
pytest tools/vision-pipeline-capture/tests
```

- `test_shm_writer.py` deckt: Header-Größe (64 B), Round-Trip (Magic/Version/Felder/Payload), Sequence-Counter-Monotonie (+2 pro Commit, immer even), Reader-Helper-Retry, **Race-Test** (Concurrent-Writer ↔ Reader für 0.5 s, Reader darf keine torn/odd Frames sehen), Oversize-Payload wird `ValueError`.

## Performance-Erwartungen (Phase 6.5c Decision 5)

| Metrik             | Ziel                     |
|--------------------|--------------------------|
| Capture-FPS        | 10 ± 0.5                 |
| JPEG-Größe         | 150–250 KB @ 1920x1080   |
| JPEG-Decode (Rust) | < 5 ms / Frame           |
| Frame-Gap-Rate     | 0 % über 30 s            |
| Producer-CPU       | < 8 % auf einem Core     |

Tatsächlich gemessene Zahlen siehe `docs/vault/01-Phases/Phase-6.5c.1-DXcam-SHM-PoC.md`.

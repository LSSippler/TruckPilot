# CVAT Setup — TruckPilot Phase 6.5e

Local CVAT instance for annotating the 793-frame TruckPilot vision dataset
produced by `tools/vision-training-collector/`. CPU-only, single user,
Windows + AMD GPU friendly (no GPU is touched).

## Layout

```
tools/cvat-setup/
├── .env                  # version pin + ports + dataset path
├── docker-compose.yml    # override on top of vendor/cvat/docker-compose.yml
├── vendor/cvat/          # cloned upstream (not in git)
├── scripts/
│   ├── import_dataset.py
│   └── export_annotations.py
└── README.md
```

## One-time setup

### 1. Vendor the upstream CVAT compose

We layer our override on top of CVAT's official compose rather than
maintaining a parallel copy.

```powershell
cd tools\cvat-setup
git clone --depth 1 --branch v2.20.0 https://github.com/cvat-ai/cvat.git vendor\cvat
```

Match the tag to `CVAT_VERSION` in `.env`. Check
<https://github.com/cvat-ai/cvat/releases> for the newest stable tag and bump
both together.

### 2. Bring the stack up

```powershell
cd tools\cvat-setup
docker compose `
  --env-file .env `
  -f vendor\cvat\docker-compose.yml `
  -f docker-compose.yml `
  up -d
```

First boot pulls ~3 GB and takes ~5 minutes. Watch with:

```powershell
docker compose `
  -f vendor\cvat\docker-compose.yml -f docker-compose.yml `
  logs -f cvat_server
```

Server is ready when you see `Listening at: http://0.0.0.0:8080`.

### 3. Create the admin user

```powershell
docker exec -it truckpilot_cvat_server `
  python ~/manage.py createsuperuser
```

(Use any local-only credentials — this CVAT is bound to `localhost` and is
not exposed to the network.)

### 4. Open the UI

<http://localhost:8080>

Log in with the credentials from step 3.

### 5. Create the TruckPilot project

In the UI:

1. **Projects → +** → name: `TruckPilot Vision Phase 1`.
2. Paste the 15 labels (one per line, exact order/casing from
   `tools/vision-training-collector/class_mapping.yaml`):

   ```
   Car
   Truck
   TruckTrailer
   Bus
   BrakeLightOn
   TurnSignalLeft
   TurnSignalRight
   TrafficLightRed
   TrafficLightYellow
   TrafficLightGreen
   StopSign
   SpeedLimitSign
   LaneSolid
   LaneDashed
   RoadEdge
   ```

   Class IDs must match indices 0–14 — `import_dataset.py` relies on this to
   translate the YOLO `.txt` files.

3. Note the project ID from the URL (e.g. `/projects/1`).

## Daily use

### Import frames + pre-labels into a task

```powershell
python scripts\import_dataset.py `
  --project-id 1 `
  --task-name "phase-6.5e-train-001" `
  --split train
```

See `scripts/import_dataset.py --help` for full flags.

### Export reviewed annotations back to YOLO

```powershell
python scripts\export_annotations.py `
  --project-id 1 `
  --out ..\vision-training-collector\data\final\labels
```

This pulls every completed task in the project, downloads as YOLO 1.1, and
overwrites the per-split label folders.

## Reviewer workflow

The 15 classes split into three tiers based on what the ETS2LA pre-labeler
can and cannot produce:

### Tier A — auto-accept, spot-check only

Pre-labeler is confident (≥ 0.85). Skim ~10% per task and only correct
boxes that are obviously wrong.

* `Car` (0)
* `Truck` (1)
* `Bus` (3)
* `TrafficLightRed` (7)
* `TrafficLightYellow` (8)
* `TrafficLightGreen` (9)
* `StopSign` (10)
* `SpeedLimitSign` (11)

### Tier B — review every box

Pre-labeler emits these but with lower confidence (0.30–0.85). Walk through
each frame, accept/reject/tighten.

* `LaneSolid` (12) — pre-labeler defaults all lane separators to this; you
  must promote some to `LaneDashed` or `RoadEdge`.

### Tier C — manual only

Pre-labeler cannot produce these. Frames come in unlabeled for these
classes; you must draw every box from scratch.

* `TruckTrailer` (2)
* `BrakeLightOn` (4)
* `TurnSignalLeft` (5)
* `TurnSignalRight` (6)
* `LaneDashed` (13)
* `RoadEdge` (14)

Recommended pass order: B first (cheapest correction), then C on the same
frames (you're already looking at them), then A spot-check at the end.

## Shutdown / cleanup

```powershell
docker compose `
  -f vendor\cvat\docker-compose.yml -f docker-compose.yml `
  down
```

Add `-v` to wipe Postgres + uploaded images (destroys all annotations —
export first).

## Troubleshooting

* **`port 8080 already in use`** — change `CVAT_PORT` in `.env`.
* **UI loads but API calls 502** — `cvat_server` not ready yet; wait for the
  `Listening at` log line.
* **Windows path mount fails** — Docker Desktop must have the TruckPilot
  drive shared (Settings → Resources → File Sharing).
* **Import script `401 Unauthorized`** — re-run with `--username` /
  `--password` or set `CVAT_USER` / `CVAT_PASS` env vars.

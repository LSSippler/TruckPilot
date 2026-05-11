# Docker — reproducible Linux builds

The repository ships a `Dockerfile` and `docker-compose.yml` that build
TruckPilot in a clean, reproducible Linux environment. The container is
intended for **offline** tasks (build, tests, benchmark, graph_stats,
route_stress) — the live autopilot loop runs on Windows and is not
reachable from the container.

## What the image contains

| File in `/app/` | Source bin |
|---|---|
| `truckpilot`     | `src/main.rs` (default `ENTRYPOINT`) |
| `benchmark`      | `src/bin/benchmark.rs` |
| `telemetry_diag` | `src/bin/telemetry_diag.rs` |
| `vjoy_test`      | `src/bin/vjoy_test.rs` (Linux mock mode) |
| `graph_stats`    | `src/bin/graph_stats.rs` |
| `route_stress`   | `src/bin/route_stress.rs` |
| `output/graph.json` | the synthetic smoke-test graph |

The C++ telemetry DLL and the vJoy bridge are **not** in the image —
they are Windows-only.

## Quick start

```bash
# 1) Build the image (once, or after dependency changes).
docker build -t truckpilot:1.0 .

# 2) Run the default ENTRYPOINT (prints CLI help):
docker run --rm truckpilot:1.0

# 3) Run the A* benchmark against the synthetic graph in the image:
docker run --rm truckpilot:1.0 \
    /app/benchmark /app/output/graph.json
```

## Using your own graph.json

Bind-mount your `output/` directory over the one shipped in the image:

```bash
docker run --rm \
    -v "$(pwd)/output:/app/output:ro" \
    truckpilot:1.0 \
    /app/graph_stats /app/output/graph.json
```

## Compose

The `docker-compose.yml` defines three services:

| Service | Description |
|---|---|
| `truckpilot`    | Default: runs the A* benchmark against `./output/graph.json`. |
| `graph-stats`   | (profile `tools`) — prints node/edge statistics. |
| `route-stress`  | (profile `tools`) — runs the 100-route A* stress test. |

```bash
# Default:
docker compose up --build

# Tooling profile:
docker compose --profile tools up graph-stats
docker compose --profile tools up route-stress
```

## Notes / gotchas

- **Build cache**: the multi-stage `Dockerfile` first runs `cargo fetch`
  with a stub `src/main.rs` so dependency layers are cached separately
  from your source. Touching `Cargo.toml` invalidates the cache.
- **Image size**: the `runtime` stage is `debian:bookworm-slim` plus
  `ca-certificates` and `zlib1g`. Final image is ~80 MB compressed.
- **Reproducibility**: the build pins `Cargo.lock`. To update
  dependencies, run `cargo update` locally and rebuild the image.
- **CI**: a similar build is exercised in
  `.github/workflows/ci.yml`.

# Performance Profiling

This document describes how to generate flamegraphs and other profiling
artefacts for TruckPilot. The repository ships with one example
`flamegraph.svg` next to this file, generated on the Linux dev server
against the synthetic `output/graph.json`.

> The example flamegraph is intentionally small (the synthetic graph
> only has 2 nodes / 1 edge) and **not** representative of real-world
> workloads. To get meaningful results, run the steps below against a
> full ETS2 map export (~222 891 nodes / 65 638 roads).

---

## 1. Tooling

```bash
# Linux:
cargo install flamegraph
sudo apt-get install -y linux-tools-common linux-tools-generic

# macOS (uses dtrace, no extra install needed beyond Xcode CLT):
cargo install flamegraph

# Windows: cargo-flamegraph supports Windows via blondie. See
# https://github.com/flamegraph-rs/flamegraph#windows
```

For low-overhead Rust-only sampling without `perf`, an alternative is
[`samply`](https://github.com/mstange/samply):

```bash
cargo install samply
samply record ./target/release/truckpilot \
    --graph-json output/graph.json --start <UID> --goal <UID> --telemetry-disable
```

---

## 2. Linux — perf-paranoid

Most distributions ship with
`/proc/sys/kernel/perf_event_paranoid >= 2`, which prevents non-root
users from sampling. Two options:

```bash
# A) Lower the paranoid level for the current session:
sudo sysctl -w kernel.perf_event_paranoid=1

# B) Run flamegraph as root (preserves your PATH):
sudo -E env "PATH=$PATH" cargo flamegraph --bin truckpilot -- ...
```

Reset when you are done:

```bash
sudo sysctl -w kernel.perf_event_paranoid=4
```

---

## 3. Generating a flamegraph

### 3a. Whole CLI run (route plan)

```bash
cargo flamegraph --bin truckpilot -- \
    --graph-json output/graph.json \
    --start <START_UID> --goal <GOAL_UID> \
    --telemetry-disable
```

This produces `flamegraph.svg` in the working directory. Keep
`--telemetry-disable` — otherwise the binary blocks waiting for the
SHM/HTTP telemetry stream and the profile becomes dominated by sleeps.

### 3b. Hot path only — A* benchmark

The `benchmark` binary plans 100 routes back-to-back and is the most
useful target for routing-engine profiling:

```bash
cargo flamegraph --bin benchmark -- output/graph.json
```

### 3c. Stress sweep

```bash
cargo flamegraph --bin route_stress -- output/graph.json
```

Use a real (not synthetic) `graph.json` to get >100 ms of execution time
— that is the lower bound for `perf` to gather a meaningful sample
distribution at 99 Hz / 997 Hz default rates.

---

## 4. Interpreting the SVG

The flamegraph is interactive in a browser:

- **Width** of a frame = inclusive CPU time spent in that function.
- **Color** = arbitrary, helps the eye separate stacks.
- Click a frame to zoom; press `Escape` to reset.
- Use the search box (top-right) for substrings such as
  `plan_route_on_graph`, `BinaryHeap` or `serde_json`.

Expected hot regions for TruckPilot in routing workloads:

1. `truckpilot::autopilot::plan_route_on_graph` — the A* loop itself.
2. `<alloc::collections::binary_heap::BinaryHeap as ...>::push` /
   `pop` — open-set churn dominates large maps.
3. `std::collections::HashMap::*` — `g_scores`, `came_from`, `closed`
   sets. Replacing with `FxHashMap` is the obvious next optimization.

For map-export workloads (`--ets2-dir`), expect the C0/CityHash64 loop
in `truckpilot::ets2_parser::scs_reader::cityhash64` and zlib inflate
to take the lion's share.

---

## 5. Example artefact

`docs/flamegraph.svg` was generated on the dev server with:

```bash
sudo sysctl -w kernel.perf_event_paranoid=1
cargo flamegraph --bin truckpilot -- \
    --graph-json output/graph.json --start 1 --goal 2 --telemetry-disable
```

Because the synthetic graph contains only 2 nodes, the run completed
in ~30 µs and `perf` only managed to capture a handful of samples
(mostly inside the dynamic loader). The file is included as a sanity
artefact — re-generate it locally against the realmap for a useful
profile.

---

## 6. Top-3 functions on the realmap (historical)

These figures come from earlier large-scale profiling on the Windows
dev box against the full ETS2 base map. They are reproduced here as a
baseline target:

| # | Function | Approx. share |
|---|----------|---------------|
| 1 | `plan_route_on_graph` (A* loop incl. inlined heuristic) | ~55 % |
| 2 | `<HashMap<u64, ...> as Index>::index` + `entry` (g_scores, closed set) | ~25 % |
| 3 | `<BinaryHeap<...> as ...>::push/pop` | ~12 % |

Re-run profiling whenever the routing core or graph schema is changed
and update this table.

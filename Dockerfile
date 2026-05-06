# syntax=docker/dockerfile:1.7
#
# TruckPilot — reproducible Linux build container.
#
# Multi-stage build: stage 1 compiles the Rust workspace, stage 2 ships
# only the resulting binaries on a slim runtime base.
#
# Notes:
#   - The Telemetry DLL and vJoy bridge are Windows-only and not built here.
#   - The container is intended for offline tasks (build, tests, benchmarks,
#     graph_stats, route_stress) — not the live ETS2 loop.

# ---------- Stage 1: build ----------------------------------------------------
FROM rust:1.85-slim AS build

ENV CARGO_TERM_COLOR=always \
    CARGO_NET_RETRY=10 \
    RUST_BACKTRACE=1

# System libraries needed by the build (zlib for SCS inflate, libssl for
# `ureq` rustls bundle, cmake/pkg-config for crates that require them).
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        pkg-config \
        zlib1g-dev \
        ca-certificates \
        git \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Cache deps: copy manifests first.
COPY Cargo.toml Cargo.lock ./
COPY crates ./crates

# Pre-fetch dependencies (works even when the source tree is absent yet,
# as long as Cargo.toml/.lock are valid).
RUN mkdir -p src && echo "fn main() {}" > src/main.rs && \
    cargo fetch && \
    rm -rf src

# Now copy the actual source tree and build.
COPY . .
RUN cargo build --release --bins

# ---------- Stage 2: runtime --------------------------------------------------
FROM debian:bookworm-slim AS runtime

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        ca-certificates \
        zlib1g \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY --from=build /app/target/release/truckpilot       /app/
COPY --from=build /app/target/release/benchmark        /app/
COPY --from=build /app/target/release/telemetry_diag   /app/
COPY --from=build /app/target/release/vjoy_test        /app/
COPY --from=build /app/target/release/graph_stats      /app/
COPY --from=build /app/target/release/route_stress     /app/

# Optional: ship the synthetic test graph so users can do a smoke test
# without bind-mounting anything.
COPY --from=build /app/output/graph.json /app/output/graph.json

# Default command prints the CLI help. Override with `docker run truckpilot
# /app/benchmark /app/output/graph.json` etc.
ENTRYPOINT ["/app/truckpilot"]
CMD ["--help"]

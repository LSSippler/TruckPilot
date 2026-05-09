#!/usr/bin/env bash
# TruckPilot release packaging script (Linux/macOS).
#
# Builds the Rust workspace in release mode and assembles a self-contained
# release/ directory plus a truckpilot-v1.0.zip archive.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

RELEASE_DIR="$ROOT_DIR/release"
ZIP_NAME="truckpilot-v1.0.zip"

echo "[1/6] cargo build --release"
cargo build --release

echo "[2/6] preparing release/ directory"
rm -rf "$RELEASE_DIR"
mkdir -p "$RELEASE_DIR" "$RELEASE_DIR/scripts" "$RELEASE_DIR/docs"

echo "[3/6] copying binary"
if [[ -f "target/release/truckpilot.exe" ]]; then
    cp "target/release/truckpilot.exe" "$RELEASE_DIR/"
elif [[ -f "target/release/truckpilot" ]]; then
    cp "target/release/truckpilot" "$RELEASE_DIR/"
else
    echo "ERROR: target/release/truckpilot[.exe] not found." >&2
    exit 1
fi

# Optional: extra binaries useful for users.
for extra in benchmark telemetry_diag vjoy_test graph_stats route_stress; do
    for cand in "target/release/${extra}" "target/release/${extra}.exe"; do
        if [[ -f "$cand" ]]; then
            cp "$cand" "$RELEASE_DIR/"
        fi
    done
done

echo "[4/6] copying telemetry DLL (if built)"
DLL_PATH="TruckPilot.TelemetryDLL/build/Release/truckpilot_telemetry.dll"
if [[ -f "$DLL_PATH" ]]; then
    cp "$DLL_PATH" "$RELEASE_DIR/"
else
    echo "  - $DLL_PATH not found (skipping; build it on Windows)"
fi

echo "[5/6] copying config, scripts and docs"
cp truckpilot.toml "$RELEASE_DIR/" 2>/dev/null || echo "  - truckpilot.toml missing (skipped)"
cp scripts/*.ps1 "$RELEASE_DIR/scripts/" 2>/dev/null || true
cp scripts/*.sh  "$RELEASE_DIR/scripts/" 2>/dev/null || true
cp README.md PROJECT_FINAL.md HANDOFF.md "$RELEASE_DIR/docs/" 2>/dev/null || true
[[ -f CHANGELOG.md ]]                    && cp CHANGELOG.md "$RELEASE_DIR/docs/"
[[ -f offline_validation_results.txt ]]  && cp offline_validation_results.txt "$RELEASE_DIR/docs/"

echo "[6/6] creating $ZIP_NAME"
rm -f "$ROOT_DIR/$ZIP_NAME"
( cd "$ROOT_DIR" && zip -qr "$ZIP_NAME" "$(basename "$RELEASE_DIR")" )

if command -v du >/dev/null 2>&1; then
    SIZE=$(du -h "$ROOT_DIR/$ZIP_NAME" | awk '{print $1}')
else
    SIZE=$(stat -c%s "$ROOT_DIR/$ZIP_NAME")
fi
echo "Release archive: $ROOT_DIR/$ZIP_NAME ($SIZE)"

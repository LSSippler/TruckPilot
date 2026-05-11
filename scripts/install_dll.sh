#!/usr/bin/env bash
# install_dll.sh — Copies truckpilot_telemetry.dll to the ETS2 plugin directory.
#
# Usage:
#   ./scripts/install_dll.sh [ETS2_DIR]
#
# If ETS2_DIR is not provided, auto-detects from standard Steam paths.

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log()  { echo -e "${GREEN}[INSTALL]${NC} $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
err()  { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

# --- Locate ETS2 directory ---
if [ -n "${1:-}" ]; then
    ETS2_DIR="$1"
else
    # Standard Steam paths (Linux Proton / Windows via WSL).
    CANDIDATES=(
        "$HOME/.steam/steam/steamapps/common/Euro Truck Simulator 2"
        "/mnt/c/Program Files (x86)/Steam/steamapps/common/Euro Truck Simulator 2"
        "$HOME/.local/share/Steam/steamapps/common/Euro Truck Simulator 2"
    )
    ETS2_DIR=""
    for d in "${CANDIDATES[@]}"; do
        if [ -d "$d" ]; then
            ETS2_DIR="$d"
            break
        fi
    done
fi

if [ -z "$ETS2_DIR" ] || [ ! -d "$ETS2_DIR" ]; then
    err "ETS2 directory not found. Provide it as argument: $0 /path/to/ets2"
fi

PLUGIN_DIR="$ETS2_DIR/bin/win_x64/plugins"
if [ ! -d "$PLUGIN_DIR" ]; then
    err "Plugin directory not found: $PLUGIN_DIR (is ETS2 installed in '$ETS2_DIR'?)"
fi

# --- Locate DLL ---
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
DLL_SRC="$PROJECT_DIR/target/release/truckpilot_telemetry_dll.dll"

if [ ! -f "$DLL_SRC" ]; then
    warn "DLL not found at $DLL_SRC"
    echo "  Build it first: cargo build --release -p truckpilot_telemetry_dll"
    echo "  Note: must be built on Windows (msvc target) or via cross-compilation."
    exit 1
fi

# --- Copy ---
cp -v "$DLL_SRC" "$PLUGIN_DIR/"
log "DLL installed to: $PLUGIN_DIR"
log "Done. Restart ETS2 to load the plugin."

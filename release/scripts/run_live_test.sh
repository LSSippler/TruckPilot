#!/usr/bin/env bash
# run_live_test.sh — Starts TruckPilot autopilot with telemetry DLL and vJoy.
#
# Usage:
#   ./scripts/run_live_test.sh [START_UID] [GOAL_UID]
#
# If UIDs are not provided, extracts the first two valid UIDs from
# output/nodes.json (if available) or prompts the user.

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log()  { echo -e "${GREEN}[LIVE]${NC} $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
err()  { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

# --- Locate project root ---
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# --- ETS2 directory ---
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

if [ -z "$ETS2_DIR" ]; then
    read -rp "ETS2 installation directory: " ETS2_DIR
fi
if [ ! -d "$ETS2_DIR" ]; then
    err "ETS2 directory not found: $ETS2_DIR"
fi

# --- Check DLL ---
PLUGIN_DLL="${ETS2_DIR}/bin/win_x64/plugins/truckpilot_telemetry_dll.dll"
if [ ! -f "$PLUGIN_DLL" ]; then
    warn "DLL not installed in $PLUGIN_DLL"
    echo "  Run ./scripts/install_dll.sh first."
    exit 1
fi
log "DLL found: $PLUGIN_DLL"

# --- Determine map source ---
MAP_ARGS=""
TEXT_MAP_FILE="$PROJECT_DIR/output/sector.txt"

if [ -f "$TEXT_MAP_FILE" ]; then
    log "Found text map file: $TEXT_MAP_FILE"
    MAP_ARGS="--text-map-file $TEXT_MAP_FILE"
elif [ -n "$ETS2_DIR" ] && [ -d "$ETS2_DIR" ]; then
    # Try binary parsing first
    log "Trying binary ETS2 parsing from: $ETS2_DIR"
    MAP_ARGS="--ets2-dir $ETS2_DIR"
else
    warn "No map source found. Export a text sector (edit_save_text in ETS2 editor)"
    warn "and save it to $TEXT_MAP_FILE"
    warn "Then re-run this script."
    exit 1
fi

# --- Determine UIDs ---
if [ -n "${1:-}" ] && [ -n "${2:-}" ]; then
    START_UID="$1"
    GOAL_UID="$2"
elif [ -f "$PROJECT_DIR/graph.json" ]; then
    log "Extracting UIDs from graph.json..."
    UIDS=$(python3 -c "
import json, sys
with open('$PROJECT_DIR/graph.json') as f:
    data = json.load(f)
nodes = data.get('nodes', [])
uids = [str(n['uid']) for n in nodes[:2]]
print(' '.join(uids) if len(uids) >= 2 else '')
" 2>/dev/null || echo "")
    if [ -n "$UIDS" ]; then
        read -r START_UID GOAL_UID <<< "$UIDS"
        log "Found UIDs: $START_UID → $GOAL_UID"
    else
        warn "Could not extract UIDs from graph.json"
        read -rp "Enter start UID (hex or dec): " START_UID
        read -rp "Enter goal UID (hex or dec): " GOAL_UID
    fi
else
    read -rp "Enter start UID (hex or dec): " START_UID
    read -rp "Enter goal UID (hex or dec): " GOAL_UID
fi

if [ -z "$START_UID" ] || [ -z "$GOAL_UID" ]; then
    err "Both start and goal UIDs are required."
fi

# --- Build (if needed) ---
if [ ! -f "$PROJECT_DIR/target/release/truckpilot" ] && [ ! -f "$PROJECT_DIR/target/release/truckpilot.exe" ]; then
    log "Building release binary..."
    (cd "$PROJECT_DIR" && cargo build --release) || err "Build failed"
fi

# --- Run ---
log "Starting TruckPilot autopilot..."
log "  ETS2 dir:  $ETS2_DIR"
log "  Start UID: $START_UID"
log "  Goal UID:  $GOAL_UID"
log "  vJoy:      device 1"
echo ""

(cd "$PROJECT_DIR" && cargo run --release -- \
    $MAP_ARGS \
    --start "$START_UID" \
    --goal "$GOAL_UID" \
    --vjoy-device 1 \
    -v \
    --telemetry-disable 2>&1 || true)
# --telemetry-disable is PRESENT above → route-only test (no live loop).
# Remove that flag to enable the live telemetry loop.

echo ""
echo "Route-only test complete."
echo "For live telemetry loop, remove --telemetry-disable and run:"
echo "  cargo run --release -- $MAP_ARGS --start \"$START_UID\" --goal \"$GOAL_UID\" --vjoy-device 1 -v"

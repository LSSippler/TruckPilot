#!/usr/bin/env bash
# scripts/compare_parsers.sh
#
# Compare the Rust and .NET parsers on the same input directory of HashFS
# sector files (`map/*/*.base`). Counts of nodes, roads and prefabs are
# extracted from both implementations, diffed and written to
# compare_report.txt.
#
# Usage:
#   scripts/compare_parsers.sh <hashfs_sectors_dir>
#
# Returns 0 if all counts match, 2 if any count differs.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <hashfs_sectors_dir>" >&2
    exit 1
fi

SECTORS="$1"
REPORT="$ROOT_DIR/compare_report.txt"

# ANSI helpers — disabled when stdout is not a TTY.
if [[ -t 1 ]]; then
    GREEN='\033[1;32m'; RED='\033[1;31m'; YELLOW='\033[1;33m'; RESET='\033[0m'
else
    GREEN=''; RED=''; YELLOW=''; RESET=''
fi

emit() {  # write to both stdout and the report file
    printf '%s\n' "$1" | tee -a "$REPORT" > /dev/null
    printf '%b\n' "$1"
}

: > "$REPORT"
emit "===================================================================="
emit "TruckPilot — Rust vs. .NET parser comparison"
emit "Input directory : $SECTORS"
emit "Date            : $(date -Iseconds)"
emit "===================================================================="

if [[ ! -d "$SECTORS" ]]; then
    emit "${RED}ERROR${RESET}: input directory does not exist."
    exit 1
fi

# ---- Rust ----------------------------------------------------------------
emit ""
emit "[1/3] Rust parser  (cargo run --quiet --release -- --hashfs-sectors ...)"
RUST_OUT=$(cargo run --quiet --release -- --hashfs-sectors "$SECTORS" -v 2>&1 || true)
echo "$RUST_OUT" >> "$REPORT"
RUST_NODES=$(  echo "$RUST_OUT" | grep -oE '[Nn]odes[^0-9]*[0-9]+'   | grep -oE '[0-9]+' | head -1 || echo 0)
RUST_ROADS=$(  echo "$RUST_OUT" | grep -oE '[Rr]oads[^0-9]*[0-9]+'   | grep -oE '[0-9]+' | head -1 || echo 0)
RUST_PREFABS=$(echo "$RUST_OUT" | grep -oE '[Pp]refabs[^0-9]*[0-9]+' | grep -oE '[0-9]+' | head -1 || echo 0)

# ---- .NET ----------------------------------------------------------------
emit ""
emit "[2/3] .NET parser  (dotnet run --project TruckPilot.NET/TruckPilot.CLI ...)"
DOTNET_OUT=$(dotnet run --project TruckPilot.NET/TruckPilot.CLI -- --hashfs-sectors "$SECTORS" -v 2>&1 || true)
echo "$DOTNET_OUT" >> "$REPORT"
DOTNET_NODES=$(  echo "$DOTNET_OUT" | grep -oE '[Nn]odes[^0-9]*[0-9]+'   | grep -oE '[0-9]+' | head -1 || echo 0)
DOTNET_ROADS=$(  echo "$DOTNET_OUT" | grep -oE '[Rr]oads[^0-9]*[0-9]+'   | grep -oE '[0-9]+' | head -1 || echo 0)
DOTNET_PREFABS=$(echo "$DOTNET_OUT" | grep -oE '[Pp]refabs[^0-9]*[0-9]+' | grep -oE '[0-9]+' | head -1 || echo 0)

# ---- Diff ----------------------------------------------------------------
emit ""
emit "[3/3] Diff"
emit "----------------------------------------------------------"
emit "                Rust          .NET          Δ"

DIFFER=0
compare_one() {
    local label=$1 a=$2 b=$3
    local diff=$(( a - b ))
    local color=$GREEN
    if [[ $diff -ne 0 ]]; then color=$RED; DIFFER=1; fi
    printf '  %-12s %12d  %12d  %s%+d%s\n' "$label" "$a" "$b" "$color" "$diff" "$RESET" \
        | tee -a "$REPORT"
}
compare_one "Nodes"   "$RUST_NODES"   "$DOTNET_NODES"
compare_one "Roads"   "$RUST_ROADS"   "$DOTNET_ROADS"
compare_one "Prefabs" "$RUST_PREFABS" "$DOTNET_PREFABS"
emit "----------------------------------------------------------"

if [[ $DIFFER -eq 0 ]]; then
    emit "${GREEN}OK${RESET}: all counts match (0 % deviation)."
    exit 0
else
    emit "${RED}DIFF${RESET}: parser outputs disagree — see $REPORT."
    exit 2
fi

# scripts/compare_parsers.ps1
#
# Compare the Rust and .NET parsers on the same HashFS-sectors directory.
# Counts of nodes, roads and prefabs are extracted from both
# implementations, diffed and written to compare_report.txt.
#
# Usage:
#   .\scripts\compare_parsers.ps1 -SectorsDir <path>
#
# Exit codes:
#   0  all counts match
#   1  invocation error (missing dir, build failure)
#   2  parser counts disagree

[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$SectorsDir
)

$ErrorActionPreference = "Stop"
$RootDir   = (Resolve-Path "$PSScriptRoot\..").Path
Set-Location $RootDir

$Report = Join-Path $RootDir "compare_report.txt"
"" | Out-File -FilePath $Report -Encoding utf8

function Emit($msg, $color = "Gray") {
    Write-Host $msg -ForegroundColor $color
    Add-Content -Path $Report -Value $msg
}

Emit "===================================================================="
Emit "TruckPilot — Rust vs. .NET parser comparison"
Emit "Input directory : $SectorsDir"
Emit ("Date            : " + (Get-Date -Format "o"))
Emit "===================================================================="

if (-not (Test-Path $SectorsDir)) {
    Emit "[ERROR] input directory does not exist." Red
    exit 1
}

function Extract-Number([string]$text, [string]$pattern) {
    $m = [regex]::Match($text, $pattern)
    if ($m.Success) {
        return [int]($m.Groups[1].Value)
    }
    return 0
}

# ---- Rust ----------------------------------------------------------------
Emit ""
Emit "[1/3] Rust parser  (cargo run --quiet --release -- --hashfs-sectors ...)"
$rustOut = (cargo run --quiet --release -- --hashfs-sectors $SectorsDir -v 2>&1) -join "`n"
Add-Content -Path $Report -Value $rustOut
$rustNodes   = Extract-Number $rustOut '[Nn]odes[^0-9]*([0-9]+)'
$rustRoads   = Extract-Number $rustOut '[Rr]oads[^0-9]*([0-9]+)'
$rustPrefabs = Extract-Number $rustOut '[Pp]refabs[^0-9]*([0-9]+)'

# ---- .NET ----------------------------------------------------------------
Emit ""
Emit "[2/3] .NET parser  (dotnet run --project TruckPilot.NET/TruckPilot.CLI ...)"
$dotnetOut = (dotnet run --project TruckPilot.NET/TruckPilot.CLI -- --hashfs-sectors $SectorsDir -v 2>&1) -join "`n"
Add-Content -Path $Report -Value $dotnetOut
$dotnetNodes   = Extract-Number $dotnetOut '[Nn]odes[^0-9]*([0-9]+)'
$dotnetRoads   = Extract-Number $dotnetOut '[Rr]oads[^0-9]*([0-9]+)'
$dotnetPrefabs = Extract-Number $dotnetOut '[Pp]refabs[^0-9]*([0-9]+)'

# ---- Diff ----------------------------------------------------------------
Emit ""
Emit "[3/3] Diff"
Emit "----------------------------------------------------------"
Emit ("                Rust          .NET          Delta")

$differ = $false
function Compare-One($label, $a, $b) {
    $diff = $a - $b
    $line = "  {0,-12} {1,12}  {2,12}  {3,+0}" -f $label, $a, $b, $diff
    if ($diff -eq 0) {
        Emit $line Green
    } else {
        Emit $line Red
        $script:differ = $true
    }
}
Compare-One "Nodes"   $rustNodes   $dotnetNodes
Compare-One "Roads"   $rustRoads   $dotnetRoads
Compare-One "Prefabs" $rustPrefabs $dotnetPrefabs
Emit "----------------------------------------------------------"

if (-not $differ) {
    Emit "[OK] all counts match (0 % deviation)." Green
    exit 0
} else {
    Emit "[DIFF] parser outputs disagree - see $Report." Red
    exit 2
}

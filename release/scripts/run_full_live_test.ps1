# run_full_live_test.ps1 — Full live test automation for TruckPilot.

param(
    [string]$HashfsSectors = "C:\temp\ets2_sectors"
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$telemetryDir = Join-Path $repoRoot "TruckPilot.TelemetryDLL"
$netDir = Join-Path $repoRoot "TruckPilot.NET"
$cliProject = Join-Path $netDir "TruckPilot.CLI\TruckPilot.CLI.csproj"

$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
$logPath = Join-Path $repoRoot ("live_test_{0}.log" -f $timestamp)
New-Item -ItemType File -Path $logPath -Force | Out-Null

function Write-Log {
    param([string]$Message)
    $Message | Tee-Object -FilePath $logPath -Append
}

function Fail {
    param([string]$Message)
    Write-Log "[ERROR] $Message"
    exit 1
}

function Require-Command {
    param([string]$Name)
    if (-not (Get-Command $Name -ErrorAction SilentlyContinue)) {
        Fail "Required command not found: $Name"
    }
}

function Invoke-Logged {
    param([string]$Label, [string]$Command, [string[]]$Arguments)
    Write-Log "[STEP] $Label"
    try {
        & $Command @Arguments 2>&1 | Tee-Object -FilePath $logPath -Append
    } catch {
        Fail "$Label failed: $($_.Exception.Message)"
    }
    if ($LASTEXITCODE -ne 0) {
        Fail "$Label failed with exit code $LASTEXITCODE."
    }
}

Write-Log "[INFO] Log: $logPath"

# Step 1: Check prerequisites
Require-Command dotnet
Require-Command cmake

# Step 2: Build C++ DLL
$buildScript = Join-Path $telemetryDir "build.ps1"
if (-not (Test-Path $buildScript)) {
    Fail "Build script not found: $buildScript"
}
Write-Log "[STEP] Build telemetry DLL"
try {
    & $buildScript -Config Release 2>&1 | Tee-Object -FilePath $logPath -Append
} catch {
    Fail "Build telemetry DLL failed: $($_.Exception.Message)"
}
if ($LASTEXITCODE -ne 0) {
    Fail "Build telemetry DLL failed with exit code $LASTEXITCODE."
}

# Step 3: Copy DLL to ETS2 plugin directory
$pluginDir = $env:ETS2_PLUGIN_DIR
if ([string]::IsNullOrWhiteSpace($pluginDir)) {
    $ets2Dir = $env:ETS2_DIR
    if ([string]::IsNullOrWhiteSpace($ets2Dir)) {
        $candidates = @(
            "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2",
            "D:\SteamLibrary\steamapps\common\Euro Truck Simulator 2",
            "$env:ProgramFiles(x86)\Steam\steamapps\common\Euro Truck Simulator 2"
        )
        foreach ($d in $candidates) {
            if (Test-Path $d) { $ets2Dir = $d; break }
        }
    }
    if ([string]::IsNullOrWhiteSpace($ets2Dir) -or -not (Test-Path $ets2Dir)) {
        Fail "ETS2 directory not found. Set ETS2_PLUGIN_DIR or ETS2_DIR."
    }
    $pluginDir = Join-Path $ets2Dir "bin\win_x64\plugins"
}

if (-not (Test-Path $pluginDir)) {
    New-Item -ItemType Directory -Path $pluginDir | Out-Null
}

$builtDll = Join-Path $telemetryDir "build\Release\truckpilot_telemetry.dll"
if (-not (Test-Path $builtDll)) {
    Fail "Built DLL not found: $builtDll"
}

$destDll = Join-Path $pluginDir "truckpilot_telemetry.dll"
Copy-Item -Path $builtDll -Destination $destDll -Force
Write-Log "[INFO] DLL copied to $destDll"

# Step 4: Sanity check
if (-not (Test-Path $cliProject)) {
    Fail "CLI project not found: $cliProject"
}
Invoke-Logged "Telemetry sanity check" "dotnet" @("run", "--project", $cliProject, "--", "--check-telemetry-dll")

if (-not (Test-Path $HashfsSectors)) {
    Fail "HashFS sectors directory not found: $HashfsSectors"
}

# Step 5: Extract connected UIDs from graph.json
$graphPath = Join-Path $repoRoot "graph.json"
if (-not (Test-Path $graphPath)) {
    Fail "graph.json not found: $graphPath"
}

try {
    $jsonText = Get-Content -Raw $graphPath
    $doc = [System.Text.Json.JsonDocument]::Parse($jsonText)
    $startUid = $null
    $goalUid = $null

    try {
        try {
            $edges = $doc.RootElement.GetProperty("Edges")
        } catch {
            $edges = $doc.RootElement.GetProperty("edges")
        }

        foreach ($edge in $edges.EnumerateArray()) {
            try {
                $startUid = $edge.GetProperty("FromNodeUid").GetUInt64()
                $goalUid = $edge.GetProperty("ToNodeUid").GetUInt64()
            } catch {
                $startUid = $edge.GetProperty("from_node_uid").GetUInt64()
                $goalUid = $edge.GetProperty("to_node_uid").GetUInt64()
            }
            break
        }
    } catch {
        # Fallback to nodes below.
    }

    if ($null -eq $startUid -or $null -eq $goalUid) {
        try {
            $nodes = $doc.RootElement.GetProperty("Nodes")
        } catch {
            $nodes = $doc.RootElement.GetProperty("nodes")
        }
        $firstNode = $null
        $lastNode = $null
        foreach ($node in $nodes.EnumerateArray()) {
            if ($null -eq $firstNode) { $firstNode = $node }
            $lastNode = $node
        }
        if ($null -eq $firstNode -or $null -eq $lastNode) {
            throw "Nodes and edges arrays are empty."
        }

        try {
            $startUid = $firstNode.GetProperty("Uid").GetUInt64()
            $goalUid = $lastNode.GetProperty("Uid").GetUInt64()
        } catch {
            $startUid = $firstNode.GetProperty("uid").GetUInt64()
            $goalUid = $lastNode.GetProperty("uid").GetUInt64()
        }
    }

    $doc.Dispose()
} catch {
    Fail "Failed to read UIDs from graph.json: $($_.Exception.Message)"
}

Write-Log "[INFO] Start UID: $startUid"
Write-Log "[INFO] Goal UID:  $goalUid"

# Step 6: Validate route planning with selected UIDs
Invoke-Logged "Route validation run" "dotnet" @("run", "--project", $cliProject, "--", "--hashfs-sectors", $HashfsSectors, "--start", "$startUid", "--goal", "$goalUid", "-v")

# Step 7-8: Final summary
Write-Log "Live-Test abgeschlossen. Log: $logPath. Naechster Schritt: Lkw im Spiel beobachten."
Write-Host "Live-Test abgeschlossen. Log: $logPath. Naechster Schritt: Lkw im Spiel beobachten."

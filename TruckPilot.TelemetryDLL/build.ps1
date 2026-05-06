param(
    [string]$Config = "Release"
)

$ErrorActionPreference = "Stop"

$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$BuildDir = Join-Path $Root "build"

if (!(Test-Path $BuildDir)) {
    New-Item -ItemType Directory -Path $BuildDir | Out-Null
}

cmake -S $Root -B $BuildDir -G "Visual Studio 17 2022" -A x64
cmake --build $BuildDir --config $Config

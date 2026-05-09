param([string]$ProjectRoot = (Get-Location).Path, [string]$OutputFile = "truckpilot_bundle.txt")
$IncludePatterns = @("crates\*\src\*.rs", "crates\*\src\**\*.rs", "crates\*\Cargo.toml", "Cargo.toml")
$ExcludePatterns = @("*\target\*", "*\node_modules\*", "*\.git\*")
if (Test-Path $OutputFile) { Remove-Item $OutputFile }
$allFiles = @()
foreach ($p in $IncludePatterns) { $allFiles += Get-ChildItem -Path (Join-Path $ProjectRoot $p) -Recurse -ErrorAction SilentlyContinue }
$allFiles = $allFiles | Where-Object { $f = $_.FullName; -not ($ExcludePatterns | Where-Object { $f -like $_ }) } | Sort-Object FullName -Unique
Add-Content $OutputFile "TRUCKPILOT 2.0 BUNDLE — $(Get-Date) — $($allFiles.Count) files`n" -Encoding UTF8
foreach ($f in $allFiles) {
    $rel = $f.FullName.Replace($ProjectRoot, "").TrimStart("\").Replace("\", "/")
    Write-Host "  + $rel"
    Add-Content $OutputFile "`n=== FILE: $rel ===`n" -Encoding UTF8
    Add-Content $OutputFile (Get-Content $f.FullName -Raw -Encoding UTF8) -Encoding UTF8
}
$size = (Get-Item $OutputFile).Length
$tokens = [math]::Round($size / 4 / 1000)
Write-Host "`nFertig: $OutputFile ($([math]::Round($size/1024,1)) KB, ~$tokens k Tokens)" -ForegroundColor Green

$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$trial = Join-Path $repo 'target/adaptive-control-implementation'
$source = Join-Path $trial 'baseline-source'
if (Test-Path -LiteralPath $source) { throw "Baseline already exists: $source" }
New-Item -ItemType Directory -Path $source | Out-Null
foreach ($folder in @('swarm-core','swarm-nfp','data')) {
    Copy-Item -LiteralPath (Join-Path $repo $folder) -Destination $source -Recurse
}
foreach ($file in @('Cargo.lock','rust-toolchain.toml')) {
    Copy-Item -LiteralPath (Join-Path $repo $file) -Destination $source
}
# The pre-task backups include D (edge traversal reuse), but not this task's changes.
$before = Join-Path $trial 'before'
foreach ($file in Get-ChildItem -LiteralPath $before -Recurse -File) {
    $relative = $file.FullName.Substring($before.Length + 1)
    if ($relative.StartsWith('swarm-core\')) {
        Copy-Item -LiteralPath $file.FullName -Destination (Join-Path $source $relative) -Force
    }
}
$manifest = Get-Content -LiteralPath (Join-Path $repo 'Cargo.toml') -Raw
$profiles = [regex]::Match($manifest, '(?s)\[profile\.dev\].*').Value
if (-not $profiles) { throw 'Missing build profiles' }
$manifestText = "[workspace]`nmembers = [`"swarm-core`", `"swarm-nfp`"]`nresolver = `"3`"`n`n" + $profiles
[IO.File]::WriteAllText((Join-Path $source 'Cargo.toml'), $manifestText)
Get-ChildItem -LiteralPath (Join-Path $source 'swarm-core/src') -Recurse -File -Filter '*.rs' |
    Get-FileHash -Algorithm SHA256 | Select-Object Path,Hash |
    ConvertTo-Json -Depth 3 | Set-Content -LiteralPath (Join-Path $trial 'baseline-source-hashes.json') -Encoding utf8
Write-Output $source

param([switch]$DepthZero)
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
$taskDeps = Join-Path $taskRoot 'target/debug/deps'
$taskExe = Join-Path $taskRoot 'target/swarm_review_frozen_probe.exe'
$taskArgs = @('--edition=2024', (Join-Path $PSScriptRoot 'swarm_review_frozen_probe.rs'), '-L', ('dependency=' + $taskDeps), '-o', $taskExe)
foreach ($crate in @('swarm_core', 'jagua_rs', 'rand', 'serde_json')) {
    $taskLib = Get-ChildItem -LiteralPath $taskDeps -Filter ('lib' + $crate + '-*.rlib') |
        Sort-Object LastWriteTime -Descending | Select-Object -First 1
    if (-not $taskLib) { throw ('Build the documented Cargo test targets first; missing rlib: ' + $crate) }
    $taskArgs += @('--extern', ($crate + '=' + $taskLib.FullName))
}
& rustc @taskArgs
if ($LASTEXITCODE -ne 0) { throw 'Probe compilation failed. Build matching Cargo targets and retry.' }
if ($DepthZero) { & $taskExe --depth-zero } else { & $taskExe }
exit $LASTEXITCODE

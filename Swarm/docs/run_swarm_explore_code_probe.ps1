param([Parameter(Mandatory=$true)][string]$Label, [string[]]$Inputs = @('data/input/fu.json', 'data/input/wsjk156.json'))
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
$taskDeps = Join-Path $taskRoot 'target/release/deps'
$taskExe = Join-Path $taskRoot ('target/explore-code-' + $Label + '.exe')
$taskArgs = @('--edition=2024', '-O', (Join-Path $PSScriptRoot 'swarm_explore_code_probe.rs'), '-L', ('dependency=' + $taskDeps), '-o', $taskExe)
foreach ($crate in @('swarm_core', 'jagua_rs', 'rand', 'serde_json')) {
    $taskLib = Get-ChildItem -LiteralPath $taskDeps -Filter ('lib' + $crate + '-*.rlib') | Sort-Object LastWriteTime -Descending | Select-Object -First 1
    if (-not $taskLib) { throw ('Missing matching Release rlib: ' + $crate) }
    $taskArgs += @('--extern', ($crate + '=' + $taskLib.FullName))
}
& rustc @taskArgs
if ($LASTEXITCODE -ne 0) { throw 'Probe compilation failed' }
& $taskExe @Inputs | Tee-Object -FilePath (Join-Path $taskRoot ('target/explore-code-' + $Label + '.jsonl'))
if ($LASTEXITCODE -ne 0) { throw 'Probe failed' }

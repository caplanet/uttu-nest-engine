param([double]$Seconds = 30, [int[]]$Seeds = @(41,42,43))
$ErrorActionPreference = 'Stop'
$trialRepo = Split-Path -Parent $PSScriptRoot
$trialRoot = Join-Path $trialRepo 'target/async-separator-trial'
$trialExe = Join-Path $trialRepo 'target/release/examples/async_separator_trial.exe'
New-Item -ItemType Directory -Force -Path $trialRoot | Out-Null
$trialCases = @(
    @{ Name='wsjk'; Input='C:\Caplanet\SwarmInputFiles\여성WSjk_비교용-G_New_Swarm.json' },
    @{ Name='ops2'; Input=(Join-Path $trialRepo 'target/phase-improvements-bench/inputs/ops2.json') }
)
$trialManifest = [ordered]@{
    created=(Get-Date -Format o); seconds=$Seconds; seeds=$Seeds; workers=3
    overlap_mm=2.2; polysimpl=0.01; logical_processors=$env:NUMBER_OF_PROCESSORS
    binary_sha256=(Get-FileHash -LiteralPath $trialExe -Algorithm SHA256).Hash
    example_sha256=(Get-FileHash -LiteralPath (Join-Path $trialRepo 'swarm-core/examples/async_separator_trial.rs') -Algorithm SHA256).Hash
    cases=@($trialCases | ForEach-Object { @{ name=$_.Name; input=$_.Input; sha256=(Get-FileHash -LiteralPath $_.Input -Algorithm SHA256).Hash } })
}
$trialManifest | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $trialRoot 'manifest.json') -Encoding utf8
function Invoke-Trial($trialCase, $mode, $seed, $duration, $output) {
    $trialInitial = Join-Path $trialRoot ($trialCase.Name + '-initial.json')
    & $trialExe $trialCase.Input $trialInitial $output $mode $duration 3 $seed 2.2
    if ($LASTEXITCODE -ne 0) { throw "Trial failed: $($trialCase.Name) $mode seed=$seed" }
}
foreach ($trialCase in $trialCases) {
    $trialPrepare = Join-Path $trialRoot ($trialCase.Name + '-prepare.json')
    Invoke-Trial $trialCase 'prepare' 901 20 $trialPrepare
    Write-Output ("PREPARED " + $trialCase.Name + ' ' + (Get-Content -Raw -LiteralPath $trialPrepare))
    Invoke-Trial $trialCase 'probe' 41 0 (Join-Path $trialRoot ($trialCase.Name + '-barrier-probe.json'))
    Write-Output ("PROBED " + $trialCase.Name)
    for ($trialIndex=0; $trialIndex -lt $Seeds.Count; $trialIndex++) {
        $trialModes = if ($trialIndex % 2 -eq 0) { @('sync','async') } else { @('async','sync') }
        foreach ($trialMode in $trialModes) {
            $trialOutput = Join-Path $trialRoot ("$($trialCase.Name)-$trialMode-$($Seeds[$trialIndex]).json")
            Invoke-Trial $trialCase $trialMode $Seeds[$trialIndex] $Seconds $trialOutput
            $trialResult = Get-Content -Raw -LiteralPath $trialOutput | ConvertFrom-Json
            Write-Output ("DONE {0} {1} seed={2} width={3:F3} density={4:P3} elapsed={5:F3}s certified={6}" -f $trialCase.Name,$trialMode,$Seeds[$trialIndex],$trialResult.width,$trialResult.density,$trialResult.elapsed_seconds,$trialResult.certified)
        }
    }
}

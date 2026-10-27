param([double]$Seconds = 20, [int[]]$Seeds = @(41,42,43), [switch]$VerifyOnly)
$ErrorActionPreference = 'Stop'
$trialRepo = Split-Path -Parent $PSScriptRoot
$trialRoot = Join-Path $trialRepo 'target/separator-variants-trial'
$trialExe = Join-Path $trialRoot 'variants.exe'
$trialCases = @(
    @{ Name='wsjk'; Input='C:\Caplanet\SwarmInputFiles\여성WSjk_비교용-G_New_Swarm.json' },
    @{ Name='ops2'; Input=(Join-Path $trialRepo 'target/phase-improvements-bench/inputs/ops2.json') }
)
function Invoke-Trial($trialCase, $mode, $seed, $duration, $output) {
    $trialInitial = Join-Path $trialRepo ('target/async-separator-trial/' + $trialCase.Name + '-initial.json')
    & $trialExe $trialCase.Input $trialInitial $output $mode $duration 3 $seed 2.2
    if ($LASTEXITCODE -ne 0) { throw "Trial failed: $($trialCase.Name) $mode seed=$seed" }
}
if ($VerifyOnly) {
    foreach ($trialCase in $trialCases) {
        foreach ($trialMode in @('sync_fixed','preparation_fixed','kernel_fixed','noop_empty_checkempty')) {
            Invoke-Trial $trialCase $trialMode 41 0 (Join-Path $trialRoot ("$($trialCase.Name)-$trialMode.json"))
            Write-Output "CHECKED $($trialCase.Name) $trialMode"
        }
    }
    & python (Join-Path $PSScriptRoot 'check_separator_variants_trial.py')
    if ($LASTEXITCODE -ne 0) { throw 'Fixed-round equivalence check failed' }
    exit
}
$trialModes = @('sync','noop_empty','preparation','shared','kernel','async')
$trialManifest = [ordered]@{
    created=(Get-Date -Format o); seconds=$Seconds; seeds=$Seeds; workers=3; modes=$trialModes
    overlap_mm=2.2; polysimpl=0.01; logical_processors=$env:NUMBER_OF_PROCESSORS
    binary_sha256=(Get-FileHash -LiteralPath $trialExe -Algorithm SHA256).Hash
    cases=@($trialCases | ForEach-Object {
        $trialInitial = Join-Path $trialRepo ('target/async-separator-trial/' + $_.Name + '-initial.json')
        @{ name=$_.Name; input=$_.Input; sha256=(Get-FileHash -LiteralPath $_.Input -Algorithm SHA256).Hash
           initial=$trialInitial; initial_sha256=(Get-FileHash -LiteralPath $trialInitial -Algorithm SHA256).Hash }
    })
}
$trialManifest | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $trialRoot 'manifest.json') -Encoding utf8
for ($trialSeedIndex=0; $trialSeedIndex -lt $Seeds.Count; $trialSeedIndex++) {
    foreach ($trialCase in $trialCases) {
        for ($trialModeIndex=0; $trialModeIndex -lt $trialModes.Count; $trialModeIndex++) {
            $trialMode = $trialModes[($trialModeIndex + 2 * $trialSeedIndex) % $trialModes.Count]
            $trialOutput = Join-Path $trialRoot ("$($trialCase.Name)-$trialMode-$($Seeds[$trialSeedIndex]).json")
            if (Test-Path -LiteralPath $trialOutput) { throw "Refusing to overwrite $trialOutput" }
            Invoke-Trial $trialCase $trialMode $Seeds[$trialSeedIndex] $Seconds $trialOutput
            $trialResult = Get-Content -Raw -LiteralPath $trialOutput | ConvertFrom-Json
            Write-Output ("DONE {0} {1} seed={2} width={3:F3} density={4:P3} elapsed={5:F3}s certified={6}" -f $trialCase.Name,$trialMode,$Seeds[$trialSeedIndex],$trialResult.width,$trialResult.density,$trialResult.elapsed_seconds,$trialResult.certified)
        }
    }
}

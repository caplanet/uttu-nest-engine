param([double]$Seconds = 20, [int[]]$Seeds = @(41,42))
$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$trial = Join-Path $repo 'target/adaptive-control-implementation'
$cases = @(
    @{ Name='wsjk'; Input='C:/Caplanet/SwarmInputFiles/여성WSjk_비교용-G_New_Swarm.json' },
    @{ Name='ops2'; Input=(Join-Path $repo 'target/phase-improvements-bench/inputs/ops2.json') }
)
$manifest = [ordered]@{
    created=(Get-Date -Format o); seconds=$Seconds; seeds=$Seeds; geometry_workers=3
    overlap_mm=2.2; polysimpl=0.01; single_explore_call=$true
    logical_processors=$env:NUMBER_OF_PROCESSORS
    binaries=@('before','after' | ForEach-Object {
        @{ mode=$_; hash=(Get-FileHash -LiteralPath (Join-Path $trial "$_.exe")).Hash }
    })
    cases=@($cases | ForEach-Object {
        $initial=Join-Path $repo "target/async-separator-trial/$($_.Name)-initial.json"
        @{ name=$_.Name; input=$_.Input; input_hash=(Get-FileHash -LiteralPath $_.Input).Hash
           initial=$initial; initial_hash=(Get-FileHash -LiteralPath $initial).Hash }
    })
}
if (@($manifest.binaries.hash | Select-Object -Unique).Count -ne 2) {
    throw 'Before/after binaries are identical; rebuild using separate example target names.'
}
$manifest | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $trial 'trial-manifest.json') -Encoding utf8
for ($index=0; $index -lt $Seeds.Count; $index++) {
    $modes=if($index % 2 -eq 0){@('before','after')}else{@('after','before')}
    foreach($case in $cases){
        foreach($mode in $modes){
            $initial=Join-Path $repo "target/async-separator-trial/$($case.Name)-initial.json"
            $output=Join-Path $trial "$($case.Name)-$mode-$($Seeds[$index]).json"
            if(Test-Path -LiteralPath $output){throw "Refusing overwrite: $output"}
            & (Join-Path $trial "$mode.exe") $case.Input $initial $output sync $Seconds 3 $Seeds[$index] 2.2
            if($LASTEXITCODE -ne 0){throw "Trial failed: $mode $($case.Name)"}
            $result=Get-Content -LiteralPath $output -Raw | ConvertFrom-Json
            Write-Output ("{0} {1} seed={2} length={3:F3} density={4:P3} elapsed={5:F3}s certified={6}" -f
                $case.Name,$mode,$Seeds[$index],$result.width,$result.density,$result.elapsed_seconds,$result.certified)
        }
    }
}

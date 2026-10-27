param(
    [string]$InputFile = 'C:\Caplanet\TestFiles\SwarmNewTestFiles\#00093_DXF 태평양-겉감_Swarm.json',
    [int]$ExploreSeconds = 60,
    [int]$CompressSeconds = 60,
    [string]$RunTag = '2026-10-22-taepyeongyang-cpguard-phases'
)
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$harness = Join-Path $repo 'scripts/full_bench/bin/Release/net10.0/FullBench.dll'
$results = Join-Path $PSScriptRoot 'results'
New-Item -ItemType Directory -Path $results -Force | Out-Null
$overrides = Join-Path $results ($RunTag + '.config.json')
@{
    'structure_first.enabled' = 0
    'initial_placement.enabled' = 0
} | ConvertTo-Json | Set-Content -LiteralPath $overrides -Encoding utf8
$sequence = @('before', 'after', 'after', 'before')
for ($i = 0; $i -lt $sequence.Count; $i++) {
    $arm = $sequence[$i]
    $dll = Join-Path $repo "target/cpguard-policy/$arm/cpswcore.dll"
    $prefix = Join-Path $results "$RunTag-r$($i + 1)-$arm"
    Write-Output "Run $($i + 1)/$($sequence.Count): $arm"
    & dotnet $harness $dll $InputFile 1 $ExploreSeconds $CompressSeconds 0 0 $prefix $overrides 2>&1 |
        Tee-Object -FilePath ($prefix + '.log')
    if ($LASTEXITCODE -ne 0) { throw "Benchmark failed: $prefix (exit $LASTEXITCODE)" }
}

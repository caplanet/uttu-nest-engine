param(
    [string]$InputJson = 'C:/Caplanet/TestFiles/SwarmNewTestFiles/#00093_DXF 태평양-겉감_Swarm.json',
    [string]$BaselineDll = "$PSScriptRoot/../../target/init-optimization/baseline/release-cpswcore.dll",
    [string]$OptimizedDll = "$PSScriptRoot/../../target/release/cpswcore.dll",
    [string]$Prefix = '2026-10-22-taepyeongyang-opt'
)
$ErrorActionPreference = 'Stop'
$savedMode = $env:SWARM_BENCH_MODE
$savedCase = $env:SWARM_INIT_CASE
$savedTiered = $env:DOTNET_TieredCompilation
$savedProfile = $env:SWARM_INIT_PROFILE_PATH
try {
    $env:SWARM_BENCH_MODE = 'init'
    $env:SWARM_INIT_CASE = 'original_legacy_json'
    $env:DOTNET_TieredCompilation = '0'
    $env:SWARM_INIT_PROFILE_PATH = $null
    $cases = @(
        @{ Name = 'before-a'; Dll = $BaselineDll },
        @{ Name = 'after-a'; Dll = $OptimizedDll },
        @{ Name = 'after-b'; Dll = $OptimizedDll },
        @{ Name = 'before-b'; Dll = $BaselineDll }
    )
    $reference = $null
    foreach ($case in $cases) {
        $output = "$PSScriptRoot/results/$Prefix-$($case.Name).json"
        if (Test-Path -LiteralPath $output) { throw "Refusing to replace measurement: $output" }
        Write-Output "Starting $($case.Name) at $([DateTime]::UtcNow.ToString('o'))"
        & dotnet "$PSScriptRoot/bin/Release/net10.0/InputConversion.dll" `
            ([IO.Path]::GetFullPath($case.Dll)) $output $InputJson
        if ($LASTEXITCODE -ne 0) { throw "Benchmark failed: $($case.Name), exit $LASTEXITCODE" }
        $result = Get-Content -LiteralPath $output -Raw | ConvertFrom-Json
        $row = $result.Rows[0]
        if (-not $row.Success -or $row.ErrorCode -ne 0 -or $row.PlacedItems -ne 4000) {
            throw "Incomplete Init: $($case.Name)"
        }
        if ($null -eq $reference) { $reference = $result }
        foreach ($field in @('LayoutSha256', 'ConfigSha256', 'StripWidth', 'Density', 'PlacedItems')) {
            if ($row.$field -ne $reference.Rows[0].$field) { throw "Output mismatch: $field in $($case.Name)" }
        }
        if ($result.SourceSha256 -ne $reference.SourceSha256) { throw 'Input changed during comparison' }
        Write-Output "Verified $($case.Name): $($row.InitMs) ms, identical layout"
    }
} finally {
    $env:SWARM_BENCH_MODE = $savedMode
    $env:SWARM_INIT_CASE = $savedCase
    $env:DOTNET_TieredCompilation = $savedTiered
    $env:SWARM_INIT_PROFILE_PATH = $savedProfile
}

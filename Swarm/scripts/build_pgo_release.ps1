[CmdletBinding()]
param(
    [ValidateSet('All', 'Generate', 'Train', 'Use', 'Validate')]
    [string]$Stage = 'All',
    [string]$RunDirectory,
    [string[]]$Inputs = @(
        'C:/Caplanet/AiWorks/InputData/여성WSjk_비교용-G_New_SwarmV1.json',
        'target/phase-improvements-bench/inputs/ops2.json'
    ),
    [int[]]$Seeds = @(41, 42),
    [float]$OverlapMm = 2.2,
    [int]$ExploreSeconds = 10,
    [int]$CompressSeconds = 5,
    [int]$PolishSeconds = 5,
    [switch]$Deploy
)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$triple = 'x86_64-pc-windows-msvc'
$features = 'simd capmods adaptive structure-first jni'
if (!$RunDirectory) {
    if ($Stage -notin @('All', 'Generate')) { throw 'Resuming requires -RunDirectory.' }
    $RunDirectory = Join-Path $root ('target/pgo-build-' + (Get-Date -Format 'yyyyMMdd-HHmmss'))
}
$run = [IO.Path]::GetFullPath($RunDirectory)
if ($Stage -in @('All', 'Generate') -and (Test-Path -LiteralPath $run)) {
    throw "Use a fresh RunDirectory; refusing to mix previous profiles: $run"
}
if ($ExploreSeconds -le 0 -or $CompressSeconds -le 0 -or $PolishSeconds -le 0 -or
    !$Inputs.Count -or !$Seeds.Count -or ![float]::IsFinite($OverlapMm) -or $OverlapMm -lt 0) {
    throw 'Inputs, seeds and positive phase budgets are required; overlap must be finite and nonnegative.'
}
$savedEnv = @{}
$envNames = @('RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'LLVM_PROFILE_FILE',
    'CARGO_PROFILE_RELEASE_CODEGEN_UNITS', 'CARGO_PROFILE_RELEASE_OPT_LEVEL', 'CARGO_PROFILE_RELEASE_LTO')
foreach ($name in $envNames) { $savedEnv[$name] = [Environment]::GetEnvironmentVariable($name, 'Process') }
function Invoke-Logged([string]$Command, [string[]]$Arguments, [string]$Log) {
    & $Command @Arguments *> $Log
    if ($LASTEXITCODE -ne 0) { throw "$Command failed ($LASTEXITCODE); see $Log" }
}
function Hash([string]$Path) { (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash }
function Source-Fingerprint {
    $files = @(& rg --files src swarm-core/src swarm-nfp/src)
    if ($LASTEXITCODE -ne 0) { throw 'Cannot enumerate Rust sources.' }
    $files += @('Cargo.toml', 'Cargo.lock', 'build.rs', '.cargo/config.toml', 'swarm-core/Cargo.toml', 'swarm-nfp/Cargo.toml')
    (($files | Sort-Object -Unique | ForEach-Object { "$_ $(Hash $_)" }) -join "`n")
}
Push-Location $root
try {
    $resolvedInputs = @($Inputs | ForEach-Object { (Resolve-Path -LiteralPath $_).Path })
    $sysroot = (& rustc --print sysroot).Trim()
    if ($LASTEXITCODE -ne 0) { throw 'rustc unavailable' }
    $profdata = Join-Path $sysroot "lib/rustlib/$triple/bin/llvm-profdata.exe"
    if (!(Test-Path -LiteralPath $profdata)) { throw 'Install the matching tool: rustup component add llvm-tools-preview' }
    New-Item -ItemType Directory -Force -Path $run, "$run/raw", "$run/host", "$run/instrumented", "$run/final" | Out-Null
    $raw = "$run/raw".Replace('\', '/')
    $merged = "$run/merged.profdata".Replace('\', '/')
    $cargoDll = "$run/cargo/$triple/release/cpswcore.dll"
    $pgoHost = "$run/host/PgoTrain.exe"
    $env:CARGO_ENCODED_RUSTFLAGS = $null
    $env:CARGO_PROFILE_RELEASE_CODEGEN_UNITS = '1'
    $env:CARGO_PROFILE_RELEASE_OPT_LEVEL = '3'
    $env:CARGO_PROFILE_RELEASE_LTO = 'fat'
    # RUSTFLAGS replaces .cargo/config.toml flags: explicitly preserve the static CRT.
    $baseFlags = '-C target-feature=+crt-static'
    $cargoArgs = @('rustc', '--locked', '--release', '--target', $triple, '--target-dir', "$run/cargo",
        '--lib', '--crate-type', 'cdylib', '--features', $features, '-vv', '--', '-C', 'lto=fat')
    if ($Stage -in @('All', 'Generate')) {
        New-Item -ItemType Directory -Force -Path "$run/before" | Out-Null
        Copy-Item -LiteralPath Cargo.toml -Destination "$run/before/Cargo.toml"
        if (Test-Path target/release/cpswcore.dll) {
            Copy-Item -LiteralPath target/release/cpswcore.dll -Destination "$run/before/cpswcore.dll"
        }
        Write-Host 'Building instrumented DLL (static CRT, fat LTO, CGU=1)...'
        $sourceBefore = Source-Fingerprint
        $env:RUSTFLAGS = "$baseFlags -C profile-generate=$raw"
        Invoke-Logged cargo $cargoArgs "$run/generate-build.log"
        if ($sourceBefore -ne (Source-Fingerprint)) { throw 'Rust sources changed during generation.' }
        Set-Content "$run/generation-sources.txt" $sourceBefore -Encoding utf8
        Copy-Item -LiteralPath $cargoDll -Destination "$run/instrumented/cpswcore.dll"
    }
    if ($Stage -in @('All', 'Train', 'Validate')) {
        Invoke-Logged dotnet @('build', 'scripts/pgo/PgoTrain.csproj', '-c', 'Release', '-o', "$run/host", '--nologo', '-v', 'minimal') "$run/host-build.log"
    }
    if ($Stage -in @('All', 'Train')) {
        if ((Test-Path "$run/generation-sources.txt") -and
            (Get-Content "$run/generation-sources.txt" -Raw).TrimEnd() -ne (Source-Fingerprint)) {
            throw 'Rust sources changed after generation; collect a fresh profile.'
        }
        if (@(Get-ChildItem -LiteralPath "$run/raw" -Filter '*.profraw').Count) {
            throw 'Raw profiles already exist. Use a fresh run instead of silently merging repeated training.'
        }
        if (!(Test-Path "$run/instrumented/cpswcore.dll")) {
            # Resume a generation performed with the documented exact command.
            if (!(Select-String -LiteralPath "$run/generate-build.log" -Pattern 'profile-generate=' -Quiet)) {
                throw 'Missing instrumentation build evidence.'
            }
            Copy-Item -LiteralPath $cargoDll -Destination "$run/instrumented/cpswcore.dll"
        }
        $dll = "$run/instrumented/cpswcore.dll"
        $trainingSources = Source-Fingerprint
        $records = @()
        for ($i = 0; $i -lt $resolvedInputs.Count; $i++) {
            foreach ($seed in $Seeds) {
                $name = "marker$i-seed$seed"
                $env:LLVM_PROFILE_FILE = "$raw/$name-%p-%m.profraw"
                Write-Host "Training $name (Init / Explore / Compress / Polish)..."
                Invoke-Logged $pgoHost @($dll, $resolvedInputs[$i], "$seed", $OverlapMm.ToString([Globalization.CultureInfo]::InvariantCulture),
                    "$ExploreSeconds", "$CompressSeconds", "$PolishSeconds", "$run/training/$name") "$run/$name.log"
                $profiles = @(Get-ChildItem -LiteralPath "$run/raw" -Filter "$name-*.profraw" | Where-Object Length -gt 0)
                if (!$profiles.Count) { throw "No profile was flushed by the DLL for $name" }
                $records += @{ name=$name; input=$resolvedInputs[$i]; input_sha256=(Hash $resolvedInputs[$i]);
                    seed=$seed; profiles=@($profiles.Name) }
            }
        }
        $env:LLVM_PROFILE_FILE = $null
        Invoke-Logged $profdata @('merge', '-o', $merged, $raw) "$run/merge.log"
        Invoke-Logged $profdata @('show', '--all-functions', '--counts', '--topn=30', $merged) "$run/profile-summary.txt"
        if ($trainingSources -ne (Source-Fingerprint)) { throw 'Rust sources changed during training.' }
        Set-Content "$run/training-sources.txt" $trainingSources -Encoding utf8
        @{ toolchain=(& rustc -vV); features=$features; target=$triple; codegen_units=1; opt_level=3; lto='fat';
            crt_static=$true; instrumented_sha256=(Hash $dll); profile_sha256=(Hash $merged); runs=$records } |
            ConvertTo-Json -Depth 8 | Set-Content -LiteralPath "$run/training-manifest.json" -Encoding utf8
    }
    if ($Stage -in @('All', 'Use')) {
        if (!(Test-Path -LiteralPath $merged)) { throw 'Run Train before Use.' }
        $manifest = Get-Content "$run/training-manifest.json" -Raw | ConvertFrom-Json
        if ((Hash $merged) -ne $manifest.profile_sha256 -or
            ((& rustc -vV) -join "`n") -ne ($manifest.toolchain -join "`n") -or
            (Get-Content "$run/training-sources.txt" -Raw).TrimEnd() -ne (Source-Fingerprint)) {
            throw 'Profile hash, Rust toolchain or sources differ from training; collect a fresh profile.'
        }
        Write-Host 'Building final DLL with profile-use...'
        $env:RUSTFLAGS = "$baseFlags -C profile-use=$merged -C llvm-args=-pgo-warn-missing-function"
        Invoke-Logged cargo $cargoArgs "$run/use-build.log"
        Copy-Item -LiteralPath $cargoDll -Destination "$run/final/cpswcore.dll"
    }
    if ($Stage -in @('All', 'Validate')) {
        $env:LLVM_PROFILE_FILE = $null
        $dll = "$run/final/cpswcore.dll"
        $readobj = Join-Path $sysroot "lib/rustlib/$triple/bin/llvm-readobj.exe"
        Invoke-Logged $readobj @('--coff-imports', $dll) "$run/final-imports.txt"
        Invoke-Logged $readobj @('--sections', $dll) "$run/final-sections.txt"
        if (Select-String -LiteralPath "$run/final-sections.txt" -Pattern 'Name: \.lprf' -Quiet) {
            throw 'Final DLL still contains LLVM profiling counters; do not deploy an instrumented DLL.'
        }
        if (Select-String -LiteralPath "$run/final-imports.txt" -Pattern '(?i)Name: (MSVCP140|VCRUNTIME140|api-ms-win-crt-)' -Quiet) {
            throw 'Final DLL unexpectedly depends on dynamic VC++/UCRT DLLs.'
        }
        $rootCommand = @(Get-Content "$run/use-build.log" | Where-Object { $_ -match '--crate-name cpswcore ' })
        if ($rootCommand.Count -ne 1) { throw 'Expected one root rustc command proving PGO application.' }
        foreach ($flag in @('-C opt-level=3', '-C codegen-units=1', '-C lto=fat', '-C target-feature=+crt-static', "-C profile-use=$merged")) {
            if (!$rootCommand[0].Contains($flag)) { throw "Missing final compiler flag: $flag" }
        }
        for ($i = 0; $i -lt $resolvedInputs.Count; $i++) {
            Write-Host "Validating final DLL on marker$i..."
            Invoke-Logged $pgoHost @($dll, $resolvedInputs[$i], '73', $OverlapMm.ToString([Globalization.CultureInfo]::InvariantCulture),
                "$ExploreSeconds", "$CompressSeconds", "$PolishSeconds", "$run/validation/marker$i-seed73") "$run/validation-marker$i.log"
        }
        @{ dll=$dll; sha256=(Hash $dll); profile_sha256=(Hash $merged); validated_inputs=$resolvedInputs;
            checks=@('managed/native ABI handshake and required exports', 'all four phases', 'placement demand',
                'native collision status', 'nonincreasing length', 'static CRT imports', 'no profiling counter sections', 'actual rustc PGO/CGU/LTO flags') } |
            ConvertTo-Json -Depth 5 | Set-Content -LiteralPath "$run/validation-manifest.json" -Encoding utf8
        if ($Deploy) {
            Copy-Item -LiteralPath $dll -Destination target/release/cpswcore.dll
            if ((Hash $dll) -ne (Hash target/release/cpswcore.dll)) { throw 'Deployed DLL hash mismatch.' }
        }
    }
    Write-Host "Stage $Stage completed: $run"
}
finally {
    foreach ($name in $envNames) { [Environment]::SetEnvironmentVariable($name, $savedEnv[$name], 'Process') }
    Pop-Location
}

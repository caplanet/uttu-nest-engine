# Runs one FullBench measurement pinned to a fixed 3-core affinity mask.
# Default mask 224 = cores 5,6,7 (leaves cores 0-4 free for other work).
param(
  [Parameter(Mandatory = $true)][string]$Dll,
  [Parameter(Mandatory = $true)][string]$Marker,
  [Parameter(Mandatory = $true)][int]$Seed,
  [Parameter(Mandatory = $true)][string]$OutPrefix,
  [double]$ExploreS = 480,
  [double]$CompressS = 60,
  [double]$PolishS = 60,
  [double]$OverlapMm = 0.8,
  [string]$Overrides = '',
  [int]$AffinityMask = 224,
  [int]$RayonThreads = 3,
  [switch]$CaptureLog
)

$ErrorActionPreference = 'Stop'
$exe = Join-Path $PSScriptRoot 'bin\Release\net10.0\FullBench.exe'
if (-not (Test-Path $exe)) { throw "FullBench.exe not found at $exe" }
$dir = Split-Path -Parent $OutPrefix
if ($dir -and -not (Test-Path $dir)) { New-Item -ItemType Directory -Force -Path $dir | Out-Null }

# Cap every rayon pool inside the native DLL as well as the affinity mask.
# Keep these two in step: a rayon pool wider than the mask just oversubscribes
# the pinned cores. -RayonThreads exists for the one-core-per-run arms, where
# three independent searches share the same three-core budget.
$env:RAYON_NUM_THREADS = "$RayonThreads"

$argList = @($Dll, $Marker, $Seed, $ExploreS, $CompressS, $PolishS, $OverlapMm, $OutPrefix)
if ($Overrides -ne '') { $argList += $Overrides }
if ($CaptureLog) { $argList += '--log' }

$stdout = "$OutPrefix.stdout.txt"
$stderr = "$OutPrefix.stderr.txt"
$sw = [System.Diagnostics.Stopwatch]::StartNew()
$p = Start-Process -FilePath $exe -ArgumentList $argList -PassThru -NoNewWindow `
  -RedirectStandardOutput $stdout -RedirectStandardError $stderr
# Windows PowerShell only reports ExitCode for a -PassThru process whose handle was
# opened before it exited. Without this the code read back as $null, so every run --
# passing or not -- printed "FullBench exit= after ...".
$null = $p.Handle
try { $p.ProcessorAffinity = [IntPtr]$AffinityMask } catch { Write-Warning "affinity set failed: $_" }
$p.WaitForExit()
$sw.Stop()

Get-Content $stdout | ForEach-Object { Write-Output $_ }
if ($p.ExitCode -ne 0) {
  Write-Warning "FullBench exit=$($p.ExitCode) after $([math]::Round($sw.Elapsed.TotalSeconds,1))s"
  Get-Content $stderr -ErrorAction SilentlyContinue | Select-Object -First 40 | ForEach-Object { Write-Warning $_ }
}
exit $p.ExitCode

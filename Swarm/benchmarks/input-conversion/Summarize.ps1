param(
    [Parameter(Mandatory=$true)][string]$OutputPath,
    [Parameter(Mandatory=$true, ValueFromRemainingArguments=$true)][string[]]$Reports
)
$ErrorActionPreference = 'Stop'
function Get-Median($values) {
    $sorted = @($values | Sort-Object)
    $middle = [int][Math]::Floor($sorted.Count / 2)
    if ($sorted.Count % 2) { return $sorted[$middle] }
    return ($sorted[$middle - 1] + $sorted[$middle]) / 2
}
$runs = @($Reports | ForEach-Object { Get-Content -LiteralPath $_ -Raw | ConvertFrom-Json })
if (@($runs.DllSha256 | Sort-Object -Unique).Count -ne 1) { throw 'DLL hashes differ' }
if (@($runs.TieredCompilation | Sort-Object -Unique).Count -ne 1) { throw 'JIT settings differ' }
$results = foreach ($fixture in $runs[0].Results) {
    $matches = @($runs | ForEach-Object { $_.Results | Where-Object SourceSha256 -eq $fixture.SourceSha256 })
    if ($matches.Count -ne $runs.Count) { throw 'Fixture is absent in one of the reports' }
    $rows = foreach ($row in $fixture.Rows) {
        $allRows = @($matches | ForEach-Object { $_.Rows | Where-Object Name -eq $row.Name })
        $times = @($allRows | ForEach-Object { $_.SamplesMs })
        $sorted = @($times | Sort-Object)
        [pscustomobject]@{
            Name = $row.Name; Samples = $times.Count
            MedianMs = Get-Median $times
            P95Ms = $sorted[[int][Math]::Ceiling($times.Count * 0.95) - 1]
            ManagedBytesMedian = Get-Median @($allRows | ForEach-Object { $_.ManagedBytes })
        }
    }
    [pscustomobject]@{ File = $fixture.File; Items = $fixture.Items; Vertices = $fixture.Vertices
        PrettyJsonBytes = $fixture.PrettyJsonBytes; CompactJsonBytes = $fixture.CompactJsonBytes; Rows = @($rows) }
}
[pscustomobject]@{ DllSha256 = $runs[0].DllSha256; Reports = $Reports; Results = @($results) } |
    ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $OutputPath -Encoding utf8
foreach ($result in $results) {
    $fullJson = ($result.Rows | Where-Object Name -eq 'csharp_json_pretty').MedianMs
    $fullDirect = ($result.Rows | Where-Object Name -eq 'csharp_struct').MedianMs
    $nativeJson = ($result.Rows | Where-Object Name -eq 'native_json_pretty').MedianMs
    $nativeDirect = ($result.Rows | Where-Object Name -eq 'native_struct').MedianMs
    '{0}: C# {1:F4} -> {2:F4} ms ({3:F1}x); native {4:F4} -> {5:F4} ms ({6:F1}x)' -f
        ([IO.Path]::GetFileName($result.File)), $fullJson, $fullDirect, ($fullJson/$fullDirect),
        $nativeJson, $nativeDirect, ($nativeJson/$nativeDirect)
}

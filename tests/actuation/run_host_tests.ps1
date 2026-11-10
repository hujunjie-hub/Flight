<#
    Builds and runs the FMAT-0013 M7 actuator transaction host test.
    The guard is compiled straight from middleware/actuator/output_guard with the
    same -Wall -Wextra settings the other host suites use, so the test cannot
    drift into a private copy of the logic.
#>
[CmdletBinding()]
param(
    [string]$Gcc = 'D:\STM32CubeCLT\mingw64\bin\gcc.exe',
    [string]$OutDir = ''
)

$ErrorActionPreference = 'Stop'
if (-not (Test-Path -LiteralPath $Gcc)) {
    # Fall back to the MinGW that ships with MATLAB before giving up.
    $candidate = 'C:\ProgramData\MATLAB\SupportPackages\R2026b\3P.instrset\mingw_w64.instrset\bin\gcc.exe'
    if (Test-Path -LiteralPath $candidate) { $Gcc = $candidate }
}

$layoutRoot = Get-Item -LiteralPath $PSScriptRoot
while (-not (Test-Path -LiteralPath (Join-Path $layoutRoot.FullName 'SConstruct'))) {
    if ($null -eq $layoutRoot.Parent) { throw 'Flight root not found' }
    $layoutRoot = $layoutRoot.Parent
}
$repoRoot = $layoutRoot.FullName
if ([string]::IsNullOrWhiteSpace($OutDir)) { $OutDir = Join-Path $repoRoot 'build\validation\m7' }
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$work = Join-Path $OutDir 'obj'
New-Item -ItemType Directory -Force -Path $work | Out-Null

$guardSource = Join-Path $repoRoot 'middleware\actuator\output_guard\actuator_guard.c'
$testSource = Join-Path $PSScriptRoot 'test_m7_main.c'
$includes = @('-I', (Join-Path $repoRoot 'middleware\actuator'),
              '-I', (Join-Path $repoRoot 'middleware\contracts'),
              '-I', (Join-Path $repoRoot 'middleware'))

$exe = Join-Path $work 'test_m7.exe'
$log = Join-Path $OutDir 'test_m7.log'
$arguments = @('-std=c11', '-O2', '-Wall', '-Wextra', '-Wconversion', '-static', '-DFLIGHT_HOST_TEST') +
             $includes + @($testSource, $guardSource, '-lm', '-o', $exe)
$lines = @('# test: test_m7', "# compiler: $Gcc", "# command: `"$Gcc`" " + ($arguments -join ' '))
$build = & $Gcc @arguments 2>&1
$lines += "# build exit: $LASTEXITCODE"
if ($build) { $lines += ($build | ForEach-Object { "  $_" }) }

$result = 'BUILD_FAILED'
$summary = $null
if ($LASTEXITCODE -eq 0) {
    $run = & $exe 2>&1
    $lines += "# run exit: $LASTEXITCODE"
    $lines += ($run | ForEach-Object { "  $_" })
    $result = if ($LASTEXITCODE -eq 0) { 'PASS' } else { 'FAIL' }
    Write-Host ($run | Out-String)
    $counts = ($run | Select-String -Pattern 'M7 checks=(\d+) failures=(\d+)' | Select-Object -Last 1)
    if ($counts) {
        $summary = [ordered]@{
            checks = [int]$counts.Matches[0].Groups[1].Value
            failures = [int]$counts.Matches[0].Groups[2].Value
        }
    }
}
$lines += "# RESULT: $result"
$lines | Set-Content -Encoding UTF8 $log

$report = [ordered]@{
    scope = 'M7_ACTUATOR_TRANSACTION'
    status = $result
    compiler = $Gcc
    checks = if ($summary) { $summary.checks } else { 0 }
    failures = if ($summary) { $summary.failures } else { 0 }
    cases = @(
        'M7-01 valid permit commits once at a frame boundary'
        'M7-02 publishing a demand is not a commit'
        'M7-03 a 500 us frame is rejected, no duplicate apply'
        'M7-04 the next 2.5 ms frame applies within the slew limit'
        'M7-05 a revoked generation cannot be replayed'
        'M7-06 a revoked generation cannot be resurrected as active'
        'M7-07 out-of-order renewal of one generation is stale'
        'M7-08 permits from another boot or time epoch are refused'
        'M7-09 an expired permit stops being bound'
        'M7-10 a demand older than 5 ms enters SAFE_ACTION'
        'M7-11 only a well-formed ground stop certificate is accepted'
        'M7-12 a ground stop is accepted after the permit is revoked'
        'M7-13 the thrust ramp never exceeds 100 N/s and never jumps'
        'M7-14 the completing receipt is ZERO_TARGET_APPLIED with zero thrust'
        'M7-15 zero thrust is confirmed within 105.5 ms of acceptance'
        'M7-16 DISARMED follows only the zero receipt and output stays zero'
    )
    permit_lifecycle = if ($result -eq 'PASS') { 'PASS' } else { 'FAIL' }
    frame_apply_500us = if ($result -eq 'PASS') { 'PASS' } else { 'FAIL' }
    ground_stop_transaction = if ($result -eq 'PASS') { 'PASS' } else { 'FAIL' }
    ground_stop_evidence = 'FIRMWARE_GROUND_STOP_CERTIFICATE'
    ground_stop_max_apply_us = 105500
    ground_stop_max_ramp_nps = 100.0
    ground_stop_log = (Select-String -Path $log -Pattern 'ground stop:' |
        Select-Object -Last 1).Line.Trim()
    log = $log
}
$report | ConvertTo-Json -Depth 5 | Set-Content -Encoding UTF8 (Join-Path $OutDir 'actuation_host_summary.json')
Write-Host ("M7 host: " + $result + " (" + $report.checks + " checks, " + $report.failures + " failures)")
if ($result -ne 'PASS') { exit 1 }
exit 0

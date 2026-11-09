<#
    Builds and runs the Sensor Integrity Manager Host test and the Cortex-M7
    compile check.  Logs go to build/validation/b_replay_fdir/.
#>

[CmdletBinding()]
param(
    [string]$Gcc = 'D:\STM32CubeCLT\mingw64\bin\gcc.exe',
    [string]$ArmGcc = 'D:\STM32CubeCLT\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe',
    [string]$OutDir = ''
)

$ErrorActionPreference = 'Stop'
$scriptDir = $PSScriptRoot
$layoutRoot = Get-Item -LiteralPath $PSScriptRoot
while (-not (Test-Path -LiteralPath (Join-Path $layoutRoot.FullName 'SConstruct'))) {
  if ($null -eq $layoutRoot.Parent) { throw 'Flight root not found' }
  $layoutRoot = $layoutRoot.Parent
}
$repoRoot = $layoutRoot.FullName
$healthDir = Join-Path $repoRoot 'middleware\sensor\source_health'

if ([string]::IsNullOrWhiteSpace($OutDir)) {
    $OutDir = Join-Path $repoRoot 'build\validation\b_replay_fdir'
}
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$work = Join-Path $OutDir 'obj'
New-Item -ItemType Directory -Force -Path $work | Out-Null

$source = Join-Path $healthDir 'sensor_integrity.c'
$testSource = Join-Path $PSScriptRoot 'test_health.c'
$include = Split-Path -Parent $healthDir

$results = @()

$exe = Join-Path $work 'test_health.exe'
$log = Join-Path $OutDir 'test_health.log'
$arguments = @('-std=c11', '-O2', '-Wall', '-Wextra', '-Wconversion', '-static',
               '-I', $include, $testSource, $source, '-o', $exe)
$lines = @('# test: test_health', "# compiler: $Gcc", "# command: `"$Gcc`" " + ($arguments -join ' '))
$build = & $Gcc @arguments 2>&1
$lines += "# build exit: $LASTEXITCODE"
if ($build) { $lines += ($build | ForEach-Object { "  $_" }) }
$result = 'BUILD_FAILED'
if ($LASTEXITCODE -eq 0) {
    $run = & $exe 2>&1
    $lines += "# run exit: $LASTEXITCODE"
    $lines += ($run | ForEach-Object { "  $_" })
    $result = if ($LASTEXITCODE -eq 0) { 'PASS' } else { 'FAIL' }
    Write-Host ($run | Out-String)
}
$lines += "# RESULT: $result"
$lines | Set-Content -Encoding UTF8 $log
$results += [pscustomobject]@{ Test = 'test_health'; Result = $result; Log = $log }

$object = Join-Path $work 'sensor_integrity_arm.o'
$armLog = Join-Path $OutDir 'arm_health.log'
$armArguments = @('-std=c11', '-mcpu=cortex-m7', '-mthumb', '-mfpu=fpv5-d16', '-mfloat-abi=hard',
                  '-O2', '-Wall', '-Wextra', '-Werror', '-I', $include,
                  '-c', $source, '-o', $object)
$armLines = @('# Cortex-M7 compile check', "# compiler: $ArmGcc",
              "# command: `"$ArmGcc`" " + ($armArguments -join ' '))
$armOutput = & $ArmGcc @armArguments 2>&1
$armLines += "# exit: $LASTEXITCODE"
if ($armOutput) { $armLines += ($armOutput | ForEach-Object { "  $_" }) }
if ($LASTEXITCODE -eq 0) {
    $bin = Split-Path $ArmGcc -Parent
    $armLines += (& (Join-Path $bin 'arm-none-eabi-size.exe') $object 2>&1 | ForEach-Object { "  $_" })
    $symbols = & (Join-Path $bin 'arm-none-eabi-nm.exe') -u $object 2>&1
    $armLines += '# undefined symbols'
    $armLines += ($symbols | ForEach-Object { "  $_" })
}
$armResult = if ($LASTEXITCODE -eq 0) { 'PASS' } else { 'FAIL' }
$armLines += "# RESULT: $armResult"
$armLines | Set-Content -Encoding UTF8 $armLog
Write-Host "ARM compile (health): $armResult"
$results += [pscustomobject]@{ Test = 'arm_compile_health'; Result = $armResult; Log = $armLog }

$results | Format-Table -AutoSize | Out-String | Write-Host
$results | ConvertTo-Json -Depth 4 | Set-Content -Encoding UTF8 (Join-Path $OutDir 'health_tests_summary.json')
$failed = @($results | Where-Object { $_.Result -ne 'PASS' })
if ($failed.Count -gt 0) { exit 1 }
exit 0

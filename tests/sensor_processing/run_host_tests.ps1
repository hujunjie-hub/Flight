<#
    Builds and runs the M2 sensor-management Host test suite plus the
    independent IMU channel round-trip, and records commands/outputs under
    build/validation/m2_adapter/.

    Usage:
        pwsh tests/sensor_processing/run_host_tests.ps1
        pwsh tests/sensor_processing/run_host_tests.ps1 -Gcc <path> -OutDir <dir>
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
$m2Dir = Join-Path $repoRoot 'middleware\sensor'
$simDir = Join-Path $repoRoot 'tests\support\sensor_sim'

if ([string]::IsNullOrWhiteSpace($OutDir)) {
    $OutDir = Join-Path $repoRoot 'build\validation\m2_adapter'
}
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$work = Join-Path $OutDir 'obj'
New-Item -ItemType Directory -Force -Path $work | Out-Null

$includes = @(
    (Join-Path $repoRoot 'middleware/contracts'),
    (Join-Path $repoRoot 'sensor_device'),
    (Join-Path $repoRoot 'sensor_device/imu'),
    (Join-Path $repoRoot 'sensor_device/barometer'),
    $m2Dir,
    (Join-Path $simDir 'include')
)
$processingSources = @(
    (Join-Path $m2Dir 'sample_normalization/sensor_sample.c'),
    (Join-Path $m2Dir 'calibration/sensor_calibration.c'),
    (Join-Path $m2Dir 'measurement_filter/sensor_filter.c')
)
$codecSources = @(
    (Join-Path $repoRoot 'sensor_device/imu/adis16505.c'),
    (Join-Path $repoRoot 'sensor_device/barometer/bmp585.c')
)

$results = @()

function Invoke-HostTest {
    param([string]$Name, [string[]]$Sources, [string[]]$ExtraFlags = @())

    $exe = Join-Path $work ($Name + '.exe')
    $log = Join-Path $OutDir ($Name + '.log')
    $arguments = @('-std=c11', '-O2', '-Wall', '-Wextra', '-Wconversion', '-static',
                   '-DFLIGHT_DRIVER_CODEC_TEST') + $ExtraFlags
    foreach ($include in $includes) { $arguments += @('-I', $include) }
    $arguments += $Sources + $processingSources + $codecSources
    $arguments += @('-lm', '-o', $exe)

    $lines = @("# test: $Name", "# compiler: $Gcc", "# command: `"$Gcc`" " + ($arguments -join ' '))
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
    $script:results += [pscustomobject]@{ Test = $Name; Result = $result; Log = $log }
}

function Invoke-ArmCompile {
    param([string]$Source)
    $name = [IO.Path]::GetFileNameWithoutExtension($Source)
    $object = Join-Path $work "$($name)_arm.o"
    $log = Join-Path $OutDir "arm_$name.log"
    $arguments = @('-std=c11', '-mcpu=cortex-m7', '-mthumb', '-mfpu=fpv5-d16', '-mfloat-abi=hard',
                   '-O2', '-Wall', '-Wextra', '-Werror', '-fno-short-enums', '-DFLIGHT_DRIVER_CODEC_TEST')
    foreach ($include in $includes) { $arguments += @('-I', $include) }
    $arguments += @('-c', $Source, '-o', $object)

    $lines = @('# Cortex-M7 compile check (spec 09 REQ-NAV-KFG-002/009 scope)',
               "# compiler: $ArmGcc",
               "# command: `"$ArmGcc`" " + ($arguments -join ' '))
    $output = & $ArmGcc @arguments 2>&1
    $result = if ($LASTEXITCODE -eq 0) { 'PASS' } else { 'FAIL' }
    $lines += "# compile: $result"
    if ($output) { $lines += ($output | ForEach-Object { "  $_" }) }
    if ($result -eq 'PASS') {
        $size = & (Join-Path (Split-Path $ArmGcc -Parent) 'arm-none-eabi-size.exe') $object 2>&1
        $symbols = & (Join-Path (Split-Path $ArmGcc -Parent) 'arm-none-eabi-nm.exe') -u $object 2>&1
        $lines += ($size | ForEach-Object { "  $_" })
        $lines += '# undefined symbols'
        $lines += ($symbols | ForEach-Object { "  $_" })
        $heap = $symbols | Select-String -Pattern 'malloc|free|operator new|__cxa_throw|_Unwind'
        $lines += '# heap/exception symbols: ' + $(if ($heap) { ($heap | ForEach-Object { $_.Line }) -join ', ' } else { 'none' })
    }
    $lines += "# RESULT: $result"
    $lines | Set-Content -Encoding UTF8 $log
    Write-Host "ARM compile $name $result"
    $script:results += [pscustomobject]@{ Test = "arm_$name"; Result = $result; Log = $log }
}

Write-Host "M2 sensor management host tests"
Write-Host "  repo: $repoRoot"
Write-Host "  logs: $OutDir"
Write-Host ""

Invoke-HostTest -Name 'test_m2' -Sources @(
    (Join-Path $PSScriptRoot 'test_m2.c'))

Invoke-HostTest -Name 'test_imu_sim' -Sources @(
    (Join-Path $simDir 'hosttest\test_imu_sim.c'),
    (Join-Path $simDir 'src\imu_sim.c'))

foreach ($source in ($processingSources + $codecSources)) {
    Invoke-ArmCompile -Source $source
}

$results | Format-Table -AutoSize | Out-String | Write-Host
$results | ConvertTo-Json -Depth 4 | Set-Content -Encoding UTF8 (Join-Path $OutDir 'host_tests_summary.json')

$failed = @($results | Where-Object { $_.Result -ne 'PASS' })
if ($failed.Count -gt 0) { exit 1 }
exit 0

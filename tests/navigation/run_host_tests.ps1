<#
    Builds and runs the M3 replay/journal Host test and the Cortex-M7 compile
    check.  Logs go to build/validation/b_replay_fdir/.
#>

[CmdletBinding()]
param(
    [string]$Gxx = 'D:\STM32CubeCLT\mingw64\bin\g++.exe',
    [string]$ArmGxx = 'D:\STM32CubeCLT\GNU-tools-for-STM32\bin\arm-none-eabi-g++.exe',
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
$m3Dir = Join-Path $repoRoot 'middleware\model\ins'
$pkg = Join-Path $m3Dir 'kf_gins'

if ([string]::IsNullOrWhiteSpace($OutDir)) {
    $OutDir = Join-Path $repoRoot 'build\validation\b_replay_fdir'
}
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$work = Join-Path $OutDir 'obj'
New-Item -ItemType Directory -Force -Path $work | Out-Null

$coreSources = @(
    (Join-Path $pkg 'trimmed_core\src\quad_core.cpp'),
    (Join-Path $pkg 'trimmed_core\src\quad_models.cpp'))
$replaySources = @((Join-Path $pkg 'observation_replay/ins_replay.cpp'))

$results = @()

function Invoke-HostTest {
    $exe = Join-Path $work 'test_replay.exe'
    $log = Join-Path $OutDir 'test_replay.log'
    $arguments = @('-std=c++14', '-fno-exceptions', '-fno-rtti', '-O2', '-Wall', '-Wextra', '-Wshadow',
                   '-static', '-static-libgcc', '-static-libstdc++',
                   '-I', (Join-Path $pkg 'trimmed_core\include'), '-I', $pkg)
    $arguments += @((Join-Path $PSScriptRoot 'test_replay.cpp')) + $replaySources + $coreSources
    $arguments += @('-o', $exe)

    $lines = @('# test: test_replay', "# compiler: $Gxx", "# command: `"$Gxx`" " + ($arguments -join ' '))
    $build = & $Gxx @arguments 2>&1
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
    $script:results += [pscustomobject]@{ Test = 'test_replay'; Result = $result; Log = $log }
}

function Invoke-ArmCompile {
    $object = Join-Path $work 'ins_replay_arm.o'
    $log = Join-Path $OutDir 'arm_m3_replay.log'
    $arguments = @('-std=c++14', '-fno-exceptions', '-fno-rtti', '-fno-threadsafe-statics',
                   '-mcpu=cortex-m7', '-mthumb', '-mfpu=fpv5-d16', '-mfloat-abi=hard',
                   '-O2', '-Wall', '-Wextra', '-Werror',
                   '-I', (Join-Path $pkg 'trimmed_core\include'), '-I', $pkg,
                   '-c', (Join-Path $pkg 'observation_replay/ins_replay.cpp'), '-o', $object)
    $lines = @('# Cortex-M7 compile check', "# compiler: $ArmGxx",
               "# command: `"$ArmGxx`" " + ($arguments -join ' '))
    $output = & $ArmGxx @arguments 2>&1
    $lines += "# exit: $LASTEXITCODE"
    if ($output) { $lines += ($output | ForEach-Object { "  $_" }) }
    if ($LASTEXITCODE -eq 0) {
        $bin = Split-Path $ArmGxx -Parent
        $lines += (& (Join-Path $bin 'arm-none-eabi-size.exe') $object 2>&1 | ForEach-Object { "  $_" })
        $symbols = & (Join-Path $bin 'arm-none-eabi-nm.exe') -u $object 2>&1
        $lines += '# undefined symbols'
        $lines += ($symbols | ForEach-Object { "  $_" })
        $heap = $symbols | Select-String -Pattern 'malloc|free|operator new|__cxa_throw|_Unwind'
        $lines += '# heap/exception symbols: ' + $(if ($heap) { ($heap | ForEach-Object { $_.Line }) -join ', ' } else { 'none' })
    }
    $lines += '# RESULT: ' + $(if ($LASTEXITCODE -eq 0) { 'PASS' } else { 'FAIL' })
    $lines | Set-Content -Encoding UTF8 $log
    Write-Host "ARM compile (m3): $(if ($LASTEXITCODE -eq 0) { 'PASS' } else { 'FAIL' })"
    $script:results += [pscustomobject]@{ Test = 'arm_compile_m3'; Result = $(if ($LASTEXITCODE -eq 0) { 'PASS' } else { 'FAIL' }); Log = $log }
}

Invoke-HostTest
Invoke-ArmCompile

$results | Format-Table -AutoSize | Out-String | Write-Host
$results | ConvertTo-Json -Depth 4 | Set-Content -Encoding UTF8 (Join-Path $OutDir 'm3_tests_summary.json')
$failed = @($results | Where-Object { $_.Result -ne 'PASS' })
if ($failed.Count -gt 0) { exit 1 }
exit 0

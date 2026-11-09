<#
    Reproduces the official-dataset evidence for T191-B-UPSTREAM:

      1. builds the pristine upstream KF-GINS from a locked checkout,
      2. runs the full shipped dataset (golden reference outputs),
      3. builds and runs the dual-engine comparison on a bounded window.

    The checkout must already exist at the locked commit (the script does not
    download sources).  See build/validation/kf_gins_golden/DATASET_EVIDENCE.md.

    Example:
        git clone https://github.com/i2Nav-WHU/KF-GINS.git C:\tmp\KF-GINS
        git -C C:\tmp\KF-GINS checkout 8291a93e49de513fe9d21f819500d39082ded611
        pwsh middleware/model/ins/kf_gins/tools/run_dataset_evidence.ps1 -Upstream C:\tmp\KF-GINS
#>

[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$Upstream,
    [string]$Work = (Join-Path $env:TEMP 'kfgins_evidence'),
    [string]$CMake = 'D:\STM32CubeCLT\CMake\bin\cmake.exe',
    [string]$NinjaDir = 'D:\STM32CubeCLT\Ninja\bin',
    [string]$Gcc = 'D:\STM32CubeCLT\mingw64\bin\gcc.exe',
    [string]$Gxx = 'D:\STM32CubeCLT\mingw64\bin\g++.exe',
    [string]$OutDir = ''
)

$ErrorActionPreference = 'Stop'

$scriptDir = $PSScriptRoot
$repoRoot = Get-Item -LiteralPath $PSScriptRoot
while (-not (Test-Path -LiteralPath (Join-Path $repoRoot.FullName 'SConstruct'))) {
  if ($null -eq $repoRoot.Parent) { throw 'Flight root not found' }
  $repoRoot = $repoRoot.Parent
}
$repoRoot = $repoRoot.FullName
$pkg = Join-Path $repoRoot 'middleware\model\ins\kf_gins'
if ([string]::IsNullOrWhiteSpace($OutDir)) {
    $OutDir = Join-Path $repoRoot 'build\validation\kf_gins_golden'
}
New-Item -ItemType Directory -Force -Path $OutDir, $Work | Out-Null

$env:PATH = "$NinjaDir;$(Split-Path $CMake -Parent);$(Split-Path $Gxx -Parent);" + $env:PATH

if (-not (Test-Path (Join-Path $Upstream 'src\kf-gins\gi_engine.cpp'))) {
    throw "no locked KF-GINS checkout at $Upstream"
}

$commit = (& git -C $Upstream rev-parse HEAD 2>$null)
if ($commit -ne '8291a93e49de513fe9d21f819500d39082ded611') {
    throw "checkout is not at the locked commit (found '$commit')"
}

# yaml-cpp 0.7.0 does not include <cstdint> itself; GCC 16 rejects it.
$emitter = Join-Path $Upstream 'ThirdParty\yaml-cpp-0.7.0\src\emitterutils.cpp'
$text = [IO.File]::ReadAllText($emitter)
if ($text -notmatch '#include <cstdint>') {
    [IO.File]::WriteAllText($emitter, $text.Replace('#include "emitterutils.h"', "#include `"emitterutils.h`"`n#include <cstdint>"))
    Write-Host "patched yaml-cpp emitterutils.cpp (one include line)"
}

$build = Join-Path $Work 'pristine_build'
& $CMake -S $Upstream -B $build -G Ninja '-DCMAKE_BUILD_TYPE=Release' `
    "-DCMAKE_C_COMPILER=$($Gcc -replace '\\','/')" "-DCMAKE_CXX_COMPILER=$($Gxx -replace '\\','/')" `
    '-DABSL_PROPAGATE_CXX_STD=ON' '-DCMAKE_POLICY_VERSION_MINIMUM=3.5' | Out-Null
& $CMake --build $build -j 8 | Out-Null

function New-DatasetConfig([string]$name, [string]$outDir, [string]$endTime) {
    $lines = Get-Content (Join-Path $Upstream 'dataset\kf-gins.yaml')
    $config = foreach ($line in $lines) {
        if ($line -match '^outputpath:') { 'outputpath: "' + ($outDir -replace '\\','/') + '"' }
        elseif ($line -match '^endtime:' -and $endTime) { "endtime: $endTime" }
        else { $line }
    }
    $path = Join-Path $Upstream "dataset\$name"
    Set-Content -Encoding UTF8 -Path $path -Value $config
    return $path
}

$exe = Join-Path $Upstream 'bin\KF-GINS.exe'
$fullOut = Join-Path $Work 'golden_full'
New-Item -ItemType Directory -Force -Path $fullOut | Out-Null
$fullConfig = New-DatasetConfig 'kf-gins-full.yaml' $fullOut ''
Push-Location $Upstream
$sw = [Diagnostics.Stopwatch]::StartNew()
& $exe $fullConfig | Out-Null
$sw.Stop()
Write-Host ("full dataset: {0:N1} s wall" -f $sw.Elapsed.TotalSeconds)
Pop-Location

$boundedOut = Join-Path $Work 'golden_120s'
New-Item -ItemType Directory -Force -Path $boundedOut | Out-Null
$boundedConfig = New-DatasetConfig 'kf-gins-120s.yaml' $boundedOut '456420'
Push-Location $Upstream
& $exe $boundedConfig | Out-Null
Pop-Location

$tool = Join-Path $Work 'dataset_dual_compare.exe'
$sources = @(
    (Join-Path $pkg 'tools\dataset_dual_compare.cpp'),
    (Join-Path $Upstream 'src\kf-gins\gi_engine.cpp'),
    (Join-Path $Upstream 'src\kf-gins\insmech.cpp'),
    (Join-Path $pkg 'trimmed_core\src\quad_core.cpp'),
    (Join-Path $pkg 'trimmed_core\src\quad_models.cpp'))
$flags = @('-std=c++14', '-O2', '-static', '-D_USE_MATH_DEFINES',
           '-I', (Join-Path $Upstream 'src'),
           '-I', (Join-Path $Upstream 'ThirdParty\eigen-3.3.9'),
           '-I', (Join-Path $pkg 'trimmed_core\include'))
& $Gxx @flags @sources '-o' $tool
if ($LASTEXITCODE -ne 0) { throw "dual compare build failed" }

Push-Location $Upstream
$dual = & $tool $boundedConfig 2>&1
$dualExit = $LASTEXITCODE
Pop-Location

$lines = @('# dual-engine comparison (Project Host Reference vs Pristine)',
           "# checkout: $Upstream @ $commit",
           "# config: $boundedConfig",
           "# exit: $dualExit", '')
$lines += ($dual | ForEach-Object { "$_" })
$lines | Set-Content -Encoding UTF8 (Join-Path $OutDir 'dual_compare.log')

$summary = [pscustomobject]@{
    commit = $commit
    full_run_wall_seconds = [math]::Round($sw.Elapsed.TotalSeconds, 2)
    dual_compare = ($dual -join ' ')
    generated_at = (Get-Date).ToString('o')
}
$summary | ConvertTo-Json | Set-Content -Encoding UTF8 (Join-Path $OutDir 'run_summary.json')
Write-Host ($dual -join "`n")
exit $dualExit

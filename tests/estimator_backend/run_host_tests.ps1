<#
    Builds and runs the KF-GINS_QUAD Host test suite and records the exact
    commands and outputs under build/validation/kf_gins_upstream/.

    The pristine-upstream comparison needs a checkout of the locked commit
    (8291a93e49de513fe9d21f819500d39082ded611) because Eigen/ThirdParty are not
    vendored into this repository. An explicit -Upstream requires a clean
    checkout of that commit. Without -Upstream the comparison is optional.

    Example:
        git clone https://github.com/i2Nav-WHU/KF-GINS.git C:\tmp\KF-GINS
        git -C C:\tmp\KF-GINS checkout 8291a93e49de513fe9d21f819500d39082ded611
        pwsh tests/estimator_backend/run_host_tests.ps1 -Upstream C:\tmp\KF-GINS
#>

[CmdletBinding()]
param(
    [string]$Upstream = (Join-Path $env:TEMP 'kfgins_probe\src'),
    [string]$Gxx = 'D:\STM32CubeCLT\mingw64\bin\g++.exe',
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
$pluginDir = Join-Path $repoRoot 'middleware\model\ins'
$pkgDir = Join-Path $pluginDir 'kf_gins'
$upstreamLock = Get-Content -LiteralPath (Join-Path $pkgDir 'UPSTREAM_LOCK.json') -Raw | ConvertFrom-Json
$upstreamSources = Join-Path $Upstream 'src'
$upstreamEigen = Join-Path $Upstream 'ThirdParty\eigen-3.3.9'
$hasUpstream = (Test-Path -LiteralPath (Join-Path $upstreamSources 'kf-gins\gi_engine.cpp') -PathType Leaf) -and
               (Test-Path -LiteralPath $upstreamEigen -PathType Container)
if ($hasUpstream) {
    $actualCommit = & git -C $Upstream rev-parse --verify HEAD 2>$null
    if ($LASTEXITCODE -ne 0 -or $actualCommit -cne $upstreamLock.commit) {
        throw "KF-GINS checkout at $Upstream must match the locked commit $($upstreamLock.commit)."
    }
    $changes = & git -C $Upstream status --porcelain --untracked-files=normal
    if ($LASTEXITCODE -ne 0 -or $changes) {
        throw "KF-GINS checkout at $Upstream must be clean."
    }
} elseif ($PSBoundParameters.ContainsKey('Upstream')) {
    throw "KF-GINS checkout at $Upstream is incomplete. Required commit: $($upstreamLock.commit)."
}
$trimInclude = Join-Path $pkgDir 'trimmed_core\include'
$trimSources = @((Join-Path $pkgDir 'trimmed_core\src\quad_core.cpp'), (Join-Path $pkgDir 'trimmed_core\src\quad_models.cpp'))
$pluginSources = @((Join-Path $pkgDir 'backend_adapter/kf_gins_adapter.cpp'), (Join-Path $pkgDir 'backend_adapter/kf_gins_quad_port.cpp'))

if ([string]::IsNullOrWhiteSpace($OutDir)) {
    $OutDir = Join-Path $repoRoot 'build\validation\kf_gins_upstream'
}
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$work = Join-Path $OutDir 'obj'
New-Item -ItemType Directory -Force -Path $work | Out-Null

if (-not (Test-Path $Gxx)) {
    throw "Host compiler not found: $Gxx (pass -Gxx)"
}

$common = @('-std=c++14', '-O2', '-Wall', '-Wextra', '-static', '-static-libgcc', '-static-libstdc++')
$results = @()

function Invoke-Test {
    param(
        [string]$Name,
        [string[]]$Sources,
        [string[]]$Includes,
        [string[]]$ExtraFlags = @()
    )

    $exe = Join-Path $work ($Name + '.exe')
    $log = Join-Path $OutDir ($Name + '.log')
    $compile = @($common + $ExtraFlags)
    foreach ($inc in $Includes) { $compile += @('-I', $inc) }
    $compile += $Sources
    $compile += @('-o', $exe)

    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $output = & $Gxx @compile 2>&1
    $buildExit = $LASTEXITCODE
    $sw.Stop()

    $lines = @()
    $lines += "# test: $Name"
    $lines += "# compiler: $Gxx"
    $lines += "# command: `"$Gxx`" " + ($compile -join ' ')
    $lines += "# build exit: $buildExit"
    if ($output) { $lines += "# build output:"; $lines += ($output | ForEach-Object { "  $_" }) }

    if ($buildExit -ne 0) {
        $lines += "# RESULT: BUILD_FAILED"
        $lines | Set-Content -Encoding UTF8 $log
        $script:results += [pscustomobject]@{ Test = $Name; Result = 'BUILD_FAILED'; Log = $log }
        return
    }

    $runOutput = & $exe 2>&1
    $runExit = $LASTEXITCODE
    $lines += "# run exit: $runExit"
    $lines += "# output:"
    $lines += ($runOutput | ForEach-Object { "  $_" })
    $lines += "# RESULT: " + $(if ($runExit -eq 0) { 'PASS' } else { 'FAIL' })
    $lines | Set-Content -Encoding UTF8 $log

    Write-Host ($runOutput | Out-String)
    $script:results += [pscustomobject]@{
        Test = $Name
        Result = $(if ($runExit -eq 0) { 'PASS' } else { 'FAIL' })
        Log = $log
        Seconds = [math]::Round($sw.Elapsed.TotalSeconds, 2)
    }
}

Write-Host "KF-GINS_QUAD host tests"
Write-Host "  repo    : $repoRoot"
Write-Host "  compiler: $Gxx"
Write-Host "  logs    : $OutDir"
Write-Host ""

$testDir = $scriptDir
$testSupport = Join-Path $testDir 'test_support.hpp'

Invoke-Test -Name 'test_jacobian' -Includes @($trimInclude, $testDir) `
    -Sources (@((Join-Path $testDir 'test_jacobian.cpp')) + $trimSources) `
    -ExtraFlags @('-fno-exceptions', '-fno-rtti')

Invoke-Test -Name 'test_core' -Includes @($trimInclude, $testDir) `
    -Sources (@((Join-Path $testDir 'test_core.cpp')) + $trimSources) `
    -ExtraFlags @('-fno-exceptions', '-fno-rtti')

Invoke-Test -Name 'test_port' -Includes @($trimInclude, $pkgDir, $testDir) `
    -Sources (@((Join-Path $testDir 'test_port.cpp')) + $pluginSources + $trimSources) `
    -ExtraFlags @('-fno-exceptions', '-fno-rtti')

Invoke-Test -Name 'test_contract_coexistence' `
    -Includes @((Join-Path $repoRoot 'middleware/contracts'), $pkgDir, $trimInclude) `
    -Sources (@((Join-Path $testDir 'test_contract_coexistence.cpp')) + $pluginSources + $trimSources) `
    -ExtraFlags @('-fno-exceptions', '-fno-rtti')

if ($hasUpstream) {
    Write-Host "KF-GINS comparison commit: $actualCommit"
    Invoke-Test -Name 'test_upstream_compare' `
        -Includes @($trimInclude, $testDir, $upstreamEigen, $upstreamSources) `
        -Sources (@((Join-Path $testDir 'test_upstream_compare.cpp'),
                    (Join-Path $upstreamSources 'kf-gins\gi_engine.cpp'),
                    (Join-Path $upstreamSources 'kf-gins\insmech.cpp')) + $trimSources) `
        -ExtraFlags @('-D_USE_MATH_DEFINES')
} else {
    Write-Host "SKIPPED test_upstream_compare: no locked upstream checkout at $Upstream"
    $log = Join-Path $OutDir 'test_upstream_compare.log'
    @('# test: test_upstream_compare',
      "# RESULT: SKIPPED - locked upstream checkout not found at $Upstream",
      '# Reproduce: git clone https://github.com/i2Nav-WHU/KF-GINS.git <dir>;',
      '#            git -C <dir> checkout 8291a93e49de513fe9d21f819500d39082ded611') |
        Set-Content -Encoding UTF8 $log
    $results += [pscustomobject]@{ Test = 'test_upstream_compare'; Result = 'SKIPPED'; Log = $log; Seconds = 0 }
}

$results | Format-Table -AutoSize | Out-String | Write-Host
$results | ConvertTo-Json -Depth 4 | Set-Content -Encoding UTF8 (Join-Path $OutDir 'host_tests_summary.json')

$failed = @($results | Where-Object { $_.Result -in @('FAIL', 'BUILD_FAILED') })
if ($failed.Count -gt 0) { exit 1 }
exit 0

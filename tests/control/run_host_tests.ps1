<#
    Builds and runs the FMAT-0012 M6 host tests.

    test_m6        drives mpc.c against the generated OSQP workspace so the QP the
                   test inspects is the one the controller assembles.
    test_m6_guard  drives controller_validate_input() with a complete context and
                   then breaks one field at a time.

    Both link the real controller translation units; nothing is re-implemented.
#>
[CmdletBinding()]
param(
    [string]$Gcc = 'D:\STM32CubeCLT\mingw64\bin\gcc.exe',
    [string]$OutDir = ''
)

$ErrorActionPreference = 'Stop'
if (-not (Test-Path -LiteralPath $Gcc)) {
    $candidate = 'C:\ProgramData\MATLAB\SupportPackages\R2026b\3P.instrset\mingw_w64.instrset\bin\gcc.exe'
    if (Test-Path -LiteralPath $candidate) { $Gcc = $candidate }
}

$layoutRoot = Get-Item -LiteralPath $PSScriptRoot
while (-not (Test-Path -LiteralPath (Join-Path $layoutRoot.FullName 'SConstruct'))) {
    if ($null -eq $layoutRoot.Parent) { throw 'Flight root not found' }
    $layoutRoot = $layoutRoot.Parent
}
$repoRoot = $layoutRoot.FullName
if ([string]::IsNullOrWhiteSpace($OutDir)) { $OutDir = Join-Path $repoRoot 'build\validation\m6' }
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$work = Join-Path $OutDir 'obj'
New-Item -ItemType Directory -Force -Path $work | Out-Null

$model = Join-Path $repoRoot 'middleware\model\control'
$inner = Join-Path $model 'inner_loop\quad_x4'
$outer = Join-Path $model 'outer_loop\quad_x4'
$mpc = Join-Path $outer 'mpc_solver'
$osqp = Join-Path $mpc 'osqp\generated'

$includes = @(
    '-I', (Join-Path $repoRoot 'middleware\contracts'),
    '-I', (Join-Path $repoRoot 'middleware\actuator'),
    '-I', (Join-Path $repoRoot 'middleware'),
    '-I', $model,
    '-I', $inner,
    '-I', $outer,
    '-I', $mpc,
    '-I', $osqp,
    '-I', (Join-Path $osqp 'inc\public'),
    '-I', (Join-Path $osqp 'inc\private')
)

$common = @(
    (Join-Path $outer 'position_tracking\control_reference.c')
    (Join-Path $outer 'position_tracking\position_control.c')
    (Join-Path $inner 'attitude_rate_control\controller.c')
    (Join-Path $inner 'attitude_rate_control\attitude_math.c')
    (Join-Path $inner 'control_allocation\control_allocation.c')
)
$osqpSources = @(Get-ChildItem (Join-Path $osqp 'src\*.c') | ForEach-Object { $_.FullName }) +
               @((Join-Path $osqp 'quad_workspace.c'))

function Invoke-HostTest {
    param([string]$Name, [string[]]$Sources, [string]$Pattern)
    $exe = Join-Path $work "$Name.exe"
    $log = Join-Path $OutDir "$Name.log"
    $arguments = @('-std=c11', '-O2', '-Wall', '-Wextra', '-static') + $includes + $Sources +
                 @('-lm', '-o', $exe)
    $lines = @("# test: $Name", "# compiler: $Gcc", "# command: `"$Gcc`" " + ($arguments -join ' '))
    $build = & $Gcc @arguments 2>&1
    $lines += "# build exit: $LASTEXITCODE"
    if ($build) {
        $lines += ($build | Where-Object { $_ -match 'error' } | Select-Object -First 20 | ForEach-Object { "  $_" })
    }
    $status = 'BUILD_FAILED'
    $checks = 0
    $failures = 0
    if ($LASTEXITCODE -eq 0) {
        $run = & $exe 2>&1
        $lines += "# run exit: $LASTEXITCODE"
        $lines += ($run | ForEach-Object { "  $_" })
        $status = if ($LASTEXITCODE -eq 0) { 'PASS' } else { 'FAIL' }
        Write-Host ($run | Out-String)
        $match = ($run | Select-String -Pattern $Pattern | Select-Object -Last 1)
        if ($match) {
            $checks = [int]$match.Matches[0].Groups[1].Value
            $failures = [int]$match.Matches[0].Groups[2].Value
        }
    }
    $lines += "# RESULT: $status"
    $lines | Set-Content -Encoding UTF8 $log
    return [pscustomobject]@{ Name = $Name; Status = $status; Checks = $checks; Failures = $failures; Log = $log }
}

$assembly = Invoke-HostTest -Name 'test_m6' -Pattern 'M6 checks=(\d+) failures=(\d+)' `
    -Sources (@((Join-Path $PSScriptRoot 'test_m6_main.c')) + $common + @((Join-Path $mpc 'mpc.c')) + $osqpSources)
$guard = Invoke-HostTest -Name 'test_m6_guard' -Pattern 'M6 guard checks=(\d+) failures=(\d+)' `
    -Sources (@((Join-Path $PSScriptRoot 'test_m6_guard_main.c')) + $common + @((Join-Path $mpc 'mpc.c')) + $osqpSources)

$result = if ($assembly.Status -eq 'PASS' -and $guard.Status -eq 'PASS') { 'PASS' }
          elseif ($assembly.Status -eq 'BUILD_FAILED' -or $guard.Status -eq 'BUILD_FAILED') { 'BUILD_FAILED' }
          else { 'FAIL' }

$report = [ordered]@{
    scope = 'M6_OUTER_LOOP_MODEL_PATH'
    status = $result
    compiler = $Gcc
    checks = $assembly.Checks + $guard.Checks
    failures = $assembly.Failures + $guard.Failures
    qp_assembly_in_model = if ($assembly.Status -eq 'PASS') { 'PASS' } else { 'FAIL' }
    strict_validation_in_model = if ($assembly.Status -eq 'PASS') { 'PASS' } else { 'FAIL' }
    complete_M6_time_context_guard = if ($guard.Status -eq 'PASS') { 'PASS' } else { 'FAIL' }
    m6_assembly_checks = $assembly.Checks
    m6_guard_checks = $guard.Checks
    log = ($assembly.Log + ' ; ' + $guard.Log)
}
$report | ConvertTo-Json -Depth 5 | Set-Content -Encoding UTF8 (Join-Path $OutDir 'm6_host_summary.json')
Write-Host ("M6 host: " + $result + " (assembly " + $assembly.Status + " " + $assembly.Checks +
            " checks, guard " + $guard.Status + " " + $guard.Checks + " checks)")
if ($result -ne 'PASS') { exit 1 }
exit 0

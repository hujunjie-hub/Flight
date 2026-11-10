<#
    Builds and runs the FMAT-0012 M6 outer-loop host test.
    Links the real MPC path (mpc.c + control_reference.c + the controller backend)
    against the generated OSQP 1.0.0 workspace, so the QP that the test inspects is
    the one the controller assembles, not a hand-built copy.
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

$sources = @(
    (Join-Path $PSScriptRoot 'test_m6_main.c')
    (Join-Path $outer 'position_tracking\control_reference.c')
    (Join-Path $outer 'position_tracking\position_control.c')
    (Join-Path $mpc 'mpc.c')
    (Join-Path $inner 'attitude_rate_control\controller.c')
    (Join-Path $inner 'attitude_rate_control\attitude_math.c')
    (Join-Path $inner 'control_allocation\control_allocation.c')
) + @(Get-ChildItem (Join-Path $osqp 'src\*.c') | ForEach-Object { $_.FullName }) +
    @((Join-Path $osqp 'quad_workspace.c'))

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

$exe = Join-Path $work 'test_m6.exe'
$log = Join-Path $OutDir 'test_m6.log'
$arguments = @('-std=c11', '-O2', '-Wall', '-Wextra', '-static') + $includes + $sources +
             @('-lm', '-o', $exe)
$lines = @('# test: test_m6', "# compiler: $Gcc", "# command: `"$Gcc`" " + ($arguments -join ' '))
$build = & $Gcc @arguments 2>&1
$lines += "# build exit: $LASTEXITCODE"
if ($build) { $lines += ($build | Where-Object { $_ -match 'error|warning: .*test_m6' } | ForEach-Object { "  $_" }) }

$result = 'BUILD_FAILED'
$summary = $null
if ($LASTEXITCODE -eq 0) {
    $run = & $exe 2>&1
    $lines += "# run exit: $LASTEXITCODE"
    $lines += ($run | ForEach-Object { "  $_" })
    $result = if ($LASTEXITCODE -eq 0) { 'PASS' } else { 'FAIL' }
    Write-Host ($run | Out-String)
    $counts = ($run | Select-String -Pattern 'M6 checks=(\d+) failures=(\d+)' | Select-Object -Last 1)
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
    scope = 'M6_OUTER_LOOP_MODEL_PATH'
    status = $result
    compiler = $Gcc
    checks = if ($summary) { $summary.checks } else { 0 }
    failures = if ($summary) { $summary.failures } else { 0 }
    cases = @(
        'M6-01 a valid reference solves inside the model path'
        'M6-02 the assembled QP keeps 15 variables / 190 constraints / 45 and 702 nonzeros'
        'M6-03 the accepted candidate satisfies the model input bounds and thrust ball'
        'M6-04 a control instant beyond the preview horizon is stale'
        'M6-05 non-finite navigation is rejected'
        'M6-06 contradictory model bounds are rejected'
        'M6-07 an envelope outside the model domain is rejected'
    )
    qp_assembly_in_model = if ($result -eq 'PASS') { 'PASS' } else { 'FAIL' }
    strict_validation_in_model = if ($result -eq 'PASS') { 'PASS' } else { 'FAIL' }
    log = $log
}
$report | ConvertTo-Json -Depth 5 | Set-Content -Encoding UTF8 (Join-Path $OutDir 'm6_host_summary.json')
Write-Host ("M6 host: " + $result + " (" + $report.checks + " checks, " + $report.failures + " failures)")
if ($result -ne 'PASS') { exit 1 }
exit 0

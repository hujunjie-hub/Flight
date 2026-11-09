<#
    Builds and runs the HIL sim-link Host test (MAVLink v2 codec, HIL decoders,
    public-sample builders, thrust normalization) and checks that the committed
    golden frames still match the independent generator.

    The firmware task itself is only compiled for ARM by scons; this script
    verifies the wire part that can be exercised without hardware.

    Usage:
        pwsh tests/simulation/run_host_tests.ps1
        pwsh tests/simulation/run_host_tests.ps1 -Gcc <path> -OutDir <dir>
#>

[CmdletBinding()]
param(
    [string]$Gcc = 'D:\STM32CubeCLT\mingw64\bin\gcc.exe',
    [string]$Python = 'python',
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
if ([string]::IsNullOrWhiteSpace($OutDir)) {
    $OutDir = Join-Path $repoRoot 'build\validation\sim'
}
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$work = Join-Path $OutDir 'obj'
New-Item -ItemType Directory -Force -Path $work | Out-Null

$includes = @(
    'middleware/contracts',
    'middleware/sensor',
    'sensor_device',
    'middleware/protocol',
    'applications/tasks',
    'tests/simulation/golden'
) | ForEach-Object { Join-Path $repoRoot $_ }

$sources = @(
    'tests/simulation/test_hil_link.c',
    'middleware/protocol/mavlink/mavlink_frame.c',
    'middleware/protocol/mavlink/mavlink_hil.c',
    'middleware/sensor/sample_normalization/sensor_sample.c',
    'middleware/sensor/calibration/sensor_calibration.c',
    'middleware/sensor/measurement_filter/sensor_filter.c'
) | ForEach-Object { Join-Path $repoRoot $_ }

$log = Join-Path $OutDir 'test_hil_link.log'
"== golden regeneration check ==" | Set-Content -LiteralPath $log -Encoding utf8
& $Python (Join-Path $repoRoot 'tests/simulation/golden/generate_golden.py') --check 2>&1 |
    Tee-Object -FilePath $log -Append
if ($LASTEXITCODE -ne 0) {
    throw 'golden frames are stale: re-run tests/simulation/golden/generate_golden.py'
}

"== bridge end-to-end (no hardware) ==" | Add-Content -LiteralPath $log -Encoding utf8
& $Python (Join-Path $repoRoot 'tests/simulation/test_bridge.py') 2>&1 |
    Tee-Object -FilePath $log -Append
if ($LASTEXITCODE -ne 0) {
    throw "bridge test failed, see $log"
}

$arguments = @('-std=c11', '-O2', '-Wall', '-Wextra', '-Werror', '-Wconversion',
    '-Wno-sign-conversion') + ($includes | ForEach-Object { "-I$_" }) +
    $sources + @('-lm', '-o', (Join-Path $work 'test_hil_link.exe'))

"== build ==" | Add-Content -LiteralPath $log -Encoding utf8
"$Gcc $($arguments -join ' ')" | Add-Content -LiteralPath $log -Encoding utf8
& $Gcc @arguments 2>&1 | Tee-Object -FilePath $log -Append
if ($LASTEXITCODE -ne 0) {
    throw "host build failed, see $log"
}

"== run ==" | Add-Content -LiteralPath $log -Encoding utf8
& (Join-Path $work 'test_hil_link.exe') 2>&1 | Tee-Object -FilePath $log -Append
if ($LASTEXITCODE -ne 0) {
    throw "host test failed, see $log"
}

"== RTCM reassembly ==" | Add-Content -LiteralPath $log -Encoding utf8
$rtcmExe = Join-Path $work 'test_rtcm.exe'
& $Gcc -std=c11 -O2 -Wall -Wextra -Werror "-I$repoRoot/middleware/protocol" `
    "$repoRoot/tests/simulation/test_rtcm.c" "$repoRoot/middleware/protocol/mavlink/mavlink_rtcm.c" `
    -o $rtcmExe 2>&1 | Tee-Object -FilePath $log -Append
if ($LASTEXITCODE -ne 0) { throw "RTCM host build failed, see $log" }
& $rtcmExe 2>&1 | Tee-Object -FilePath $log -Append
if ($LASTEXITCODE -ne 0) { throw "RTCM host test failed, see $log" }
"host test log: $log"

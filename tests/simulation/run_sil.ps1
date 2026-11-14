<#
    Builds and runs the SIL flight-logic scenario (tests/simulation/sil_flight.c).

    The scenario links the production runtime and owners against the same Host
    stubs the runtime integration test uses, and additionally defines
    FLIGHT_HOST_SIL so the composition runs with SIM_CONTROLLED_STOP (2) and a
    recording actuator driver instead of DShot.

    Usage:
        pwsh tests/simulation/run_sil.ps1
        pwsh tests/simulation/run_sil.ps1 -GccDir D:\STM32CubeCLT\mingw64\bin
#>

[CmdletBinding()]
param(
    [string]$GccDir = 'D:\STM32CubeCLT\mingw64\bin',
    [string]$OutDir = '',
    [ValidateRange(0, 65535)][int]$FmtHilPort = 0,
    [string]$FmtSimExe = '',
    [ValidateRange(0, 15)][int]$FmtGraphicsAdapter = 1,
    [ValidateRange(0.1, 0.95)][double]$FmtHover = 0.62,
    [ValidateSet('Land', 'Return', 'Mission')][string]$FmtScenario = 'Land',
    [ValidateRange(350, 1000)][int]$FmtStateTimeoutMs = 350,
    [ValidateRange(0, 65535)][int]$GroundPort = 0,
    [switch]$SkipAbiLock,
    [switch]$BuildOnly
)

$ErrorActionPreference = 'Stop'
if ($FmtSimExe -and -not $FmtHilPort) { throw '-FmtSimExe requires -FmtHilPort' }
if ($GroundPort -and $GroundPort -eq $FmtHilPort) {
    throw '-GroundPort and -FmtHilPort must use different ports'
}

$scriptDir = $PSScriptRoot
$simPath = $null
if ($FmtSimExe) {
    $simPath = (Resolve-Path -LiteralPath $FmtSimExe).Path
    $simDirectory = Get-Item -LiteralPath (Split-Path $simPath)
    if ($simDirectory.Name -ne 'Win64' -or $simDirectory.Parent.Name -ne 'Binaries') {
        throw 'Pass the FMT-SIM executable under Binaries/Win64'
    }
    $settingsPath = Join-Path $simDirectory.Parent.Parent.FullName 'Content/Settings/settings.jason'
    if (-not (Test-Path -LiteralPath $settingsPath -PathType Leaf)) {
        throw "FMT-SIM settings missing: $settingsPath"
    }
    $settings = Get-Content -LiteralPath $settingsPath -Raw | ConvertFrom-Json
    $hilVehicles = @($settings.Vehicles | Where-Object {
        $_.SimMode -eq 'HIL' -and $_.MavproxyFCS.Interface -eq 'UDP' -and
            $_.MavproxyFCS.LocalPort -eq $FmtHilPort
    })
    if ($hilVehicles.Count -ne 1) {
        throw "Configure one FMT-SIM HIL vehicle on UDP port $FmtHilPort in $settingsPath"
    }
}
$layoutRoot = Get-Item -LiteralPath $PSScriptRoot
while (-not (Test-Path -LiteralPath (Join-Path $layoutRoot.FullName 'SConstruct'))) {
    if ($null -eq $layoutRoot.Parent) { throw 'Flight root not found' }
    $layoutRoot = $layoutRoot.Parent
}
$repoRoot = $layoutRoot.FullName
if ([string]::IsNullOrWhiteSpace($OutDir)) { $OutDir = Join-Path $repoRoot 'build\validation\sim\sil' }
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$OutDir = (Resolve-Path -LiteralPath $OutDir).Path
$log = Join-Path $OutDir 'sil_flight.log'

$includes = @(
    'tests/runtime/stubs', 'applications/tasks', 'applications/runtime',
    'middleware/contracts', 'middleware/communication', 'middleware/protocol', 'middleware/param',
    'middleware/sensor', 'middleware/model/ins', 'middleware/model/fms', 'middleware/model/fms/quad_x4', 'middleware/guidance',
    'middleware/model/control', 'middleware/model/control/inner_loop/quad_x4',
    'middleware/actuator', 'sensor_device', 'sensor_device/time_base',
    'sensor_device/actuator', 'libraries/HAL_Drivers/drivers',
    'sensor_device/imu', 'sensor_device/mag', 'sensor_device/barometer',
    'sensor_device/pmu', 'sensor_device/gnss',
    'middleware/model/ins/kf_gins', 'middleware/model/ins/kf_gins/trimmed_core/include',
    'middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated', 'middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated/inc/public',
    'middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated/inc/private', 'board'
) | ForEach-Object { Join-Path $repoRoot $_ }
if ($GroundPort) {
    & python (Join-Path $repoRoot 'tools/generate_ground_config.py') --check
    if ($LASTEXITCODE -ne 0) { throw 'Flight ground parameter catalog is stale' }
    $includes += Join-Path $repoRoot 'middleware/protocol/mavlink/parameters'
}

$sources = @()
foreach ($pattern in @('middleware/sensor/*/*.c', 'middleware/model/fms/*.c', 'middleware/model/fms/quad_x4/*/*.c',
        'middleware/guidance/*/*.c', 'middleware/model/control/*.c', 'middleware/model/ins/*.c',
        'middleware/model/control/inner_loop/quad_x4/*/*.c', 'middleware/model/control/outer_loop/quad_x4/*/*.c',
        'middleware/model/ins/kf_gins/navigation_estimator/*.cpp', 'middleware/model/ins/kf_gins/observation_replay/*.cpp',
        'middleware/model/ins/kf_gins/trimmed_core/src/*.cpp',
        'middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated/src/*.c')) {
    $sources += Get-ChildItem -Path (Join-Path $repoRoot $pattern) -File | Select-Object -ExpandProperty FullName
}
$sources += @(
    'middleware/model/fms/geofence_geometry/geofence_geometry.c',
    'middleware/model/ins/kf_gins/backend_adapter/kf_gins_adapter.cpp',
    'middleware/model/plant/plant_interface.c',
    'middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generated/quad_workspace.c',
    'middleware/protocol/mavlink/mavlink_frame.c',
    'middleware/protocol/mavlink/mavlink_action.c',
    'middleware/protocol/mavlink/mavlink_hil.c',
    'middleware/param/parameter_store.c',
    'middleware/communication/sim/fmt_sim_host.c',
    'middleware/protocol/unicore_ascii/unicore_ascii.c',
    'middleware/actuator/device_adapter/actuator_device.c',
    'middleware/actuator/output_guard/actuator_guard.c',
    'libraries/HAL_Drivers/drivers/drv_wdt.c',
    'sensor_device/gnss/um982.c',
    'sensor_device/imu/adis16505.c',
    'sensor_device/barometer/bmp585.c',
    'sensor_device/telemetry/elrs/elrs.c',
    'sensor_device/telemetry/data_radio/data_radio.c',
    'tests/simulation/sil_flight.c'
) | ForEach-Object { Join-Path $repoRoot $_ }
if ($GroundPort) {
    $sources += Join-Path $repoRoot 'tests/simulation/ground_station.c'
    $sources += Join-Path $repoRoot 'tests/simulation/ground_log.c'
}

$objects = @()
"== compile ==" | Set-Content -LiteralPath $log -Encoding utf8
foreach ($source in $sources) {
    $relative = $source.Substring($repoRoot.Length + 1)
    $obj = Join-Path $OutDir (($relative -replace '[\\/]', '_') + '.o')
    $isCpp = $source.EndsWith('.cpp')
    $compiler = if ($isCpp) { Join-Path $GccDir 'g++.exe' } else { Join-Path $GccDir 'gcc.exe' }
    $flags = @(
        $(if ($isCpp) { '-std=c++14' } else { '-std=c99' }), '-O2', '-g', '-Wall', '-Wextra',
        '-Wno-unused-function', '-fno-short-enums', '-DFLIGHT_HOST_TEST', '-DFLIGHT_HOST_SIL'
    )
    if ($GroundPort) { $flags += '-DFLIGHT_GROUND_STATION' }
    if ($SkipAbiLock) { $flags += '-DFLIGHT_SKIP_ABI_LOCK' }
    if ($relative -notmatch 'osqp') { $flags += '-Werror' }
    if ((Split-Path $source -Leaf) -in @('adis16505.c', 'bmp585.c')) {
        $flags += '-DFLIGHT_DRIVER_CODEC_TEST'
    }
    if ($isCpp) { $flags += @('-fno-exceptions', '-fno-rtti', '-fno-threadsafe-statics') }
    $flags += $includes | ForEach-Object { "-I$_" }
    & $compiler @flags -c $source -o $obj 2>&1 | Tee-Object -FilePath $log -Append
    if ($LASTEXITCODE -ne 0) { throw "compile failed: $relative" }
    $objects += $obj
}

"== link ==" | Add-Content -LiteralPath $log -Encoding utf8
$exe = Join-Path $OutDir 'sil_flight.exe'
& (Join-Path $GccDir 'g++.exe') -static -static-libgcc -static-libstdc++ @objects -lm -lws2_32 -o $exe 2>&1 |
    Tee-Object -FilePath $log -Append
if ($LASTEXITCODE -ne 0) { throw 'link failed' }
if ($BuildOnly) {
    "Built $exe"
    return
}

"== run ==" | Add-Content -LiteralPath $log -Encoding utf8
$runArgs = if ($GroundPort) {
    @($FmtHilPort, $FmtHover.ToString([Globalization.CultureInfo]::InvariantCulture),
      'Ground', $FmtStateTimeoutMs, $GroundPort)
} elseif ($FmtHilPort) {
    @($FmtHilPort, $FmtHover.ToString([Globalization.CultureInfo]::InvariantCulture),
      $FmtScenario, $FmtStateTimeoutMs)
} else {
    @()
}
$simProcess = $null
try {
    if ($simPath) {
        $network = [Net.NetworkInformation.IPGlobalProperties]::GetIPGlobalProperties()
        if ($network.GetActiveUdpListeners().Port -contains $FmtHilPort) {
            throw "UDP port $FmtHilPort is already in use"
        }
        $engineLog = Join-Path (Resolve-Path -LiteralPath $OutDir).Path 'fmt_sim_engine.log'
        $simArgs = @('/Game/Maps/MainMenu', '-windowed', '-ResX=320', '-ResY=240',
            '-nosound', '-unattended', '-RenderOffscreen', '-NoVSync', '-NoTextureStreaming',
            '-d3d11', "-graphicsadapter=$FmtGraphicsAdapter", '-AllowSoftwareRendering', '-preferMS',
            '-ExecCmds="t.MaxFPS 45,r.ScreenPercentage 50,sg.ShadowQuality 0,sg.PostProcessQuality 0,sg.EffectsQuality 0,sg.ViewDistanceQuality 0"',
            "-abslog=`"$engineLog`"")
        $simProcess = Start-Process -FilePath $simPath -WorkingDirectory $simDirectory.FullName `
            -ArgumentList $simArgs -WindowStyle Hidden -PassThru
        $startup = [Diagnostics.Stopwatch]::StartNew()
        while ($network.GetActiveUdpListeners().Port -notcontains $FmtHilPort) {
            if ($simProcess.HasExited -or $startup.Elapsed.TotalSeconds -ge 45) {
                throw "FMT-SIM did not open UDP port $FmtHilPort. See $engineLog"
            }
            Start-Sleep -Milliseconds 100
        }
    }
    & $exe @runArgs 2>&1 | Tee-Object -FilePath $log -Append
    $code = $LASTEXITCODE
} finally {
    if ($simProcess -and -not $simProcess.HasExited -and $simProcess.Path -eq $simPath) {
        [void]$simProcess.CloseMainWindow()
        if (-not $simProcess.WaitForExit(3000)) {
            Stop-Process -InputObject $simProcess
            [void]$simProcess.WaitForExit(3000)
        }
    }
}
"exit=$code, log: $log"
exit $code

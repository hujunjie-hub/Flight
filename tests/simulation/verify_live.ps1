<#
    Live HIL verification against a running FMT-SIM (no flight controller).

    What it proves, in order:
      1. FMT-SIM is running and has an active SIH session on its MavproxyFCS port
         (it answers the firmware's HEARTBEAT/HIL_ACTUATOR_CONTROLS);
      2. the stream it sends is CRC-valid MAVLink v2 HIL_SENSOR/HIL_GPS
         (independent Python decoder in tools/fmt_sim_probe.py);
      3. the *firmware* codec decodes the very same captured bytes
         (tests/simulation/decode_capture.c includes task_sim.c).

    It does NOT cover the UART/bridge leg or the target board; that needs the
    board on a COM port plus middleware/communication/sim/fmt_sim_udp_bridge.py.

    Usage:
        pwsh tests/simulation/verify_live.ps1
        pwsh tests/simulation/verify_live.ps1 -Seconds 15 -Peer 127.0.0.1:14552
#>

[CmdletBinding()]
param(
    [string]$Peer = '127.0.0.1:14553',
    [double]$Seconds = 8,
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
if ([string]::IsNullOrWhiteSpace($OutDir)) { $OutDir = Join-Path $repoRoot 'build\validation\sim' }
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$log = Join-Path $OutDir 'live_link.log'
$capture = Join-Path $OutDir 'sim_capture.bin'

"== 1. simulator presence ==" | Set-Content -LiteralPath $log -Encoding utf8
$process = Get-Process FMT_SIM -ErrorAction SilentlyContinue
if (-not $process) {
    "FMT_SIM is not running; start it and load an SIH vehicle first." |
        Tee-Object -FilePath $log -Append
    throw 'FMT_SIM not running'
}
"FMT_SIM pid $($process.Id -join ','), window '$($process.MainWindowTitle -join '; ')'" |
    Tee-Object -FilePath $log -Append
$port = [int]($Peer -split ':')[-1]
$endpoint = Get-NetUDPEndpoint -LocalPort $port -ErrorAction SilentlyContinue
if (-not $endpoint) {
    "warning: nothing is bound to UDP $port; the sim may be on another port or idle" |
        Tee-Object -FilePath $log -Append
}

"== 2. MAVLink HIL exchange (python probe) ==" | Add-Content -LiteralPath $log -Encoding utf8
& $Python (Join-Path $repoRoot 'tools/fmt_sim_probe.py') --peer $Peer --seconds $Seconds --save $capture 2>&1 |
    Tee-Object -FilePath $log -Append
if ($LASTEXITCODE -ne 0) {
    throw 'no HIL stream: open FMT-SIM, start the SIH multicopter session, then rerun'
}

"== 3. firmware codec on the captured bytes ==" | Add-Content -LiteralPath $log -Encoding utf8
$includes = @('middleware/contracts', 'middleware/sensor', 'sensor_device', 'middleware/protocol',
    'applications/tasks', 'tests/simulation/golden') | ForEach-Object { Join-Path $repoRoot $_ }
$decoder = Join-Path $OutDir 'decode_capture.exe'
& $Gcc -std=c11 -O2 -Wall -Wextra -Werror @($includes | ForEach-Object { "-I$_" }) `
    (Join-Path $repoRoot 'tests/simulation/decode_capture.c') `
    (Join-Path $repoRoot 'middleware/protocol/mavlink/mavlink_frame.c') `
    (Join-Path $repoRoot 'middleware/protocol/mavlink/mavlink_hil.c') `
    (Join-Path $repoRoot 'middleware/sensor/sample_normalization/sensor_sample.c') `
    (Join-Path $repoRoot 'middleware/sensor/calibration/sensor_calibration.c') `
    (Join-Path $repoRoot 'middleware/sensor/measurement_filter/sensor_filter.c') -lm -o $decoder 2>&1 |
    Tee-Object -FilePath $log -Append
if ($LASTEXITCODE -ne 0) { throw 'decoder build failed' }
& $decoder $capture 2>&1 | Tee-Object -FilePath $log -Append
if ($LASTEXITCODE -ne 0) { throw 'firmware codec rejected the captured stream' }

"== PASS: simulator HIL stream accepted by the firmware codec ==" | Add-Content -LiteralPath $log -Encoding utf8
"live log: $log"

<#
    Starts the Flight SIL and the Flight-customized QGroundControl.

    QGroundControl and the Qt 6.11.1 runtime under build/validation/qgc_native
    were compiled with MinGW 13.1; using any other MinGW runtime makes Qt DLLs
    fail with a missing __emutls_v._ZSt11__once_call entry point.

    Usage:
        pwsh tools/run_qgc_flight.ps1
        pwsh tools/run_qgc_flight.ps1 -UdpPort 14550 -Scenario Ground
#>
[CmdletBinding()]
param(
    [ValidateRange(0, 65535)][int]$UdpPort = 14550,
    [string]$Sil = '',
    [string]$SilMode = 'Ground',
    [ValidateRange(1, 20)][int]$SilRateHz = 5
)

$ErrorActionPreference = 'Stop'
$repo = $PSScriptRoot
while (-not (Test-Path -LiteralPath (Join-Path $repo 'SConstruct'))) {
    $parent = Split-Path $repo -Parent
    if (-not $parent) { throw 'Flight root not found' }
    $repo = $parent
}

$qt = Join-Path $repo 'build/validation/qgc_native/qt'
$mingw = Join-Path $repo 'build/validation/qgc_native/mingw1310/Tools/mingw1310_64/bin'
$qgc = Join-Path $repo 'build/validation/qgc_native/cmake_delivery/Release/QGroundControl.exe'
if (-not (Test-Path $qgc)) { throw "QGroundControl not built: $qgc" }
if (-not (Test-Path (Join-Path $mingw 'g++.exe'))) { throw "MinGW 13.1 not found: $mingw" }

if (-not $Sil) { $Sil = Join-Path $repo 'build/validation/qgc/sil_r5/sil_flight.exe' }
if (-not (Test-Path $Sil)) { throw "SIL not built: $Sil" }

$env:PATH = "$mingw;$(Join-Path $qt 'bin');" + $env:PATH

Write-Host "SIL  : $Sil 0 $SilRateHz $SilMode 350 $UdpPort"
$silProcess = Start-Process -FilePath $Sil `
    -ArgumentList @('0', '0.62', $SilMode, '350', "$UdpPort") -PassThru

Start-Sleep -Seconds 1
Write-Host "QGC  : $qgc"
$qgcProcess = Start-Process -FilePath $qgc -PassThru
Write-Host "PIDs : SIL=$($silProcess.Id) QGC=$($qgcProcess.Id)"

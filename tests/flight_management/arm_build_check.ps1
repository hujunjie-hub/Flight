# Cortex-M7 production-source compile check. ABI is verified separately by
# tests/abi/abi_report.py. No hardware execution or WCET claim.
$ErrorActionPreference = 'Stop'

$layoutRoot = Get-Item -LiteralPath $PSScriptRoot
while (-not (Test-Path -LiteralPath (Join-Path $layoutRoot.FullName 'SConstruct'))) {
  if ($null -eq $layoutRoot.Parent) { throw 'Flight root not found' }
  $layoutRoot = $layoutRoot.Parent
}
$root = $layoutRoot.FullName
Set-Location $root

$gcc = if ($env:ARM_GCC) { $env:ARM_GCC } else { 'D:\STM32CubeCLT\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe' }
$size = if ($env:ARM_SIZE) { $env:ARM_SIZE } else { 'D:\STM32CubeCLT\GNU-tools-for-STM32\bin\arm-none-eabi-size.exe' }
if (-not (Test-Path $gcc)) { throw "ARM compiler not found: $gcc" }

$out = 'build\m4_arm'
New-Item -ItemType Directory -Force -Path $out | Out-Null

$flags = @(
  '-mcpu=cortex-m7', '-mthumb', '-mfpu=fpv5-d16', '-mfloat-abi=hard',
  '-std=c11', '-Wall', '-Wextra', '-Werror', '-fno-short-enums',
  '-Imiddleware/contracts', '-Imiddleware/model/fms', '-Imiddleware/model/fms/quad_x4'
)

$sources = @(
  'middleware/model/fms/fms_interface.c',
  'middleware/model/fms/geofence_geometry/geofence_geometry.c',
  'middleware/model/fms/quad_x4/flight_state_machine/fms.c',
  'middleware/model/fms/quad_x4/mission_lifecycle/mission_lifecycle.c',
  'middleware/model/fms/quad_x4/operator_authority/operator_authority.c',
  'middleware/model/fms/quad_x4/flight_safety/actuation_permit.c',
  'middleware/model/fms/quad_x4/flight_safety/fms_safety.c',
  'middleware/model/fms/quad_x4/flight_policy/quad_x4_fms.c'
)

$objects = foreach ($src in $sources) {
  $obj = Join-Path $out ((Split-Path $src -Leaf).Replace('.c', '.o'))
  & $gcc @flags -c $src -o $obj
  if ($LASTEXITCODE -ne 0) { throw "ARM compile failed: $src" }
  $obj
}

& $size @objects
if ($LASTEXITCODE -ne 0) { throw 'arm-none-eabi-size failed' }
Write-Output 'M4_ARM_COMPILE_OK'

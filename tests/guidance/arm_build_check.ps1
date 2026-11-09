# ARM (Cortex-M7) compile check for T191-C-M5 (M5 Guidance).
# Not a target build, WCET or hardware result.
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

$out = 'build\m5_arm'
New-Item -ItemType Directory -Force -Path $out | Out-Null

$flags = @(
  '-mcpu=cortex-m7', '-mthumb', '-mfpu=fpv5-d16', '-mfloat-abi=hard',
  '-std=c11', '-Wall', '-Wextra', '-Werror', '-fno-short-enums',
  '-Imiddleware/contracts', '-Imiddleware/model/fms',
  '-Imiddleware/guidance'
)

$sources = @(
  'middleware/model/fms/geofence_geometry/geofence_geometry.c',
  'middleware/guidance/guidance_update/guidance.c',
  'middleware/guidance/trajectory_planning/trajectory_math.c',
  'middleware/guidance/guidance_update/guidance_input_checks.c',
  'middleware/guidance/trajectory_planning/mission_trajectory.c',
  'middleware/guidance/pilot_and_landing/pilot_and_landing.c',
  'middleware/guidance/trajectory_planning/trajectory_validation.c',
  'middleware/guidance/guidance_update/guidance_config.c'
)

$objects = foreach ($src in $sources) {
  $obj = Join-Path $out ((Split-Path $src -Leaf).Replace('.c', '.o'))
  & $gcc @flags -c $src -o $obj
  if ($LASTEXITCODE -ne 0) { throw "ARM compile failed: $src" }
  $obj
}

& $size @objects
if ($LASTEXITCODE -ne 0) { throw 'arm-none-eabi-size failed' }
Write-Output 'M5_ARM_COMPILE_OK'

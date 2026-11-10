# ARM (Cortex-M7) compile check for T191-C-M5 (M5 Guidance), including the
# Host/SIL-only advanced planner and M5-06 validator translation units.
# Not a target build, WCET, runtime stack watermark or hardware result.
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
  '-std=c11', '-Wall', '-Wextra', '-Werror', '-fno-short-enums', '-fstack-usage',
  '-fno-fast-math', '-fno-unsafe-math-optimizations',
  '-ffp-contract=off', '-frounding-math', '-fexcess-precision=standard',
  '-Imiddleware/contracts', '-Imiddleware/model/fms',
  '-Imiddleware/guidance'
)

# Sources compiled into the H723/NONE product slice.
$productionSources = @(
  'middleware/model/fms/geofence_geometry/geofence_geometry.c',
  'middleware/guidance/guidance_update/guidance.c',
  'middleware/guidance/trajectory_planning/trajectory_math.c',
  'middleware/guidance/guidance_update/guidance_input_checks.c',
  'middleware/guidance/trajectory_planning/mission_trajectory.c',
  'middleware/guidance/pilot_and_landing/pilot_and_landing.c',
  'middleware/guidance/trajectory_planning/trajectory_validation.c',
  'middleware/guidance/guidance_update/guidance_config.c'
)

# Host/SIL-only M5 advanced planner and M5-06 validator translation units.
$hostOnlySources = @(
  'middleware/guidance/trajectory_planning/m5_local_map.c',
  'middleware/guidance/trajectory_planning/m5_kinodynamic_astar.c',
  'middleware/guidance/trajectory_planning/m5_time_scaler.c',
  'middleware/guidance/trajectory_planning/m5_path_smoother.c',
  'middleware/guidance/trajectory_planning/minimum_jerk_optimizer.c',
  'middleware/guidance/trajectory_planning/m5_trajectory_safety.c'
)

$objects = @()
foreach ($src in $productionSources) {
  $obj = Join-Path $out ((Split-Path $src -Leaf).Replace('.c', '.o'))
  & $gcc @flags -c $src -o $obj
  if ($LASTEXITCODE -ne 0) { throw "ARM compile failed: $src" }
  $objects += $obj
}
foreach ($src in $hostOnlySources) {
  $obj = Join-Path $out ((Split-Path $src -Leaf).Replace('.c', '.o'))
  & $gcc @flags -DFLIGHT_M5_HOST_PLANNER -c $src -o $obj
  if ($LASTEXITCODE -ne 0) { throw "ARM compile failed: $src" }
  $objects += $obj
}

& $size @objects
if ($LASTEXITCODE -ne 0) { throw 'arm-none-eabi-size failed' }
Write-Output 'M5_ARM_COMPILE_OK'

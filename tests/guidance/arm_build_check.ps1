# ARM (Cortex-M7) compile check for T191-C-M5 (M5 Guidance), including the
# Host/SIL-only advanced planner and M5-06 validator translation units.
# Not a target build, WCET, runtime stack watermark or hardware result.
#
# Optimization defaults to the product release level (-O2, see rtconfig.py).
# Measuring stack at the compiler default (-O0) is not representative of the
# shipped code; pass -Optimization '-O0' to reproduce the historical baseline.
param(
  [string]$Optimization = '-O2'
)
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
$objdump = if ($env:ARM_OBJDUMP) { $env:ARM_OBJDUMP } else { 'D:\STM32CubeCLT\GNU-tools-for-STM32\bin\arm-none-eabi-objdump.exe' }
if (-not (Test-Path $gcc)) { throw "ARM compiler not found: $gcc" }

$out = 'build\m5_arm'
New-Item -ItemType Directory -Force -Path $out | Out-Null

$flags = @(
  '-mcpu=cortex-m7', '-mthumb', '-mfpu=fpv5-d16', '-mfloat-abi=hard',
  '-std=c11', '-Wall', '-Wextra', '-Werror', '-fno-short-enums', '-fstack-usage',
  $Optimization,
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

# Emit the disassembly in the same build step as the object so the stack
# analyzer can never consume a .dis older than the .o it describes.
$validatorObj = Join-Path $out 'm5_trajectory_safety.o'
$validatorDis = Join-Path $out 'm5_trajectory_safety.dis'
& $objdump -d -C $validatorObj | Set-Content -LiteralPath $validatorDis -Encoding utf8
if ($LASTEXITCODE -ne 0) { throw 'arm-none-eabi-objdump failed' }
$disHash = (Get-FileHash $validatorDis -Algorithm SHA256).Hash.ToLowerInvariant()

$codeCommit = if ($env:FLIGHT_CODE_COMMIT) { $env:FLIGHT_CODE_COMMIT } else { (git rev-parse HEAD).Trim() }
$sourceHash = (Get-FileHash 'middleware/guidance/trajectory_planning/m5_trajectory_safety.c' -Algorithm SHA256).Hash.ToLowerInvariant()
$toolchain = @(
  "commit=$codeCommit",
  "source_sha256=$sourceHash",
  "compiler=$(& $gcc --version | Select-Object -First 1)",
  "size_tool=$(& $size --version | Select-Object -First 1)",
  "optimization=$Optimization",
  "disassembly=m5_trajectory_safety.dis",
  "disassembly_sha256=$disHash",
  "production_objects=$($productionSources.Count)",
  "host_only_objects=$($hostOnlySources.Count)",
  "flags=$($flags -join ' ')",
  'M5-06 scope=Host/SIL-only validator object cross-compiled for Cortex-M7; not linked into product'
)
Set-Content -LiteralPath (Join-Path $out 'toolchain.txt') -Value $toolchain -Encoding utf8
Write-Output 'M5_ARM_COMPILE_OK'

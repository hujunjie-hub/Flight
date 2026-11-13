# Link the M5-06 validator with the project objects and the toolchain libm, then
# analyse the whole image. This closes the "external libm edge" gap of the
# single-object analyzer: every reachable call resolves to a real function.
#
# Evidence only. The image is NOT the product firmware (B03/B04 still block the
# RT-Thread package tree and the product link), and a static image analysis is
# NOT an H723 stack watermark, WCET, RTA, HIL or flight result.
param(
  [string]$Optimization = '-O2',
  [string]$OutJson = 'build/validation/regenerated/stack_image.json',
  [string]$OutDir = 'build/stack_image'
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
$objdump = if ($env:ARM_OBJDUMP) { $env:ARM_OBJDUMP } else { 'D:\STM32CubeCLT\GNU-tools-for-STM32\bin\arm-none-eabi-objdump.exe' }
$python = if ($env:PYTHON) { $env:PYTHON } else { 'python' }

# 1. Rebuild every object and the validator disassembly at the requested level.
& pwsh -NoProfile -File tests/guidance/arm_build_check.ps1 -Optimization $Optimization
if ($LASTEXITCODE -ne 0) { throw "arm_build_check failed at $Optimization" }

New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$stubObj = Join-Path $OutDir 'stack_image_stubs.o'
& $gcc -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard $Optimization `
  -ffp-contract=off -c tests/guidance/stack_image_stubs.c -o $stubObj
if ($LASTEXITCODE -ne 0) { throw 'stub compile failed' }

# 2. Link the method under test with the real libm/libc.
$objects = @(
  'geofence_geometry', 'guidance', 'trajectory_math', 'guidance_input_checks',
  'mission_trajectory', 'pilot_and_landing', 'trajectory_validation',
  'guidance_config', 'm5_local_map', 'm5_kinodynamic_astar', 'm5_time_scaler',
  'm5_path_smoother', 'minimum_jerk_optimizer', 'm5_trajectory_safety'
) | ForEach-Object { "build/m5_arm/$_.o" }
$elf = Join-Path $OutDir 'validator_image.elf'
& $gcc -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -nostartfiles `
  '-Wl,-e,m5_trajectory_safety_validate' -o $elf @objects $stubObj -lm
if ($LASTEXITCODE -ne 0) { throw 'stack image link failed' }

$dis = Join-Path $OutDir 'validator_image.dis'
& $objdump -d -C $elf | Set-Content -LiteralPath $dis -Encoding utf8
if ($LASTEXITCODE -ne 0) { throw 'objdump failed' }

# 3. Analyse and propagate the analyzer's exit code (0 PASS, 2 BLOCKED, 3 UNKNOWN).
& $python tools/analyze_stack_image.py --dis $dis --su-dir build/m5_arm `
  --output $OutJson --csv-dir (Join-Path $OutDir 'csv')
exit $LASTEXITCODE

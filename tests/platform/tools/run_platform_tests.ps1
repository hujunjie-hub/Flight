# T191-A-PLATFORM host/ARM test runner (independent preparation)
# Builds and runs the service unit tests on the Windows host and compiles the
# same sources for the Cortex-M7 firmware target (C99, hard-float).  It does
# not build or flash the firmware.
param(
  [string]$HostGcc = "D:\STM32CubeCLT\mingw64\bin\gcc.exe",
  [string]$ArmGcc  = "D:\STM32CubeCLT\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe"
)

$ErrorActionPreference = "Stop"
$PSNativeCommandUseErrorActionPreference = $false
$layoutRoot = Get-Item -LiteralPath $PSScriptRoot
while (-not (Test-Path -LiteralPath (Join-Path $layoutRoot.FullName 'SConstruct'))) {
  if ($null -eq $layoutRoot.Parent) { throw 'Flight root not found' }
  $layoutRoot = $layoutRoot.Parent
}
$root = $layoutRoot.FullName
$evidence = Join-Path $root "tests\platform\evidence"
New-Item -ItemType Directory -Force $evidence | Out-Null

$inc = @(
  "-I$(Join-Path $root 'middleware/contracts')",
  "-I$(Join-Path $root 'sensor_device\time_base')"
)
$armFlags = @("-mcpu=cortex-m7","-mthumb","-mfpu=fpv5-d16","-mfloat-abi=hard",
              "-Wall","-Wextra","-Werror","-fno-short-enums")

function Read-Log([string]$path) {
  $text = Get-Content -Path $path -Raw
  if ($null -eq $text) { return "" }
  return [string]$text
}

$tests = @()
$armCompiles = @()

# --- time service host test -----------------------------------------------
$timeSources = @(
  (Join-Path $root "sensor_device\time_base\systime.c"),
  (Join-Path $root "tests\time\test_time_service.c")
)
$timeExe = Join-Path $env:TEMP "platform_time_service.exe"
$timeBuildLog = Join-Path $evidence "build_time_service.log"
$timeRunLog = Join-Path $evidence "run_time_service.log"
$timeBuildCmd = (@($HostGcc, "-std=c11", "-Wall", "-Wextra", "-Werror") + $inc + $timeSources + @("-o", $timeExe)) -join " "
& $HostGcc -std=c11 -Wall -Wextra -Werror @inc @timeSources -o $timeExe *> $timeBuildLog
$timeBuildCode = $LASTEXITCODE
if ($timeBuildCode -ne 0) { throw "host build failed: time_service (see $timeBuildLog)" }
& $timeExe *> $timeRunLog
$timeRunCode = $LASTEXITCODE
if ($timeRunCode -ne 0) { throw "host test failed: time_service (see $timeRunLog)" }
$tests += [ordered]@{
  name = "time_service"
  sources = $timeSources
  build_command = $timeBuildCmd
  build_exit = $timeBuildCode
  run_exit = $timeRunCode
  output = (Read-Log $timeRunLog).Trim()
}

# --- ARM compile-only checks ----------------------------------------------
$timeArmLog = Join-Path $evidence "arm_time_service_c99.log"
& $ArmGcc @armFlags "-std=c99" @inc -c (Join-Path $root "sensor_device\time_base\systime.c") -o (Join-Path $env:TEMP "arm_time_service.o") *> $timeArmLog
$timeArmCode = $LASTEXITCODE
if ($timeArmCode -ne 0) { throw "ARM compile failed: time_service (see $timeArmLog)" }
$armCompiles += [ordered]@{
  name = "time_service_c99"
  source = "sensor_device/time_base/systime.c"
  std = "c99"
  exit = $timeArmCode
  output = (Read-Log $timeArmLog).Trim()
}

$results = [ordered]@{
  baseline = "V1.9.1-GLOBAL-AUDIT-R3"
  scope = "HOST_UNIT + ARM_COMPILE_ONLY"
  production_target_validation = "NOT_RUN"
  generated_at = (Get-Date).ToString("yyyy-MM-ddTHH:mm:sszzz")
  tests = $tests
  arm_compile = $armCompiles
}
$results | ConvertTo-Json -Depth 6 | Out-File -FilePath (Join-Path $evidence "platform_tests.json") -Encoding utf8
Write-Output ($results | ConvertTo-Json -Depth 3 -Compress)

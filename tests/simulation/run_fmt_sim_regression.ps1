<#
    Run the formal local ENG-07 FMT-SIM regression set.

    Each scenario gets a fresh FMT-SIM process and its own immutable evidence
    directory. The SIL executable is built once and reused for the scenarios.
    This script is intentionally a local Windows workflow; CI must not launch
    the native FMT-SIM application.

    Usage:
        pwsh -NoProfile -File tests/simulation/run_fmt_sim_regression.ps1 -FmtSimExe <path>
        pwsh -NoProfile -File tests/simulation/run_fmt_sim_regression.ps1 -FmtSimExe <path> -Scenario Land
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$FmtSimExe,
    [string]$GccDir = 'D:\STM32CubeCLT\mingw64\bin',
    [string]$OutRoot = '',
    [ValidateRange(1, 65535)][int]$FmtHilPort = 14653,
    [ValidateRange(0, 15)][int]$FmtGraphicsAdapter = 1,
    [ValidateRange(0.1, 0.95)][double]$FmtHover = 0.62,
    [ValidateRange(350, 1000)][int]$FmtStateTimeoutMs = 350,
    [ValidateSet('All', 'Land', 'Return', 'Mission')][string]$Scenario = 'All',
    [switch]$ConfirmLoopbackAddress,
    [switch]$AllowDirty
)

$ErrorActionPreference = 'Stop'
$scriptPath = Join-Path $PSScriptRoot 'run_sil.ps1'
if (-not (Test-Path -LiteralPath $scriptPath -PathType Leaf)) {
    throw "SIL runner missing: $scriptPath"
}
$parseTokens = $null
$parseErrors = $null
[void][System.Management.Automation.Language.Parser]::ParseFile($scriptPath, [ref]$parseTokens, [ref]$parseErrors)
if ($parseErrors.Count -gt 0) {
    $parseText = ($parseErrors | ForEach-Object { $_.Message }) -join '; '
    throw "SIL runner PowerShell parse failed: $parseText"
}

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
    throw "Configure exactly one FMT-SIM HIL vehicle on UDP port $FmtHilPort in $settingsPath"
}
if ($hilVehicles[0].MavproxyFCS.Autobound -ne $true) {
    throw "The FMT-SIM HIL vehicle must set MavproxyFCS.Autobound=true in $settingsPath"
}
if (-not (Test-Path -LiteralPath (Join-Path $GccDir 'gcc.exe') -PathType Leaf) -or
    -not (Test-Path -LiteralPath (Join-Path $GccDir 'g++.exe') -PathType Leaf)) {
    throw "MinGW gcc.exe/g++.exe not found under $GccDir"
}

$layoutRoot = Get-Item -LiteralPath $PSScriptRoot
while (-not (Test-Path -LiteralPath (Join-Path $layoutRoot.FullName 'SConstruct'))) {
    if ($null -eq $layoutRoot.Parent) { throw 'Flight repository root not found' }
    $layoutRoot = $layoutRoot.Parent
}
$repoRoot = $layoutRoot.FullName
$head = (& git -C $repoRoot rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0 -or $head -notmatch '^[0-9a-f]{40}$') {
    throw 'Unable to resolve the exact Git HEAD'
}
$branch = (& git -C $repoRoot branch --show-current).Trim()
if ($LASTEXITCODE -ne 0) { throw 'Unable to resolve the current Git branch' }
$expectedBranch = 'main'
if ($branch -ne $expectedBranch) {
    throw "Expected branch '$expectedBranch', found '$branch'."
}
$gitStatus = @(& git -C $repoRoot status --porcelain=v2 --untracked-files=all)
if ($LASTEXITCODE -ne 0) { throw 'Unable to read Git working-tree status' }
$dirty = $gitStatus.Count -gt 0
if ($dirty -and -not $AllowDirty) {
    throw 'Working tree is not clean. Commit or otherwise preserve local changes; use -AllowDirty only for diagnostic runs (not issue closure evidence).'
}
$fetchOutput = @(& git -C $repoRoot fetch --prune origin 2>&1)
$fetchExit = $LASTEXITCODE
if ($fetchExit -ne 0) {
    $fetchDetail = ($fetchOutput | ForEach-Object { [string]$_ }) -join [Environment]::NewLine
    throw "git fetch --prune origin failed (exit $fetchExit): $fetchDetail"
}
$upstream = (& git -C $repoRoot rev-parse --abbrev-ref --symbolic-full-name '@{u}' 2>$null)
$upstreamExit = $LASTEXITCODE
$upstreamHead = ''
if ($upstreamExit -eq 0 -and $upstream) {
    $upstream = $upstream.Trim()
    $upstreamHead = (& git -C $repoRoot rev-parse $upstream).Trim()
}
$expectedUpstream = "origin/$expectedBranch"
$upstreamIsExpected = $upstream -eq $expectedUpstream
$upstreamMatchesHead = $upstreamIsExpected -and
    -not [string]::IsNullOrWhiteSpace($upstreamHead) -and $upstreamHead -eq $head

$pwshCommand = Get-Command pwsh -ErrorAction Stop | Select-Object -First 1
$pwshPath = $pwshCommand.Source
if (-not $pwshPath) { throw 'Unable to resolve pwsh executable path' }
$pythonCommand = Get-Command python -ErrorAction Stop | Select-Object -First 1
$pythonPath = $pythonCommand.Source
if (-not $pythonPath) { throw 'Unable to resolve Python executable path' }

function Invoke-ChildPowerShell {
    param(
        [string[]]$Arguments,
        [string]$StdoutPath,
        [string]$StderrPath
    )
    $psi = [System.Diagnostics.ProcessStartInfo]::new()
    $psi.FileName = $pwshPath
    $psi.WorkingDirectory = $repoRoot
    $psi.UseShellExecute = $false
    $psi.RedirectStandardOutput = $true
    $psi.RedirectStandardError = $true
    $psi.ArgumentList.Add('-NoLogo')
    $psi.ArgumentList.Add('-NoProfile')
    foreach ($argument in $Arguments) {
        $psi.ArgumentList.Add([string]$argument)
    }
    $process = [System.Diagnostics.Process]::Start($psi)
    if ($null -eq $process) { throw 'Unable to start child PowerShell process' }
    $stdoutTask = $process.StandardOutput.ReadToEndAsync()
    $stderrTask = $process.StandardError.ReadToEndAsync()
    $process.WaitForExit()
    $stdout = $stdoutTask.GetAwaiter().GetResult()
    $stderr = $stderrTask.GetAwaiter().GetResult()
    [System.IO.File]::WriteAllText($StdoutPath, $stdout, [System.Text.UTF8Encoding]::new($false))
    [System.IO.File]::WriteAllText($StderrPath, $stderr, [System.Text.UTF8Encoding]::new($false))
    return [int]$process.ExitCode
}

function Get-Sha256 {
    param([string]$Path)
    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) { return $null }
    return (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant()
}

function Get-FirstVersionLine {
    param([string]$Executable, [string[]]$Arguments)
    try {
        $lines = @(& $Executable @Arguments 2>&1)
        if ($LASTEXITCODE -eq 0 -and $lines.Count -gt 0) { return [string]$lines[0] }
    } catch { }
    return 'UNAVAILABLE'
}

$timestamp = [DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$runId = "FMT-SIM-LOCAL-$timestamp-$($head.Substring(0, 8))"
if (-not $OutRoot) { $OutRoot = Join-Path $repoRoot 'build\validation\fmt-sim-local' }
$runDir = Join-Path $OutRoot $runId
if (Test-Path -LiteralPath $runDir) { throw "Evidence directory already exists; refusing to overwrite: $runDir" }
New-Item -ItemType Directory -Force -Path $runDir | Out-Null
$buildDir = Join-Path $runDir 'build'
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null

$gcc = Join-Path $GccDir 'gcc.exe'
$gxx = Join-Path $GccDir 'g++.exe'
$buildStdout = Join-Path $buildDir 'build.stdout.log'
$buildStderr = Join-Path $buildDir 'build.stderr.log'
$buildArgs = @(
    '-File', $scriptPath,
    '-GccDir', $GccDir,
    '-OutDir', $buildDir,
    '-FmtHilPort', "$FmtHilPort",
    '-FmtSimExe', $simPath,
    '-FmtGraphicsAdapter', "$FmtGraphicsAdapter",
    '-FmtHover', $FmtHover.ToString([Globalization.CultureInfo]::InvariantCulture),
    '-FmtStateTimeoutMs', "$FmtStateTimeoutMs",
    '-BuildOnly'
)
$buildStarted = [DateTime]::UtcNow
$buildExit = Invoke-ChildPowerShell -Arguments $buildArgs -StdoutPath $buildStdout -StderrPath $buildStderr
$buildEnded = [DateTime]::UtcNow
$builtExe = Join-Path $buildDir 'sil_flight.exe'
if ($buildExit -ne 0 -or -not (Test-Path -LiteralPath $builtExe -PathType Leaf)) {
    $buildStatus = 'FAIL'
} else {
    $buildStatus = 'PASS'
}
$buildExeSha = Get-Sha256 $builtExe

$selectedScenarios = if ($Scenario -eq 'All') { @('Land', 'Return', 'Mission') } else { @($Scenario) }
$scenarioResults = @()
if ($buildStatus -eq 'PASS') {
    foreach ($scenarioName in $selectedScenarios) {
        $scenarioDir = Join-Path $runDir $scenarioName
        New-Item -ItemType Directory -Force -Path $scenarioDir | Out-Null
        $scenarioExe = Join-Path $scenarioDir 'sil_flight.exe'
        Copy-Item -LiteralPath $builtExe -Destination $scenarioExe
        $stdoutPath = Join-Path $scenarioDir 'runner.stdout.log'
        $stderrPath = Join-Path $scenarioDir 'runner.stderr.log'
        $scenarioArgs = @(
            '-File', $scriptPath,
            '-GccDir', $GccDir,
            '-OutDir', $scenarioDir,
            '-FmtSimExe', $simPath,
            '-FmtHilPort', "$FmtHilPort",
            '-FmtScenario', $scenarioName,
            '-FmtGraphicsAdapter', "$FmtGraphicsAdapter",
            '-FmtHover', $FmtHover.ToString([Globalization.CultureInfo]::InvariantCulture),
            '-FmtStateTimeoutMs', "$FmtStateTimeoutMs",
            '-ReuseBuild'
        )
        $started = [DateTime]::UtcNow
        $exitCode = Invoke-ChildPowerShell -Arguments $scenarioArgs -StdoutPath $stdoutPath -StderrPath $stderrPath
        $ended = [DateTime]::UtcNow
        $exitPath = Join-Path $scenarioDir 'exit.txt'
        [System.IO.File]::WriteAllText($exitPath, "exit_code=$exitCode$([Environment]::NewLine)", [System.Text.UTF8Encoding]::new($false))
        $silLog = Join-Path $scenarioDir 'sil_flight.log'
        $engineLog = Join-Path $scenarioDir 'fmt_sim_engine.log'
        $logText = if (Test-Path -LiteralPath $silLog -PathType Leaf) {
            Get-Content -LiteralPath $silLog -Raw
        } else { '' }
        $summaryMatch = [regex]::Match($logText, '(?m)^sil_flight:\s*(\d+)\s+checks,\s*(\d+)\s+failures,\s*commits=(\d+)\s+inhibits=(\d+)')
        $gapMatch = [regex]::Match($logText, '\[fmt\]\s+wall timeout\s+(\d+)\s+ms,\s+longest successful state gap\s+([\d.]+)\s+ms')
        $checks = $null
        $failures = $null
        $commits = $null
        $inhibits = $null
        if ($summaryMatch.Success) {
            $checks = [int]$summaryMatch.Groups[1].Value
            $failures = [int]$summaryMatch.Groups[2].Value
            $commits = [int]$summaryMatch.Groups[3].Value
            $inhibits = [int]$summaryMatch.Groups[4].Value
        }
        $longestGapMs = $null
        if ($gapMatch.Success) { $longestGapMs = [double]::Parse($gapMatch.Groups[2].Value, [Globalization.CultureInfo]::InvariantCulture) }
        $settleCount = ([regex]::Matches($logText, '(?m)^\[settle\]')).Count
        $hasFinalState = $logText.Contains('[final]')
        $hasDescentMetric = $logText.Contains('[fmt] measured maximum descent in the last 0.5 m:')
        $hasReturnDistance = $scenarioName -eq 'Land' -or $logText.Contains('[fmt] return touchdown distance')
        $hasMissionMetrics = $scenarioName -ne 'Mission' -or $logText.Contains('[mission] closest waypoint distances')
        $validSummary = $summaryMatch.Success -and $checks -gt 0 -and $failures -eq 0
        $reportedTimeoutMs = if ($gapMatch.Success) { [int]$gapMatch.Groups[1].Value } else { $null }
        $validGap = $gapMatch.Success -and $reportedTimeoutMs -eq $FmtStateTimeoutMs -and $longestGapMs -lt $FmtStateTimeoutMs
        $hasEngineLog = Test-Path -LiteralPath $engineLog -PathType Leaf
        $passed = $exitCode -eq 0 -and $validSummary -and $validGap -and
            $settleCount -gt 0 -and $hasFinalState -and $hasDescentMetric -and
            $hasReturnDistance -and $hasMissionMetrics -and $hasEngineLog
        $scenarioResults += [pscustomobject]@{
            scenario = $scenarioName
            status = $(if ($passed) { 'PASS' } else { 'FAIL' })
            log = "$scenarioName/sil_flight.log"
            engine_log = "$scenarioName/fmt_sim_engine.log"
            exit_file = "$scenarioName/exit.txt"
            started_utc = $started.ToString('o')
            ended_utc = $ended.ToString('o')
            exit_code = $exitCode
            checks = $checks
            failures = $failures
            commits = $commits
            inhibits = $inhibits
            wall_timeout_ms = $FmtStateTimeoutMs
            reported_timeout_ms = $reportedTimeoutMs
            longest_successful_state_gap_ms = $longestGapMs
            settle_trace_lines = $settleCount
            final_state_logged = $hasFinalState
            descent_metric_logged = $hasDescentMetric
            return_distance_logged = $hasReturnDistance
            mission_waypoint_metrics_logged = $hasMissionMetrics
            engine_log_present = $hasEngineLog
            executable_sha256 = (Get-Sha256 $scenarioExe)
            sil_log_sha256 = (Get-Sha256 $silLog)
            engine_log_sha256 = (Get-Sha256 $engineLog)
            stdout_sha256 = (Get-Sha256 $stdoutPath)
            stderr_sha256 = (Get-Sha256 $stderrPath)
        }
    }
}
$headAfter = (& git -C $repoRoot rev-parse HEAD).Trim()
$branchAfter = (& git -C $repoRoot branch --show-current).Trim()
$gitStatusAfter = @(& git -C $repoRoot status --porcelain=v2 --untracked-files=all)
$worktreeStable = $headAfter -eq $head -and $branchAfter -eq $branch -and $gitStatusAfter.Count -eq 0
$failedScenarioCount = @($scenarioResults | Where-Object { $_.status -ne 'PASS' }).Count
$allScenarioPass = $scenarioResults.Count -eq $selectedScenarios.Count -and
    $scenarioResults.Count -gt 0 -and $failedScenarioCount -eq 0
$overall = if ($buildStatus -eq 'PASS' -and $allScenarioPass) { 'PASS' } else { 'FAIL' }
$closureReasons = @()
if ($Scenario -ne 'All') { $closureReasons += 'Only a subset of scenarios was requested.' }
if ($FmtStateTimeoutMs -ne 350) { $closureReasons += 'State timeout differs from the strict 350 ms gate.' }
if ($FmtHilPort -ne 14653) { $closureReasons += 'HIL UDP port differs from the required 14653 port.' }
if ([Math]::Abs($FmtHover - 0.62) -gt 0.000000001) { $closureReasons += 'Hover calibration differs from the required baseline 0.62.' }
if (-not $ConfirmLoopbackAddress) { $closureReasons += 'Owner has not confirmed the FMT-SIM HIL address is loopback 127.0.0.1.' }
if ($buildStatus -ne 'PASS') { $closureReasons += 'SIL build did not pass.' }
if (-not $allScenarioPass) { $closureReasons += 'One or more requested scenarios did not pass.' }
if ($dirty -or $gitStatusAfter.Count -gt 0) { $closureReasons += 'Git working tree was dirty before or after the run.' }
if ($branch -ne $expectedBranch) { $closureReasons += 'Branch does not match the required branch.' }
if (-not $upstreamIsExpected) { $closureReasons += 'Branch upstream is not origin/opt/stage-02-m5-06-safety-validator.' }
if (-not $upstreamMatchesHead) { $closureReasons += 'Local HEAD does not match the freshly fetched upstream tracking HEAD.' }
if (-not $worktreeStable) { $closureReasons += 'Branch, HEAD or working-tree state changed during the run.' }
$strictGateEligible = $overall -eq 'PASS' -and $Scenario -eq 'All' -and
    $FmtStateTimeoutMs -eq 350 -and $FmtHilPort -eq 14653 -and
    [Math]::Abs($FmtHover - 0.62) -le 0.000000001 -and $ConfirmLoopbackAddress -and
    -not $dirty -and $gitStatusAfter.Count -eq 0 -and
    $branch -eq $expectedBranch -and $upstreamIsExpected -and $upstreamMatchesHead -and $worktreeStable

$logRecords = @()
foreach ($scenarioResult in $scenarioResults) {
    foreach ($relativePath in @($scenarioResult.log, $scenarioResult.engine_log, $scenarioResult.exit_file)) {
        $absolutePath = Join-Path $runDir $relativePath
        $present = Test-Path -LiteralPath $absolutePath -PathType Leaf
        $logRecords += [pscustomobject]@{
            path = $relativePath
            present = $present
            size_bytes = $(if ($present) { (Get-Item -LiteralPath $absolutePath).Length } else { 0 })
            sha256 = (Get-Sha256 $absolutePath)
        }
    }
}
$toolVersions = [ordered]@{
    powershell = (Get-FirstVersionLine $pwshPath @('-NoProfile', '-Command', '$PSVersionTable.PSVersion.ToString()'))
    gcc = (Get-FirstVersionLine $gcc @('--version'))
    gxx = (Get-FirstVersionLine $gxx @('--version'))
    python = (Get-FirstVersionLine $pythonPath @('--version'))
    git = (Get-FirstVersionLine 'git' @('--version'))
}
$manifest = [ordered]@{
    schema = 'FMT_SIM_LOCAL_HANDOFF_V1'
    run_id = $runId
    overall = $overall
    issue_closure_eligible = $strictGateEligible
    closure_eligibility_reason = $closureReasons
    started_utc = $buildStarted.ToString('o')
    completed_utc = [DateTime]::UtcNow.ToString('o')
    repository = 'hujunjie-hub/Flight'
    branch = $branch
    branch_after = $branchAfter
    expected_branch = $expectedBranch
    commit = $head
    head_after = $headAfter
    dirty_before = $gitStatus
    dirty_after = $gitStatusAfter
    dirty_worktree_before = $dirty
    dirty_worktree_after = $gitStatusAfter.Count -gt 0
    worktree_stable = $worktreeStable
    git_fetch_remote = 'origin'
    git_fetch_exit_code = $fetchExit
    upstream = [string]$upstream
    expected_upstream = $expectedUpstream
    upstream_head = [string]$upstreamHead
    upstream_is_expected = $upstreamIsExpected
    upstream_matches_head = $upstreamMatchesHead
    fmt_sim_exe = [System.IO.Path]::GetFileName($simPath)
    fmt_sim_sha256 = (Get-Sha256 $simPath)
    settings_path = 'Content/Settings/settings.jason'
    settings_sha256 = (Get-Sha256 $settingsPath)
    gcc_dir_name = [System.IO.Path]::GetFileName($GccDir.TrimEnd([System.IO.Path]::DirectorySeparatorChar))
    fmt_hil_port = $FmtHilPort
    graphics_adapter = $FmtGraphicsAdapter
    hover = $FmtHover
    loopback_address_confirmed = [bool]$ConfirmLoopbackAddress
    state_timeout_ms = $FmtStateTimeoutMs
    scenarios_requested = $selectedScenarios
    tool_versions = $toolVersions
    build = [ordered]@{
        status = $buildStatus
        exit_code = $buildExit
        started_utc = $buildStarted.ToString('o')
        ended_utc = $buildEnded.ToString('o')
        executable = 'build/sil_flight.exe'
        executable_sha256 = $buildExeSha
        stdout_sha256 = (Get-Sha256 $buildStdout)
        stderr_sha256 = (Get-Sha256 $buildStderr)
    }
    scenarios = $scenarioResults
    logs = $logRecords
}
$manifestPath = Join-Path $runDir 'manifest.json'
$manifestJson = $manifest | ConvertTo-Json -Depth 12
[System.IO.File]::WriteAllText($manifestPath, $manifestJson + [Environment]::NewLine, [System.Text.UTF8Encoding]::new($false))

$summaryLines = @(
    '# FMT-SIM Local Regression',
    '',
    "- RUN_ID: $runId",
    "- Overall: $overall",
    "- Issue closure eligible: $strictGateEligible",
    "- Branch / HEAD: $branch / $head",
    "- Fetched origin successfully: $($fetchExit -eq 0)",
    "- Upstream matches HEAD: $upstreamMatchesHead",
    "- Worktree stable through run: $worktreeStable",
    "- FMT-SIM executable SHA256: $(Get-Sha256 $simPath)",
    "- Settings SHA256: $(Get-Sha256 $settingsPath)",
    "- HIL port / timeout: $FmtHilPort / $FmtStateTimeoutMs ms",
    "- Graphics adapter / hover: $FmtGraphicsAdapter / $FmtHover",
    "- Owner confirmed 127.0.0.1 loopback: $ConfirmLoopbackAddress",
    '',
    '| Scenario | Result | Exit | Checks | Failures | Longest state gap | settle lines |',
    '|---|---:|---:|---:|---:|---:|---:|'
)
foreach ($result in $scenarioResults) {
    $summaryLines += "| $($result.scenario) | $($result.status) | $($result.exit_code) | $($result.checks) | $($result.failures) | $($result.longest_successful_state_gap_ms) ms | $($result.settle_trace_lines) |"
}
$summaryLines += @(
    '',
    'The script starts and stops a fresh FMT-SIM process for each selected scenario.',
    'A PASS at a timeout above 350 ms, a dirty/changed worktree, an upstream mismatch, or a single-scenario run is diagnostic only and cannot close ENG-07 Issues #17/#18/#19.',
    '',
    "Evidence directory: $runDir"
)
$summaryPath = Join-Path $runDir 'summary.md'
[System.IO.File]::WriteAllLines($summaryPath, $summaryLines, [System.Text.UTF8Encoding]::new($false))

$sumPath = Join-Path $runDir 'SHA256SUMS.txt'
$evidenceFiles = Get-ChildItem -LiteralPath $runDir -File -Recurse |
    Where-Object { $_.FullName -ne $sumPath -and $_.Name -ne 'manifest.sha256' } |
    Sort-Object FullName
$sumLines = foreach ($file in $evidenceFiles) {
    $relative = $file.FullName.Substring($runDir.Length + 1).Replace('\', '/')
    "$(Get-Sha256 $file.FullName)  $relative"
}
[System.IO.File]::WriteAllLines($sumPath, @($sumLines), [System.Text.UTF8Encoding]::new($false))
$manifestSha = Get-Sha256 $manifestPath
[System.IO.File]::WriteAllText((Join-Path $runDir 'manifest.sha256'), "$manifestSha  manifest.json$([Environment]::NewLine)", [System.Text.UTF8Encoding]::new($false))

Write-Host "FMT-SIM local regression: $overall"
Write-Host "RUN_ID: $runId"
Write-Host "Evidence: $runDir"
Write-Host "Issue closure eligible: $strictGateEligible"
$archivePath = "$($runDir).zip"
Compress-Archive -Path (Join-Path $runDir '*') -DestinationPath $archivePath -CompressionLevel Optimal -Force
$archiveSha = Get-Sha256 $archivePath
[System.IO.File]::WriteAllText("$($archivePath).sha256", "$archiveSha  $([System.IO.Path]::GetFileName($archivePath))$([Environment]::NewLine)", [System.Text.UTF8Encoding]::new($false))
Write-Host "ARCHIVE=$archivePath"
Write-Host "ARCHIVE_SHA256=$archiveSha"
if ($overall -ne 'PASS') { exit 1 }
exit 0

[CmdletBinding()]
param(
    [string]$RunId = (Get-Date).ToUniversalTime().ToString("yyyyMMddTHHmmssZ"),
    [string]$CiRunUrl = "",
    [string]$InputBaselineCodeCommit = "",
    [string]$InputBaselineArchiveHead = ""
)

$ErrorActionPreference = "Stop"
$OutputEncoding = [System.Text.UTF8Encoding]::new($false)
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new($false)

$rootItem = Get-Item -LiteralPath $PSScriptRoot
while (-not (Test-Path -LiteralPath (Join-Path $rootItem.FullName "SConstruct"))) {
    if ($null -eq $rootItem.Parent) { throw "Flight repository root not found (SConstruct missing)." }
    $rootItem = $rootItem.Parent
}
$repoRoot = $rootItem.FullName

if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
    throw "git is required to create a traceable validation archive."
}
if ($RunId -notmatch '^[A-Za-z0-9._-]{6,64}$') {
    throw "RunId must be 6-64 characters and contain only letters, digits, dot, underscore or dash."
}

$archiveRoot = Join-Path $repoRoot "work/validation/archives"
$runDir = Join-Path $archiveRoot $RunId
$zipPath = Join-Path $archiveRoot ($RunId + ".zip")
if (Test-Path -LiteralPath $runDir) { throw "Archive directory already exists: $runDir" }
if (Test-Path -LiteralPath $zipPath) { throw "Archive ZIP already exists: $zipPath" }

New-Item -ItemType Directory -Force -Path $runDir | Out-Null

function Get-GitValue {
    param([string[]]$GitArgs)
    $output = & git -C $repoRoot @GitArgs 2>&1
    if ($LASTEXITCODE -ne 0) {
        return ("GIT_ERROR: " + ($output -join [Environment]::NewLine))
    }
    return (($output | ForEach-Object { "$_" }) -join [Environment]::NewLine).Trim()
}

$copyPaths = @(
    "build/m4_m5_implementation/evidence",
    "build/ci/m4-m5",
    "build/m4_host",
    "build/m5_host",
    "build/m4_m5_host",
    "build/m5_arm",
    "build/validation",
    "tests/guidance/evidence",
    "spec/validation/flight_acceptance_manifest.json",
    "spec/quality/issue-workflow.json",
    "spec/quality/ISSUE_REGISTER.md",
    "spec/quality/BLOCKER_BOARD.md",
    "spec/quality/ISSUE_WORKFLOW.md",
    "spec/history/implementation/guidance/M5_06_7_JERK_INTERVAL_PROOF.md",
    "spec/history/implementation/guidance/MR009_R17_REVIEW_MATERIAL.md",
    "spec/09_需求验证实施与发布管理.md"
)

$copied = @()
foreach ($relative in $copyPaths) {
    $source = Join-Path $repoRoot $relative
    if (-not (Test-Path -LiteralPath $source)) { continue }

    $destination = Join-Path $runDir $relative
    $destinationParent = Split-Path -Parent $destination
    New-Item -ItemType Directory -Force -Path $destinationParent | Out-Null
    Copy-Item -LiteralPath $source -Destination $destination -Recurse -Force
    $copied += $relative
}

# Preserve local tracked edits as a binary-capable patch. Inspect the archive
# before sharing it; never include credentials, tokens, private keys or secrets.
$patch = & git -C $repoRoot diff --binary HEAD 2>&1
if ($LASTEXITCODE -eq 0 -and $patch.Count -gt 0) {
    $patchPath = Join-Path $runDir "working_tree.patch"
    [System.IO.File]::WriteAllText(
        $patchPath,
        ($patch -join [Environment]::NewLine),
        [System.Text.UTF8Encoding]::new($false)
    )
}

$untracked = & git -C $repoRoot ls-files --others --exclude-standard 2>&1
if ($LASTEXITCODE -eq 0) {
    $untrackedRoot = Join-Path $runDir "working_tree_untracked"
    foreach ($relative in $untracked) {
        $relative = "$relative".Trim()
        if ([string]::IsNullOrWhiteSpace($relative)) { continue }
        if ($relative -match '(^|[\\/])(\.env($|\.)|secrets?($|[\\/])|credentials?($|[\\/]))|\.(pem|p12|pfx|key)$') {
            continue
        }
        $source = Join-Path $repoRoot $relative
        if (-not (Test-Path -LiteralPath $source -PathType Leaf)) { continue }
        $destination = Join-Path $untrackedRoot $relative
        New-Item -ItemType Directory -Force -Path (Split-Path -Parent $destination) | Out-Null
        Copy-Item -LiteralPath $source -Destination $destination -Force
    }
}

$branch = Get-GitValue -GitArgs @("branch", "--show-current")
$head = Get-GitValue -GitArgs @("rev-parse", "HEAD")
$status = Get-GitValue -GitArgs @("status", "--short", "--branch")
$commit = Get-GitValue -GitArgs @("show", "-s", "--format=%H%n%cs%n%s", "HEAD")
$remoteHead = Get-GitValue -GitArgs @("rev-parse", "origin/opt/stage-02-m5-06-safety-validator")

$pythonVersion = "NOT_AVAILABLE"
$pythonOutput = & python --version 2>&1
if ($LASTEXITCODE -eq 0) { $pythonVersion = ($pythonOutput -join " ").Trim() }

$wslVersion = "NOT_AVAILABLE"
if (Get-Command wsl.exe -ErrorAction SilentlyContinue) {
    $wslOutput = & wsl.exe -d Ubuntu-26.04 -- bash -lc "uname -srmo" 2>&1
    if ($LASTEXITCODE -eq 0) { $wslVersion = ($wslOutput -join " ").Trim() }
}

$evidenceFiles = @()
Get-ChildItem -LiteralPath $runDir -Recurse -File | ForEach-Object {
    $relative = $_.FullName.Substring($runDir.Length).TrimStart([IO.Path]::DirectorySeparatorChar)
    $hash = Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256
    $evidenceFiles += [ordered]@{
        path = $relative.Replace([string][char]92, "/")
        size_bytes = $_.Length
        sha256 = $hash.Hash.ToLowerInvariant()
    }
}

$codeCommit = "NOT_RECORDED"
try {
    $manifestPath = Join-Path $repoRoot "spec/validation/flight_acceptance_manifest.json"
    if (Test-Path -LiteralPath $manifestPath) {
        $manifestJson = Get-Content -LiteralPath $manifestPath -Raw | ConvertFrom-Json
        $codeCommit = $manifestJson.latest_local_validation.validated_code_commit
    }
} catch { $codeCommit = "NOT_RECORDED" }

$archiveManifest = [ordered]@{
    schema_version = "1.0"
    project = "Flight"
    run_id = $RunId
    created_utc = (Get-Date).ToUniversalTime().ToString("o")
    repository_root = $repoRoot
    branch = $branch
    head_sha = $head
    code_commit = $codeCommit
    archive_head_sha = $head
    archive_commit_sha = $head
    validated_code_commit = $codeCommit
    input_baseline_code_commit = $InputBaselineCodeCommit
    input_baseline_archive_head = $InputBaselineArchiveHead
    remote_m5_branch_sha_at_archive_time = $remoteHead
    head_commit = $commit
    git_status = $status
    powershell = $PSVersionTable.PSVersion.ToString()
    python = $pythonVersion
    wsl = $wslVersion
    ci_run_url = $CiRunUrl
    copied_evidence_paths = $copied
    evidence_files = $evidenceFiles
    scope_warning = "Archive contents are evidence only. PASS is not inferred from file presence. Review exact commands, exit codes, counts and commit binding."
    hardware_warning = "Host/SIL and Cortex-M7 compile evidence do not establish H723 runtime, WCET/RTA, HIL or flight authorization."
}

$manifestPath = Join-Path $runDir "archive_manifest.json"
$archiveManifest | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath $manifestPath -Encoding UTF8

$sumLines = @()
Get-ChildItem -LiteralPath $runDir -Recurse -File | Where-Object { $_.Name -ne "SHA256SUMS.txt" } | Sort-Object FullName | ForEach-Object {
    $hash = Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256
    $relative = $_.FullName.Substring($runDir.Length).TrimStart([IO.Path]::DirectorySeparatorChar).Replace([string][char]92, "/")
    $sumLines += ($hash.Hash.ToLowerInvariant() + "  " + $relative)
}
[System.IO.File]::WriteAllLines(
    (Join-Path $runDir "SHA256SUMS.txt"),
    [string[]]$sumLines,
    [System.Text.UTF8Encoding]::new($false)
)

# Re-verify every SHA256SUMS entry after writing.
foreach ($line in $sumLines) {
    $parts = $line -split '  ', 2
    if ($parts.Count -ne 2) { throw "Malformed SHA256SUMS line: $line" }
    $file = Join-Path $runDir $parts[1]
    if (-not (Test-Path -LiteralPath $file)) { throw "Missing evidence file: $file" }
    $actual = (Get-FileHash -LiteralPath $file -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($actual -ne $parts[0]) { throw "SHA256 mismatch: $file" }
}

Compress-Archive -LiteralPath $runDir -DestinationPath $zipPath -CompressionLevel Optimal
$zipHash = (Get-FileHash -LiteralPath $zipPath -Algorithm SHA256).Hash.ToLowerInvariant()
$zipName = [IO.Path]::GetFileName($zipPath)
$sidecarPath = $zipPath + ".sha256"
"$zipHash  $zipName" | Set-Content -LiteralPath $sidecarPath -Encoding ascii

$aheadCount = (& git -C $repoRoot rev-list --count "origin/opt/stage-02-m5-06-safety-validator..HEAD" 2>&1)
$externalReport = [ordered]@{
    schema_version = "1.0"
    run_id = $RunId
    code_commit = $codeCommit
    validated_code_commit = $codeCommit
    archive_head_sha = $head
    archive_commit_sha = $head
    input_baseline_code_commit = $InputBaselineCodeCommit
    input_baseline_archive_head = $InputBaselineArchiveHead
    remote_head = $remoteHead
    ahead_commit_count = ("$aheadCount".Trim())
    archive_zip_sha256 = $zipHash
    archive_zip_path = $zipPath
    archive_zip_size_bytes = (Get-Item -LiteralPath $zipPath).Length
    sidecar_path = $sidecarPath
    created_utc = (Get-Date).ToUniversalTime().ToString("o")
    scope = "LOCAL_EXECUTION_ONLY; not GitHub CI, H723 runtime, HIL or flight"
}
$externalReportPath = Join-Path $archiveRoot ($RunId + ".archive_report.json")
$externalReport | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath $externalReportPath -Encoding UTF8

Write-Output "FLIGHT_VALIDATION_ARCHIVE_CREATED"
Write-Output ("RUN_ID=" + $RunId)
Write-Output ("HEAD=" + $head)
Write-Output ("BRANCH=" + $branch)
Write-Output ("DIRECTORY=" + $runDir)
Write-Output ("ZIP=" + $zipPath)
Write-Output ("ZIP_SHA256=" + $zipHash)
Write-Output ("ZIP_SIDECAR=" + $sidecarPath)
Write-Output ("ARCHIVE_REPORT=" + $externalReportPath)
Write-Output "Review the ZIP for secrets before sharing it."

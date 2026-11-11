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

function Get-GitValue {
    param([string[]]$GitArgs)
    $output = & git -C $repoRoot @GitArgs 2>&1
    if ($LASTEXITCODE -ne 0) {
        return ("GIT_ERROR: " + ($output -join [Environment]::NewLine))
    }
    return (($output | ForEach-Object { "$_" }) -join [Environment]::NewLine).Trim()
}

# Prepare the in-ZIP acceptance manifest before copying evidence. The ZIP digest
# is intentionally external-only because a ZIP cannot contain its own SHA256.
$branch = Get-GitValue -GitArgs @("branch", "--show-current")
$head = Get-GitValue -GitArgs @("rev-parse", "HEAD")
$remoteRef = "origin/$branch"
$remoteHead = Get-GitValue -GitArgs @("rev-parse", $remoteRef)
$aheadCountText = Get-GitValue -GitArgs @("rev-list", "--count", "$remoteRef..HEAD")
if ($branch -ne "opt/stage-02-m5-06-safety-validator") { throw "Unexpected branch: $branch" }
if ($head -notmatch '^[0-9a-f]{40}$') { throw "Unable to resolve HEAD: $head" }
if ($remoteHead -notmatch '^[0-9a-f]{40}$') { throw "Unable to resolve remote HEAD: $remoteHead" }
if ($aheadCountText -notmatch '^\d+$') { throw "Unable to compute ahead count: $aheadCountText" }
$metadataArgs = @(
    (Join-Path $repoRoot "tools/prepare_archive_metadata.py"),
    "--repo", $repoRoot,
    "--manifest", "spec/validation/flight_acceptance_manifest.json",
    "--run-id", $RunId,
    "--expected-branch", $branch
)
$metadataOutput = & python @metadataArgs 2>&1
if ($LASTEXITCODE -ne 0) {
    throw "Archive metadata preparation failed: $($metadataOutput -join [Environment]::NewLine)"
}

New-Item -ItemType Directory -Force -Path $runDir | Out-Null

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
$patchPath = Join-Path $runDir "working_tree.patch"
$patchStderr = Join-Path $runDir "working_tree.patch.stderr.log"
$patch = & git -C $repoRoot diff --binary HEAD 2> $patchStderr
if ($LASTEXITCODE -eq 0 -and $patch.Count -gt 0) {
    $patchText = ($patch -join "`n").TrimEnd("`r", "`n") + "`n"
    [System.IO.File]::WriteAllText(
        $patchPath,
        $patchText,
        [System.Text.UTF8Encoding]::new($false)
    )
} else {
    Set-Content -LiteralPath $patchPath -Value "" -Encoding utf8NoBOM
}

# Every archive must carry a self-contained bundle with both HEAD and the
# named working branch. Never silently reuse a bundle for a different HEAD.
$bundleRelative = "build/validation/regenerated/$RunId.bundle"
$bundlePath = Join-Path $runDir $bundleRelative
New-Item -ItemType Directory -Force -Path (Split-Path -Parent $bundlePath) | Out-Null
if (-not (Test-Path -LiteralPath $bundlePath)) {
    $bundleOutput = & git -C $repoRoot bundle create $bundlePath HEAD "refs/heads/$branch" 2>&1
    if ($LASTEXITCODE -ne 0) {
        throw "Git Bundle creation failed: $($bundleOutput -join [Environment]::NewLine)"
    }
}
$bundleVerify = & git -C $repoRoot bundle verify $bundlePath 2>&1
if ($LASTEXITCODE -ne 0) {
    throw "Git Bundle verification failed: $($bundleVerify -join [Environment]::NewLine)"
}
$bundleRefs = & git -C $repoRoot bundle list-heads $bundlePath 2>&1
if ($LASTEXITCODE -ne 0) { throw "Git Bundle list-heads failed." }
$headRef = "$head HEAD"
$branchRef = "$head refs/heads/$branch"
if (($bundleRefs -notcontains $headRef) -or ($bundleRefs -notcontains $branchRef)) {
    throw "Git Bundle must contain exact refs: '$headRef' and '$branchRef'. Found: $($bundleRefs -join '; ')"
}
$bundleHash = (Get-FileHash -LiteralPath $bundlePath -Algorithm SHA256).Hash.ToLowerInvariant()

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

$status = Get-GitValue -GitArgs @("status", "--short", "--branch")
$commit = Get-GitValue -GitArgs @("show", "-s", "--format=%H%n%cs%n%s", "HEAD")

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
$manifestJson = $null
try {
    $manifestPath = Join-Path $repoRoot "spec/validation/flight_acceptance_manifest.json"
    if (Test-Path -LiteralPath $manifestPath) {
        $manifestJson = Get-Content -LiteralPath $manifestPath -Raw | ConvertFrom-Json
        $codeCommit = $manifestJson.latest_local_validation.validated_code_commit
    }
} catch { throw "Unable to read prepared acceptance manifest: $_" }
if ($null -eq $manifestJson -or $codeCommit -notmatch '^[0-9a-f]{40}$') {
    throw "Prepared acceptance manifest has no valid validated_code_commit."
}

$archiveManifest = [ordered]@{
    schema_version = "2.0"
    project = "Flight"
    run_id = $RunId
    created_utc = (Get-Date).ToUniversalTime().ToString("o")
    repository_root = $repoRoot
    branch = $branch
    head_sha = $head
    code_commit = $codeCommit
    validated_code_commit = $codeCommit
    tooling_commit = $manifestJson.latest_local_validation.tooling_commit
    collector_commit = $manifestJson.latest_local_validation.collector_commit
    archive_script_commit = $manifestJson.latest_local_validation.archive_script_commit
    collector_worktree_sha256 = $manifestJson.latest_local_validation.collector_worktree_sha256
    archive_script_worktree_sha256 = $manifestJson.latest_local_validation.archive_script_worktree_sha256
    tooling_source_manifest_sha256 = $manifestJson.latest_local_validation.tooling_source_manifest_sha256
    tooling_sources = $manifestJson.latest_local_validation.tooling_sources
    archive_head_sha = $head
    archive_commit_sha = $head
    remote_head = $remoteHead
    ahead_commit_count = [int]$aheadCountText
    archive_zip_hash_location = "EXTERNAL_ARCHIVE_REPORT_AND_SIDECAR"
    git_bundle = [ordered]@{
        path = $bundleRelative.Replace([string][char]92, "/")
        sha256 = $bundleHash
        refs = @($bundleRefs | ForEach-Object { "$($_)".Trim() })
    }
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

$externalReport = [ordered]@{
    schema_version = "2.0"
    run_id = $RunId
    branch = $branch
    code_commit = $codeCommit
    validated_code_commit = $codeCommit
    tooling_commit = $manifestJson.latest_local_validation.tooling_commit
    collector_commit = $manifestJson.latest_local_validation.collector_commit
    archive_script_commit = $manifestJson.latest_local_validation.archive_script_commit
    collector_worktree_sha256 = $manifestJson.latest_local_validation.collector_worktree_sha256
    archive_script_worktree_sha256 = $manifestJson.latest_local_validation.archive_script_worktree_sha256
    tooling_source_manifest_sha256 = $manifestJson.latest_local_validation.tooling_source_manifest_sha256
    tooling_sources = $manifestJson.latest_local_validation.tooling_sources
    archive_head_sha = $head
    archive_commit_sha = $head
    archive_commit_sha_equals_archive_head_sha = $true
    archive_commit_sha_equals_reason = "The archive commit is the final commit captured by the ZIP; its tree contains the evidence and archive manifest, so archive_commit_sha equals archive_head_sha. The code commit is an earlier commit."
    input_baseline_code_commit = $InputBaselineCodeCommit
    input_baseline_archive_head = $InputBaselineArchiveHead
    remote_head = $remoteHead
    ahead_commit_count = [int]$aheadCountText
    archive_zip_hash_location = "EXTERNAL_ARCHIVE_REPORT_AND_SIDECAR"
    archive_zip_sha256 = $zipHash
    archive_zip_path = $zipPath
    archive_zip_size_bytes = (Get-Item -LiteralPath $zipPath).Length
    sidecar_path = $sidecarPath
    archive_manifest = (Join-Path $runDir "archive_manifest.json")
    bundle = $bundleRelative.Replace([string][char]92, "/")
    bundle_path = $bundlePath
    bundle_sha256 = $bundleHash
    bundle_refs = @($bundleRefs | ForEach-Object { "$($_)".Trim() })
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

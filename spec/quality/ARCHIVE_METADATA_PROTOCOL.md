# Flight Validation Archive Metadata Protocol

Status: **IMPLEMENTED IN WORKING PATCH / WAITING PROJECT INTEGRATION TEST**

This protocol removes the archive self-reference and the post-archive metadata
commit pattern. It is mandatory for new validation runs; historical R1-R28
archives remain immutable.

## 1. Authoritative identities

The archive has four distinct Git identities:

- `validated_code_commit`: the exact code revision whose tests were executed.
- `tooling_commit`: latest committed revision affecting the validation collector,
  evidence checker, stack analyzer and metadata preparation helper.
- `collector_commit`: latest committed revision affecting
  `tools/collect_validation_results.py`.
- `archive_script_commit`: latest committed revision affecting the archive
  entrypoint/finalizer.
- `archive_head_sha`: current named-branch HEAD captured into the bundle.

These values must not be collapsed into one SHA. The worktree patch and its raw
file copies preserve any uncommitted evidence-only metadata edits.

## 2. Two-phase archive metadata

### Phase A — before ZIP creation

`tools/prepare_archive_metadata.py` reads the real Git repository and updates
`spec/validation/flight_acceptance_manifest.json` atomically. It records:

- RUN_ID and named branch;
- validated code, tooling, collector and archive-script commits;
- current archive HEAD and remote HEAD;
- dynamically computed `remote..HEAD` ahead count;
- external report and sidecar paths;
- `archive_zip_hash_location = EXTERNAL_ARCHIVE_REPORT_AND_SIDECAR`;
- `archive_zip_sha256 = null` and `archive_zip_size_bytes = null`.

The manifest update occurs before the archive script copies evidence and
captures `working_tree.patch`. No `PENDING_*` placeholder is allowed.

### Phase B — after ZIP creation

The archive script computes the final ZIP SHA256 and byte size and writes them
only to:

- `<RUN_ID>.zip.sha256`;
- `<RUN_ID>.archive_report.json`.

The ZIP's internal `archive_manifest.json` repeats Git identities and the
external-only digest policy, but must not contain its own ZIP SHA256 or size.

The archive report must match the in-ZIP manifest and acceptance manifest for
RUN_ID, branch, code/tooling/collector/archive-script commits, archive HEAD,
remote HEAD and ahead count. The checker independently hashes the ZIP, verifies
the sidecar, tests CRC, reads the internal manifest and verifies every listed
evidence file's SHA256 and size.

## 3. Forbidden patterns

The following are invalid for new runs:

- `PENDING_ARCHIVE_HEAD`, `PENDING_ARCHIVE_ZIP` or `ahead_commit_count: null`;
- stale manually copied ahead counts;
- hard-coded R25/R28 commit identities in a generic finalizer;
- writing the ZIP's own hash inside the ZIP;
- a metadata-only post-archive commit used to make the manifest appear current;
- treating `collector_commit == tooling_commit` as a universal invariant;
- accepting only field presence without comparing values to Git and the archive.

## 4. Required commands

Before archive staging:

```bash
python tools/prepare_archive_metadata.py \
  --run-id M5-06-Rxx-YYYYMMDD \
  --expected-branch opt/stage-02-m5-06-safety-validator
```

After the archive is built and the external report exists:

```bash
python tools/check_validation_evidence.py --check manifest
python tools/check_validation_evidence.py --check archive
python tools/check_validation_evidence.py --check all
```

A missing remote tracking ref, invalid Git SHA, missing path history, mismatched
branch, stale ahead count, missing ZIP, bad sidecar or internal/external metadata
mismatch must fail closed.

## 5. Historical treatment

R28's ZIP and external report remain unchanged. Its acceptance manifest
snapshot (`PENDING_ARCHIVE_*`, ahead 87), blocker board and handover values are
historical defects, not values to rewrite retroactively. R29 and later archives
must use this protocol and record the R28 defect in the issue register.

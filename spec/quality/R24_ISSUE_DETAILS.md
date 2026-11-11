# R24 detailed issue records: FLIGHT-LOCAL-35..41

## FLIGHT-LOCAL-35 — analysis_p2 historical/current conflict
- First seen: R21; current baseline: R23/R24.
- Root cause: old 32,840 B/3.6 KB sections remained unmarked while top summary changed.
- Owner: AGENT; safety impact: reviewer may use stale stack number.
- Fix: top R24 summary; sections 1/2/4/5 marked HISTORICAL/SUPERSEDED; raw data retained.
- Repro: `Select-String -Path build/validation/analysis_p2.md -Pattern '^## '`.
- Expected: only R24 summary current; all old values historical.
- Actual: current 128 B map, 4,248 B validator; old 32,840 B/3.6 KB marked.
- Log: `build/validation/analysis_p2.md`; SHA256 in R24 archive.
- Unblock: independent reviewer confirms no stale number remains.
- Next check: R24 review.
- Status: FIXED_WAITING_RETEST.

## FLIGHT-LOCAL-36 — archive/metadata definitions
- First seen: R21; current: R23/R24.
- Root cause: archive_head/commit/zip/ahead fields mixed; external report lacked equality field.
- Owner: AGENT + OWNER (archive semantics); impact: non-reproducible archive metadata.
- Fix: archive report has archive_commit_sha, equality boolean/reason, input baseline, remote, integer ahead; ZIP hash external only.
- Repro: `Get-Content work/validation/archives/M5-06-R24-20261111.archive_report.json`.
- Expected: validated_code_commit, archive head/commit, remote, integer ahead, ZIP SHA/size.
- Actual: R24 report generated; status BLOCKED pending independent check.
- Log: external archive report; SHA256 in sidecar.
- Unblock: independent reviewer verifies fields against Git/ZIP.
- Next check: R24 review.
- Status: BLOCKED.

## FLIGHT-LOCAL-37 — environment evidence
- First seen: R22; current R23/R24.
- Root cause: old environment.txt dated 2026-11-04 and missing exact toolchain/durations.
- Owner: AGENT; impact: non-reproducible test environment.
- Fix: `r24_environment.json` with Host/ARM/Windows versions and validation durations; old file marked HISTORICAL.
- Repro: `python -m json.tool build/validation/regenerated/r24_environment.json`.
- Expected/actual: complete fields; status FIXED_WAITING_RETEST.
- Log: r24_environment.json; SHA256 in archive.
- Unblock: independent reviewer confirms versions match commands.
- Next check: R24 review.

## FLIGHT-LOCAL-38 — per-test evidence
- First seen: R22; current R23/R24.
- Root cause: results collector hardcoded values and some logs did not bind final files.
- Owner: AGENT; impact: tests could appear PASS without reproducible evidence.
- Fix: `tools/collect_validation_results.py` parses actual logs/Git; `r24_test_results.json` has 28 records; logs/SHA bound.
- Repro: `python tools/collect_validation_results.py --self-test`; `--run-id ...`.
- Expected: 11 negative tests pass; all PASS have exit 0/failures 0/checks>0; actual: verified.
- Log: `build/validation/regenerated/collector_selftest.log`; SHA256 in archive.
- Unblock: independent reviewer confirms no hardcoded counts and all SHAs.
- Next check: R24 review.
- Status: BLOCKED.

## FLIGHT-LOCAL-39 — S06-112 JSON
- First seen: R22; root cause: PASS text appended to JSON.
- Fix: R23/R24 `s06_112_r24.json` JSON-only, `s06_112_r24.log` separate; six R14b exact records checked.
- Repro: `python -c "import json; json.load(open(...))"`.
- Actual: JSON_OK; 6/6 exact; 0 failures.
- Unblock: independent math reviewer parses JSON and confirms six Fractions.
- Status: FIXED_WAITING_RETEST (independent support recorded; not CLOSED).

## FLIGHT-LOCAL-40 — collector hardcoding
- First seen: R23; root cause: S06-108/113/product/issues/stack counts and statuses embedded.
- Fix: collector reads validation.json/logs/JSON/issue register; counts parsed; negative self-tests 11, run under `python` and `python -O`; log SHA.
- Repro: `python tools/collect_validation_results.py --self-test`; `python -O ...`.
- Actual: both PASS (11 cases).
- Unblock: independent review confirms no hardcoded results; status BLOCKED.
- Next check: R24 review.

## FLIGHT-LOCAL-41 — R22 worktree/archive tool
- First seen: R23; root cause: archive patch mixed `git diff` stderr warnings and did not prove byte-exact recovery.
- Fix: patch stdout/stderr separated; raw modified/untracked copies stored under `build/validation/regenerated/r24_worktree/`; patch apply-check run; ABI failure evidence retained; no reset.
- Repro: `git apply --check working_tree.patch` in clean worktree; SHA comparison documented in `R24_WORKTREE_REVIEW.md`.
- Actual: patch applies; CRLF normalization means byte-exact hashes require raw copies (documented).
- Unblock: independent reviewer verifies patch/copy/SHA; status BLOCKED.
- Next check: R24 review.

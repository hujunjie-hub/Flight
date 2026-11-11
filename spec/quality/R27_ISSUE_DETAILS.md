# R27 issue details FLIGHT-LOCAL-35..41

RUN_ID `M5-06-R27-20261111`; validated code `b2a4c3779e9a2ccc22aebe209dce435b74ccc7d6`; tooling `cf4b3e3b7d9c50abd019711725891a64c7b592fc`.

## FLIGHT-LOCAL-35
- Root cause/current: R26/R27 evidence issue.
- Owner: AGENT; independent acceptance: ChatGPT.
- Safety impact: evidence reproducibility, not flight release.
- Fix: analysis resource summary R27; old 3.6KB/32840 marked HISTORICAL
- Command: see R27 HANDOVER and r27_test_results.json.
- Expected/actual: see test status; no CLOSED.
- Log: `build/validation/analysis_p2.md`; SHA256 `677a4dd09b6633110a6c8eede0e9418f28e4761d63db70abb99694417fc4d46e`.
- Unblock: independent reviewer verifies R27 artifacts.
- Next checkpoint: R27 review.
- Status: FIXED_WAITING_RETEST.

## FLIGHT-LOCAL-36
- Root cause/current: R26/R27 evidence issue.
- Owner: AGENT; independent acceptance: ChatGPT.
- Safety impact: evidence reproducibility, not flight release.
- Fix: archive report fields must include tooling/collector/script and match manifest
- Command: see R27 HANDOVER and r27_test_results.json.
- Expected/actual: see test status; no CLOSED.
- Log: `work/validation/archives/M5-06-R27-20261111.archive_report.json`; SHA256 `MISSING`.
- Unblock: independent reviewer verifies R27 artifacts.
- Next checkpoint: R27 review.
- Status: BLOCKED.

## FLIGHT-LOCAL-37
- Root cause/current: R26/R27 evidence issue.
- Owner: AGENT; independent acceptance: ChatGPT.
- Safety impact: evidence reproducibility, not flight release.
- Fix: r27_environment binds Host/WSL/ARM/Windows, CFLAGS, sanitizer, durations
- Command: see R27 HANDOVER and r27_test_results.json.
- Expected/actual: see test status; no CLOSED.
- Log: `build/validation/regenerated/r27_environment.json`; SHA256 `b0843d17304282357e3f7e6b1fc9331b8580ca22ea857d46012f484d499fe397`.
- Unblock: independent reviewer verifies R27 artifacts.
- Next checkpoint: R27 review.
- Status: FIXED_WAITING_RETEST.

## FLIGHT-LOCAL-38
- Root cause/current: R26/R27 evidence issue.
- Owner: AGENT; independent acceptance: ChatGPT.
- Safety impact: evidence reproducibility, not flight release.
- Fix: r27_test_results source/log/env SHA and 29 records; independent review pending
- Command: see R27 HANDOVER and r27_test_results.json.
- Expected/actual: see test status; no CLOSED.
- Log: `build/validation/regenerated/r27_test_results.json`; SHA256 `3c09e4ee8a7c0c26228eed1f3d5c8521bc2bc9431667298376564237d6235f32`.
- Unblock: independent reviewer verifies R27 artifacts.
- Next checkpoint: R27 review.
- Status: BLOCKED.

## FLIGHT-LOCAL-39
- Root cause/current: R26/R27 evidence issue.
- Owner: AGENT; independent acceptance: ChatGPT.
- Safety impact: evidence reproducibility, not flight release.
- Fix: s06_112_r27 JSON parses; 6/6 R14b; 0 failures
- Command: see R27 HANDOVER and r27_test_results.json.
- Expected/actual: see test status; no CLOSED.
- Log: `build/validation/regenerated/s06_112_r27.json`; SHA256 `f50d6a9663004579affb4e76b14c4010b0abf510858d67e05c1ade008e4c132a`.
- Unblock: independent reviewer verifies R27 artifacts.
- Next checkpoint: R27 review.
- Status: FIXED_WAITING_RETEST.

## FLIGHT-LOCAL-40
- Root cause/current: R26/R27 evidence issue.
- Owner: AGENT; independent acceptance: ChatGPT.
- Safety impact: evidence reproducibility, not flight release.
- Fix: collector 16 negative tests; PASS requires real metrics/source/log/env
- Command: see R27 HANDOVER and r27_test_results.json.
- Expected/actual: see test status; no CLOSED.
- Log: `tools/collect_validation_results.py`; SHA256 `130de10fab9cac8be748a4ea7b3829bd4792c908c60a69d4e8c30a4b80062b67`.
- Unblock: independent reviewer verifies R27 artifacts.
- Next checkpoint: R27 review.
- Status: BLOCKED.

## FLIGHT-LOCAL-41
- Root cause/current: R26/R27 evidence issue.
- Owner: AGENT; independent acceptance: ChatGPT.
- Safety impact: evidence reproducibility, not flight release.
- Fix: R27 worktree patch/copies preserved; R22 bytes not recovered
- Command: see R27 HANDOVER and r27_test_results.json.
- Expected/actual: see test status; no CLOSED.
- Log: `build/validation/regenerated/r27_worktree/working_tree.patch`; SHA256 `MISSING`.
- Unblock: independent reviewer verifies R27 artifacts.
- Next checkpoint: R27 review.
- Status: BLOCKED.


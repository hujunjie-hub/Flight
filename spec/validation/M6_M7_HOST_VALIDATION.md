# M6/M7 Host validation record

> Historical sections below describe the earlier standalone `test/fmat-0012-m6-controller` branch at the time of each recorded run. Their branch-lineage and PR-state statements are historical snapshots. The current integrated main branch result is recorded in the final section below.

## Baseline

- Repository: `hujunjie-hub/Flight`
- Branch: `test/fmat-0012-m6-controller`
- Baseline HEAD before CI bootstrap: `b2f33ee00fd7cc36fb51803d38ab0ff1267cbf0f`
- Scope: M6 MPC model path and input/time-context guard; M7 actuator transaction and ground-stop guard; M4/M5 host regression.

## Validation entry points

Run from the repository root:

```bash
bash tests/flight_management/run_host_tests.sh
bash tests/guidance/run_host_tests.sh
bash tests/guidance/run_integration_tests.sh
bash tests/control/run_host_tests.sh
bash tests/actuation/run_host_tests.sh
```

The M6 runner compiles the real controller translation units and generated OSQP workspace. The M7 runner compiles the real actuator output guard with `FLIGHT_HOST_TEST`; it does not substitute a parallel implementation.

## First complete CI run

- Commit: `13f1e6b87fbe449c0a1348f35b985c9b2a824bc0`
- Workflow run: [43362327368](https://github.com/hujunjie-hub/Flight/actions/runs/43362327368)
- Overall result: **SUCCESS** (3/3 jobs)

| Gate | Result | Checks | Failures |
|---|---|---:|---:|
| M4 host | PASS | 993 | 0 |
| M5 host | PASS | 9,515 | 0 |
| M4/M5 integration | PASS | 1,066 | 0 |
| M6 MPC | PASS | 13 | 0 |
| M6 input/time guard | PASS | 11 | 0 |
| M7 actuator transaction | PASS | 230 | 0 |
| M6/M7 ASan + UBSan | PASS | Re-ran M6/M7 suites | 0 |
| Cortex-M7 target build | NOT_RUN | — | — |
| HIL / DShot waveform / flight | NOT_RUN | — | — |

The 11,828 checks above are the unique M4/M5/M6/M7 Host checks; the sanitizer job repeats the M6/M7 tests and is reported separately. The first run proves the new test runners and current Host baseline work. It does not yet cover the newly identified M6 reset/deadline-accounting regression cases; those are added in the following commit.

## Second CI run — M6 lifecycle and deadline accounting

- Commit: `2fea8ce32316efde2ebcabfb367769b2466fab4b`
- Workflow run: [43362749296](https://github.com/hujunjie-hub/Flight/actions/runs/43362749296)
- Overall result: **SUCCESS** (3/3 jobs)

| Gate | Result | Checks | Failures |
|---|---|---:|---:|
| M4 host | PASS | 993 | 0 |
| M5 host | PASS | 9,515 | 0 |
| M4/M5 integration | PASS | 1,066 | 0 |
| M6 MPC | PASS | 15 | 0 |
| M6 input/time + outer lifecycle | PASS | 26 | 0 |
| M7 actuator | PASS | 230 | 0 |
| M6/M7 ASan + UBSan | PASS | Re-ran M6/M7 suites | 0 |

M6 now verifies OSQP absolute termination is applied, reset clears run statistics, the outer update rejects NULL, actuator telemetry TTL does not cap the MPC plan, and a 2 ms deadline overrun is not counted as solved or covered. The deadline case returns `STATUS_DEADLINE_MISSED`, publishes an expired snapshot and increments the deadline/rejected counters.

M7 certificate epoch binding, in-flight permit blocking and late-apply overrun cases are added in the following commit; they remain pending until their own CI run.

## Third CI run — M7 ground-stop transaction hardening

- Commit: `d8e2d9afa7c070f72616f34d36764fcb2b215874`
- Workflow run: [43363536926](https://github.com/hujunjie-hub/Flight/actions/runs/43363536926)
- Overall result: **SUCCESS** (3/3 jobs)

| Gate | Result | Checks | Failures |
|---|---|---:|---:|
| M4 host | PASS | 993 | 0 |
| M5 host | PASS | 9,515 | 0 |
| M4/M5 integration | PASS | 1,066 | 0 |
| M6 MPC | PASS | 15 | 0 |
| M6 input/time + outer lifecycle | PASS | 26 | 0 |
| M7 actuator / ground stop | PASS | 254 | 0 |
| M6/M7 ASan + UBSan | PASS | Re-ran M6/M7 suites | 0 |

The third run verifies certificate authority/directive epoch binding, rejection of new permits and demands while the ramp owns output, actual driver-apply deadline reporting, and a late final zero frame that must retain the OVERRUN flag. The 11,869 unique Host checks pass; the sanitizer job repeats M6/M7 tests.

The concurrency follow-up is committed as `907ce7fac83f476fa0d840004a9559f220d7e9a7`; CI run [43363758248](https://github.com/hujunjie-hub/Flight/actions/runs/43363758248) completed SUCCESS (3/3 jobs). It serializes the terminal ground-stop latch, permit updates, and actuator stop/reset through the same lock.

## Fifth CI run — require permit revocation before ground stop

- Commit: `80510c7d6575ec728663118a3b9e403706e2c74f`
- Workflow run: [43363924238](https://github.com/hujunjie-hub/Flight/actions/runs/43363924238)
- Overall result: **SUCCESS** (3/3 jobs)

| Gate | Result | Checks | Failures |
|---|---|---:|---:|
| M4 host | PASS | 993 | 0 |
| M5 host | PASS | 9,515 | 0 |
| M4/M5 integration | PASS | 1,066 | 0 |
| M6 MPC | PASS | 15 | 0 |
| M6 input/time + outer lifecycle | PASS | 26 | 0 |
| M7 actuator / ground stop | PASS | 255 | 0 |
| M6/M7 ASan + UBSan | PASS | Re-ran M6/M7 suites | 0 |

M7 now rejects an epoch-matching ground-stop certificate while the CONTROL_ACTIVE permit is still live. The certificate must match the current permit's authority/directive epochs and the current permit must already carry `PERMIT_FLAG_REVOKED`. The 11,870 unique Host checks pass; Sanitizer repeats M6/M7 without failures.

## Safety boundary

Host tests prove only the exercised software paths. They do not establish target WCET, board integration, DShot waveform timing, physical motor response, HIL acceptance, or flight readiness. The production H723 build remains `BENCH_INHIBIT`; `SIM_CONTROLLED_STOP` is enabled only for the host test build.


## M4→M7 ground-stop integration status (2026-11-13)

The production software path is now connected in this branch:

- `task_mission_step()` runs the FMS policy, translates an eligible ground-only disarm output with `fms_build_ground_stop_certificate()`, and publishes the certificate in `ControlBundle.ground_stop_certificate`.
- `task_actuator_step()` consumes the certificate from the runtime bundle, suppresses queued M6 demands while the stop owns output, calls `actuator_guard_ground_stop()`, advances the frame-boundary owner, and publishes the resulting actuator receipt.
- Certificate publication now uses the shared `task_mission_update_ground_stop_certificate()` helper in both the production mission owner and the SIL fixture. The fixture checks valid publication, bounded retention only when there is no current FMS output, immediate clearing when a current in-air output supersedes the ground decision, expiry clearing, and republishing.
- The SIL task-owner scenario now first supplies a valid 5 N M6 demand alongside an invalid in-air ground-stop candidate. It asserts that the invalid candidate does not suppress the demand. It then revokes the permit, publishes a valid landed/disarmed certificate through the shared helper, and drives the real actuator owner from the committed 5 N frame through multiple bounded ramp frames to the zero-target receipt.
- The SIL fixture still supplies a constructed `FmsOutputView`; it does **not** drive `task_mission_step()` / `fms_policy_update()` through a live FMS input and command/lifecycle sequence. Full live FMS-task-to-actuator-owner integration coverage therefore remains incomplete.

### Driver apply-time evidence

`actuator_device_commit()` now treats `rt_device_write()` as DMA submission only and returns no apply timestamp. `actuator_device_poll_apply()` waits until the DShot driver reports the submitted frame completed, then samples the monotonic clock. This is a conservative post-completion timestamp, not a hardware-captured timestamp of the exact DShot waveform edge. H723 runtime timing and waveform evidence remain NOT_RUN.

### Latest integration regression — Run #19

- Branch: `test/fmat-0012-m6-controller`
- Base HEAD: `467c99904488943ba8861ae84cc75dc74a2b332a`
- Workflow: [Run #19](https://github.com/hujunjie-hub/Flight/actions/runs/43369239983)
- M4/M5 regression: PASS.
- M6 MPC / M7 actuator Host tests: PASS.
- M6/M7 ASan/UBSan: PASS.
- Runtime task / ground-stop SIL: FAILED one assertion: the state was no longer `ACTUATOR_GUARD_DISARMED` after the zero-target receipt.

Root cause: after the ground-stop zero frame was confirmed, `actuator_poll()` treated the expected revoked permit as an ordinary unbound permit and called `safe(..., ACTUATOR_GUARD_SAFE_ACTION)`. That downgraded the terminal DISARMED latch. The correction preserves DISARMED after `gs.latched`, clears demand ownership, and reports no actuator capability after the terminal stop. The M7 regression test now polls status and capability after the zero receipt and checks that subsequent frame boundaries cannot leave DISARMED or commit another frame.

**Status: FIXED / CI_PASS.**

- Fix commit: `764493751290d0cad56897e1a4dcee0f3ee54755`.
- Revalidation: [Run #20](https://github.com/hujunjie-hub/Flight/actions/runs/43374017388), exact fix commit, **4/4 jobs SUCCESS**.
- M4 Host: 993 checks / 0 failures.
- M5 Host: 9,515 checks / 0 failures.
- M4/M5 Integration: 1,066 checks / 0 failures.
- M6 MPC: 15 checks / 0 failures; M6 guard/lifecycle: 26 checks / 0 failures.
- M7 actuator guard: 274 checks / 0 failures, including post-receipt polling and capability-latch regressions.
- Runtime task ground-stop SIL: 13 checks / 0 failures; one frame commit, zero-target receipt, terminal DISARMED state and apply-time budget all passed.
- M6/M7 ASan + UBSan: PASS, reran M6/M7 suites with 0 failures.

The CI failure on Run #19 is closed for this Host/SIL path. This does not close the remaining acceptance gates below.

## Remaining acceptance gates (not implied by Run #20)

1. **Live FMS-task integration coverage — OPEN, partially improved.** The M4 Host test now feeds the actual `fms_policy_update()` output for first-reference-window ground disarm into `fms_build_ground_stop_certificate()`, proving the real FMS decision/permit pair produces a valid certificate. The SIL smoke exercises the production mission publication helper and real actuator owner. It still does not drive the live `task_mission_step()` / `runtime_data_take_requests()` path with a real command/lifecycle sequence; that full task-to-task integration remains open.
2. **DShot apply-time characterization — PARTIAL.** `actuator_device_commit()` treats `rt_device_write()` as DMA submission. `actuator_device_poll_apply()` waits until the driver reports the submitted frame completed, then samples the monotonic clock. This is a conservative post-completion timestamp, not a hardware-captured timestamp of the exact DShot waveform edge. Validate the bound on H723 or document the approved timing semantics in the interface contract.
3. **Target and physical validation — NOT_RUN.** Cortex-M7/H723 target build, H723 WCET/RTA, target RAM/stack watermark, DShot waveform timing, physical ESC response, HIL, sensor calibration and staged flight validation remain outstanding.
4. **Separate M5-06 acceptance gate — BLOCKED.** The M5-06 PR #2 still requires independent mathematical review of Hermite→Bernstein conversion, derivative convex-hull bounds and outward interval assumptions, plus broader randomized dynamic uncertainty/acceleration-bound proof coverage. The M4/M5 regression passing here is not independent M5-06 approval; M5-07/physical release must not be inferred from this Host/SIL result.

Production `BENCH_INHIBIT` remains unchanged. Host/SIL evidence is not H723 target build, HIL, physical ESC response, or flight authorization.


### Mission certificate publication seam regression — Run #23 PASS

The SIL smoke calls the same certificate-publication helper used by `task_mission_step()`. Run #23 verified the then-current helper behavior, including retaining an unexpired certificate after an in-air output. A subsequent safety review found that this cache behavior was too permissive: a current, explicitly ineligible FMS output must revoke the old certificate immediately. The follow-up correction is tracked below. Run #22 initially failed to compile because the new helper was declared `void` while the test asserted its result, and the SIL `CHECK` macro accepts one argument only. Commit `a8f62f5b270355d699b8a42861198cfa93894337` corrected both compile issues.

[Run #23](https://github.com/hujunjie-hub/Flight/actions/runs/43375102383), exact commit `a8f62f5b270355d699b8a42861198cfa93894337`: **4/4 jobs SUCCESS**. M4 Host 993/0, M5 Host 9,515/0, M4/M5 integration 1,066/0, M6 MPC 15/0, M6 guard 26/0, M7 guard 274/0, SIL publication/owner smoke 18/0, and ASan/UBSan PASS. This remains a seam test rather than a live `fms_policy_update()` command/lifecycle integration test.

### Branch lineage / integration gate

At the time of this check, M6 branch HEAD `63fa80e960a752bcef97887a16a22a38905bfad2` and M5-06 branch HEAD `51e7e923419733f5aa3b21efa1e46588629623ed` are **diverged**, with common ancestor `edf32b4e39cdae014aa76870dc821882807a6905`. GitHub compare reports M6 ahead by 57 commits and behind M5-06 by 395 commits. The M6/M7 Host CI therefore validates this independent branch only; it does not contain or supersede the current M5-06 branch. Do not rebase, merge, or claim unified milestone acceptance without an explicit integration plan and independent M5-06 approval.


### Invalid certificate demand-continuity regression — Run #25 PASS

The actuator owner now suppresses a queued M6 demand only when `actuator_guard_ground_stop()` returns `STATUS_OK`. A malformed, stale or unauthorized certificate candidate cannot take ownership of output. The SIL fixture proves that an invalid in-air candidate does not block a valid 5 N demand, then verifies that a subsequently accepted certificate drives a multi-frame 5 N→0 N ground-stop ramp and terminal zero receipt.

[Run #25](https://github.com/hujunjie-hub/Flight/actions/runs/43375635462), exact commit `41ba12f2d1fbb3b4525407f5b621db8cddc6dbc4`: **4/4 jobs SUCCESS**. M4 Host 993/0, M5 Host 9,515/0, M4/M5 integration 1,066/0, M6 MPC 15/0, M6 guard 26/0, M7 guard 274/0, and ASan/UBSan PASS. The runtime SIL scenario completed **19 checks / 0 failures**, with **18 frame commits** (one 5 N demand plus the bounded ramp frames) and 58 inhibit calls; the final receipt confirmed zero thrust without an apply-time overrun.


### Superseded FMS ground-stop certificate revocation — Run #27 PASS

A current FMS output that is not eligible for a ground-stop certificate now clears the cached certificate immediately. Only an absence of current FMS output may retain the previous certificate, and then only until its original expiry. The SIL fixture verifies no-output bounded retention, immediate clearing on a current IN_AIR output, expiry clearing, and fresh republishing. This closes the stale-ground-decision window in the mission certificate cache.

[Run #27](https://github.com/hujunjie-hub/Flight/actions/runs/43376273060), exact commit `1bbc3d8d716a723dabc51528b86c3d6fc17be7d2`: **4/4 jobs SUCCESS**. The SIL runtime ground-stop scenario completed **27 checks / 0 failures**, with 18 frame commits (one 5 N demand plus the bounded ramp frames) and 58 inhibit calls. M4 Host 993/0, M5 Host 9,515/0, M4/M5 integration 1,066/0, M6 MPC 15/0, M6 guard 26/0, M7 guard 274/0, and ASan/UBSan PASS.


### Real FMS output → ground-stop certificate adapter test — Run #30 PASS

The M4 Host test `test_first_reference_window_expires_to_ground_stop` validates the real FMS first-reference timeout output: LANDED state, DISARMED state, revoked permit reason, forced-safe decision reason, and a successful `fms_build_ground_stop_certificate()` conversion with matching authority/directive epochs. Run #29 initially exposed that `run_host_tests.sh` did not link the production `ground_stop_adapter.c`; commit `63fa80e960a752bcef97887a16a22a38905bfad2` added the production adapter source to the Host link.

[Run #30](https://github.com/hujunjie-hub/Flight/actions/runs/43376885571), exact commit `63fa80e960a752bcef97887a16a22a38905bfad2`: **4/4 jobs SUCCESS**. M4 Host 1,002/0, M5 Host 9,515/0, M4/M5 integration 1,075/0, M6 MPC 15/0, M6 guard 26/0, M7 guard 274/0, runtime SIL 27/0, and ASan/UBSan PASS. This closes the synthetic-output gap at the FMS-policy-to-adapter boundary. The complete `task_mission_step()` queue/scheduling path remains open.


## Latest branch handover — CI #31

- Checked at: 2026-11-13 14:39 +08:00.
- Latest active branch: `test/fmat-0012-m6-controller`.
- Exact branch HEAD: `f8a8211bb690fc90cf69d34200af32976f1fcdef`.
- Latest commit: `docs(validation): record real FMS certificate adapter run 30`.
- Latest workflow: [M6/M7 Host validation #31](https://github.com/hujunjie-hub/Flight/actions/runs/43377043882), exact HEAD above, **4/4 jobs SUCCESS**.

### Run #31 observed results

| Gate | Result |
|---|---:|
| M6 MPC Host | 15 checks / 0 failures |
| M6 input/time + outer lifecycle guard | 26 checks / 0 failures |
| M7 actuator guard Host | 274 checks / 0 failures |
| M4 Host | 1,002 checks / 0 failures |
| M5 Host | 9,515 checks / 0 failures |
| M4/M5 integration | 1,075 checks / 0 failures |
| Runtime task / ground-stop SIL | 27 checks / 0 failures; 18 frame commits, 58 inhibit calls |
| M6/M7 ASan + UBSan | PASS; M6/M7 suites re-run without failures |

Run #29's M4 Host failure was caused by the Host runner omitting the production `ground_stop_adapter.c` translation unit. Commit `63fa80e960a752bcef97887a16a22a38905bfad2` linked the production adapter; Run #30 and Run #31 then passed. This closes the adapter-link/test-fixture gap, not the remaining full task-flow acceptance gate.

### Current branch lineage (rechecked at HEAD above)

- M6 branch vs M5-06 branch `51e7e923419733f5aa3b21efa1e46588629623ed`: **diverged**; M6 is 58 commits ahead and 395 behind; merge base `edf32b4e39cdae014aa76870dc821882807a6905`.
- M6 branch vs M7 branch `test/fmat-0013-m7-actuation` at `24eee8acb3c10b79b8e306b00b1725d3fbc2614f`: **diverged**; M6 is 52 commits ahead and 2 behind; merge base `b4ce6d0b3a2429d186fd5ec0563453322731db9b`.
- These are independent workstreams. Do not merge/rebase either lineage or claim cross-branch acceptance without a reviewed integration plan.

### Remaining work — priority order

1. **P0 — Live FMS-task-to-actuator-owner SIL integration remains OPEN.** The production code path exists: `task_mission_step()` calls `fms_policy_update()`, creates a certificate from its real output and publishes it; `task_actuator_step()` consumes the certificate and invokes the M7 guard. Current SIL proves the real actuator owner and shared certificate-publication helper, but its fixture still constructs `FmsOutputView`; it does not drive the real request queue, mission task scheduling, FMS command/lifecycle sequence and actuator owner as one end-to-end scenario. Add that test before calling the task-to-task path verified.
2. **P0 — DShot apply-time characterization is PARTIAL.** The timestamp is sampled after the driver reports DMA/frame completion; it is conservative software observation, not a hardware-captured DShot edge timestamp. Characterize the bound on H723 and document the measured timing contract.
3. **P1 — Target/physical validation is NOT_RUN.** H723/Cortex-M7 product build, WCET/RTA, target RAM/stack watermark, DShot waveform, ESC response, HIL, sensor calibration and staged flight validation remain outstanding. Keep `BENCH_INHIBIT`.
4. **P1 — M5-06 independent safety approval is BLOCKED.** Independent mathematical review of Hermite→Bernstein conversion, derivative convex-hull bounds and outward interval assumptions, plus broader randomized dynamic uncertainty/acceleration proof coverage, remains open on the separate M5-06 branch. M4/M5 regression passing on this branch is not M5-06 approval.

The M7 PR #4 remains Draft on its own branch and its stated integration blockers are branch-specific; it must not be treated as evidence that the newer M6 branch lacks the production adapter/owner code. Conversely, the M6 branch's Host/SIL pass does not close the physical and independent-review gates above.


## Live FMS-to-M7 task integration — Run #33

- Exact tested commit: `f28b7b72500cf892e12c757022819dca5f6f0d66`.
- Workflow: [M6/M7 Host validation #33](https://github.com/hujunjie-hub/Flight/actions/runs/43381782424), **4/4 jobs SUCCESS**.
- The new `--live-fms-ground-stop-smoke` drives fresh navigation/actuator/control contracts and operator input through `runtime_submit_request()`, the real request mailbox, `task_mission_step()`, `fms_policy_update()`, the production ground-stop certificate adapter, and `task_actuator_step()`. It does not manufacture an `FmsOutputView`.
- Live scenario evidence: **455 checks / 0 failures**; LandDetector reported LANDED, PreArm reached READY, the real ground DISARM command was accepted, the mission owner published a certificate with matching authority/directive epochs, and the actuator owner produced `ACTUATOR_RECEIPT_ZERO_TARGET_APPLIED`.
- Existing non-zero-to-zero ramp scenario remains separate: **27 checks / 0 failures**, 18 frame commits, 58 inhibit calls.
- Regression gates on the same commit: M4 Host 1,002/0; M5 Host 9,515/0; M4/M5 integration 1,075/0; M6 MPC 15/0; M6 guard 26/0; M7 guard 274/0; M6/M7 ASan/UBSan PASS.

### Updated acceptance status after Run #33

1. **Live FMS-task-to-actuator-owner path for explicit ground DISARM: PASS (Host/SIL).** The prior queue/scheduling gap is closed for this command path.
2. **First-reference-timeout ground-disarm through the live task queue: OPEN (P0).** M4 Host validates the real FMS timeout decision and certificate adapter, but the new task-level SIL scenario currently covers explicit ground DISARM. Add a separate live timeout scenario with no accepted first reference before the configured deadline.
3. **DShot apply-time characterization: PARTIAL (P0).** Host observes the driver completion callback; H723 hardware capture is still required to bound actual output-application timing.
4. **Target and physical validation: NOT_RUN (P1).** H723 product build, WCET/RTA, RAM/stack watermark, waveform/ESC response, HIL, calibration and staged flight remain outstanding. Keep `BENCH_INHIBIT`.
5. **M5-06 independent safety approval: BLOCKED (P1).** The separate branch's mathematical and dynamic-uncertainty review remains outstanding; M6/M4/M5 Host regression is not a substitute.

No flight-readiness claim is made from this Host/SIL result. The zero-target receipt is a software/bench contract result, not proof of physical motor response.


## First-reference timeout task integration — Run #38

- Tested code: `a314458f4ea35e06ed6145124ba63b1010d0b833`.
- [CI #38](https://github.com/hujunjie-hub/Flight/actions/runs/43382987050): **4/4 jobs SUCCESS**.
- Live first-reference scenario: **467 checks / 0 failures**. The test uses the real request queue and `task_mission_step()`; it withholds guidance through pre-arm and the first-reference grace period, then injects a valid reference at 110 ms. The production timeout uses strict `>` against 100 ms, so 100 ms exactly is still inside the window. FMS ground-disarms, publishes a matching certificate, and the real actuator owner produces `ZERO_TARGET_APPLIED`.
- Live explicit DISARM scenario: **455 / 0**. Existing non-zero-to-zero ramp scenario: **27 / 0**, 18 commits, 58 inhibits.
- Regression: M4 Host 1,002/0; M5 Host 9,515/0; M4/M5 integration 1,075/0; M6 MPC 15/0; M6 guard 26/0; M7 guard 274/0; ASan/UBSan PASS.
- Runs #35–#37 were failed attempts at the new test fixture; the final correction removed pre-arm guidance and moved the late candidate beyond the strict timeout boundary. Do not count those runs as passing.

### Current remaining acceptance gates

1. **P0 — DShot physical apply-time characterization: PARTIAL.** Host/SIL observes software completion; H723 waveform capture and a bounded ESC application-time result are still required.
2. **P1 — Target/physical validation: NOT_RUN.** H723 product build, WCET/RTA, RAM/stack, ESC response, HIL, calibration and staged flight remain outstanding. Keep `BENCH_INHIBIT`.
3. **P1 — M5-06 independent safety approval: BLOCKED.** Independent review of Hermite→Bernstein conversion, derivative convex-hull bounds, outward interval assumptions and dynamic uncertainty proof coverage remains outstanding on the separate branch.

### Branch lineage at tested code HEAD

- M6 vs M5-06 HEAD `51e7e923419733f5aa3b21efa1e46588629623ed`: diverged, **65 ahead / 395 behind**, merge base `edf32b4e39cdae014aa76870dc821882807a6905`.
- M6 vs M7 HEAD `24eee8acb3c10b79b8e306b00b1725d3fbc2614f`: diverged, **59 ahead / 2 behind**, merge base `b4ce6d0b3a2429d186fd5ec0563453322731db9b`.
- No merge/rebase performed. PRs #1–#4 remain Draft and branch-specific.

Host/SIL now covers both explicit ground DISARM and first-reference-timeout task-to-actuator software paths. It does not establish physical output timing, ESC response, target timing or flight readiness.

## Pre-sync stage-02 implementation baseline — 2026-11-13

- Repository: `hujunjie-hub/Flight`
- Branch: `opt/stage-02-m5-06-safety-validator`
- Implementation baseline: `2a0237bf42963f5a4457af24da1b713cd6f957d1`
- [Host validation](https://github.com/hujunjie-hub/Flight/actions/runs/43400605776): 6/6 jobs SUCCESS.
- [M6/M7 host validation](https://github.com/hujunjie-hub/Flight/actions/runs/43400605749): 5/5 jobs SUCCESS.
- [M7 host validation](https://github.com/hujunjie-hub/Flight/actions/runs/43400605682): 2/2 jobs SUCCESS.

| Test | Checks / failures |
|---|---:|
| M4 Host | 1,002 / 0 |
| M5 Host | 322,179 / 0 |
| M4/M5 Integration | 1,075 / 0 |
| M6 MPC | 15 / 0 |
| M6 guard/input lifecycle | 26 / 0 |
| M7 actuator guard | 274 / 0 |
| Live FMS→M7 explicit DISARM SIL | 455 / 0 |
| Live first-reference timeout SIL | 467 / 0 |
| Ground-stop ramp SIL | 27 / 0 (18 frame commits, 58 inhibits) |

M5 and M4/M5 integration ASan/UBSan each completed with 0 failures (322,179 and 1,075 checks). M6/M7 ASan/UBSan reran the 15-check MPC, 26-check M6 guard and 274-check M7 guard suites with 0 failures. Cortex-M7 M4/M5/M6/M7 module compile passed; MPC RAM 29,400 B, worker stack 8,192 B, largest static frame 2,024 B.

**Remaining gates:** physical DShot apply-time/ESC response characterization, H723 product runtime, WCET/RTA, target RAM/stack watermark, physical HIL, calibration and staged flight remain NOT_RUN. H723 remains `BENCH_INHIBIT`. M5-06 independent mathematical approval remains BLOCKED; the M6/M7 software integration does not authorize M5-07 or flight. These results apply only to the implementation baseline above; a new SPEC-only commit requires fresh CI.

## Post-sync stage-02 validation — 2026-11-13 (b6b2c8)

- Validated commit: `b6b2c8d1fbe2483ac76164993e7798a18af1e697`
- [Host validation](https://github.com/hujunjie-hub/Flight/actions/runs/43410120128): 6/6 jobs SUCCESS.
- [M6/M7 host validation](https://github.com/hujunjie-hub/Flight/actions/runs/43410120115): 5/5 jobs SUCCESS.
- [M7 host validation](https://github.com/hujunjie-hub/Flight/actions/runs/43410120239): 2/2 jobs SUCCESS.

| Test | Checks / failures |
|---|---:|
| M6 MPC | 15 / 0 |
| M6 guard/input lifecycle | 26 / 0 |
| M7 actuator guard | 274 / 0 |
| Live FMS→M7 explicit DISARM SIL | 455 / 0 |
| Live first-reference timeout SIL | 467 / 0 |
| Ground-stop ramp SIL | 27 / 0 (18 frame commits, 58 inhibits) |

M6/M7 ASan/UBSan and M4/M5 regression/sanitizer jobs all completed with zero failures. Production ABI 144 types / 0 mismatches / 0 unexpected Host/ARM; Spec Audit 145 types / 8 checks PASS; Cortex-M7 M4/M5/M6/M7 module compile PASS. MPC RAM 29,400 B; worker stack 8,192 B; largest static frame 2,024 B.

H723 product runtime, physical DShot apply-time, ESC thrust response, WCET/RTA, target RAM/stack watermark, physical HIL and flight remain NOT_RUN. M5-06 independent mathematical review remains BLOCKED; M5-07 is not authorized. FLIGHT-LOCAL-27 remains OPEN / RISK with estimated 8 B static headroom pending memory-budget and final-map recheck. These runs validate the commit above; subsequent commits require fresh CI.

## Current integrated main branch — 2026-11-14

PR #20 merged the complete stage-02 history into `main` at `eacb966b502b7f9cb547617e866641504346a5fd`. The integration source `bcbe4db5e665e5f678069afc1b206a00ae1cd36d` had 13/13 completed CI checks SUCCESS, including M6/M7 Host, ASan/UBSan, Cortex-M7 M4/M5/M6/M7 module compile and runtime ground-stop SIL. Main merge commit Host validation #43566259362 completed 6/6 jobs SUCCESS. The final main commit `d7666c5aa093dbef36fef11b82909e91fdc61491` passed all three main-push workflows: [Host validation #43567633122](https://github.com/hujunjie-hub/Flight/actions/runs/43567633122) 6/6, [M6/M7 host validation #43567633103](https://github.com/hujunjie-hub/Flight/actions/runs/43567633103) 5/5, and [M7 host validation #43567633072](https://github.com/hujunjie-hub/Flight/actions/runs/43567633072) 2/2. The Host workflow's PowerShell runner parse step also passed. Historical standalone feature-branch records above remain historical only.

H723 runtime, physical DShot apply-time, ESC response, WCET/RTA, target RAM/stack watermark, physical HIL and flight remain NOT_RUN. This integration does not authorize physical actuator output, M5-07 or flight.

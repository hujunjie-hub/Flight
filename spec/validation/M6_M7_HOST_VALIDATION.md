# M6/M7 Host validation record

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

1. **Live FMS-task integration coverage — OPEN.** The production `task_mission_step()` code creates and publishes the certificate into the runtime bundle, and `task_actuator_step()` consumes it. The current SIL smoke fixture still constructs a valid `FmsOutputView` and injects the adapter-produced certificate directly into the runtime bundle; it does not drive the live `task_mission_step()` input/output path. Add a test that exercises that publication seam before treating the full FMS-task-to-actuator-owner path as end-to-end verified.
2. **DShot apply-time characterization — PARTIAL.** `actuator_device_commit()` treats `rt_device_write()` as DMA submission. `actuator_device_poll_apply()` waits until the driver reports the submitted frame completed, then samples the monotonic clock. This is a conservative post-completion timestamp, not a hardware-captured timestamp of the exact DShot waveform edge. Validate the bound on H723 or document the approved timing semantics in the interface contract.
3. **Target and physical validation — NOT_RUN.** Cortex-M7/H723 target build, H723 WCET/RTA, target RAM/stack watermark, DShot waveform timing, physical ESC response, HIL, sensor calibration and staged flight validation remain outstanding.
4. **Separate M5-06 acceptance gate — BLOCKED.** The M5-06 PR #2 still requires independent mathematical review of Hermite→Bernstein conversion, derivative convex-hull bounds and outward interval assumptions, plus broader randomized dynamic uncertainty/acceleration-bound proof coverage. The M4/M5 regression passing here is not independent M5-06 approval; M5-07/physical release must not be inferred from this Host/SIL result.

Production `BENCH_INHIBIT` remains unchanged. Host/SIL evidence is not H723 target build, HIL, physical ESC response, or flight authorization.


### Mission certificate publication seam regression — Run #23 PASS

The SIL smoke calls the same certificate-publication helper used by `task_mission_step()`. Run #23 verified the then-current helper behavior, including retaining an unexpired certificate after an in-air output. A subsequent safety review found that this cache behavior was too permissive: a current, explicitly ineligible FMS output must revoke the old certificate immediately. The follow-up correction is tracked below. Run #22 initially failed to compile because the new helper was declared `void` while the test asserted its result, and the SIL `CHECK` macro accepts one argument only. Commit `a8f62f5b270355d699b8a42861198cfa93894337` corrected both compile issues.

[Run #23](https://github.com/hujunjie-hub/Flight/actions/runs/43375102383), exact commit `a8f62f5b270355d699b8a42861198cfa93894337`: **4/4 jobs SUCCESS**. M4 Host 993/0, M5 Host 9,515/0, M4/M5 integration 1,066/0, M6 MPC 15/0, M6 guard 26/0, M7 guard 274/0, SIL publication/owner smoke 18/0, and ASan/UBSan PASS. This remains a seam test rather than a live `fms_policy_update()` command/lifecycle integration test.

### Branch lineage / integration gate

At the time of this check, M6 branch HEAD `41ba12f2d1fbb3b4525407f5b621db8cddc6dbc4` and M5-06 branch HEAD `51e7e923419733f5aa3b21efa1e46588629623ed` are **diverged**, with common ancestor `edf32b4e39cdae014aa76870dc821882807a6905`. GitHub compare reports M6 ahead by 52 commits and behind M5-06 by 395 commits. The M6/M7 Host CI therefore validates this independent branch only; it does not contain or supersede the current M5-06 branch. Do not rebase, merge, or claim unified milestone acceptance without an explicit integration plan and independent M5-06 approval.


### Invalid certificate demand-continuity regression — Run #25 PASS

The actuator owner now suppresses a queued M6 demand only when `actuator_guard_ground_stop()` returns `STATUS_OK`. A malformed, stale or unauthorized certificate candidate cannot take ownership of output. The SIL fixture proves that an invalid in-air candidate does not block a valid 5 N demand, then verifies that a subsequently accepted certificate drives a multi-frame 5 N→0 N ground-stop ramp and terminal zero receipt.

[Run #25](https://github.com/hujunjie-hub/Flight/actions/runs/43375635462), exact commit `41ba12f2d1fbb3b4525407f5b621db8cddc6dbc4`: **4/4 jobs SUCCESS**. M4 Host 993/0, M5 Host 9,515/0, M4/M5 integration 1,066/0, M6 MPC 15/0, M6 guard 26/0, M7 guard 274/0, and ASan/UBSan PASS. The runtime SIL scenario completed **19 checks / 0 failures**, with **18 frame commits** (one 5 N demand plus the bounded ramp frames) and 58 inhibit calls; the final receipt confirmed zero thrust without an apply-time overrun.


### Superseded FMS ground-stop certificate revocation (CI pending)

A current FMS output that is not eligible for a ground-stop certificate now clears the cached certificate immediately. Only an absence of current FMS output may retain the previous certificate, and then only until its original expiry. The SIL fixture verifies no-output bounded retention, immediate clearing on a current IN_AIR output, expiry clearing, and fresh republishing. This closes the stale-ground-decision window in the mission certificate cache; the updated workflow run is pending.

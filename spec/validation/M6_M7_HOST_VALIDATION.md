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
- The `--ground-stop-smoke` SIL case exercises the real actuator task owner, guard and receipt path. Its fixture currently constructs a valid `FmsOutputView` and injects the adapter-produced certificate into the runtime bundle; it does **not** drive `task_mission_step()` with a live FMS input sequence. Full mission-task-to-actuator-owner integration coverage therefore remains incomplete.

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

**Fix committed; CI revalidation pending.** Keep production `BENCH_INHIBIT` unchanged. Host/SIL evidence is not H723 target build, HIL, physical ESC response, or flight authorization.

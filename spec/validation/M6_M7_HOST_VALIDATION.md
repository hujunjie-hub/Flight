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

## Safety boundary

Host tests prove only the exercised software paths. They do not establish target WCET, board integration, DShot waveform timing, physical motor response, HIL acceptance, or flight readiness. The production H723 build remains `BENCH_INHIBIT`; `SIM_CONTROLLED_STOP` is enabled only for the host test build.


## Remaining M7 end-to-end integration blocker

The ground-stop guard transaction is covered by Host tests, but the production call path is not yet connected:

- `fms_permit_tick()` revokes the permit / calls `fms_permit_ground_disarm()`, but no FMS output currently publishes a `GroundStopCertificate`.
- `task_actuator_step()` consumes `ActuatorDemand`, polls status, and reads receipts; it does not consume a ground-stop certificate or call `actuator_guard_ground_stop()`.
- `actuator_device_commit()` timestamps the write with `clock_now_us()` after `rt_device_write()`. That is a submission-return timestamp, not yet independently proven to be the physical DShot frame-application time.

Therefore **FMS→M7 ground-stop integration = NOT_IMPLEMENTED / NOT_RUN**. The M7 Host test proves the guard API in isolation only. Do not claim that the FMS first-reference timeout or landed disarm path executes a bounded physical thrust ramp on H723. Before enabling physical output, add the certificate to the FMS/runtime contract, consume it in the actuator owner task, obtain a defensible driver-application timestamp (or explicitly redefine the evidence as enqueue time), and test the full revoked-permit→ramp→zero-receipt→DISARMED chain. Keep production `BENCH_INHIBIT` unchanged.

# FMAT-0013 M7 Host validation record

## Scope

- Branch: `test/fmat-0013-m7-actuation`
- Baseline before M7 guard hardening: `b4ce6d0b3a2429d186fd5ec0563453322731db9b`
- The test compiles the real `actuator_guard.c`; it does not use a parallel guard implementation.

## Run

```bash
bash tests/actuation/run_host_tests.sh
EXTRA_CFLAGS="-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer" bash tests/actuation/run_host_tests.sh
```

The workflow runs both commands on every push to this branch and on pull requests targeting main.

## M7 Host coverage

The suite includes permit generation/watermark, demand freshness, frame-boundary commit, ground-stop certificate validation, revoked-permit epoch binding, repeated certificate idempotence, in-flight permit/demand rejection, bounded thrust ramp, physical apply-time overrun reporting, zero-target receipt and terminal DISARMED latch.

## Acceptance boundary

- Host tests and ASan/UBSan are software evidence only.
- Cortex-M7 target build, DShot waveform timing, physical ESC response, HIL and flight remain NOT_RUN on this branch.
- FMS currently does not publish GroundStopCertificate through FmsOutputView, and task_actuator does not consume it or call actuator_guard_ground_stop(). End-to-end FMS→M7 ground-stop integration is NOT_IMPLEMENTED.
- actuator_device_commit() currently timestamps after rt_device_write(); physical DShot frame application timing has not been independently established.
- Keep the PR Draft and production BENCH_INHIBIT unchanged.


## CI result for commit c6db294d47e5b98bfa22a1b93db192e780c2b073

- Push workflow: [43364882966](https://github.com/hujunjie-hub/Flight/actions/runs/43364882966) — SUCCESS, 2/2 jobs.
- Pull-request workflow: [43364890728](https://github.com/hujunjie-hub/Flight/actions/runs/43364890728) — SUCCESS, 2/2 jobs.
- M7 Host: 255 checks / 0 failures.
- M7 ASan/UBSan: 255 checks / 0 failures.

Both independently triggered workflows compiled and ran the same real actuator guard source. This closes the M7 Host/Sanitizer gate for this commit only. It does not close the FMS→M7 integration, target build, DShot physical-application timing, HIL or flight gates listed above.

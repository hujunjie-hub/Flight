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

## Status at CI bootstrap

| Gate | Status | Evidence |
|---|---|---|
| M4 host | NOT_RUN | Awaiting first branch CI run |
| M5 host | NOT_RUN | Awaiting first branch CI run |
| M4/M5 integration | NOT_RUN | Awaiting first branch CI run |
| M6 host | NOT_RUN | Awaiting first branch CI run |
| M7 host | NOT_RUN | Awaiting first branch CI run |
| M6/M7 ASan + UBSan | NOT_RUN | Awaiting first branch CI run |
| Cortex-M7 target build | NOT_RUN | Not covered by this host workflow |
| HIL / DShot waveform / flight | NOT_RUN | Requires target hardware and independent evidence |

No item is marked PASS until the corresponding job completes successfully against the commit under review.

## Safety boundary

Host tests prove only the exercised software paths. They do not establish target WCET, board integration, DShot waveform timing, physical motor response, HIL acceptance, or flight readiness. The production H723 build remains `BENCH_INHIBIT`; `SIM_CONTROLLED_STOP` is enabled only for the host test build.

# FLIGHT-29 Navigation Health and Divergence — Change Request Package

Status: **ROOT CAUSE IDENTIFIED (NIS deadlock) / AWAITING OWNER APPROVAL — NO ESTIMATOR CODE CHANGED**

Scope: QUAD_X4 / STM32H723ZGT6 / `OBSTACLE_SOURCE_MODE=NONE` / `BENCH_INHIBIT`.
All evidence below is Host/SIL and Windows-native FMT-SIM software evidence. It is
not H723 target, HIL, WCET or flight evidence.

## 1. Known facts

| Fact | Value | Source |
|---|---|---|
| Navigation vs FMT truth separation | up to **10.441 m** horizontal | RUN_ID `FMT-SIM-LOCAL-20261114T191012782Z` |
| Mission touchdown error while diverged | **16.927 m** (limit 0.50) | RUN_ID `FMT-SIM-LOCAL-20261114T185231385Z` |
| `nav.valid_mask` during divergence | **0xff** (all fields still "valid") | same runs |
| `horizontal_position_std` | 0.062 m -> **0.81 m**, monotonic | same runs |
| Navigation acceleration | **-80.7 / -137.6 m/s^2** | RUN_ID `...T185231385Z` |
| Measurement rejections | `meas_rej = 2` only | same run |
| Reproducible without controller tuning | yes, unmodified `controller_tuning.yaml` | `f0f21cf36` run |
| First detectable separation | 1.37 m at `pos_std = 0.11` | detector run |

Consequences already implemented (no estimator change):

- **SIL detector** `7fd57508`: an armed scenario fails with one explicit
  `ESTIMATOR_DIVERGENCE` line when navigation and truth stay separated by more
  than 1.0 m horizontal / 0.8 m vertical for 500 ms.
- **M6 consumer gate** `7b55218e`: a valid-flagged navigation acceleration above
  20 m/s^2 per axis returns `STATUS_MODEL_OUT_OF_DOMAIN`, so the firmware stops
  instead of flying on the diverged estimate.

These two only catch the symptom. They do not stop the estimate from diverging.

## 2. Root cause (static analysis, 2026-11-15)

The filter has a normalised-innovation-squared gate but **no recovery path**:

| Code | Behaviour |
|---|---|
| `trimmed_core/src/quad_core.cpp:699-703` | `if (config_.nis_gate_enable && nis > threshold) { ++diagnostics_.nis_rejected; return kErrNisRejected; }` |
| `trimmed_core/include/kfgins/quad_types.hpp:58-59` | `kNisThresholdDof1 = 6.6348966`, `kNisThresholdDof3 = 11.3448667` (99.5 % chi-square) |
| `kf_gins_quad_port.cpp:107,229,232` | `nis_gate_enable` is configurable and `nis_rejected` / `last_nis` are exposed by the port |
| repo-wide search | no covariance inflation, no reset, no "re-accept after N rejections" anywhere |

This produces a deadlock: once the INS has drifted, the GNSS position innovation is
large compared with the (still small) declared covariance, so NIS exceeds the
threshold and **every** position fix is rejected for ever. The filter then
integrates without position aiding, `horizontal_position_std` grows, and the
estimate can only diverge further - while `valid_mask` stays `0xff`.

It also explains the measured signature better than a "sudden divergence":

- navigation **velocity** follows the truth (e.g. `ve = 1.766 m/s` at t = 37.24 s)
  because velocity observations are still accepted;
- navigation **position** does not follow, because position observations are
  rejected;
- `meas_rej = 2` from the runtime is a different counter from
  `diagnostics_.nis_rejected`, so the rejection storm is invisible to every
  consumer.

The remaining question is what caused the *first* large innovation (a ground
contact transient, a baro step at touchdown, or an attitude excursion). It sets
the trigger but not the defect: a filter that cannot recover from a single bad
innovation is unsafe regardless of the trigger.

### 2.1 Deterministic reproduction (2026-11-15)

`tests/estimator_backend/test_nis_recovery.cpp` (commit `4a3fdfaa`) drives the
production port ABI and records the deadlock:

```
first displaced fix (40 m, 0.2 m std): result=5 nis=69932240.851060 nis_rejected 0 -> 1
after 20 displaced fixes over 20 s:    accepted=1 rejected=21 recovered=0 last_nis=3018.585036
```

Twenty seconds of covariance growth brought NIS from 6.99e7 down to 3.0e3, which
is still 266 times the dof-3 threshold: the gate cannot re-open by itself. The
test is the red baseline for Stage B (the `recovered` and `gnss_pos_updates`
assertions are the lines a fix must flip).

Related gap: the estimator suite is not executed by any CI workflow
([FLIGHT-35](https://github.com/hujunjie-hub/Flight/issues/35)), which is why this
defect was never observed automatically.

## 3. Trigger candidates (Stage A confirms which one fired)

The NIS deadlock explains the divergence; it does not explain which event produced
the first large innovation. Candidates, all reachable at touchdown:

1. a vertical ground-contact transient that steps the position/velocity estimate;
2. a barometer step when the airframe reaches the surface;
3. an attitude excursion during the landing flare that corrupts the mechanisation.

Each is a different upstream defect and each can trigger the same unrecoverable
NIS rejection, so Stage A must identify the trigger before any additional change.

## 4. Staged plan (each stage is separately approvable)

### Stage A — read-only instrumentation (no behaviour change)

Add a Host/SIL-only diagnostic that logs, per GNSS sample: the implied local NED
position, the navigation position at that instant, the innovation, and which
`ins` adapter branch consumed it. Also count applied/rejected samples per
rejection reason.

- Risk: none in flight code if the counters live behind `FLIGHT_HOST_TEST`
  (precedent: the existing MPC diagnostic records).
- Exit criterion: the trigger from section 3 is identified, and the NIS
  rejection storm is quantified (`nis_rejected`, `last_nis`) from the same trace.

### Stage B — NIS recovery (behaviour change, needs approval)

Keep the gate for transient outliers, add a bounded recovery so a single bad
innovation cannot brick the filter. Candidate shapes:

1. count consecutive NIS rejections per observation kind; after N (for example
   5) rejections in a row, inflate the corresponding position covariance (or
   re-seed the position from the accepted fix) once, then resume normal gating;
2. allow a rejected position fix to update position but not velocity (or the
   reverse) instead of rejecting the whole record;
3. bound the gate by an absolute innovation limit sized from the declared
   variance *and* the expected dynamics, so a genuinely diverged estimate is
   detected as "reset", not "reject for ever".

Shape 1 is the smallest change and is testable from a recorded trace. All shapes
must keep rejecting isolated outliers: the NIS thresholds and the
`nis_gate_enable` switch stay unchanged.

- Risk: navigation response changes on every flight phase; needs a full
  M4/M5/M6/M7 regression plus a native three-scenario batch.
- Rollback: single commit revert; no ICD change.

### Stage C — contract-level health (ICD change, needs separate approval)

If oscillation remains possible after Stage B, the contract needs a navigation
health signal that consumers can trust. Options:

1. reuse the existing `NavigationCapability.horizontal_position_std_m` and let
   consumers gate on a threshold (no ICD change, but late: the estimate was
   already 1.37 m wrong at `pos_std = 0.11`);
2. add an explicit validity bit or a bounded `position_std` ceiling to the
   navigation snapshot (**ICD + ABI change**, needs the ABI lock and message
   registry update).

Option 1 is cheap but demonstrably late. Option 2 is the only one that lets a
consumer fail closed *before* the position is unusable.

## 5. Test plan attached to any approved change

- Fixed Host regression that reproduces the divergence from a recorded trace
  (the `[settle]` diagnostics added in `9676b07d` carry the needed fields).
- SIL detector must stay silent for the full Land/Return/Mission nominal runs.
- M4 969/0, M5 330,302/0, M6 15/0 + guard 28/0, M7 274/0, M4/M5 integration
  1,042/0, ASan/UBSan, Cortex-M7 compile, control-loop step/sweep.
- Native FMT-SIM Land -> Return -> Mission batch on a clean `main` with the
  formal gates, archived with SHA256.
- H723 target/HIL/WCET/flight stay NOT_RUN and must not be claimed.

## 6. What the owner is being asked to approve

1. **Stage A** (instrumentation only) — no flight behaviour change.
2. In principle, the *scope* of Stage B/C so the work is not blocked again after
   Stage A lands.

Until this is approved, `middleware/model/ins/**` stays untouched and FLIGHT-29
remains OPEN.

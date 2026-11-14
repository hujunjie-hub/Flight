# FLIGHT-29 Navigation Health and Divergence — Change Request Package

Status: **DESIGN / AWAITING OWNER APPROVAL — NO ESTIMATOR CODE CHANGED**

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

## 2. The open question that must be answered before any fix

`horizontal_position_std` growing from 0.062 m to 0.81 m means the filter is not
fusing position observations. The SIL feeds a synthetic GNSS sample every 100 ms
at RTK_FIXED with `position_variance_m2 = 0.04` (0.2 m std) taken directly from
the FMT truth, and the submissions return `STATUS_OK`. Therefore one of these is
true:

1. the samples are accepted by the runtime but **not applied** by the estimator
   adapter (mode/source gating, origin or epoch filter, innovation gate);
2. they are applied but the filter's process noise grows faster than the
   correction can pull it back (a tuning/gating defect);
3. the sample path selected in Mission differs from the path used by Land/Return
   (which do track truth).

Nothing in the current evidence distinguishes these three, and each implies a
different fix. Guessing risks masking a real navigation fault.

## 3. Staged plan (each stage is separately approvable)

### Stage A — read-only instrumentation (no behaviour change)

Add a Host/SIL-only diagnostic that logs, per GNSS sample: the implied local NED
position, the navigation position at that instant, the innovation, and which
`ins` adapter branch consumed it. Also count applied/rejected samples per
rejection reason.

- Risk: none in flight code if the counters live behind `FLIGHT_HOST_TEST`
  (precedent: the existing MPC diagnostic records).
- Exit criterion: one of the three hypotheses above is proven, or all three are
  disproven.

### Stage B — estimator gating fix (behaviour change, needs approval)

Depending on Stage A. Likely shape: apply the GNSS position/velocity update when
the sample is fresh, same clock and same origin, with an innovation gate sized
from the declared variance, and stop requiring the mode that currently disables
it.

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

## 4. Test plan attached to any approved change

- Fixed Host regression that reproduces the divergence from a recorded trace
  (the `[settle]` diagnostics added in `9676b07d` carry the needed fields).
- SIL detector must stay silent for the full Land/Return/Mission nominal runs.
- M4 969/0, M5 330,302/0, M6 15/0 + guard 28/0, M7 274/0, M4/M5 integration
  1,042/0, ASan/UBSan, Cortex-M7 compile, control-loop step/sweep.
- Native FMT-SIM Land -> Return -> Mission batch on a clean `main` with the
  formal gates, archived with SHA256.
- H723 target/HIL/WCET/flight stay NOT_RUN and must not be claimed.

## 5. What the owner is being asked to approve

1. **Stage A** (instrumentation only) — no flight behaviour change.
2. In principle, the *scope* of Stage B/C so the work is not blocked again after
   Stage A lands.

Until this is approved, `middleware/model/ins/**` stays untouched and FLIGHT-29
remains OPEN.

# FLIGHT-29 Navigation Health and Divergence — Change Request Package

Status: **NATIVE FMT-SIM FRAME DELIVERY FIX VERIFIED LOCALLY / GENERAL ESTIMATOR RECOVERY OPEN**

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

This can produce a deadlock: once the INS has drifted, the GNSS position innovation is
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

The native trace has no per-observation innovation or disposition, so it does
not establish that this mechanism caused the first divergence. In the
`...T191012782Z-0f21cf36` run the horizontal estimate was already 0.93 m from
truth at t=30.000 s while the plant was 2.50 m above the ground. The first
divergence flag followed at t=30.610 s, with the plant still airborne.

### 2.1 Deterministic reproduction (2026-11-15)

`tests/estimator_backend/test_nis_recovery.cpp` (commit `4a3fdfaa`) drives the
production port ABI and records the deadlock:

```
first displaced fix (40 m, 0.2 m std): result=5 nis=69932240.851060 nis_rejected 0 -> 1
after 20 displaced fixes over 20 s:    accepted=1 rejected=21 recovered=0 last_nis=3018.585036
```

Twenty seconds of covariance growth brought NIS from 6.99e7 down to 3.0e3, still
266 times the dof-3 threshold. The fixture injects a displaced GNSS sequence
into a nearly stationary INS. It measures persistent disagreement without an
independent source that identifies which side is wrong. Preserve rejection of
an isolated fix and of a sustained faulty source when validating recovery.

The estimator suite now runs in Host CI and in the local T31 regression. The
coverage gap is tracked in [FLIGHT-35](https://github.com/hujunjie-hub/Flight/issues/35).

## 3. Trigger candidates (Stage A confirms which one fired)

Correlate the first divergence with the last fused GNSS position, the position
and velocity innovations, aircraft attitude and the raw plant state. Candidate
events for subsequent runs include:

1. a vertical ground-contact transient that steps the position/velocity estimate;
2. a barometer step when the airframe reaches the surface;
3. an attitude excursion during the landing flare that corrupts the mechanisation.

Each is a different upstream defect and each can trigger the same unrecoverable
NIS rejection, so Stage A must identify the trigger before any additional change.

## 4. Staged plan (each stage is separately approvable)

### Stage A — read-only instrumentation (no behaviour change)

`[gnss-fusion]` logs each replayed GNSS position or velocity observation with
sample time, aligned IMU time, source/journal sequence, replay path, observed
and predicted antenna values, innovation, result and NIS. Result `0` means the
update passed and `5` means the NIS gate rejected it. Position values are
metres in the NED tangent frame at the pre-update INS position. Velocity values
are m/s in NED. `[gnss-commit]` identifies the latest position and velocity
evaluated by each successfully committed candidate. A replay row can recur or
belong to a probe, so use the sample times and commit rows when tallying inputs.

- Scope: both log sites are gated by `FLIGHT_HOST_SIL`. The ARM build uses the
  original observation and replay logic without diagnostic I/O.
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

FLIGHT-29 remains OPEN pending the Stage B/C decisions and formal verification.

## 7. Native FMT-SIM trigger and local batch

In `FMT-SIM-LOCAL-20261115T010454526Z-3652a608`, the Mission receiver had
dequeued 18 more states than the SIL clock had advanced by t=26 s. GNSS
velocity rejected at t=26.290 s, then horizontal/vertical divergence was
reported at t=29.350 s. The old receive routine overwrote earlier decoded
states when a scheduler turn received a burst. A loopback test reproduced
the loss from two frames in one datagram and a third in a queued datagram.

Commit `ada6bd65` delivers one state at a time and preserves queued samples.
The next native batch, `FMT-SIM-LOCAL-20261115T011452219Z-ada6bd65`, passed all
three scenarios. Mission had 535 position/velocity pairs, no NIS rejection,
and a maximum horizontal separation of 0.055 m. This verifies the corrected
simulation transport. The Stage B/C production decisions remain open.

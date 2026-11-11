# MR-009 Independent Review Matrix — R29

Status: **WAITING_INDEPENDENT_REVIEW — AGENT CANNOT APPROVE**

## 1. Scope

The claim under review is that the binary32 M6 consumer's position, velocity and
acceleration Horner evaluations are enclosed by the M5-06 validator's error
budget for every validated sub-interval and every axis.

The proof must cover the actual consumer implementation, not an idealized
polynomial alone.

## 2. Evidence already present in R28

- `consumer_power_coefficients()` mirrors the M6 coefficient construction.
- `consumer_eval_error_bound()` compares binary32 consumer coefficients with
  exact interval coefficient bounds.
- Position, velocity and acceleration have separate error bounds.
- Velocity and acceleration coefficient scaling is represented in `vf`/`af`
  and `vc`/`ac`.
- Horner magnitude recurrence `H_n=|c_n|`,
  `H_k=|c_k|+s_max*H_(k+1)` is present.
- The recurrence rejects non-finite values and exact upper bounds above
  `FLT_MAX`.
- Host compile flags disable fast-math and FP contraction.
- Floating environment is checked at runtime.
- R28 has direct test-API extreme cases and a full-validator extreme input test.

## 3. Test result interpretation correction

The full-validator extreme test contains three classes of input:

1. All position/velocity/acceleration fields set to `FLT_MAX`: rejected by
   `validate_reference_shape()` as `M5_SAFETY_INVALID /
   M5_SAFETY_REASON_REFERENCE_INVALID` before `build_curve()`.
2. Alternating-sign cancellation at half `FLT_MAX`: also rejected by reference
   shape validation before the consumer polynomial.
3. Normal positions with acceleration endpoints `+/-FLT_MAX`: rejected by the
   validator numeric path as `M5_SAFETY_NUMERIC_FAILURE /
   M5_SAFETY_REASON_NUMERIC`.

The first two are fail-closed input-boundary tests, not proof that the Horner
branch executed. The third reaches a numeric failure but still needs branch
trace evidence to identify whether the failure came from coefficient
construction, interval conversion, derivative scaling or Horner evaluation.

The R29 patch strengthens these status/reason assertions; it does not claim
that branch-level proof is complete.

## 4. Required independent proof checks

| Proof obligation | Required evidence | Reviewer result |
|---|---|---|
| Consumer coefficient identity | Exact operation order and binary32 types compared against M6 source | PENDING |
| Position Horner | Degree-5 bound and all intermediate rounded values finite | PENDING |
| Velocity Horner | Degree-4 bound after `5c5..c1` scaling | PENDING |
| Acceleration Horner | Degree-3 bound after `20c5..2c2` scaling | PENDING |
| Binary32 rounding | RN model, relative error and subnormal absolute allowance | PENDING |
| Intermediate overflow | Bound on each rounded product and sum, not only exact real H_k | PENDING |
| Coefficient scaling overflow | Explicit checks or proof for each `vf`/`af` multiplication | PENDING |
| Time conversion | `gamma_2`, enlarged `s_eval_max`, derivative shift | PENDING |
| FP contraction | Actual production compiler flags and object-code inspection | PENDING |
| Rounding mode / FTZ / DAZ | Host MXCSR and Cortex-M7 FPSCR equivalence | PENDING |
| M6 interval domain | No extrapolation outside validated `s_eval_max` | PENDING |
| Production branch reachability | Trace proves each targeted rejection site executes | PENDING |

## 5. R29 required branch instrumentation

The test harness must distinguish these rejection points without changing
production behavior:

- `consumer_power_coefficients()` finite/overflow rejection;
- velocity coefficient scaling rejection;
- acceleration coefficient scaling rejection;
- position Horner intermediate rejection;
- velocity Horner intermediate rejection;
- acceleration Horner intermediate rejection.

A test-only trace/counter may be compiled under
`FLIGHT_M5_SAFETY_TEST_API`, but it must be absent from production builds and
must not change stack layout or timing in the production object.

Each test must assert:

- the expected branch counter increments exactly once or the documented count;
- final status is `M5_SAFETY_NUMERIC_FAILURE`;
- reason is `M5_SAFETY_REASON_NUMERIC`;
- `result.passed == 0`.

The test must also include a valid legal companion case that still passes.

## 6. Approval rule

Agent may update proof text and add tests. Agent must not mark MR-009 APPROVED.
The independent reviewer must fill each row, identify assumptions and sign the
final conclusion. Until then B05 remains WAITING_INDEPENDENT_REVIEW and M5-06
cannot be accepted.

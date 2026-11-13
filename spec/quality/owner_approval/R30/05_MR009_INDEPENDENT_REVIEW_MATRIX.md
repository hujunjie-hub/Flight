# MR-009 Independent Review Matrix — R30

Status: **WAITING_INDEPENDENT_REVIEW — AGENT CANNOT APPROVE**

## 1. Scope

The claim under review is that the binary32 M6 consumer's position, velocity and
acceleration Horner evaluations are enclosed by the M5-06 validator's error
budget for every validated sub-interval and every axis. The proof must cover
the actual consumer implementation and each rounded intermediate, not only an
ideal-real polynomial.

## 2. R29 gap corrected in R30

R29 used the ideal-real recurrence

~~~text
H_n = |c_n|
H_k = |c_k| + s_max * H_(k+1)
~~~

and compared each H_k with FLT_MAX. This did not propagate the actual binary32
rounding of the previous Horner product and sum. S06-115 now contains a
deterministic counterexample where every ideal H_k is below FLT_MAX and the
magnitude term is below FLT_MAX/16, but the real binary32 chain rounds an
intermediate add to the round-to-nearest overflow midpoint.

R30 replaces that check with a rounded recurrence. Each exact product and sum
upper bound must be strictly below

~~~text
T_RN = 2^128 - 2^103
~~~

the binary32 round-to-nearest/ties-to-even overflow midpoint. The rounded
product upper bound is passed to the next sum; the rounded sum upper bound is
passed to the next Horner level. Only after proving an operation below T_RN may
its rounded bound be capped at FLT_MAX.

## 3. Error model

For every no-overflow binary32 RN operation:

~~~text
|fl(x)-x| <= u*|x| + eta
u = 2^-24
eta = 2^-149
~~~

The relative term covers normal results; the absolute term covers subnormal
results. The propagated Horner bound, gamma_n term and separately propagated
subnormal floor must be checked together. The existing conservative magnitude
thresholds remain unchanged.

## 4. Test evidence and interpretation

- S06-115: deterministic exact-H counterexample; all ideal H values remain
  finite and below FLT_MAX; the actual volatile binary32 Horner chain becomes
  non-finite at degree-5 stage k=3; the production helper must return +Inf.
- Test-only 0/1 counters distinguish coefficient construction, velocity
  scaling, acceleration scaling, position Horner, velocity Horner and
  acceleration Horner.
- Full validator asserts NUMERIC_FAILURE, NUMERIC, passed=0 for coefficient
  construction and both derivative-scaling rejection paths.
- Degree-5/4/3 Horner rejection branches are exercised directly through the
  same production consumer_eval_error_bound() helper and must each increment
  exactly one corresponding counter.
- A legal full-validator companion must pass with all six counters equal to 0.
- The focused runner executes Horner tests, branch-stage tests, extreme
  reference tests, floating-environment tests and the real C time-conversion
  path.

**Important limitation:** the degree-5/4/3 Horner branch tests use the production
helper directly. They do not claim that all three Horner rejection sites have
been reached through a complete m5_trajectory_safety_validate() input. The
independent reviewer must decide whether the helper-level proof plus full
validator status tests satisfy the production-path obligation, or request
additional full-validator fixtures.

## 5. Independent proof checklist

| Proof obligation | R30 evidence | Reviewer result |
|---|---|---|
| Consumer coefficient identity | Existing shared coefficient construction and consumer source comparison | PENDING |
| Position Horner | Degree-5 rounded recurrence; S06-115 exact-H counterexample | PENDING |
| Velocity Horner | Degree-4 rounded recurrence and stage counter | PENDING |
| Acceleration Horner | Degree-3 rounded recurrence and stage counter | PENDING |
| Binary32 RN rounding | u=2^-24, eta=2^-149, propagated rounded intermediate bounds | PENDING |
| Intermediate overflow | Strict comparison against T_RN=2^128-2^103 for every product and sum | PENDING |
| Coefficient scaling overflow | Full-validator trace for velocity and acceleration scaling | PENDING |
| Time conversion | Existing gamma_2, enlarged s_eval_max, derivative shift evidence | PENDING |
| FP contraction | Existing compile flags and object-code audit; re-run for R30 commit | PENDING |
| Rounding mode / FTZ / DAZ | Focus runner includes existing Host negative tests; Cortex-M7/FPSCR equivalence review remains | PENDING |
| M6 interval domain | No extrapolation outside validated s_eval_max | PENDING |
| Full-validator Horner reachability | Helper-level branch counters only; full-validator reachability explicitly not claimed | PENDING |

## 6. Required result semantics

For each full-validator branch test:
- expected branch counter is exactly 1; all other rejection counters are 0;
- status is M5_SAFETY_NUMERIC_FAILURE;
- reason is M5_SAFETY_REASON_NUMERIC;
- result.passed == 0.

For each helper-level Horner branch test:
- return value is +Inf;
- the expected degree-specific Horner counter is exactly 1;
- all other rejection counters are 0.

For the legal full-validator companion, status is PASS and every counter is 0.

## 7. Approval rule

The Agent may update proof text and add tests. The Agent must not mark MR-009
APPROVED. The independent reviewer must fill every row, identify assumptions
and sign the final conclusion. Until then B05 remains WAITING_INDEPENDENT_REVIEW
and M5-06 cannot be accepted.


## 8. R30.1 branch-counter semantics

The first R30 CI run found three counter assertions failing because the production
builder continued into downstream derivative scaling after an upstream
coefficient/scaling value became non-finite. R30.1 now returns immediately at
the first failed numerical stage:
- non-finite consumer coefficients stop before derivative scaling;
- non-finite velocity scaling stops before acceleration scaling;
- non-finite acceleration scaling stops before Horner error evaluation.

This makes the six test-only 0/1 counters mutually exclusive and consistent
with the first-stage enum. The change is fail-fast only; it does not relax a
threshold or permit an invalid value.

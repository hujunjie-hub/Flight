# MR-009 Review Matrix — R31 (internal three-pass disposition)

Status: **INTERNAL_THREE_PASS_REVIEW_COMPLETE_WITH_LIMITATIONS — EXTERNAL REVIEW WAIVED BY PROJECT OWNER**

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
results. Under RN, a subnormal rounding residual is at most 2^-150; its maximum
propagation through up to n later relative roundings is absorbed by the factor
2 in the implemented n*2^-149*max(1,s_eval_max)^n floor. The propagated Horner
bound, gamma_n term and subnormal floor are checked together. Conservative
magnitude thresholds remain unchanged.

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

**Accepted explicit limitation:** the degree-5/4/3
Horner branch tests call the production helper directly. They do not claim that
all three Horner rejection sites have been reached through a complete
m5_trajectory_safety_validate() input.

S06-115 uses `s=0x1.8f5c28p-2f` (~0.39), deliberately outside the current
M5-06 product time domain. The product contract sets
`PRODUCT_PREVIEW_STEP_US=20000u`; the M5-06 validator requires the 20 ms
preview spacing and `u1<=1`, so production `s_eval_max` is approximately
0.020000002. The degree-specific helper stress tests also include `s_max=1.0`
cases and a separate `s_max=0.02` branch-stage case, but their coefficients
are directly constructed and are not claimed to be reachable from a full
validator input. The M6 locator also supports 40 ms route previews; M5-06
does not claim to validate those 40 ms references.

Internal three-pass review accepts the explicit boundary: the degree-5/4/3
Horner rejection branches are tested through the production helper; no claim is
made that each branch is reachable through a complete product-domain validator
input. Full-validator coefficient/scaling failure paths and a legal companion
are tested separately. This limitation remains visible and must not be hidden.

## 5. Internal three-pass proof checklist

| Proof obligation | R30/R31 evidence | Evidence status | Internal three-pass outcome |
|---|---|---|---|
| Consumer coefficient identity | S06-086 real C `control_sample_reference()` differential (314,731/0); validator coefficient operation-order replica; consumer/validator source hashes recorded by FP audit | READY | PASS |
| Position Horner | Degree-5 rounded recurrence; S06-115 exact-H counterexample; production-helper 0/1 counter | READY; S06-115 is outside product time domain | PASS |
| Velocity Horner | Degree-4 rounded recurrence; production-helper 0/1 counter; helper-level reachability only | READY; full-validator reachability not claimed | PASS |
| Acceleration Horner | Degree-3 rounded recurrence; production-helper 0/1 counter; helper-level reachability only | READY; full-validator reachability not claimed | PASS |
| Binary32 RN rounding | `u=2^-24`, `eta=2^-149`; normal/subnormal upward bound propagated after every product and sum | READY | PASS |
| Gamma upper arithmetic | R30.3: for n=2/10/12, `nu` up, `1-nu` down, quotient up; non-positive denominator fails closed; dual-source token audit + 18 mutation tests PASS | READY | PASS |
| Intermediate overflow | Strict `P_k/A_k < T_RN=2^128-2^103` before capping a rounded upper bound at FLT_MAX | READY | PASS |
| Coefficient scaling overflow | Full-validator coefficient/velocity-scaling/acceleration-scaling fail-fast traces; NUMERIC_FAILURE / NUMERIC / passed=0 | READY | PASS |
| Time conversion | S06-099: 60,002 offsets; physical-time max ratio 4.897714e-08 and production h*offset/step max ratio 0.6601743 of gamma2; S06-108/109/113: 96/96/76 checks; R31 time shift uses cf', rounded vf', rounded af'; S06-107 oracle updated | READY | PASS |
| FP contraction | R30.3 ARM audit: GCC 14.2.1; consumer/validator objects `fused_operations=0`; consumer/validator source-token MATCH; 18 mutation self-tests PASS; both objects 0 fused operations | READY | PASS |
| Rounding mode / FTZ / DAZ | Host negative tests cover FE_DOWNWARD/UPWARD/TOWARDZERO and MXCSR FTZ/DAZ; ARM runtime gate reads FPSCR.RMode/FZ/DN; H723 runtime remains NOT_RUN | HOST_TESTED / ARM_RUNTIME_NOT_RUN | PARTIAL — target gate remains open |
| M6 interval domain | M5-06 requires 20 ms intervals; `s_eval_max≈0.020000002`; M6 locator rejects offsets beyond the preview horizon; 40 ms route previews are outside M5-06 validation scope | READY | PASS — 20 ms claim only |
| Full-validator Horner reachability | Direct production-helper branches are tested; no full-validator reachability claim; full-validator coefficient/scaling status tests and legal companion pass | ACCEPTED_LIMITATION | ACCEPTED — no reachability claim |

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

## 7. Review disposition rule

The project owner has waived the external independent-review requirement for
MR-009 and replaced it with a three-pass internal review. This is an owner-directed
process change, not an independent third-party approval. The matrix records the
internal findings and explicit limitations; it must not be represented as an
independent signature. H723 runtime, WCET/RTA, target RAM/stack watermark, HIL,
flight, and S06-100..102 remain separate release gates.



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


## 9. R30.1 CI evidence

Validated code commit: 80670f118c5fdccce5a1f00f218a58a66c28bd5e.

- Host validation #43417453111: 6/6 jobs SUCCESS.
- M6/M7 host validation #43417453109: 5/5 jobs SUCCESS.
- M7 host validation #43417452817: 2/2 jobs SUCCESS.
- M5 Host 322,299/0; focused runner 430/0; M5 ASan/UBSan 322,299/0; focused ASan/UBSan 430/0; integration ASan/UBSan 1,075/0.
- S06-115 deterministic no-seed exact-H counterexample passed.
- Independent reviewer results remain PENDING; full-validator reachability for all three Horner rejection sites is not claimed.


## 10. R30.2 review handoff packet

### Code and review baselines

- Branch: `opt/stage-02-m5-06-safety-validator`.
- R30 rounded-intermediate implementation: `6dc0e5411409244a137f9b5663c5daea08b5f81e`.
- R30.1 fail-fast code baseline: `80670f118c5fdccce5a1f00f218a58a66c28bd5e`.
- R30.3 gamma-factor directed-rounding hardening: implementation commit `9af2d4864dfcc8bd4b9434749b67f3e5c2fd4e5a`; FP audit source-token/mutation-gate correction: `882028c6771180dbd997bcff19de48c88b617307`; proof, matrix and CI self-test wiring: `00339febd045a85285a3df571268f3333c867aae`.
- R30 proof: `spec/history/implementation/guidance/M5_06_6_MR009_PROOF.md`.
- Production validator: `middleware/guidance/trajectory_planning/m5_trajectory_safety.c`.
- M6 consumer: `middleware/model/control/outer_loop/quad_x4/position_tracking/control_reference.c`.
- Focused C tests/runner: `tests/guidance/test_m5_main.c`, `tests/guidance/mr009_focus_main.c`, `tests/guidance/run_mr009_focus.sh`.
- FP build/object audit: `rtconfig.py`, `tests/guidance/check_consumer_fp_contraction.py`.

### Reproducible remote evidence

- Host validation #43417453111: 6/6 jobs SUCCESS; M5 Host 322,299/0; MR-009 focused 430/0; M5 ASan/UBSan 322,299/0; focused ASan/UBSan 430/0; integration ASan/UBSan 1,075/0.
- M6/M7 host validation #43417453109: 5/5 jobs SUCCESS.
- M7 host validation #43417452817: 2/2 jobs SUCCESS.
- FP audit job 131858854716, within Host validation #43417453111: `model_check=MATCH`, compiler `arm-none-eabi-gcc 14.2.1 20241119`, production consumer and validator objects each `fused_operations=0`. The consumer object uses the exact `rtconfig.py` production flags including `-ffp-contract=off -frounding-math -fexcess-precision=standard`.
- FP source SHA256: validator `0335e8322ae68a8ed244d845e5cf332ae90748be62aad8a6dc2a31e89b3c9300`; consumer `48fa1f629380ce354617af7082e7f19bd81974f00f9b9c126c9b4c690ff1c31b`.
- R30.3 validation on proof/audit commit `00339febd045a85285a3df571268f3333c867aae`: Host #43424620489 (6/6 jobs SUCCESS), M6/M7 #43424620578 (5/5 SUCCESS), M7 #43424620424 (2/2 SUCCESS). Host M5 regression 322,299/0; MR-009 focused 430/0; M4/M5 integration 1,075/0; S06-099 60,002 offsets/0 failures; S06-108/109/113 96/96/76 checks with 0 failures. FP audit job 131882878679: `M5_FP_MODEL_SELF_TEST_PASS (9 mutations detected)`, `model_check=MATCH`, both objects `fused_operations=0`.
- Documentation synchronization HEAD `241784357e7a8f3b92514612307df66a13d51fe9` was also revalidated: Host #43425062263 (6/6), M6/M7 #43425062339 (5/5), M7 #43425062346 (2/2) all SUCCESS; code/proof baseline remains `00339febd045a85285a3df571268f3333c867aae`.
- Remote artifacts: `m4-m5-host-regression` ID `14032860881`, `m5-cortex-m7-compile` ID `14032780901`, `abi-evidence` ID `14032786372`. No local evidence ZIP is claimed.
- R30.3 CI artifacts: Host `m5-cortex-m7-compile` ID `14036950533`, `abi-evidence` ID `14036490022`, `m4-m5-host-regression` ID `14035697038`; M6/M7 `cortex-m7-module-compile` ID `14036700759`, `m6-m7-host-evidence` ID `14036135841`, `sil-ground-stop-evidence` ID `14037190065`; M7 `m7-host-evidence` ID `14035836365`.

### Historical R30.2 handoff state (superseded)

The following was the R30.2 handoff state before the project owner waived external
review. It is retained only as history and is not the current disposition:
`READY_FOR_INDEPENDENT_REVIEW`, reviewer `NOT_ASSIGNED`, reviewer fields `TBD`.
The current internal disposition is recorded in §11; no independent reviewer or
third-party approval is claimed.


## 11. R31 owner-directed three-pass internal review (2026-11-13)

**Disposition: INTERNAL_THREE_PASS_REVIEW_COMPLETE_WITH_LIMITATIONS.**
External independent review is waived by the project owner. No independent
reviewer identity or third-party approval is claimed.

1. **Mathematical pass:** corrected the time-shift derivative for velocity and
   acceleration to use the actual rounded consumer coefficient arrays `vf` and
   `af`, rather than unrounded second/third derivatives of `cf`. Confirmed
   outward `s_max`, `delta_s`, and `s_eval_max`.
2. **Implementation pass:** rechecked coefficient operation order, 10/12/10
   rounding counts, gamma directed rounding, normal/subnormal bounds, strict
   RN overflow midpoint rejection, runtime FP environment checks, expanded
   `s_eval_max` propagation through coefficient/Horner/overflow/subnormal terms,
   and fail-fast behavior. No further production arithmetic defect found.
3. **Test/evidence pass:** updated S06-107 Python Fraction oracle to model
   binary32-rounded velocity/acceleration coefficients and their own derivative
   polynomials; S06-099 now checks both physical time and the exact production
   h*offset/step parameter. The dual-source FP audit checks 18 mutations.

**Accepted explicit limitations:** degree-5/4/3 Horner rejection branches are
tested through the production helper, not claimed reachable through a complete
product-domain validator input; H723 FPSCR runtime equivalence remains NOT_RUN;
40 ms M6 route preview is outside the 20 ms M5-06 proof domain.

### R31 follow-up audit hardening

The FP source audit now checks both compiled translation units and the validator
replica, including the expanded s_eval_max use and the position/rounded-velocity/
rounded-acceleration time derivatives. Its mutation suite contains 18 cases.
The latest audit result is recorded against the final branch HEAD after CI.

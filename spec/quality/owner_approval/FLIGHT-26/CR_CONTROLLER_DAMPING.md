# FLIGHT-26 Near-Ground Horizontal Limit Cycle — Change Request Package

Status: **EXPERIMENTS COMPLETE / FIX NOT MERGED / AWAITING CONTROL REVIEW**

Scope: QUAD_X4 / STM32H723ZGT6 / `OBSTACLE_SOURCE_MODE=NONE` / `BENCH_INHIBIT`.
Windows-native FMT-SIM plus Flight Host SIL only. H723 target, HIL, WCET and
flight stay NOT_RUN.

## 1. Symptom

After touchdown the airframe keeps a rotating horizontal limit cycle of roughly
0.2 m/s at ~1.4 s period, measured identically in navigation velocity and FMT
truth. M4's `LandDetector` needs `|vh| <= 0.20 m/s`, `|vz| <= 0.15 m/s`,
`|a_z| <= 0.5 m/s^2` continuously for 2 s, so `LANDED` never latches and the
terminal ACK / DISARM chain cannot complete (FLIGHT-17/18/19/23/24).

The acceleration clause is not the binding one: reconstructing the FMS
velocity-derived filter from an R39 Return trace (251 samples, last 5 s) gives
`|a_z| <= 0.237 m/s^2` (0.263 m/s^2 margin) and `|v_z| <= 0.113 m/s`, while
`|v_h|` exceeds 0.20 in 100 of 251 samples.

## 2. FMT-SIM tuning experiments

The tuning batches changed tracked files and have diagnostic status in their
run manifests. The acceptance gates still require a stable, clean checkout.

| # | Change | Result |
|---|---|---|
| 1 | `mpc_q_diag` velocity 3/3/4 -> 10/10/12 | worse: 2 m hover still 0.29-0.37 m/s, descent never starts |
| 2 | `mpc_r_diag` 0.8/0.8/1.0 -> 2.4/2.4/3.0 | touchdown bounce, final `contact=NO_CONTACT` |
| 3 | `rate_kp_nm_per_radps` 0.12 -> 0.3, `rate_kd_nm_per_radps2` 0.002 -> 0.004 | **Land PASS: 47,125 checks / 0 failures**, `landed=1 evidence=0x3 stable_ms=2020`, terminal ACK and DISARM complete, near-ground descent 0.323 m/s |

Experiment 3 shows that the inner attitude/rate gains affect settling. The
two tested outer-loop adjustments failed to restore settling. The available
data leave the relative contributions of the inner loop and ground dynamics open.

## 3. Why it is not merged

- The full batch under experiment 3 kept Land passing (43,166/0) but Return only
  ran out of the fixed 32 s horizon, and Mission did not produce a usable
  verification in either attempt.
- The Mission instability was later traced to FLIGHT-29 (navigation estimate
  diverging from truth), which reproduces **without** any controller change. The
  earlier attribution "tuning caused Mission divergence" is withdrawn.
- A SIM_ONLY rate-loop change still alters closed-loop behaviour in every phase
  and needs an independent control review, not a single Land pass.

## 4. Proposed disposition (two independent decisions)

### Decision A — Return scenario horizon

Return touches down at 28-29 s inside a fixed 32 s horizon; the 2 s dwell plus
the ACK/DISARM exchange cannot finish. Extending only the horizon (Land/Return
40 s) was tested **without** any controller change and did not close Return
(61,002/3), so the horizon is a secondary factor, not a fix. It is still needed
once the damping works, and it changes no threshold. Approval is requested to
extend the FMT-SIM horizon independently of the tuning decision.

### Decision B — SIM_ONLY rate-loop damping

Ship `rate_kp 0.3` / `rate_kd 0.004` **after** FLIGHT-29 stops poisoning the
Mission estimate, with the full test plan below. Alternatively keep the current
gains and treat FLIGHT-26 as a design defect that needs a control-identification
campaign (step/sweep on the Host plant) before any gain change.

## 5. Test plan for Decision B

- M4/M5/M6/M7 Host, ASan/UBSan, M4/M5 integration, Cortex-M7 compile, Spec Audit,
  production ABI, default Release link (the T01-T30 runner).
- Control-loop step and frequency sweep on the Host plant to show the rate loop
  is not merely slower but better damped.
- Native FMT-SIM Land -> Return -> Mission on a clean `main` with the formal
  gates; Land must stay at 0 failures and Return must complete the full dwell and
  disarming.
- Rollback: revert `config/controller_tuning.yaml`, regenerate
  `quad_x4_controller_sim_config.c` and `ground_parameters.json`, rerun the
  runner.

## 6. What the owner is asked to decide

1. **A**: approve the FMT-SIM horizon extension (no threshold change).
2. **B**: approve in principle the SIM_ONLY rate-loop damping, to be executed
   after FLIGHT-29 and validated by the plan above.

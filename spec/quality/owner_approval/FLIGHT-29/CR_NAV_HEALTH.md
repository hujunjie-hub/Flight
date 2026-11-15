# FLIGHT-29 Navigation Health and Divergence — Change Request Package

Status: **R45 TRANSPORT TRIGGER FIX VERIFIED / STAGE B1 FAIL-CLOSED CAPABILITY LATCH IMPLEMENTED / R46 NATIVE RETEST PENDING**

Scope: QUAD_X4 / STM32H723ZGT6 / `OBSTACLE_SOURCE_MODE=NONE` / `BENCH_INHIBIT`.
All evidence below is Host/SIL and Windows-native FMT-SIM software evidence. It is
not H723 target, HIL, WCET or flight evidence.

## 1. Known facts

| Fact | Value | Source |
|---|---|---|
| Historical R41 navigation vs FMT truth separation (pre transport fix) | up to **10.441 m** horizontal | RUN_ID `FMT-SIM-LOCAL-20261114T191012782Z` |
| Mission touchdown error while diverged | **16.927 m** (limit 0.50) | RUN_ID `FMT-SIM-LOCAL-20261114T185231385Z` |
| `nav.valid_mask` during divergence | **0xff** (all fields still "valid") | same runs |
| `horizontal_position_std` | 0.062 m -> **0.81 m**, monotonic | same runs |
| Navigation acceleration | **-80.7 / -137.6 m/s^2** | RUN_ID `...T185231385Z` |
| Measurement rejections | `meas_rej = 2` only | same run |
| Reproducible without controller tuning | yes, unmodified `controller_tuning.yaml` | `f0f21cf36` run |
| First detectable separation | 1.37 m at `pos_std = 0.11` | detector run |

Existing symptom guards and the new navigation-capability response:

- **SIL detector** `7fd57508`: an armed scenario fails with one explicit
  `ESTIMATOR_DIVERGENCE` line when navigation and truth stay separated by more
  than 1.0 m horizontal / 0.8 m vertical for 500 ms.
- **M6 consumer gate** `7b55218e`: a valid-flagged navigation acceleration above
  20 m/s^2 per axis returns `STATUS_MODEL_OUT_OF_DOMAIN`.
- **M3 capability latch** `28748c71`: after five consecutive GNSS position or
  velocity NIS rejections, revokes only the affected navigation valid bits and
  makes the capability DEGRADED. The NIS gate remains unchanged; no automatic
  re-acceptance or source-level quarantine occurs.

## 2. Root cause (static analysis, 2026-11-15)

The filter has a normalised-innovation-squared gate but **no recovery path**:

| Code | Behaviour |
|---|---|
| `middleware/model/ins/kf_gins/trimmed_core/src/quad_core.cpp` | `if (config_.nis_gate_enable && nis > threshold) return kErrNisRejected;`；NIS rejection still does not auto-reseed the core |
| `middleware/model/ins/kf_gins/trimmed_core/include/kfgins/quad_types.hpp` | `kNisThresholdDof1 = 6.6348966`, `kNisThresholdDof3 = 11.3448667` (99 % chi-square) |
| `kf_gins_quad_port.cpp:107,229,232` | `nis_gate_enable` is configurable and `nis_rejected` / `last_nis` are exposed by the port |
| repo-wide search | no covariance inflation, no reset, no "re-accept after N rejections" anywhere |

The core can remain in a rejection deadlock: after the estimate and GNSS
position disagree, subsequent fixes can remain above the NIS threshold. This is
a real generic estimator hazard, but it is **not proven to be the cause of the
R41 Mission divergence**. Before commit 28748c71, M3 could continue publishing
the affected position/velocity capability as valid while the core rejected
updates. Stage B1 now revokes the affected navigation capability after five
consecutive rejects; it does not make the disputed source automatically
acceptable.

The historical trace signature was consistent with the hazard but did not
prove causality. R45 after the FMT-SIM transport fix showed 535 GNSS
position/velocity pairs, zero NIS rejections and maximum horizontal
navigation/truth separation of 0.055 m.

The original R41 trace alone did not identify the trigger. Stage A/R45 later
identified a separate simulation transport defect: the UDP receiver overwrote
earlier decoded state frames when a scheduler turn received a burst. Commit
ada6bd65 now delivers one complete state per SIL tick and preserves queued
frames. The subsequent native batch had no NIS rejection and 0.055 m maximum
horizontal separation. The earlier contact/barometer/attitude candidates are
withdrawn as explanations for this reproduced FMT-SIM failure.

### 2.1 Deterministic reproduction (2026-11-15)

`tests/estimator_backend/test_nis_recovery.cpp` (commit `4a3fdfaa`) drives the
production port ABI and records the deadlock:

```
first displaced fix (40 m, 0.2 m std): result=5 nis=69932240.851060 nis_rejected 0 -> 1
after 20 displaced fixes over 20 s:    accepted=1 rejected=21 unexpected_accepts=0 last_nis=3018.585036
```

Twenty seconds of covariance growth brought NIS from 6.99e7 down to 3.0e3, still
266 times the dof-3 threshold. The fixture injects a displaced GNSS sequence
into a nearly stationary INS. It measures persistent disagreement without an
independent source that identifies which side is wrong. Preserve rejection of
an isolated fix and of a sustained faulty source when validating recovery.

The estimator suite now runs in Host CI and the local T31 regression; FLIGHT-35
was closed after the CI workflow was added.

## 3. Stage A outcome — FMT-SIM transport trigger identified

R41's large navigation/truth separation was caused in the reproduced native test
path by state-frame loss in fmt_sim_host_receive(): one socket burst could
contain multiple valid states, but the SIL clock advances once per frame. The
old receive path decoded several frames into one output object and overwrote
earlier states.

- Commit ada6bd65 now buffers the datagram and returns at most one complete
  state per SIL scheduler turn; queued frames retain wire order.
- The loopback regression reproduces two states in one datagram plus a third
  queued datagram and verifies three distinct receive calls.
- Native R45 batch
  FMT-SIM-LOCAL-20261115T011452219Z-ada6bd65: Land 46,313/0, Return
  48,196/0, Mission 83,130/0; all three completed terminal ACK, DISARM ACK and
  DISARMED. Mission delivered 535 position/velocity pairs, had no NIS rejection,
  and maximum horizontal nav/truth separation was 0.055 m.
- This establishes the trigger for the reproduced FMT-SIM divergence. It does
  not prove that the production estimator is free of independent NIS-recovery
  hazards. The old contact/barometer/attitude candidates are not supported by
  the R45 evidence.

## 4. Stage B — fail-closed navigation capability latch

### Stage B1 — implemented in main

The safety response is not to lower the NIS threshold, inflate covariance until
a disputed source passes, or auto-accept a rejected fix.

- QuadCore maintains independent consecutive NIS rejection streaks for GNSS
  position and velocity. An accepted update of the same kind resets its streak;
  each streak saturates at five.
- Checkpoint schema version 2 stores the position and velocity streaks in
  reserved[0] and reserved[1]. The checkpoint remains 2,304 bytes, and
  replay/commit preserves the streak deterministically.
- After five consecutive rejects, M3 latches only the affected navigation
  capability invalid. Position and velocity are independent; barometer-derived
  vertical capability remains available where independently valid.
- NavigationCapability.solution_state becomes DEGRADED and
  EstimatorHealth.last_error_code records STATUS_NUMERIC_ERROR. M4's existing
  navigation-loss path pauses Mission and requests AUTO_LAND when the remaining
  capability supports it.
- This is a navigation-output fail-closed latch, not source-level quarantine:
  SensorUsePolicy is unchanged, GNSS is not marked FAILED, and no disputed
  measurement is auto-accepted. The latch is cleared only by the existing
  authorized estimator reset/re-alignment flow.
- NIS thresholds and nis_gate_enable are unchanged.

### Stage B2 — controlled estimator re-alignment (still open)

Automatic in-flight covariance inflation/re-seeding remains deliberately
unimplemented. The current evidence cannot determine whether a sustained
position disagreement means INS drift or a persistently displaced GNSS source.
A future recovery path must require independent source evidence and the
existing ground/DISARMED reset authorization, preserve rejection of isolated
outliers, and define the new state/origin epoch. This requires a separate
reviewed change request; it must not silently clear the Stage B1 capability
latch.

## 4.1 Stage C — contract-level health

No new ICD field is required for Stage B1: existing
NavigationSnapshot.valid_mask, NavigationCapability.degraded_mask and
solution_state already communicate the revoked capability. Any future request
for a more specific cross-module NIS reason code or explicit bounded
position-uncertainty contract must be reviewed as a separate ICD/ABI change.

## 5. Verification attached to Stage B1

- tests/estimator_backend/test_core.cpp: transient rejection resets on a
  consistent update; sustained position/velocity rejection counters saturate;
  checkpoint v2 saves/restores both counters and rejects malformed reserved
  values.
- tests/navigation/test_m3_bridge.cpp: replay of five displaced GNSS position
  fixes carries the rejection streak through shadow checkpoint and commit; the
  M3 capability latch is sticky until explicit reset.
- tests/estimator_backend/test_nis_recovery.cpp: a repeated displaced source
  remains rejected; the fixture has no independent truth and does not label
  either INS or GNSS as correct.
- CI on commit 28748c717e83b868ae58957c04bcd9dbe1c23e1b: Host validation
  #43667830819 7/7 jobs SUCCESS; M6/M7 #43667830805 5/5; M7 #43667830814
  2/2. Navigation/control 4,620/0; M3 bridge 131/0; KF-GINS core 2,790/0;
  NIS mechanism 16/0; M4/M5 integration 1,042/0; Spec Audit 145 types / 8
  checks PASS; Cortex-M7 production compile PASS.
- Native R46 FMT-SIM Land -> Return -> Mission on the exact new main remains
  required. H723 target, physical HIL, WCET/RTA and flight remain NOT_RUN.

## 6. What the owner is being asked to approve

1. Stage A trigger identification is complete: the reproduced R41 divergence was caused by FMT-SIM UDP frame overwrite; R45 passed after the transport fix.
2. The owner instruction to start repair authorized Stage B1. The fail-closed capability latch is implemented and all three CI workflows passed.
3. FLIGHT-29 remains OPEN pending R46 native batch, plus a separately reviewed Stage B2 controlled re-alignment design. No automatic estimator re-seed has been authorized.

## 7. Native R45 batch and required R46 retest

R45 RUN_ID FMT-SIM-LOCAL-20261115T011452219Z-ada6bd65 passed Land
46,313/0, Return 48,196/0 and Mission 83,130/0. All three scenarios completed
terminal ACK, DISARM ACK and DISARMED. Mission had 535 position/velocity pairs,
zero NIS rejections and maximum horizontal nav/truth separation 0.055 m. The
transport fix is verified locally; R45 is not the post-Stage-B1 baseline.

R46 must run the full Land -> Return -> Mission batch after fetching the latest
main commit 28748c717e83b868ae58957c04bcd9dbe1c23e1b, with the existing
350 ms status period, UDP 14653, hover 0.62 and explicit loopback. Archive the
ZIP, sidecar SHA256, manifest, per-scenario sil_flight.log, exact HEAD and
before/after worktree state. A nominal run must not spuriously latch either
navigation capability. H723 target/HIL/WCET/flight remain NOT_RUN.


## 8. 2026-11-15 implementation record

- Commit: 28748c717e83b868ae58957c04bcd9dbe1c23e1b —
  fix(ins): fail closed on sustained GNSS NIS rejection.
- Files: trimmed-core NIS streak state and checkpoint v2; M3 sticky capability
  latch; core, replay and M3 bridge regression tests.
- No public ICD/ABI layout, NIS threshold, controller tuning or source-level
  SensorUsePolicy was changed. Checkpoint remains 2,304 bytes.
- Rollback is a single commit revert. Keep FLIGHT-29 OPEN until R46 and the
  separate controlled re-alignment decision are complete.

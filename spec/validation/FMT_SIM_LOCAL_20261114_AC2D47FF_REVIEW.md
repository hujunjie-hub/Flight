# R35 FMT-SIM Local Archive Review — 2026-11-14

## 1. Identity and integrity

- RUN_ID: `FMT-SIM-LOCAL-20261114T134046188Z-ac2d47ff`
- Tested branch: `main`
- Tested HEAD: `ac2d47ff3d36153960d6103056ae2aff1832a8a1`
- HEAD/upstream: `origin/main` exact match; clean before/after; worktree stable; fetch exit 0.
- Formal gates: explicit loopback confirmation = true; UDP 14653; hover 0.62; state timeout 350 ms; graphics adapter 1.
- FMT-SIM EXE SHA256: `4f9a0f12c78e3964b3e472ee579c547d13ea629456dc6a4c7fe72ea18ae2a6d6`
- Settings SHA256: `f556717bcd96a32206065d5dbb97a8f9e57e2267c055bd75e3d6bc501102948b`
- SIL build: PASS, exit 0; binary SHA256 `3d88ddba5aab3f1293e8e5712e32836acb8c3077110f7bd61bd1aa18ddfcaa43`.
- ZIP SHA256: `00c926b92fd95d28494bff2b5daa3b7cc9488402ab18643ece6f7e0e090b4e7d` (sidecar matches).
- Manifest SHA256: `4d1155034f09dc993c18aa5433706caea462a881d3ef21039189b1c78b28c93c`.
- ZIP: 104 entries; CRC test PASS; 102 entries in SHA256SUMS; all 102 hashes and manifest sidecar verified.
- Manifest overall: FAIL; `issue_closure_eligible=false`.

The R35 tested HEAD predates the new M5 low-speed contact-hold fix `69626899249e1fd926a0a903447f2ad763f24ba1`. The three commits between R35 HEAD and the then-current documentation HEAD were documentation-only; no R35 result is evidence for `69626899`.

## 2. Scenario results

| Scenario | Checks | Failures | Longest state gap | Final 0.50 m max descent | Result |
|---|---:|---:|---:|---:|---|
| Land | 48,801 | 4 | 95.0 ms | 0.340 m/s | FAIL |
| Return | 39,389 | 4 | 108.8 ms | 0.316 m/s | FAIL |
| Mission | 67,112 | 3 | 92.7 ms | 0.330 m/s | FAIL |
| **Total** | **155,302** | **11** | **108.8 ms max** | **all <= 0.40 m/s** | **FAIL** |

All state gaps are below the formal 350 ms gate. The previous R34 Mission near-ground descent failure (0.674 m/s) is not reproduced: all three R35 scenarios satisfy the 0.40 m/s limit.

Mission waypoint closest distances: 0.257 m and 0.327 m. Return touchdown distance from Home: 0.067 m.

### Land

- Four failed assertions: terminal ACK, DISARM ACK, final DISARMED state, and ground-contact confirmation.
- At t=23.100 s the advisory contact candidate reached M5 while measured horizontal speed was approximately 0.248 m/s.
- M5 latched the measured contact point before horizontal motion had settled. The final sample at t=32.000 s reports contact=NO_CONTACT, landed=UNKNOWN, evidence=0x1, and the vehicle remains ARMED in AUTO_LAND.
- Near-ground descent metric: 0.340 m/s PASS.

### Return

- Four failed assertions: terminal failsafe guard, terminal ACK, DISARM ACK, final DISARMED state.
- At t=25.780 s, the advisory candidate was published while horizontal speed was approximately 0.200 m/s.
- At t=25.800 s, M5 rejected the contact-hold replan with reason 1550 / JERK constraint=4. The control path then rejected with status -11 and FMS entered FAILSAFE_TERMINAL.
- Near-ground descent metric: 0.316 m/s PASS; touchdown distance: 0.067 m.

### Mission

- Three failed assertions: terminal ACK, DISARM ACK, final DISARMED state.
- At t=37.020 s, the advisory candidate reached M5 while horizontal speed was approximately 0.301 m/s. The contact-hold stage latched before horizontal motion settled.
- Final state remains ARMED, landed=UNKNOWN, evidence=0x1. Both waypoint and near-ground speed metrics pass.
- Near-ground descent metric: 0.330 m/s PASS; waypoint closest distances: 0.257 m / 0.327 m; touchdown distance: 0.219 m.

## 3. Root cause

The R34 fixes preserved the active M5 trajectory as the replan start and separated horizontal Home alignment from vertical descent. R35 shows a remaining defect in the consumption of the M4 advisory contact hint:

- M4 publishes UNKNOWN + GROUND_CONTACT as an advisory candidate. This is not LANDED and does not grant DISARM authority.
- M5 treated that candidate as immediately sufficient to enter CONTACT_HOLD and retarget to the instantaneous measured position.
- At touchdown, both the measured navigation velocity and the active reference velocity could still be significant. The new target was too close to the active reference's moving start state, so Return's quintic replan exceeded the jerk envelope. Land and Mission could create a reference but stopped the Home-approach/settling behavior too early, and did not reach continuous LANDED evidence or DISARM.
- This is a post-handoff M5 consumer/readiness defect, not a regression in the M4 advisory evidence delivery. R34/R35 both show evidence=0x1 reaching M5; #22 remains the separate handoff/safety-review issue.

## 4. Fix committed after R35

Commit `69626899249e1fd926a0a903447f2ad763f24ba1`:

- M5 now consumes UNKNOWN + GROUND_CONTACT only when both measured navigation velocity and the active validated trajectory-reference velocity are within 0.15 m/s in horizontal and vertical components.
- A confirmed LANDED snapshot preserves the current validated trajectory target instead of introducing an unnecessary last-moment replan during the ground-stop/disarm handoff.
- Added `landing_contact_waits_for_low_speed`: compares the same high-speed Home-return trajectory with and without an advisory contact hint and verifies that the hint does not retarget prematurely.
- Existing low-speed contact-hold regression still covers raw nav vertical velocity 0.128 m/s and raw acceleration 2.95 m/s² while the active reference remains the safe planning boundary.
- M4 ground-contact, horizontal/vertical velocity, acceleration, 2 s dwell and DISARM gates were not relaxed.

### CI on exact fixed-code HEAD

- Host [#43603447251](https://github.com/hujunjie-hub/Flight/actions/runs/43603447251): 6/6 jobs SUCCESS.
- M6/M7 [#43603447288](https://github.com/hujunjie-hub/Flight/actions/runs/43603447288): 5/5 jobs SUCCESS.
- M7 [#43603447340](https://github.com/hujunjie-hub/Flight/actions/runs/43603447340): 2/2 jobs SUCCESS.
- M4 Host: 969 checks / 0 failures.
- M5 Host and ASan/UBSan: 329,670 checks / 0 failures each.
- M4/M5 integration and integration sanitizer: 1,042 checks / 0 failures each.
- MR-009 focused: 430 checks / 0 failures.
- Spec Audit round 3: 145 types / 8 checks PASS.
- Cortex-M7 M4/M5/M6/M7 production-module compile: PASS.

## 5. Disposition

R35 is a valid formal native FMT-SIM run, but all three scenarios fail terminal completion. Do not close #17/#18/#19/#21/#23. #21's near-ground speed metric is now passing, but its existing full Mission/manifest closure rule remains unmet. #22 remains open for independent review of the M4 advisory handoff.

Native FMT-SIM after `69626899`: **NOT_RUN**. Owner must fetch the latest `origin/main` and rerun the full Land → Return → Mission batch under the unchanged 350 ms / UDP 14653 / hover 0.62 / explicit loopback gates. Do not relax safety thresholds, dwell or DISARM authorization.

FMT-SIM software SIL does not replace B07 H723 target execution, physical HIL, WCET/RTA, target stack/RAM watermark or flight evidence.

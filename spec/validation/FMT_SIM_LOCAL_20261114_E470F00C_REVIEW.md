# FMT-SIM Local Archive Review — R32

> **Disposition: FORMAL_ARCHIVE_VALID / LAND_PASS / RETURN_PASS / MISSION_FAIL / FIX_COMMITTED / NATIVE_RETEST_REQUIRED**

## 1. Run identity and formal-gate integrity

- RUN_ID: `FMT-SIM-LOCAL-20261114T072713056Z-e470f00c`
- Tested branch / HEAD: `main` / `e470f00cd539f438b1923b0faa844456ba2d3b59`
- `origin/main` HEAD matched tested HEAD; fetch exit code 0.
- Worktree clean before and after; branch and HEAD stable for the run.
- Explicit loopback confirmation: true; UDP/HIL port 14653; graphics adapter 1; hover 0.62; timeout 350 ms.
- PowerShell 7.6.10; MinGW GCC/G++ 16.2.0; Python 3.14.8; Git 2.55.0.windows.5.
- FMT-SIM executable SHA256: `4f9a0f12c78e3964b3e472ee579c547d13ea629456dc6a4c7fe72ea18ae2a6d6`
- SIL executable SHA256: `9bb05090d6b4a537209442c35a9f72af2ba7f3e4547cdcb46fd8b52a2a4c2e42`
- Settings SHA256: `f556717bcd96a32206065d5dbb97a8f9e57e2267c055bd75e3d6bc501102948b`
- Build: PASS / exit 0.

ZIP SHA256 `93f9ad1289a0b30f0a92afa3dee039b37b1e8c03435db54354a2986cdf3eb76d` matches the external sidecar. ZIP has 104 entries and passes CRC validation. Manifest SHA256 `50cd771b3b810bdf8d5dd08c761b201b4e5a19770ee4b9405d93799d0e039234` matches `manifest.sha256`. All 102 entries in `SHA256SUMS.txt` match. Archive integrity is PASS; this does not make the failed scenario pass.

## 2. Scenario results

| Scenario | Status | Checks | Failures | Commits | Inhibits | Longest state gap |
|---|---:|---:|---:|---:|---:|---:|
| Land | PASS | 48,043 | 0 | 10,968 | 5,721 | 145.3 ms |
| Return | PASS | 44,667 | 0 | 10,080 | 5,721 | 64.4 ms |
| Mission | FAIL | 62,793 | 3 | 14,808 | 5,806 | 65.5 ms |
| **Total** | **FAIL** | **155,503** | **3** | **35,856** | **17,248** | **145.3 ms max** |

All state gaps are below the strict 350 ms limit. Manifest overall is FAIL and `issue_closure_eligible=false` because one requested scenario did not pass.

## 3. Mission's three remaining failures

Exact assertions:

```text
FAIL 1703: near_ground_max_down > 0.0f && near_ground_max_down <= 0.40f
FAIL 1705: disarm_ack == COMMAND_ACK_COMPLETED
FAIL 1710: runtime_data.fms.arm_state == ARM_STATE_DISARMED
```

Key samples:

- At 24,000 ms: height 1.397 m, NED down velocity 1.023 m/s, horizontal velocity components -1.097 / -0.579 m/s.
- At 25,000 ms: height 0.484 m, NED down velocity 0.665 m/s, horizontal velocity components -1.270 / -0.742 m/s. The measured maximum in the final 0.50 m was 0.665 m/s, above the 0.40 m/s acceptance bound.
- Return request 2163 completed at 41,140 ms: `[ack] ... state=3 reason=0`.
- The harness submitted DISARM request 3064; at 41,160 ms FMS rejected it: `state=0 reason=11`.
- Final snapshot: arm state 2, mode 6, ground contact 1, landed state 0 (UNKNOWN), nav horizontal speed 0.204 m/s, vertical speed 0.027 m/s, vertical acceleration 0.389 m/s², return touchdown distance 0.352 m.

The FMS disarm gate requires a current LANDED state and stable dwell; it was not relaxed. The final horizontal speed is just above the LandDetector's 0.20 m/s stable-ground gate. The return reference still targeted the exact Home XY point after LANDED confirmation, so the vehicle could continue moving horizontally during the disarm handoff. The touchdown XY hold correction below latches the measured contact point and retains it if the LANDED snapshot briefly drops.

## 4. Source-level causes and correction

Two separate corrections were made on main:

1. **Near-ground descent envelope.** Before this change, M5 constrained vertical speed to 0.12 m/s only in the final 0.20 m. The native FMT-SIM acceptance measures the entire final 0.50 m and requires no more than 0.40 m/s. The Mission return trajectory crossed this band at 0.665 m/s. M5 now proves a 0.40 m/s bound after crossing the 0.50 m plane, then retains the stricter 0.12 m/s bound after crossing the 0.20 m plane.
2. **Touchdown horizontal hold.** Once the FMS LANDED snapshot is observed, M5 now latches measured touchdown north/east as the horizontal target. It does not continue chasing Home XY during disarm handoff, and it retains this target if the LANDED snapshot briefly drops. The FMS LandDetector thresholds, ground-contact evidence, dwell, and disarm rejection policy are unchanged.

The native run did not include these changes; it was executed on pre-fix HEAD `e470f00c`. Do not describe the changes as FMT-SIM-verified until the owner runs the next archive on the updated main.

## 5. Fix commit and automated verification

Fix commit: `3d8d29b5dfda21237f8ed02db3c31223c656bfbe`.

- [Host validation #43574875444](https://github.com/hujunjie-hub/Flight/actions/runs/43574875444): 6/6 jobs SUCCESS.
- [M6/M7 host validation #43574875363](https://github.com/hujunjie-hub/Flight/actions/runs/43574875363): 5/5 jobs SUCCESS.
- [M7 host validation #43574875394](https://github.com/hujunjie-hub/Flight/actions/runs/43574875394): 2/2 jobs SUCCESS.
- M5 Host: 326,004 checks / 0 failures; M5 ASan/UBSan: 326,004 / 0.
- M4 Host: 945 / 0; M4/M5 integration: 1,018 / 0; integration sanitizer: 1,018 / 0; MR-009 focused: 430 / 0.
- New Host tests `landing_alignment_and_speed` and `landing_contact_latches_xy` both PASS.
- Cortex-M7 production-module compile, MPC RAM checks, ABI report, Spec Audit, source-layout/time-service, M6/M7 sanitizer, M7 guard, and runtime ground-stop SIL all PASS.

## 6. Disposition and next action

- Keep GitHub Issues #17 Land, #18 Return and #19 Mission OPEN. Land and Return scenarios passed in this run, but the formal manifest says issue closure is ineligible because Mission failed.
- Owner must fetch the latest `main`, confirm clean worktree and exact `HEAD == origin/main`, then rerun the formal three-scenario batch with the same 350 ms / UDP 14653 / hover 0.62 / explicit loopback settings.
- The updated SIL logs will include LandDetector `stable_duration_ms`, `evidence_mask`, and confidence, plus the exact time and height of the maximum descent in the final 0.50 m.
- Do not close ENG-07 issues based on Host/CI PASS. FMT-SIM PASS does not replace B07 H723 runtime, physical HIL, WCET/RTA, target stack watermark, or flight evidence.

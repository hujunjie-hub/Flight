# Agent C status submissions (D-REV-04, C part)

C's consolidated submissions so D can close the C part of `D-REV-04`
(HANDOFF vs worktree inconsistency). C does not edit `AGENT_HANDOFF` or `09`;
merging/closing is a D action.

| Task | Revision | State | Submission |
|---|---|---|---|
| `T191-C-FMS` | 3 | independent preparation delivered (Host 16/16, ARM compile PASS) | `m4/status_request_T191-C-FMS.json` |
| `T191-C-M5` | 2 | partial: NONE-obstacle-source slice only (Host 10/10, ARM PASS) | `m5/status_request_T191-C-M5.json` |
| `T191-C-M6` | 1 | NOT_STARTED, blocked (MPC solver package/ABI, ControlModelPort, C-M5/B-REPLAY acceptance) | `m6/status_request_T191-C-M6.json` |
| `T191-C-M7` | 1 | NOT_STARTED, blocked (C-M6, A DShot/actuator submit contract) | `m7/status_request_T191-C-M7.json` |

Machine index (revision, state, path, SHA256): `index.json`; regenerate with
`python applications/core/status_requests/build_submissions.py`
(no spec/HANDOFF file is touched).

## T191-D-REVIEW opinion

C's non-author opinion (revision 2, post-fix) is on file and **submitted for D to merge**:

- `applications/core/m4/reviews/C_REVIEW_T191-D-REVIEW.md`
- `applications/core/m4/reviews/c_review_checks.json` (22 checks: 15 PASS / 1 FAIL / 6 OBS)\n- `applications/core/m4/reviews/C_REVIEW_SUBMISSION.json` (submission record + SHA256)
- `applications/core/m4/reviews/c_review_checks.py`

Conclusion: `ACCEPT_WITH_CONDITIONS`. Key C findings: C-REV-01 (`mission_epoch`
missing from `OperationRequest`/`ProductOperationPayload`), C-REV-02 (no
defined geofence configuration entry), C-REV-03 (no registry for real
`output_policy_id>=0x100`). This opinion is not an ACCEPTED state, not a Gate A
sign-off, and does not replace B's opinion or user approval.

## What D needs to do for the C part of D-REV-04

1. Merge/record the four submissions (or record `BLOCKED` for M6/M7 as
   requested) and update `AGENT_HANDOFF`.
2. Keep `D-REV-04` open until B submits its own `T191-D-REVIEW` opinion.
3. Register executor-run Host/ARM evidence in `09` only where actually run;
   keep target/WCET/SIL/HIL and independent review `NOT_RUN`.
4. Open/route the `mission_epoch` and geofence-entry CRs from C's review.
## Current state (2026-11-05)

| Task | HANDOFF | C state | What is needed |
|---|---|---|---|
| T191-C-FMS | REVIEW/seq5 | CR-C01/C02 implemented; waiting for CR finalization | D/A: CR independent review + user approval |
| T191-C-M5 | READY/seq4 | NONE-source slice prepared | Start after C-FMS leaves REVIEW |
| T191-C-M6 | READY/seq3 | NOT_STARTED | D/A: MPC solver package + ControlModelPort ABI; accept C-M5/B-REPLAY |
| T191-C-M7 | READY/seq3 | NOT_STARTED | A: DShot/actuator submit contract (OUT-DSHOT missing); then C-M6 |

Machine-readable: status_update_20261105_C.json and index.json.
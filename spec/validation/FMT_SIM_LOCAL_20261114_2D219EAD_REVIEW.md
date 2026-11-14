# FMT-SIM-LOCAL-20261114T160601185Z-2d219ead 归档复核（R37）

> **Disposition: RECEIVED / HASH_VALID / BUILD_PASS / LAND_FAIL / RETURN_FAIL (NEW #25) / MISSION_FAIL (4 mm RETURN_ERROR)**
>
> Scope: Windows native FMT-SIM + Flight Host SIL software simulation only. Not H723 target, physical HIL, WCET/RTA, stack watermark or flight evidence.

## 1. Source and environment

| Item | Record |
|---|---|
| RUN_ID | `FMT-SIM-LOCAL-20261114T160601185Z-2d219ead` |
| Branch / tested HEAD | `main` / `2d219ead6e3bcdbc5adff2fb7dfa365ab2a15bbb` |
| Upstream | `origin/main`, freshly fetched by the runner, identical to HEAD |
| Worktree | clean before/after, stable |
| New commits in build | `866f2d79` (brake horizontal motion before landing descent), `50713306` (keep landing brake inside horizontal settle envelope), `dd82cfd6` / `2d219ead` docs |
| FMT-SIM SHA256 | `4f9a0f12c78e3964b3e472ee579c547d13ea629456dc6a4c7fe72ea18ae2a6d6` |
| settings SHA256 | `f556717bcd96a32206065d5dbb97a8f9e57e2267c055bd75e3d6bc501102948b` |
| Gates | UDP 14653, 350 ms, hover 0.62, adapter 1, explicit loopback, `main`/clean/HEAD==origin/main all satisfied |
| Host / toolchain | Windows 11 Pro 10.0.26300, PowerShell 7.6.10, MinGW GCC/G++ 16.2.0, Python 3.14.8, Git 2.55.0.windows.5 |
| Command | `pwsh -NoProfile -File tests/simulation/run_fmt_sim_regression.ps1 -FmtSimExe build/validation/sim/fmt_release/FMT_SIM/Binaries/Win64/FMT_SIM.exe -FmtHilPort 14653 -FmtStateTimeoutMs 350 -FmtGraphicsAdapter 1 -FmtHover 0.62 -ConfirmLoopbackAddress` |

The previous local R36 documentation commit was superseded by `origin/main`; preserved as `build/validation/fmt-sim-local/0001-docs-eng-07-record-R36-FMT-SIM-retest-on-main-3fb4dc.patch`. Nothing was pushed.

Note: the wrapper's first `git fetch origin` hit the same TLS/proxy failure, but `git pull --ff-only` and the runner's own fetch succeeded; the manifest records `fetched origin successfully: True` and `upstream matches HEAD: True`.

## 2. Integrity

| Item | Result |
|---|---|
| SIL build | PASS |
| manifest.json SHA256 | `dca7ed7059beeb14bb83d41d7a4cfc7c5aca9d6a6945fa804fead14c3366eca1` (matches `manifest.sha256`) |
| SHA256SUMS.txt | 102 entries, all verified, 0 mismatches |
| ZIP SHA256 | `f08b39235e99265abed27520b84a8998a569e3fdd552d93fd534431fd213073c` (matches sidecar) |

## 3. Scenario results

| Scenario | Result | Exit | Checks | Failures | Longest gap | Near-ground descent |
|---|---|---:|---:|---:|---:|---:|
| Land | FAIL | 1 | 48,801 | 3 | 144.9 ms | 0.315 m/s |
| Return | FAIL | 1 | 17,088 | 6 | 93.0 ms | n/a (aborted at ~2 m) |
| Mission | FAIL | 1 | 67,112 | 4 | 102.8 ms | 0.340 m/s |
| **Total** | — | — | **133,001** | **13** | — | Land/Mission ≤ 0.40 |

Failing assertions:

- Land: `terminal_ack`, `disarm_ack`, `arm_state` only.
- Return: the same three plus `FAIL 1695` (terminal failsafe), `FAIL 1719` (near-ground metric zero because it never reached the band) and `FAIL 1727` (no ground contact).
- Mission: the three plus `FAIL 1732: return_error <= .5f` — touchdown distance `0.504 m` against the `0.50 m` limit.

## 4. What changed versus R36

### Land — horizontal gate cleared, acceleration gate remains

Final: `landed=0 stable_ms=0 evidence=0x1`, `contact=1`, `nav_vh=0.166` (inside the 0.20 m/s horizontal gate) but `nav_ad=0.529 m/s²` (0.029 above the 0.5 m/s² gate). LANDED still never latches. Near-ground descent improved to `0.315 m/s`; the vehicle no longer loses ground contact.

### Return — new regression: RETURN handoff rejected at ~2 m (#25)

About 80 ms after the RETURN command, at truth height ~1.99 m and with `contact=2 landed=2` (still airborne):

```text
[settle] t=11180 ... v=0.1008,-0.0041,0.2816 a=0.0020,0.1434,8.6231 ... guidance=4/1550 constraint=4
[settle] t=11200 ... scope=4 control_sample=0 guidance=4/1550 constraint=2
[final] t=12.200s arm=2 mode=6 (failsafe=3, cause=18)
```

The plan built at the RETURN handoff is rejected as `M5_VALIDATION_VEHICLE_ENVELOPE_INVALID` (JERK then SPEED) while the estimator carries an ~8.6 m/s² vertical transient. Permit revoked, terminal failsafe, RETURN ACK cancelled. This is not the contact-hold path; it is a new failure of the horizontal-brake/staging change (`866f2d79`/`50713306`) and is tracked as new issue #25.

### Mission — 4 mm touchdown-distance miss

Near-ground descent `0.340 m/s` (pass), waypoints 0.238/0.285 m (pass), no failsafe, but the return touchdown distance became `0.504 m` (limit `0.50`) — the first `return_error` failure in this campaign. `landed=1` still never latches (final `nav_vh=0.236`, `nav_ad=-1.762`).

## 5. Issue disposition

- New **#25** (S1, M5): RETURN handoff re-plan rejected at ~2 m, terminal failsafe.
- #24 (horizontal settle before descent): partially effective for Land/Mission, but the handoff regression and LANDED gate remain; keep OPEN.
- #23: LANDED still never latches; Land is now blocked only by the acceleration gate (`0.529`), Mission by horizontal speed (`0.236`). Keep OPEN.
- #19: add the 4 mm `return_error` failure; keep OPEN.
- #17 / #18: still fail; #18 is now blocked first by #25.
- #21: Land/Mission pass (`0.315`/`0.340`); Return not measurable this run; keep OPEN.
- #22: M4 advisory handoff unchanged; keep OPEN.
- No threshold, assertion, scenario input or timing gate was relaxed.

## 6. Scope limits

- Windows native FMT-SIM software SIL with `SIM_CONTROLLED_STOP`; synthetic Host map/plant/sensors.
- Not H723 target, physical HIL, WCET/RTA, stack watermark, DShot/ESC timing or flight evidence. B07 remains NOT_RUN; M5-07 must not start.

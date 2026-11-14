# FMT-SIM-LOCAL-20261114T045144621Z-304fbd12 归档复核

> **Disposition: RECEIVED / HASH_VALID / BUILD_PASS / THREE_SCENARIOS_FAIL / OLD_HEAD_NOT_CURRENT**

## 1. 归档与代码来源

- RUN_ID：`FMT-SIM-LOCAL-20261114T045144621Z-304fbd12`
- Tested branch：`opt/stage-02-m5-06-safety-validator`（历史分支）
- Tested HEAD：`304fbd12078679d46b80cb626aba9eb0727b6914`
- Tested upstream：`origin/opt/stage-02-m5-06-safety-validator`，与 tested HEAD 完全一致
- Worktree：测试前/后均 clean，run 中 HEAD/branch 未变化，fetch 成功
- 当前整合源 HEAD：`bcbe4db5e665e5f678069afc1b206a00ae1cd36d`
- Compare：测试 HEAD 后续有 7 个提交，0 behind；这些提交包含 FMS Return control-binding handoff 和 M5 landing stage/probe/contact reference 修正。PR #20 已将完整 177 提交历史合入 main，当前 merge commit 为 `eacb966b502b7f9cb547617e866641504346a5fd`。

因此本归档是 `304fbd12` 的有效动态失败证据，但不是后续 7 个修复提交、也不是当前 main 的复测证据。

## 2. 完整性与运行环境

| 项目 | 记录 |
|---|---|
| Overall | FAIL |
| Issue closure eligible | false |
| SIL build | PASS / exit 0 |
| FMT-SIM EXE SHA256 | `4f9a0f12c78e3964b3e472ee579c547d13ea629456dc6a4c7fe72ea18ae2a6d6` |
| Settings SHA256 | `94c30638ded616d60f13e36b0a8c4f265dcc2c38c662db9e6f179ac9cee6a2f2` |
| UDP / timeout | 14653 / 350 ms |
| Graphics adapter / hover | 1 / 0.62 |
| Loopback | Owner explicitly confirmed 127.0.0.1 |
| PowerShell | 7.6.10 |
| MinGW GCC/G++ | 16.2.0 |
| Python | 3.14.8 |
| Git | 2.55.0.windows.5 |

ZIP SHA256 `af97c2ac451c2937fab186ccd5e1a9f553b4ee6d699470ab9963e0b105505d26` 与外部 sidecar 完全一致。Manifest SHA256 为 `5ddea977a8d267ebff908cc0df4dcd79d3aad3adcca44c555fe1b92f453f1031`；内部 SHA256SUMS 包含 102 项，全部通过。归档字节完整性 PASS，不代表场景通过。

## 3. 场景结果

| Scenario | Checks | Failures | Exit | Longest state gap | [settle] lines |
|---|---:|---:|---:|---:|---:|
| Land | 36,600 | 4 | 1 | 78.7 ms | 640 |
| Return | 36,601 | 3 | 1 | 48.0 ms | 644 |
| Mission | 43,692 | 6 | 1 | 48.5 ms | 301 |
| **Total** | **116,893** | **13** | — | all <350 ms | 1,585 |

### Land — 4 failures

- `near_ground_max_down > 0.0f && near_ground_max_down <= 0.40f`
- `terminal_ack == COMMAND_ACK_COMPLETED`
- `disarm_ack == COMMAND_ACK_COMPLETED`
- `runtime_data.fms.arm_state == ARM_STATE_DISARMED`

### Return — 3 failures

- `terminal_ack == COMMAND_ACK_COMPLETED`
- `disarm_ack == COMMAND_ACK_COMPLETED`
- `runtime_data.fms.arm_state == ARM_STATE_DISARMED`

### Mission — 6 failures

- `near_ground_max_down > 0.0f && near_ground_max_down <= 0.40f`
- `terminal_ack == COMMAND_ACK_COMPLETED`
- `disarm_ack == COMMAND_ACK_COMPLETED`
- `runtime_data.fms.arm_state == ARM_STATE_DISARMED`
- `flight_sil_ground_contact() == GROUND_CONTACT_CONTACT`
- `return_error <= .5f`

The longest successful state gaps are all below the configured 350 ms threshold. This rules out a recorded state-gap violation for these three runs, but does not itself explain the terminal/landing failures. The final snapshots and full `[settle]` logs must be used for root-cause work; no threshold relaxation is justified.

## 4. Disposition and next action

- Keep GitHub Issues #17 Land, #18 Return and #19 Mission OPEN.
- Do not mark the current main branch as failing based only on this old tested HEAD.
- The formal local runner has been updated to require branch `main` and upstream `origin/main`.
- Owner should fetch/switch/pull main, confirm a clean worktree and exact `HEAD == origin/main`, then rerun all three scenarios with the same isolated FMT-SIM setup, UDP 14653, 350 ms, hover 0.62 and explicit loopback confirmation.
- Do not use Host/CI PASS to close native FMT-SIM Issues. FMT-SIM PASS also does not replace B07 H723 runtime, physical HIL, WCET/RTA, target stack watermark or flight evidence.

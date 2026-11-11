# R28 issue details FLIGHT-LOCAL-35..41

RUN_ID `M5-06-R28-20261111`; input baseline `00baed0575e6db1212248315adf214853ecc62e3`;
validated code/tooling `31463cb351d2f55e5da978c75db64e9d25fc2523`.
R27 detail file is retained as history and must not be treated as current.

## FLIGHT-LOCAL-35

- 根因: R27 `analysis_p2.md` 仍以 R25 作为摘要标题，资源叙述与 R28 当前对象不一致。
- 首次发现: R21 文档审计；R27 复核仍存在。
- 当前基线: `31463cb351d2f55e5da978c75db64e9d25fc2523`。
- 责任方: AGENT；独立验收 ChatGPT。
- 安全影响: 证据可复现性；不改变轨迹算法或飞行放行。
- 修复步骤: 文首更新为 R28 摘要；R25/R16 段落标记 HISTORICAL/SUPERSEDED；保留 4,248 B 与 232 B 两个独立数字。
- 复测命令: `python tools/check_validation_evidence.py --check blocker`。
- 预期结果: R28 摘要、4,248 B、4,096 B、232 B 均出现；旧段落有历史标记。
- 实际结果: 已更新；等待独立复验。
- 日志路径: `build/validation/analysis_p2.md`。
- 日志 SHA256: `94802e68fcc4741f81f09ed6f64d5a0cfee9a2a15dcfb06f8bb969e3b4ce688f`。
- 解除条件: 独立审查确认 R28 资源摘要与 `.su`/分析文件一致。
- 下一检查点: R28 独立验收。

## FLIGHT-LOCAL-36

- 根因: R27 Manifest 的 archive/ahead 元数据在归档后才补齐，且 R27 快照实际是 R26 数据。
- 首次发现: R21/R22 归档审计。
- 当前基线: `31463cb351d2f55e5da978c75db64e9d25fc2523`。
- 责任方: AGENT；OWNER 批准归档元数据口径。
- 安全影响: 证据可复现性；不直接影响飞控输出。
- 修复步骤: R28 Manifest 统一 run_id/branch/code/tooling/collector/archive-script/remote/ahead；外部归档报告与其逐项比较；旧 R27 快照标 HISTORICAL。
- 复测命令: `python tools/check_validation_evidence.py --check manifest` 与 `--check archive`。
- 预期结果: Manifest、外部报告、ZIP、Bundle 的 commit/ahead/size/SHA 一致；不一致即 FAIL。
- 实际结果: Manifest 已更新；归档报告和 ZIP 在最终归档步骤生成并复测。
- 日志路径: `work/validation/archives/M5-06-R28-20261111.archive_report.json`。
- 日志 SHA256: `PENDING_UNTIL_ARCHIVE`（归档后写回真实 SHA256，不使用 R27 值）。
- 解除条件: ZIP/报告/Manifest/Bundle 独立核对通过。
- 下一检查点: R28 最终归档。

## FLIGHT-LOCAL-37

- 根因: R27 环境字段仍有 placeholder 和历史环境混用风险。
- 首次发现: R22 环境证据审计。
- 当前基线: `31463cb351d2f55e5da978c75db64e9d25fc2523`。
- 责任方: AGENT。
- 安全影响: 证据可复现性；不得把 Host/SIL 当作 H723。
- 修复步骤: `r28_environment.json` 记录 Windows/WSL/Cortex-M7 实际版本；采集器禁止 `see r25_environment`、`unknown`、`deterministic` 等值。
- 复测命令: `python tools/check_validation_evidence.py --check environment`。
- 预期结果: 三类环境均有真实版本，采集器拒绝 placeholder。
- 实际结果: 已生成；等待独立复验。
- 日志路径: `build/validation/regenerated/r28_environment.json`。
- 日志 SHA256: `ee91fefa8001f19d87258972d07bd1563bfcedfa13706691eadf83af527f8e5f`。
- 解除条件: 独立审查确认环境与实际执行一致。
- 下一检查点: R28 独立验收。

## FLIGHT-LOCAL-38

- 根因: R27 逐项测试记录仍有派生字段、旧日志和 `NOT_RECORDED` 时长。
- 首次发现: R22 测试台账审计。
- 当前基线: `31463cb351d2f55e5da978c75db64e9d25fc2523`。
- 责任方: AGENT。
- 安全影响: 证据可复现性；不允许用旧日志冒充 R28。
- 修复步骤: 采集器从 wrapped log 解析 checks/failures，绑定 command/exit/duration/seed/toolchain/source/log SHA；S06-108/112/113/MAP-RANDOM 从真实标记派生。
- 复测命令: `python tools/collect_validation_results.py --run-id M5-06-R28-20261111 --code-commit 31463cb351d2f55e5da978c75db64e9d25fc2523 --output build/validation/regenerated/r28_test_results.json`。
- 预期结果: 30+ 条记录，PASS 字段完整，FAIL/BLOCKED/NOT_RUN 也有原始日志或明确原因。
- 实际结果: 待最终全量回归后重建。
- 日志路径: `build/validation/regenerated/r28_test_results.json`。
- 日志 SHA256: `14586a7071124d16b3dbd04bda0737fbcf6f28c211f97d2cb86dbd765786c984`。
- 解除条件: 采集器严格模式通过且独立复跑一致。
- 下一检查点: R28 最终回归。

## FLIGHT-LOCAL-39

- 根因: S06-112 JSON 曾含 PASS 文本/`deterministic` seed，无法独立解析。
- 首次发现: R23/R24 证据审计。
- 当前基线: `31463cb351d2f55e5da978c75db64e9d25fc2523`。
- 责任方: AGENT。
- 安全影响: 数值证据可复现性。
- 修复步骤: `verify_jerk_continuous.py` 使用真实 `--seed`，JSON 与测试台账同样字段；采集器比较 run/code/checks/failures/seed。
- 复测命令: `wsl.exe -d Ubuntu-26.04 -- python3 tests/guidance/verify_jerk_continuous.py --library build/m5_host/libm5_safety_oracle.so --run-id M5-06-R28-20261111 --code-commit 31463cb351d2f55e5da978c75db64e9d25fc2523 --seed 0x53313632 --output build/validation/regenerated/s06_112_r28.json`。
- 预期结果: `status=PASS`、`seed=0x53313632`、420 axis checks、0 failures，JSON 可 `json.loads`。
- 实际结果: PASS，420/0。
- 日志路径: `build/validation/regenerated/s06_112_r28.json`。
- 日志 SHA256: `ca70bad85dcc0435b41b5366e96fa2b4acf7235b6468a5eca32cdc22c41ae3c8`。
- 解除条件: 独立复跑 JSON/日志/台账一致。
- 下一检查点: R28 独立复验。

## FLIGHT-LOCAL-40

- 根因: R27 采集器仍可接受只有 PASS 文本、人工填写 `checks=1/failures=0` 的记录；工具链/seed/时长可为占位值。
- 首次发现: R23 首次登记；R27 直接探测复现。
- 当前基线: `31463cb351d2f55e5da978c75db64e9d25fc2523`。
- 责任方: AGENT。
- 安全影响: S1 证据完整性；可掩盖真实失败。
- 修复步骤: 采集器改为解析 `FLIGHT_VALIDATION_LOG_V1` 包装日志，PASS 必须 exit 0、failures 0、checks>0、RUN_ID/code/command/duration 一致、toolchain 真实、seed 合规、source manifest 完整、log SHA 精确；禁止调用者提供 checks/failures。
- 复测命令: `python tools/collect_validation_results.py --self-test` 与 `python -O tools/collect_validation_results.py --self-test`。
- 预期结果: 两种模式均输出 `COLLECTOR_SELF_TEST_PASS (28 negative cases)`；R27 虚假 PASS 反例被拒绝。
- 实际结果: 两模式均 PASS，28/28。
- 日志路径: `build/m4_m5_implementation/evidence/collector_selftest.log`。
- 日志 SHA256: `4e4ba83fe37756d0891fd0d7d3e54c1a484abd6195a6897ad30f2c2edc9d9fcb`。
- 解除条件: 独立审查确认负向场景覆盖与真实集成测试。
- 下一检查点: R28 独立验收。

## FLIGHT-LOCAL-41

- 根因: R22 原始工作区字节在后续归档中无法完整恢复；只能保留 R22/R27 的派生副本和历史补丁。
- 首次发现: R23 工作区审计。
- 当前基线: `31463cb351d2f55e5da978c75db64e9d25fc2523`。
- 责任方: AGENT/OWNER；外部证据不可恢复。
- 安全影响: 历史证据可追溯性；不改变当前代码。
- 修复步骤: 保留 R27 worktree patch、modified/untracked 原始副本和 SHA；R28 不覆盖、不删除；R22 原始字节明确标 NOT_RECOVERABLE。
- 复测命令: `python tools/check_validation_evidence.py --check handover` 与 R27 worktree manifest 逐一 SHA 比对。
- 预期结果: 当前 8 个 modified + 2 个 untracked 文件与 R27 清单完全一致；R22 原始字节仍标 NOT_RECOVERABLE。
- 实际结果: R27 文件逐一 SHA 一致；R22 原始字节不可恢复。
- 日志路径: `build/validation/regenerated/r27_worktree/working_tree.patch`。
- 日志 SHA256: `df9f19b586dc8a47ecedb4683f666f4ffa6511748954fd0890ff558af9f5670f`。
- 解除条件: 找到可验证的 R22 原始字节备份并由独立审查确认；否则保持 BLOCKED。
- 下一检查点: OWNER/外部证据提供者。


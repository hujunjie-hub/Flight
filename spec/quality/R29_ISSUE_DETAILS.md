# R29 issue details FLIGHT-LOCAL-35..41

RUN_ID `M5-06-R29-20261111`; validated code `fd8198d2ac216fef87dafba73c252bb254ea3dd3`.
R28 detail file remains historical.

## FLIGHT-LOCAL-35

- 根因: R28 资源摘要仍引用 R28 基线，未反映 R29 M5 Host 320,650 检查、4,248 B 栈下界和 232 B 静态余量。
- 首次发现: R21 文档审计；R28 复核仍滞后。
- 当前基线: `fd8198d2ac216fef87dafba73c252bb254ea3dd3`。
- 责任方: AGENT；独立验收 ChatGPT。
- 安全影响: 证据可复现性；不改变轨迹算法。
- 修复步骤: 更新 `analysis_p2.md` 的 R29 摘要；R28/R25 段落标 HISTORICAL；保留 4,248 B、4,096 B、232 B 三个独立数字。
- 复测命令: `python tools/check_validation_evidence.py --check blocker`。
- 预期结果: R29 摘要、4,248 B、4,096 B、232 B 同时出现，旧段落有历史标记。
- 实际结果: 已更新；等待独立复验。
- 日志路径: `build/validation/analysis_p2.md`。
- 日志 SHA256: `951e6e053c36254f4878660f6dc3b5c359b1d327392bfbdeb4d16eb21f090e31`。
- 解除条件: 独立审查确认 R29 资源摘要与实际 `.su`/分析一致。
- 下一检查点: R29 独立验收。

## FLIGHT-LOCAL-36

- 根因: R28 使用归档后元数据提交补齐 Manifest，导致 Archive HEAD 与当前 HEAD 分离；R28 issue detail 含 PENDING/旧 SHA。
- 首次发现: R27/R28 归档审计。
- 当前基线: `fd8198d2ac216fef87dafba73c252bb254ea3dd3`。
- 责任方: AGENT；OWNER 批准归档元数据口径。
- 安全影响: 证据可复现性；不直接影响飞控输出。
- 修复步骤: 应用 R29 `prepare_archive_metadata.py` 两阶段协议；归档前写入 Git 动态元数据，ZIP 自身 SHA 只写外部报告和 sidecar；禁止 PENDING 和后置元数据提交。
- 复测命令: `python tools/prepare_archive_metadata.py --run-id M5-06-R29-20261111 --expected-branch opt/stage-02-m5-06-safety-validator`；归档后 `python tools/check_validation_evidence.py --check archive`。
- 预期结果: archive head == 当前 HEAD；ahead 动态计算；ZIP 内部无自身 SHA；外部报告/sidecar/ZIP 一致。
- 实际结果: R29 补丁已提交；归档报告在最终归档后生成并复测。
- 日志路径: `work/validation/archives/M5-06-R29-20261111.archive_report.json`。
- 日志 SHA256: `NOT_APPLICABLE_PRE_ARCHIVE`（归档后由外部报告记录；不使用 R28 值）。
- 解除条件: R29 ZIP/报告/Manifest/Bundle 独立核对通过。
- 下一检查点: R29 最终归档。

## FLIGHT-LOCAL-37

- 根因: R28 环境字段与工具链绑定需要按 R29 重新记录。
- 首次发现: R22 环境证据审计。
- 当前基线: `fd8198d2ac216fef87dafba73c252bb254ea3dd3`。
- 责任方: AGENT。
- 安全影响: 证据可复现性；Host/SIL 不得当作 H723。
- 修复步骤: `r29_environment.json` 记录 Windows/WSL/Cortex-M7 实际版本和 CFLAGS；collector/checker 拒绝 placeholder。
- 复测命令: `python tools/check_validation_evidence.py --check environment --environment build/validation/regenerated/r29_environment.json`。
- 预期结果: 三类环境均有真实版本，无 `NOT_RECORDED`/`see r25_environment`。
- 实际结果: 已生成；等待独立复验。
- 日志路径: `build/validation/regenerated/r29_environment.json`。
- 日志 SHA256: `aa8de081b38abbba41b587a2345ece2fb8d0c449d43e0f9a40e178335c9b1b9b`。
- 解除条件: 独立审查确认环境与实际执行一致。
- 下一检查点: R29 独立验收。

## FLIGHT-LOCAL-38

- 根因: R28 逐项测试记录不能覆盖 R29 新工具、MR-009 focus 与 33 项单元测试。
- 首次发现: R22 测试台账审计。
- 当前基线: `fd8198d2ac216fef87dafba73c252bb254ea3dd3`。
- 责任方: AGENT。
- 安全影响: 证据可复现性；旧日志不得冒充 R29。
- 修复步骤: 重建 `r29_test_results.json`，逐项绑定 RUN_ID/code/command/exit/checks/failures/duration/toolchain/source/log SHA；M5 Host 320,650/0；MR-009 focus 15/0；单元测试 33/0 两种模式。
- 复测命令: `python tools/check_validation_evidence.py --check test-results --test-results build/validation/regenerated/r29_test_results.json`。
- 预期结果: 11 条记录，PASS/FAIL/BLOCKED 均有真实日志与 SHA。
- 实际结果: 已生成；等待独立复验。
- 日志路径: `build/validation/regenerated/r29_test_results.json`。
- 日志 SHA256: `322513877bfc8d60cb9f431d65295536570c94f48770609d07de9ea59bad9084`。
- 解除条件: 独立复跑或独立核对日志一致。
- 下一检查点: R29 最终回归。

## FLIGHT-LOCAL-39

- 根因: S06-112 需要按 R29 RUN_ID/code 重新绑定。
- 首次发现: R23/R24 证据审计。
- 当前基线: `fd8198d2ac216fef87dafba73c252bb254ea3dd3`。
- 责任方: AGENT。
- 安全影响: 数值证据可复现性。
- 修复步骤: 用真实 `--seed 0x53313632` 运行 S06-112；JSON 与 test_results 的 run/code/checks/failures/seed 一致。
- 复测命令: `wsl.exe -d Ubuntu-26.04 -- python3 tests/guidance/verify_jerk_continuous.py --library build/m5_host/libm5_safety_oracle.so --run-id M5-06-R29-20261111 --code-commit fd8198d2ac216fef87dafba73c252bb254ea3dd3 --seed 0x53313632 --output build/validation/regenerated/s06_112_r29.json`。
- 预期结果: PASS、420 axis checks、0 failures、seed `0x53313632`。
- 实际结果: PASS，420/0。
- 日志路径: `build/validation/regenerated/s06_112_r29.json`。
- 日志 SHA256: `48082e83a3b50d0341d28fb52e512ed87c2eda68e3f03c905766785ba33907d2`。
- 解除条件: 独立复跑 JSON/日志/台账一致。
- 下一检查点: R29 独立复验。

## FLIGHT-LOCAL-40

- 根因: R28 collector 与 checker 需要在 R29 重新验证；归档 checker 曾允许后置元数据。
- 首次发现: R23 首次登记；R27 直接探测复现。
- 当前基线: `fd8198d2ac216fef87dafba73c252bb254ea3dd3`。
- 责任方: AGENT。
- 安全影响: S1 证据完整性；可掩盖真实失败。
- 修复步骤: 应用 R29 collector/checker；collector 28 负向；checker 33 项单元测试（含 26 负向 scenario）在普通/`-O` 模式通过；归档 checker 禁止 PENDING 和后置元数据。
- 复测命令: `python tools/collect_validation_results.py --self-test`；`python tools/check_validation_evidence.py --self-test`；`python -O ... --self-test`。
- 预期结果: collector 28/28；checker 33/33；两种模式一致。
- 实际结果: 全部通过。
- 日志路径: `build/validation/regenerated/r29/collector_selftest.log`；`build/validation/regenerated/r29/checker_selftest.log`。
- 日志 SHA256: `295e25248d3ee1ac760aecb0b3f8e9aaa2485c309f6ae0d6b5d42cfc703fa28e`（checker 日志）。
- 解除条件: 独立审查确认负向场景覆盖与真实集成测试。
- 下一检查点: R29 独立验收。

## FLIGHT-LOCAL-41

- 根因: R22 原始工作区字节无法完整恢复。
- 首次发现: R23 工作区审计。
- 当前基线: `fd8198d2ac216fef87dafba73c252bb254ea3dd3`。
- 责任方: AGENT/OWNER；外部证据不可恢复。
- 安全影响: 历史证据可追溯性；不改变当前代码。
- 修复步骤: R29 有明确范围搜索 `E:\Flight`、用户目录、Downloads 和 Git 对象；未发现 R22 原始字节；保留 R27/R28 原始副本与 patch。
- 复测命令: `Get-ChildItem E:\Flight -Recurse -File -Filter '*R22*'`；R27 worktree manifest 逐项 SHA。
- 预期结果: R22 原始字节不可恢复时明确标 NOT_RECOVERABLE，不得用 R27/R28 文件冒充。
- 实际结果: NOT_RECOVERABLE。
- 日志路径: `build/validation/regenerated/r27_worktree/working_tree.patch`。
- 日志 SHA256: `df9f19b586dc8a47ecedb4683f666f4ffa6511748954fd0890ff558af9f5670f`。
- 解除条件: 找到可验证的 R22 原始字节备份并由独立审查确认；否则保持 BLOCKED。
- 下一检查点: OWNER/外部证据提供者。

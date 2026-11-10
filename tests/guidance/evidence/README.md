# 历史测试证据（不是 R7 运行结果）

本目录中的文件是 **2026-11-04** 的 T191-C-M5 早期 Host 证据，早于 M5-06 独立验证器、MR-009 误差包络和 S06-080..103 测试矩阵。

- 生成日期：2026-11-04
- 生成时源码基线：T191-C-M5 阶段提交（早于 `opt/stage-02-m5-06-safety-validator`）
- 现状：**HISTORICAL / SUPERSEDED**，不得作为当前 HEAD 或 R7 的证据引用。

当前运行的证据只写入：

- `build/m4_m5_implementation/evidence/`（`run_validation.py` 产出的日志与 JSON）
- `build/m5_arm/`（Cortex-M7 对象、`.su`、`toolchain.txt`、`consumer_fp.json`）
- `build/validation/`（HANDOVER、issue 草稿、rebaseline 提案、regenerated 证据）
- `work/validation/archives/<RUN_ID>/`（归档与 SHA256 清单）

每份当前证据都由归档脚本记录生成时间、Git HEAD 和 SHA256。

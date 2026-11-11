# R25 detailed issue records FLIGHT-LOCAL-35..41

RUN_ID: `M5-06-R25-20261111`  
Validated code: `e89cb7d94e6c992c8dfa71c3cb565ae4c5f6248a`

## FLIGHT-LOCAL-35 — 资源文档历史/当前混用
- Root cause/current: R24/R25 evidence issue.
- Fix: analysis_p2 顶部 R25 摘要；旧 32,840 B/3.6 KB 标 HISTORICAL
- Log: `build/validation/analysis_p2.md`; SHA256 `677a4dd09b6633110a6c8eede0e9418f28e4761d63db70abb99694417fc4d46e`.
- Repro: see R25 HANDOVER and `r25_test_results.json`.
- Unblock: independent reviewer verifies this R25 evidence.
- Status: FIXED_WAITING_RETEST.

## FLIGHT-LOCAL-36 — 归档元数据定义
- Root cause/current: R24/R25 evidence issue.
- Fix: archive report 含 validated/collector/archive script/head/remote/ahead/zip；Manifest 外部 sidecar；BLOCKER R25
- Log: `M5-06-R25-20261111.archive_report.json`; SHA256 `MISSING`.
- Repro: see R25 HANDOVER and `r25_test_results.json`.
- Unblock: independent reviewer verifies this R25 evidence.
- Status: BLOCKED.

## FLIGHT-LOCAL-37 — 环境证据
- Root cause/current: R24/R25 evidence issue.
- Fix: r25_environment.json 含 Host/ARM/Windows、CFLAGS、sanitizer、逐项 duration；R24/R23 历史
- Log: `build/validation/regenerated/r25_environment.json`; SHA256 `96e7509fecf405419683e08f15a87e642cf9414f018c20ad66f8b8eddb19f1b5`.
- Repro: see R25 HANDOVER and `r25_test_results.json`.
- Unblock: independent reviewer verifies this R25 evidence.
- Status: FIXED_WAITING_RETEST.

## FLIGHT-LOCAL-38 — 逐项测试证据
- Root cause/current: R24/R25 evidence issue.
- Fix: r25_test_results.json 由 collector 从日志/JSON 生成，29 项；S06-112/113、地图、产品、Manifest、Issue、STACK 独立
- Log: `build/validation/regenerated/r25_test_results.json`; SHA256 `7465d4f057025dde36635b54884001040f5d2fd63460188efdf1f3143784b09d`.
- Repro: see R25 HANDOVER and `r25_test_results.json`.
- Unblock: independent reviewer verifies this R25 evidence.
- Status: BLOCKED.

## FLIGHT-LOCAL-39 — S06-112 JSON
- Root cause/current: R24/R25 evidence issue.
- Fix: s06_112_r25.json 仅 JSON；log 独立；R14b 6/6；json.loads OK
- Log: `build/validation/regenerated/s06_112_r25.json`; SHA256 `dca5e0fb088faa56b598aeecf57b4dff69e5bd8f26b647d016dc3e992b841408`.
- Repro: see R25 HANDOVER and `r25_test_results.json`.
- Unblock: independent reviewer verifies this R25 evidence.
- Status: FIXED_WAITING_RETEST.

## FLIGHT-LOCAL-40 — 收集器硬编码
- Root cause/current: R24/R25 evidence issue.
- Fix: collector 16 负向（含 -O）；PASS 需 exit0/failures0/checks>0/source/log/env SHA
- Log: `tools/collect_validation_results.py`; SHA256 `42586f6b0445d379ef21a6ba9a65d445a700aa21fde745471cab848322894281`.
- Repro: see R25 HANDOVER and `r25_test_results.json`.
- Unblock: independent reviewer verifies this R25 evidence.
- Status: BLOCKED.

## FLIGHT-LOCAL-41 — R22 工作区/补丁
- Root cause/current: R24/R25 evidence issue.
- Fix: R25 patch stdout/stderr 分离；apply-check；原始字节副本；R22 原始字节不可恢复的差异保留
- Log: `build/validation/regenerated/r24_worktree/working_tree.patch`; SHA256 `988e5560684f4b5290a48710082b92fd6f958780b9f53f5b381f9f21aec4046c`.
- Repro: see R25 HANDOVER and `r25_test_results.json`.
- Unblock: independent reviewer verifies this R25 evidence.
- Status: BLOCKED.


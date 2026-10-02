# build_host/ — 主机侧调试与质量工具 (不参与固件编译)

所有脚本面向"OpenOCD(telnet 4444 / gdb 3333) + ST-LINK V3"或 USART1 调试串口。
工程根目录以脚本自身位置推算 (`Path(__file__).parent.parent`), 移动脚本需同步检查。

## 质量门禁 (可复用, 勿删)

| 脚本 | 用途 |
|---|---|
| `strict_scan.py` / `strict_scan_cpp.py` | C / C++ 严格告警扫描 (读 `cmake-build-debug/compile_commands.json`), 基线 0 报警 |
| `fanalyzer_scan.py` | GCC `-fanalyzer` 静态分析扫描 |
| `warningscan.py` / `warnfix.py` / `warnfix2.py` | 告警清点 / 半自动修复 (历史一次性, 留作参考) |
| `cfgcheck.py` | `.config` ↔ `rtconfig.h` 一致性检查 |
| `rtt53_config_audit.py` / `rtt53_depend_audit.py` | RT-Thread 5.3.0 迁移后的配置/依赖审计 |
| `mapcmp.py` | 两次构建 `Flight.map` 的段占用对比 |

## SWD 在线探针 (py, 经 OpenOCD)

- 常驻观测: `swd_live.py` `swd_now.py` `swd_status.py` `swd_monitor.py` `swd_snap.py` `swd_quality.py`
- 收敛/稳定性: `swd_converge.py` `swd_stability.py` `swd_stab_alt.py` `swd_10rounds.py` `swd_freeze_watch.py` (README 主文档引用)
- 数据链路诊断: `swd_rxdiag.py` `swd_diag.py` `swd_dmascan.py` `swd_fifowatch.py` `swd_bdma_fill.py` `swd_fill2.py` `swd_zerorun.py` `swd_uart2.py` `swd_gnssdump.py` `swd_navlast.py` `swd_watchpoint.py` `swd_bufview.py` (DMA 缓冲区快照回溯)
- 标定会话: `swd_magcal_session.py` (配套符号表生成 `magcal_syms.py` → `data/magcal_syms.json`)
- 其他: `swo_test.py` `probe_cs.py` `probe_dma_transient.py` `probe_spi_dma.py` `probe_runtime.py` `fault_probe.py` `fault_probe2.py` `catch_moder_writer.py` `check_halt_resume.py` `reset_and_verify.py` `flash_and_verify_dma.py` `check_gins_ocd.py` `check_um982_gins.py` `baro_chain_monitor.py` `fix_path.py`

## gdb / OpenOCD 片段

断点·线程·堆·现场转储: `*.gdb` (`state*` `thrd*` `heapdbg` `finshdbg*` `adis` `bp*` `ctx*` `irq*` `forensic` 等), `crashdump.ocd`, `finsh_check.gdb`。

## 串口 PowerShell 辅助

`bootlog*.ps1` `cap.ps1` `flash.ps1` (CubeProgrammer 烧录) `monitor.ps1` `finsh*.ps1` `rate.ps1` `peek.ps1` `diag*.ps1` `baudscan.ps1` `bootcap.ps1` `um982rate.ps1` `ginscheck.ps1` `so3check.ps1` `solcheck.ps1` `suffixcheck.ps1` `ocdcheck.ps1` `telnet_resume.ps1` `test460800.ps1` `baro_check.ps1` `baro_bootlog.ps1`。

## 数据与证据 (子目录)

- `data/` — 一次性数据捕获: 磁标定数据集 (`magcal_*.csv` `maglog_*.csv` `synth.csv`)、标定槽位读写样本 (`slot*.bin`)、气压计记录 (`barocal_rec.bin` + `_barocal_ctx.json`)
- `test10/` — 2026-09-30 SWD 10 轮回归 17 判据证据日志 (README 主文档"已知问题 #17"引用, 勿删)
- `ell_test.exe` — 主机端椭球拟合单元测试 (源: `middleware/calibration/ellipsoid_fit.c`)

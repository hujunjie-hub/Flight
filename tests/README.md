# 固件测试

本目录保存 Host 测试、ARM 编译检查、集成调度器和仿真夹具。从工程根运行这些验证入口。

```powershell
python tools/check_layout.py
python tests/runtime/run_tests.py
python tests/algorithms/run_tests.py
python tests/abi/abi_report.py
python tests/drivers/check_drivers.py
pwsh tests/simulation/run_host_tests.ps1
uv run --no-project --with pymavlink==2.4.49 python tests/runtime/check_wire.py
wsl.exe -d Ubuntu-26.04 -- python3 tests/runtime/run_tests.py --sanitize
```

`tools/check_layout.py` 面向未构建的源码树（`.o`/`.su` 属于构建产物）；先跑它再构建。
`tests/simulation/` 覆盖 HIL 链路：MAVLink v2 编解码与独立金帧比对，以及
UDP 桥的端到端转发（不需要硬件）。接线与边界见 `doc/simulation/HIL_LINK.md`。

各功能目录保留原Host测试入口。`support/sensor_sim`为Host专用仿真输入。结果按入口保存在`build`或`tests/*/evidence`；迁移前报告属于历史证据，不能代替当前源码重跑结果。

运行时和线协议校验开始时将本次结果标为 `INCOMPLETE`。运行时检查完成后记录测试日志 SHA256，线协议校验只接受与成功记录匹配的日志。生成发布清单时执行 `python tests/runtime/verify_firmware.py`，它以 GNU Arm Release 构建，并核对源码、配置、产物和已有测试结果的对应关系。

## M6/M7 Host 回归

从工程根运行真实 M6 MPC/输入时序测试和 M7 执行器事务测试：

```bash
bash tests/control/run_host_tests.sh
bash tests/actuation/run_host_tests.sh
bash tests/simulation/run_sil.sh
```

前两项分别编译实际控制器/OSQP 工作区和实际 actuator output guard；`tests/simulation/run_sil.sh` 编译真实 runtime owner，覆盖 M4→M7 ground-stop handoff、显式 DISARM 和 first-reference timeout。GitHub Actions 在唯一开发分支 `main` 的 push 上执行 M4/M5、M6/M7 Host 回归、M6/M7 ASan/UBSan、Cortex-M7 模块编译和 SIL。Host/SIL 通过不代表 H723 目标板、物理 ESC、HIL 或飞行验收通过。

## 板上 utest

当前配置启用 `FLIGHT_TARGET_UTEST`，由 RT-Thread 自带的 utest 注册
[`flight.protocol_ipc`](target/protocol_utest.c)。用例覆盖 NMEA 校验、MAVLink 封帧和静态消息队列的 FIFO、满队列与空队列。
在板上 FinSH 手动执行。

```text
utest_list
utest_run -thread flight.protocol_ipc
```

测试启动由操作者触发，`RT_UTEST_USING_AUTO_RUN` 关闭。运行日志通过 RT-Thread ulog 的控制台后端输出。

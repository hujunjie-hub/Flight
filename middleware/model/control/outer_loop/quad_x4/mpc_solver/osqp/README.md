# OSQP 1.0.0 — QUAD embedded workspace

本目录保存 M6 控制器的固定求解器、生成工作区、稀疏模式、来源锁和生成器。替换求解器时，
在控制模块内同步调整本目录与 `mpc.c` 的求解器接入。对外入口见 ControllerPort。

来源及完整 commit/archive/license SHA-256 见 `SOURCE_LOCK.json`；生成文件逐项 SHA-256 见
`UPSTREAM_LOCK.json`。许可证为本目录 `LICENSE`（Apache-2.0），保留生成源码中的原始版权声明。
`bindings.cpp` 是 OSQP Python 1.0.0 的 pywrapper 模板按 `prefix=quad_`、`extension_name=emosqp` 渲染的 Host 扩展文件；它由锁文件校验，但不参与固件构建。
`UPSTREAM_LOCK.json` 的 SHA-256 使用规范 CRLF 字节表示；生成器与验证器先统一换行再计算，避免 Windows/Unix 工作区的行尾差异造成误报。
`inc/private/qdldl_version.h` 对应 QDLDL v0.1.8 的 `configure/qdldl_version.h.in`，其 tag commit 与版本宏渲染值同时记录在 `SOURCE_LOCK.json` / `UPSTREAM_LOCK.json`。
运行期不使用 Python、NumPy、SciPy、动态内存或桌面 OSQP 库。

配置固定为 float32、embedded mode 2、15 变量/190 约束、45/702 个 P/A 非零元素，
80 次迭代，绝对/相对容差 1e-3，rho 每 25 次迭代检查。关闭 polishing、printing、profiling、
interrupt、数值 scaling；使用 scalar rho。输入本身按项目约定使用 SI 单位和单位归一化尺度。

从 Flight 根目录重新生成：

```powershell
uv venv --python 3.12 build/host_deps/osqp-env
uv pip install --python build/host_deps/osqp-env/Scripts/python.exe osqp==1.0.0 numpy scipy jinja2 setuptools
build/host_deps/osqp-env/Scripts/python.exe middleware/model/control/outer_loop/quad_x4/mpc_solver/osqp/generate_position_mpc.py
```

`SConscript` 仅选择 `generated/src/*.c` 与 `quad_workspace.c`。生成器附带的 Python 扩展、
CMake/Makefile、示例入口不参与固件构建。根工程仍由原 SCons 管理。

为满足 32 KiB 静态 RAM 预算，生成器执行三项固定修改：

- 将固定 CSC 索引、KKT 映射、消元树及符号计数存为 const；只在静态结构初始化时适配上游指针类型。
- QDLDL 数值重分解对 L 的索引执行一致性检查，替代重复写入同一符号模式；不一致返回失败。
- embedded 模式未对外使用的 infeasibility certificate 缓冲与对应 delta 工作向量共用存储。

修改由生成器重现；运行时仍保留独立原约束检查和求解残差检查。最终 ARM 静态 RAM 报告由
`tests/algorithms/run_tests.py` 生成并强制检查，见
`build/validation/controller_navigation/summary.json`。这不构成 H723 WCET 证据。

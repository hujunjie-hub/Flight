# 浮点运行环境审计（MR-009 圆整模式前提）

状态：**AUDIT RECORD**。本文回答一个问题：仓库里是否有可能改变运行时浮点圆整模式（或 FZ/DN）的代码？结论不能只靠编译参数，必须看运行时状态，因此验证器增加了运行时检查。

## 1. 搜索范围

- 文件类型：`*.c`、`*.h`、`*.s`、`*.S`、`*.py`、`*.sh`、`*.ps1`、`SConscript`、`SConstruct`、`*.lds`
- 排除目录：`build/`、`doc/`、`.git/`、`work/`
- 文件数：3,362
- 模式：`FPSCR`、`fpscr`、`__set_FPSCR`、`vmsr`、`fesetround`、`fegetround`、`feclearexcept`、`fenv.h`、`setround`、`CPACR`、`FPU`

## 2. 结果

| 类别 | 命中 | 结论 |
|---|---|---|
| `__set_FPSCR` 定义 | `libraries/HAL_Drivers/CMSIS/Include/cmsis_{gcc,armcc,armclang,iccarm}.h` | 只是 CMSIS 内联定义；**仓库内无任何调用点** |
| 实际 `vmsr fpscr` 写 | `rt-thread/libcpu/arm/cortex-{a,r4,r52}/…`、`zynqmp-r5` | 这些 CPU 端口不参与 STM32H723 构建 |
| Cortex-M7 上下文切换 | `rt-thread/libcpu/arm/cortex-m7/context_gcc.S` | 只 `VSTMDB/VLDMIA` 保存 s16–s31；**没有 FPSCR 写**。S0–S15/FPSCR 由异常硬件压栈（FPCCR.ASPEN/LSPEN） |
| `fesetround` / `fegetround`（固件） | 仅 `m5_trajectory_safety.c`（新增检查）与 `test_m5_main.c`（回归用例） | 生产代码没有修改圆整模式 |
| `CPACR` 写（FPU 使能） | **仓库内没有**（只有 CMSIS 头文件里的寄存器定义） | 见 §3 |
| 启动文件 / `system_stm32h7xx.c` | **仓库内没有** | 见 §3 |

## 3. 关键发现：FPU 使能与 FPSCR 复位值不在本仓库

- 仓库没有 Cortex-M7 启动汇编，也没有 `system_stm32h7xx.c`；它们属于 RT-Thread 的 STM32H7 CMSIS device 包，而 `packages/` 自首次提交起被 `.gitignore` 排除（见 `spec/09` 的固件链接 BLOCKED 记录）。
- 因此**无法从源码审计**：CPACR 是否使能 CP10/CP11、启动阶段是否写 FPSCR、FPSCR 复位值（ARM 规定为 implementation-defined）是否等于 0。
- 推论：任何"运行时是 RN（且 FZ=0、DN=0）"的说法，在仓库范围内只能靠运行时读取寄存器来确认。

## 4. 处置：运行时检查而不是假设

`middleware/guidance/trajectory_planning/m5_trajectory_safety.c` 新增 `floating_environment_is_supported()`：

```c
#if defined(__arm__) && defined(__ARM_FP) && (__ARM_FP != 0)
  __asm__ volatile ("vmrs %0, fpscr" : "=r" (fpscr));
  return ((fpscr >> 22) & 3u) == 0u &&   /* RMode = round to nearest */
         ((fpscr >> 24) & 1u) == 0u &&   /* FZ = 0 */
         ((fpscr >> 25) & 1u) == 0u;     /* DN = 0 */
#else
  return fegetround() == FE_TONEAREST;
#endif
```

`m5_trajectory_safety_validate()` 在入口调用它；不满足时返回 `M5_SAFETY_NUMERIC_FAILURE`（fail-closed），不使用失效的误差模型。

回归用例 `m5_safety_floating_environment`：把 host 圆整模式设为 `FE_DOWNWARD` → 必须 `NUMERIC_FAILURE`；恢复 `FE_TONEAREST` → 必须 `PASS`。

## 5. 结论与剩余风险

**结论**：仓库内没有修改 FPSCR/圆整模式的代码；Cortex-M7 上下文切换不保存/恢复 FPSCR（由硬件异常压栈保持）；由于启动代码不在仓库内，运行时状态改为在验证器入口强制检查，不再依赖假设。

**剩余风险**：

1. 若目标启动代码把 FPSCR 设为非 RN 或开启 FZ，验证器会拒绝（不会误判），但固件其它浮点模块仍按该模式运行——这是固件初始化问题，需要 `packages/` 进入仓库后单独审计。
2. Host 的 x86 FTZ/DAZ（MXCSR）不在 `fenv.h` 语义内；Host/SIL 默认未开启，目标端由 `FZ/DN` 位检查覆盖。
3. 该审计不覆盖硬件勘误或调试器在运行时改写 FPSCR 的情况；验证器每次调用都会重新检查，因此这类改写会被拒绝而不是被忽略。

 
## 15. R6 更新：Host MXCSR

Host/SIL 现在同时检查 x86 MXCSR：RC=round-to-nearest、FTZ=0、DAZ=0（ARM 侧继续检查
FPSCR.RMode/FZ/DN）。负向用例覆盖 `FE_DOWNWARD`、`FE_UPWARD`、`FE_TOWARDZERO`、
`MXCSR.FTZ=1`、`MXCSR.DAZ=1`，每项都要求 `NUMERIC_FAILURE`，测试结束恢复原环境。

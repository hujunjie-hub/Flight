# T191-A-ABI 交付报告（独立准备）

| 项目 | 内容 |
|---|---|
| 任务 ID | `T191-A-ABI` |
| 类型 | IMPLEMENTATION |
| Owner | A（PLATFORM/HARDWARE/ICD） |
| 基线 | `V1.9.1-GLOBAL-AUDIT-R3` / Schema 6.0 / Product-Port `0x00010903` |
| Profile | `QUAD_X4_H723`（`0x00010001`） |
| 前置 | `T191-D-REVIEW`（**未满足**，见 §1） |
| 验证范围 | `HOST_PRODUCTION_ABI` |
| 本次结果 | 生产头编译与 Host/ARM 布局：**PASS**；独立审核、目标固件构建等：`NOT_RUN` |
| 交付时间 | 2026-11-04（Asia/Shanghai） |

本报告由实际执行该 prompt 的 Agent A 执行上下文编写，不是 D 的任务记录更新。
任务主状态、证据台账与发布状态仍由 D/09 维护。

## 0. 结论摘要

1. 在 `applications/contracts/` 交付 QUAD 生产头（版本、公共基础、消息、Port 类型/API、消息 ID 注册）共 8 个头 + 1 个 README。
2. 在 `applications/products/QUAD_X4_H723/` 交付 `product_contract.h` 与生成的 `abi_lock.h`。
3. 在 `applications/tests/abi/` 交付 Host/ARM ABI 夹具：探针生成器、报告工具、负向对照。
4. 用真实生产头（非 spec C 片段）完成：
   - 142 个公共类型、985 个字段、1269 个 `sizeof/alignof/offsetof` 表达式；
   - ARM Cortex-M7（EABI hard-float）与 x86_64 Host 两目标，C11 与 C++14 双语言；
   - 与 `spec/validation/abi_layout.json` 的 980 个规范 offset 逐项比对，**0 差异**；
   - Host↔ARM 仅 5 个指针承载聚合按预期不同（0 个意外差异）；
   - 55 个消息类型 ID 与 `spec/contracts/message_registry.json` 一致；
   - ABI hash 写入 `abi_lock.h`，并由 CI 模式（无 `--emit-lock`）校验漂移。
5. 负向对照证明检查器确实能发现布局漂移（注入 `NavStateCore.valid_mask` 位宽变化 → 14 处 size/align/offset 差异 → 检查器 `FAIL`）。
6. 未执行、不得声称通过的项目见 §6；对 D 的依赖与需要登记的台账动作见 §7。

## 1. 前置状态与“不伪称”声明

读取 `spec/AGENT_HANDOFF.md`（文件时间 21:09:48）时：

- `T191-D-REVIEW`: `task_status=READY_FOR_ASSIGNMENT`、`acknowledgement=null`、`verification_status=NOT_RUN`。
- 因此 `T191-A-ABI` 的前置**未满足**。按 prompt 要求，本次只做“独立准备并向 D 报告依赖”。

执行期间观察到另一执行上下文在 22:27 写入了 `spec/validation/T191-D-REVIEW/review_checks.py`
与 `review_checks.json`（scope `DOCUMENT_STATIC`，其中 `independent_non_author_review=NOT_RUN`），
但 `AGENT_HANDOFF.md` 未被更新，也没有可核验的 ASSIGNED/ACCEPTED 迁移或批准记录。

因此本报告：

- **不声称**已接单为 `ASSIGNED`、已审核、已通过 Gate A 或已 ACCEPTED；
- 仅声明“执行上下文收到并执行了该 prompt，且提交了可独立复核的交付与证据”；
- 状态迁移请求交 D 守卫（§9）。

## 2. 交付物与落点

### 2.1 生产头（`applications/contracts/`）

| 文件 | 作用 |
|---|---|
| `fc_contract_version.h` | Schema 6.0 / Port `0x00010903`、编译器抽象、C99 兼容断言 |
| `fc_common_base.h` | `Status`、`ContractHeader`、时间/原始采集服务、传感器样本、`NavStateCore`、公共诊断 |
| `fc_product_contract.h` | 唯一 Product Profile 选择入口（含未知 Profile 拒绝） |
| `fc_control_model_types.h` | `PRODUCT_CONTROL_NX/NU` 维度的 ControlModel 类型 |
| `fc_common_messages.h` | 全部 55 类公共消息/健康/证据结构的强类型定义 |
| `fc_port_types.h` | FMS/Guidance/Controller/Plant Port 聚合输入输出 |
| `fc_port_api.h` | M3～M7、ControlModelPort、PlantSimulationPort、PLATFORM 时间/原始采集 API |
| `fc_message_ids.h` | 55 个 `FC_MSG_*` 消息类型 ID（append-only） |
| `README.md` | 包含顺序、冻结值、ABI lock 与验证方法 |

分层严格遵循 02 §1.3：`common_base.h → product_contract.h → common_messages.h → port_api.h`。
公共头只包含 `<stdint.h>`/`<stddef.h>` 与本套 `fc_*`/产品头；无 HAL、RT-Thread、CMSIS、Simulink 依赖。

### 2.2 产品契约（`applications/products/QUAD_X4_H723/`）

| 文件 | 作用 |
|---|---|
| `product_contract.h` | QUAD 全部 `Product*` 强类型、容量/布局 ID/命令码/valid-mask 宏、配置 payload |
| `abi_lock.h` | **生成文件**：Schema/Port/Product/Profile、`ContractHeader` 字节数、ARM/Host ABI hash、消息注册表 SHA256 |

`abi_lock.h` 由 `abi_report.py --emit-lock` 生成，文件头明确“do not edit”；其常量由探针编译期断言使用。

### 2.3 ABI 夹具（`applications/tests/abi/`）

| 文件 | 作用 |
|---|---|
| `abi_report.py` | 解析生产头 → 生成探针 → ARM/Host × C11/C++14 编译 → 提取布局 → 比对规范/交叉目标/消息表/锁 → 写证据 |
| `negative_control.py` | 复制头文件、注入一处位宽变化，断言检查器必须 `FAIL` |
| `evidence/abi_arm.json` | ARM 布局、hash、编译器与 flags、头文件 SHA256 |
| `evidence/abi_host.json` | Host 布局、hash、编译器与 flags |
| `evidence/abi_compare.json` | 规范比对、Host/ARM delta、消息表、头文件卫生、锁校验 |
| `evidence/commands.json` | 全部实际命令、返回码、stdout/stderr、耗时 |
| `evidence/abi_report.md` | 人类可读证据摘要 |
| `evidence/abi_probe.c` | 实际编译的探针源码（含 1269 个布局表达式） |
| `evidence/abi_canonical_arm.txt` / `abi_canonical_host.txt` | ABI hash 的规范文本（可独立 SHA256 复算） |
| `evidence/artifact_sha256.json` | 交付物 SHA256 清单（不含自身） |

## 3. 冻结值

| 常量 | 值 |
|---|---|
| `FC_CONTRACT_SCHEMA_MAJOR` / `MINOR` | 6 / 0 |
| `FC_PORT_API_REVISION` | `0x00010903` |
| `PRODUCT_CONTRACT_REVISION` | `0x00010903` |
| `PRODUCT_PROFILE_ID` | `0x00010001` |
| `ContractHeader` sizeof | 88（编译期断言 + ABI 锁） |
| ARM ABI hash | `f1e36c57b13dccba75bf745f831f395901d4fe9ded42e38c0fea2e22ec910b08` |
| Host ABI hash | `e288b875c20315d4fc7e3568eccfb9bb6a1cc3c6902bf97edd86ea48e4eb4741` |
| 消息注册表 SHA256 | `e2893aebd8c1121b1ede80591849730ec722c5f9e0616443507c2ff99179bf41` |

ABI hash 的规范文本格式见 `abi_report.py::canonical_abi`（类型按名称排序、字段按名称排序、含 pointer_bits/schema/port/product/profile）。
ARM 与 Host 的 hash 不同是**必然且被显式分类**的：仅指针承载聚合不同；不得跨目标共享/序列化这些结构。

本次**不是**对 02 的 ABI 变更：生产头与规范布局 0 差异，因此不触发 CR。`abi_lock.h` 是把既有 Schema 6.0
首次固化为可校验基线的生成物；它是否作为冻结基线仍需 D 复核与用户按 Gate A 批准，本任务不自行宣称冻结。

## 4. 实际命令与输出

工具链（由 `commands.json` 记录的实际调用）：

- ARM：`D:\STM32CubeCLT\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe`
  —— `GNU Tools for STM32 14.3.rel1.20251027-0700`，gcc 14.3.1 20250623
- Host：`D:\STM32CubeCLT\mingw64\bin\gcc.exe`
  —— `x86_64-win32-seh-rev1`，gcc 16.2.0

ARM flags：`-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -Wall -Wextra -Werror -fno-short-enums`，
C11/C++14（C++ 另加 `-fno-exceptions -fno-rtti`）；Host 为 `-Wall -Wextra -Werror`。
独立包含性检查额外使用 ARM `-std=c99`（匹配现有 `rtconfig.py` 的固件标准）。

### 4.1 生成并校验 ABI 锁

```text
> python applications/tests/abi/abi_report.py --emit-lock
{"status": "PASS", "types": 142, "arm_hash": "f1e36c57...910b08",
 "host_hash": "e288b875...b4741", "spec_mismatches": 0,
 "unexpected_host_arm": 0, "lock": "PASS",
 "evidence": "applications/tests/abi/evidence"}
[exit 0]
```

### 4.2 CI 模式（不带 `--emit-lock`，校验漂移）

```text
> python applications/tests/abi/abi_report.py
{"status": "PASS", ... "lock": "PASS" ...}
[exit 0]
```

### 4.3 负向对照

```text
> python applications/tests/abi/negative_control.py
{"status": "FAIL", "spec_mismatches": 14, "lock": "FAIL", ...}   <- 注入后的检查器结果
{"negative_control": "PASS", "checker_exit_code": 1, "checker_status": "FAIL",
 "mismatch_count": 14,
 "navstate_mismatches": [
   {"type":"NavStateCore","kind":"size","expected":104,"actual":112},
   {"type":"NavStateCore","kind":"alignment","expected":4,"actual":8},
   {"type":"NavStateCore","kind":"offset","field":"position_ned_m","expected":4,"actual":8},
   {"type":"NavStateCore","kind":"offset","field":"velocity_ned_mps","expected":16,"actual":20}]}
[exit 0]
```

### 4.4 其他实测

- 9 个头中，除仅允许经选择器包含的 `product_contract.h` 外，其余 8 个均通过 ARM C99 与 Host C++14
  独立包含编译；`product_contract.h` 经 `fc_product_contract.h` 编译通过。
- 头文件卫生检查：0 个禁止 include、0 个 `void*`、注册消息中 0 个指针字段。
- 辅助一致性检查（一次性、非长期工具）：从 02 规范 C 块解析 136 个共享类型的**字段名与顺序**，
  与生产头逐类型比较，0 差异（`MpcOutputSnapshot` 按设计排除）。该检查复用 D 的
  `spec/tools/audit_spec.py` 抽取逻辑，未纳入长期 CI，避免与 D 侧工具耦合。
- ABI hash 可独立复算：`evidence/abi_canonical_arm.txt` / `abi_canonical_host.txt` 是用普通
  `Get-FileHash -Algorithm SHA256` 复算得到的，分别等于上表 ARM/Host hash（不依赖本工具内部实现）。
- `python` 语法/运行：以上脚本均实际执行通过；无 `__pycache__` 残留。

## 5. 验证结果映射

| 验证项 | scope | 结果 | 证据 |
|---|---|---|---|
| 生产头 ARM/Host 编译（C11/C++14） | `HOST_PRODUCTION_ABI` | PASS | `abi_arm.json` / `abi_host.json` / `commands.json` |
| ARM 布局 vs 规范 `abi_layout.json` | `SPEC_ARM_ABI_COMPILE` 对照 | PASS（142/142 类型，980 offsets，0 差异） | `abi_compare.json` |
| Host↔ARM 差异分类 | `HOST_PRODUCTION_ABI` | PASS（137 相同，5 指针承载，0 意外） | `abi_compare.json` |
| 消息 ID 注册表 | `HOST_PRODUCTION_ABI` | PASS（55/55） | `abi_compare.json` |
| ABI 锁漂移 | `HOST_PRODUCTION_ABI` | PASS | `abi_lock.h` + CI 模式 |
| 检查器负向能力 | `HOST_PRODUCTION_ABI` | PASS | `negative_control.py` 输出 |
| 独立审核（D） | — | **NOT_RUN** | 需 D 实际复核 |
| 目标固件构建/map/WCET/栈 | `ARM_TARGET_BUILD`/`TARGET_HARDWARE` | **NOT_RUN** | 不属于本任务 |
| 消费者重编译、SIL/HIL、飞行 | — | **NOT_RUN** | 后续任务 |

说明：本次 PASS 只表示“生产头在已验证工具链上编译成功且布局与规范一致”。
它**不是** Gate A 会签，**不是**目标工程 ABI 通过，也**不是**固件可发布。

## 6. 明确未执行（NOT_RUN）

1. 目标固件 `scons` 构建、ARM ELF/MAP、RAM/栈/WCET/RTA。
2. 现有消费者（M1～M7 应用代码）切换到新头后的重编译；当前 `applications/` 尚无这些消费者。
3. Keil/armcc、IAR/iccarm 编译验证（本任务只验证 GNU ARM 与 Host）。
4. `applications/SConscript` 的 include 路径/Profile 解析接入、`build/generated` Manifest 与 Profile hash 生成器。
5. `applications/tools/contract_layout.py`（09 中规划的 D 侧工具）与 `build/validation/` 证据登记。
6. 目标板/HIL/飞行；以及 UGV/USV 产品的生产化（本 Schema 只有 QUAD 规范）。
7. `MpcOutputSnapshot`（04 的 M6 私有类型）**故意不进入公共头**；规范 ABI 表中仅此 1 个类型被排除。

## 7. 未决项与对 D 的依赖（重要）

1. **前置未满足**：`T191-D-REVIEW` 仍为 `READY_FOR_ASSIGNMENT`/`NOT_RUN`。请 D 先记录接单/评审状态；
   若 D 要求前置通过后才可 ASSIGNED，请按 10 §4 记 `BLOCKED` 并保留本交付为独立准备。
2. **09 台账未登记**：本报告不能代替 09。请 D 将 §5 的 `HOST_PRODUCTION_ABI` 结果登记为真实证据，
   并明确“生产头 ABI PASS / 目标构建 NOT_RUN”。
3. **规范侧证据漂移**：`spec/validation/protected_sources_report.json` 与 `round3` 的 outside-spec 树 hash
   断言会因本轮实现文件（含本交付及 B/C 的并行交付）而失败。D 的 `review_checks.json` 已记录 `EVID-03/EVID-06 DRIFT`；
   需要 D 提供 rebaseline 工具/新快照，A 不修改 `spec/`。
4. **ABI 锁再生成权限**：`abi_lock.h` 是生成物；任何字段/容量/消息表变化都必须走 ICD CR 并由 D 复核后
   才能重新运行 `--emit-lock`。请勿手工编辑。
5. **C99 兼容**：现有固件 `rtconfig.py` 使用 `-std=c99`。公共头已提供可移植断言回退并通过 ARM C99 编译；
   若后续切到 C11，行为不变。
6. **私有类型边界**：若 D 的比对脚本要求 143/143 类型全等，请以本报告的排除说明为准，
   或由 D 决定是否将 `MpcOutputSnapshot` 纳入独立的 M6 私有 ABI 检查（不应进入公共头）。

## 8. 回滚

- 工作区**无 Git 元数据**，`commit=null`；以 `evidence/artifact_sha256.json` 作为源文件快照追溯。
- 本次**只新增文件，未修改任何既有文件**。回滚即删除以下新增文件/目录；若目录中已有其他 Agent 的并行产物，
  只删除本报告 §2 列出的具体文件，不要整目录删除：
  - `applications/contracts/`（本任务独占）
  - `applications/products/QUAD_X4_H723/product_contract.h`、`abi_lock.h`
  - `applications/tests/abi/`（本任务独占）
- 由于尚无构建接入或消费者引用，删除后不会影响现有固件构建。
- 若只回滚 ABI 锁：删除 `abi_lock.h` 后，`abi_report.py` 的锁校验会失败（预期），
  需同时删除 `evidence/` 或在 D 批准后重新生成。
- 本次未修改 `spec/`、`board/`、`libraries/`、`packages/`、`rt-thread/`；执行中生成的
  `applications/tests/abi/__pycache__` 与 `spec/tools/__pycache__` 已删除，不构成交付。

## 9. 状态请求

结构化请求见 `applications/tests/abi/status_request_T191-A-ABI.json`。要点：

- `task_id=T191-A-ABI`、`baseline=V1.9.1-GLOBAL-AUDIT-R3`、`update_sequence=2`；
- 请求 `READY_FOR_ASSIGNMENT → ASSIGNED`（执行侧确认收到 prompt/基线），
  但 `T191-D-REVIEW` 未满足，D 可改记 `BLOCKED` + `resume_state`；
- `verification_scope=HOST_PRODUCTION_ABI`、结果 PASS（真实命令/证据见 §4/§5）；
- `reviewer=D`、`independent_review=NOT_RUN`；
- 不请求 ACCEPTED/CLOSED；不修改 HANDOFF 或 09。

## 10. 可复现性

```text
cd D:/STM32Project/FMT-Firmware-1.2.0/Flight
python applications/tests/abi/abi_report.py --emit-lock
python applications/tests/abi/abi_report.py
python applications/tests/abi/negative_control.py
```

交付物哈希清单：`applications/tests/abi/evidence/artifact_sha256.json`。
关键文件 SHA256：

| 文件 | SHA256 |
|---|---|
| `applications/contracts/fc_contract_version.h` | `96ba5a7fb3086a5e872f06d49885531d16d9b7896e226991e61a541b2ddb4cf6` |
| `applications/contracts/fc_common_base.h` | `5ea5eec31af37237abfe4332def04c09da5399cb8439d64e8a50f13c25803596` |
| `applications/contracts/fc_common_messages.h` | `ec342762cf2d1ff35f6c1ac3847b88bb89c376ddf3e7113fd288a33ba5e0fad5` |
| `applications/contracts/fc_port_types.h` | `361aae78677916eaa7192c4c15e97edcfe2ef889a6d5dc95d074d35ee34fef03` |
| `applications/contracts/fc_port_api.h` | `b1395d2d20a899e2dc22918c9d8dd0eb5a9a7fadd162aa3e7eb074acbed97fbc` |
| `applications/products/QUAD_X4_H723/product_contract.h` | `2d01458e93eefc5a3e878b1afff27056a238719da7e20f686eaeddbaf8217656` |
| `applications/products/QUAD_X4_H723/abi_lock.h` | `793b2f5f78a751f43d5da96fb5e28711a3c3c17e2354a072eb3402b65ac11111` |
| `applications/tests/abi/abi_report.py` | `247df4a31706cc2f8669d1f266ad50f88783e8b1198f2088d749c8ed73e52a8f` |

（清单还包含本报告与状态请求；清单文件自身除外。）

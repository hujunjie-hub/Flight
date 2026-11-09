# A 对 D-REVIEW §18.2 中 A 项（B-R03/B-R05）的处置

适用范围：`T191-D-REVIEW` 记录 §18.2（B 的意见）中 Owner=A 的两项；不代替 D 复核，
不改变 Gate A/ABI 冻结状态。B-R08（28KiB 余量）仍未处置，理由见 §4。

## 1. B-R03 — `rtconfig.py` 缺 `-fno-short-enums`（已修复）

**根因（已实测）**：GNU Arm GCC 默认 `__ARM_SIZEOF_MINIMAL_ENUM=1`（short enums）。公共 ICD
把状态/原因枚举作为 32-bit 值存储（02 §1.4.3），生产头的编译期断言要求
`sizeof(TimeQuality)==4`。用现有 `rtconfig.py` 的 flag 编译 `fc_port_api.h` 时断言失败：

```text
arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard \
  -std=c99 -Iapplications/contracts -Iapplications/products/QUAD_X4_H723 ...
applications/contracts/fc_contract_version.h:58: error:
  size of array 'fc_static_assert_4' is negative
```

加 `-fno-short-enums` 后 exit 0。

**修复**：`rtconfig.py` GCC 分支 CFLAGS 增加 `-fno-short-enums`（CXXFLAGS 在该行之后复制
CFLAGS，因此 C++ 同步生效）；armcc/iccarm 分支未改（其默认枚举宽度不同，需另行目标核验）。

**ABI 证据加固**：`applications/tests/abi/abi_report.py` 新增 `check_firmware_flags()`，
要求 `rtconfig.py` 含 `-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
-fno-short-enums`，作为 ABI 证据的一部分（`firmware_build_flags`，本次 PASS）。
ABI hash 未变化（ARM `f1e36c57…`、Host `e288b875…`）。

## 2. B-R05 — Kconfig 聚合可达性（已核实，补充规则）

**现状核实**：

- 根 `Kconfig` → `rsource "applications/Kconfig"`；
- `applications/Kconfig` → `rsource "core/Kconfig"`、`core/m2/Kconfig`、`core/m3/Kconfig`、
  `services/health/Kconfig`；
- `core/Kconfig` → `rsource "m4/Kconfig"`、`m5/Kconfig`；
- `FMS_QUAD_X4_ENABLE` 定义于 `core/m4/Kconfig:1`，因此当前已可达（B 复核时的
  `false` 结论已被 C 新增的 `core/Kconfig` 消除）。

**仍存在的结构事实**：

- `core/m6/`、`core/m7/`、`plugins/*/` 目前没有任何 `Kconfig` 文件；`plugins/` 下也没有
  聚合器。没有可 source 的对象，不能凭空创建 C/B 的配置符号。
- 原缺陷的另一半是“没有 source 规则”。已在 `applications/Kconfig` 写入聚合规则：
  core/<module>/Kconfig、services/<service>/Kconfig 必须显式接入；新增
  `plugins/<area>/Kconfig` 时在此加一行 rsource（或统一 `plugins/Kconfig` 聚合器）并记录到 10。

**结论**：可达性缺陷当前已消除；规则已写入。后续 M6/M7/插件新增 Kconfig 时按该规则接入，
不另建第二套聚合器。

## 3. 验证

```text
arm-none-eabi-gcc <rtconfig flags without -fno-short-enums> -c fc_port_api.h
  -> FAIL (enum static assert)          # 复现 B-R03
python applications/tests/abi/abi_report.py
  -> PASS; firmware_build_flags.missing = []
```

用 `rtconfig.py` GCC 分支的实际 CFLAGS（`RTT_CC=gcc` 加载后为
`-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -ffunction-sections
-fdata-sections -Dgcc -fno-short-enums -O0 ... -std=c99`）编译生产头：
C exit 0、加 `-std=c++14 -fno-exceptions -fno-rtti` 的 C++ exit 0；
记录在 `evidence/firmware_flags_compile.log`。

证据：`applications/tests/abi/evidence/abi_compare.json`（`firmware_build_flags`）、
`abi_report.md`、`applications/platform/rtthread53/evidence/firmware_flags_compile.log`。

## 4. 未处置

- **B-R08**（28KiB 预算余量 464B）：需要 A/B 提供每实例运行时开销并更新 07 表；不阻塞
  A-ABI/A-PLATFORM 当前阶段，保留 OPEN。
- B-R01/B-R05 的 05/10 正文改动属 B/D Owner，A 只改了 `applications/Kconfig` 规则注释。

## 5. 文件

| 文件 | 变化 |
|---|---|
| `rtconfig.py` | GCC CFLAGS 增加 `-fno-short-enums` |
| `applications/Kconfig` | 增加 Kconfig 聚合规则注释（不改变现有符号） |
| `applications/tests/abi/abi_report.py` | 新增 `check_firmware_flags()` |
| `applications/tests/abi/evidence/abi_compare.json` / `abi_report.md` | 重新生成，含该检查 |

回滚：`rtconfig.py` 去掉该 flag、`applications/Kconfig` 删除注释块、`abi_report.py` 删除
该检查并重跑；ABI 布局/hash 不受影响，但固件枚举宽度会回到 short-enum，禁止在此状态下
声称 ABI 兼容。

# CR-V191-C01 / CR-V191-C02 实施记录（A）

| 项目 | 内容 |
|---|---|
| CR | `CR-V191-C01`（mission_epoch）、`CR-V191-C02`（围栏编译期入口） |
| 基线 | `V1.9.1-GLOBAL-AUDIT-R3`；实施后 Product/Port `0x00010904`，Schema 6.0 不变 |
| Owner | A（02/ABI）；06/08 为 C 文件，本次按用户指示提交 A patch，C/D 复核 |
| 复核状态 | `independent_review_status=NOT_RUN`；`user_approval=NOT_RUN`；不是 ABI 冻结 |
| 日期 | 2026-11-05（Asia/Shanghai） |

## 1. CR-V191-C01：mission_epoch

- `02` §4.2 `ProductOperationPayload` 末尾增加 `uint32_t mission_epoch;`（44→48B）；
  §1.4.3 注册 `valid_mask` bit8 `MISSION_EPOCH`；新增 CR 段说明 START/PAUSE/RESUME/
  ABORT/CLEAR 必须置 bit8 且非零并匹配当前任务 epoch，旧 epoch/0/未置位拒绝。
- `06` §12 命令表：上述五类任务命令改为必须 `VALID_MISSION_EPOCH` + 非零 epoch；旧 epoch
  拒绝且保留当前任务。
- `applications/products/QUAD_X4_H723/product_contract.h`：字段、bit8 宏、
  `PRODUCT_CONTRACT_REVISION=0x00010904`。
- 布局影响：`ProductOperationPayload` 44→48B；`OperationRequest` 仍 160B（原 padding 吸收），
  既有字段 offset 不变；`mission_epoch=0` 表示不适用，旧生产者产生 0 时任务命令 fail-closed。

## 2. CR-V191-C02：围栏编译期入口

- `02` §4.2 新增 `ProductGeofenceDefinition`、`ProductGeofenceConfigPayload`、
  `ProductGeofenceConfig`（`ProductGeofenceSnapshot` 之后）；新增 CR 段明确 M4 唯一持有、
  编译期 `.rodata` 来源、`fms_policy_init` 校验、无运行时 `SET_GEOFENCE`、Companion 只读候选。
- `02` 明确 `FmsInputView` 不新增围栏指针（围栏不是 M1 输入）。
- `06` §GeofenceManager：补编译期配置来源、M4 init 校验与无运行时命令；Companion 不得携带/覆盖围栏。
- `08` 新增 §3.2 围栏编译期配置入口（`product_manifest.yaml` → `.rodata`、hash 入 Profile、
  变更走 CR/重编译、测试 `TEST-V191-GEOFENCE`），并更新“不进入日常调参”表。
- 新增类型不改变任何既有消息布局；ABI surface 增加 3 个类型。尺寸（ARM）：
  `ProductGeofenceDefinition=280B`、`ProductGeofenceConfigPayload=1132B`、
  `ProductGeofenceConfig=1144B`。

## 3. ABI 升级与重跑锁

| 项 | 旧 | 新 |
|---|---|---|
| Schema | 6.0 | **6.0（不变）** |
| Product/Port revision | `0x00010903` | **`0x00010904`** |
| ARM ABI hash | `f1e36c57…` | **`588707eb4e722b7d9d885d9ab2c40767a69bfad99f8e214da54d17ba4cc8d9fc`** |
| Host ABI hash | `e288b875…` | **`2e8c855d1b6522cccc725cd582007bd141306982fcc69ac68c8bbd7ffe166a4b`** |
| 公共类型 | 142 | **145** |
| 规范比对 | — | 145 类型 / 998 offsets，0 差异 |
| Host↔ARM | 137 相同/5 指针 | 140 相同/5 指针，0 意外 |

版本决策：只升 Product/Port，不升 Schema Major/Minor。理由：`ContractHeader` 与公共消息
宽度/顺序/对齐未变，变化限于 product payload（+4B）和新增编译期产品配置类型；`mission_epoch=0`
对旧生产者 fail-closed；所有消费者必须按新 Port revision 重编译并匹配新 ABI 锁。若 D/用户
认为必须升 Schema Major，需另开 CR 并全量替换版本声明。

命令与结果：

```text
python applications/tests/abi/abi_report.py --emit-lock
{"status":"PASS","types":145,
 "arm_hash":"588707eb...","host_hash":"2e8c855d...",
 "spec_mismatches":0,"unexpected_host_arm":0,"lock":"PASS"}
python applications/tests/abi/negative_control.py
{"negative_control":"PASS","checker_exit_code":1,"checker_status":"FAIL",
 "mismatch_count":14,"lock_problems":2}
```

`spec/validation/abi_layout.json` 已由 D 在 00:08 重生成（146 类型，含 `MpcOutputSnapshot`），
当前与 02 的 `icd_sha256` 匹配；A 未修改该文件。

## 4. B-R03 / B-R08

- **B-R03**：`rtconfig.py` GCC 分支已加 `-fno-short-enums`；`abi_report.py` 的
  `firmware_build_flags` 检查强制该旗标（否则 ABI 证据 FAIL）。
- **B-R08**：`07` §6.2 增列 M3 estimator arena：
  active/shadow 各 15,360B = 14,104B workspace + 1,256B per-instance 固定开销
  （control block 64 + fusion/checkpoint 640 + guard/align 128 + diagnostics 424），
  两实例 30,720B=30KiB；`07` §6.3 DTCM 改为 94.5KiB、余量 33.5KiB（≥32KiB）；
  `05` §9 的 28KiB 同步改为 30KiB/15,360B。`05` 为 B 文件，本次为 A patch，B 复核。

## 5. 未执行 / 待 D

1. 09 未登记本 CR 的 ABI 证据；`TEST-T191-D-REVIEW`、`TEST-V191-GEOFENCE` 仍按 D 维护。
2. 00/AGENT/README/09/11/HANDOFF 中仍有 `0x00010903` 或 CR `PROPOSED` 的旧表述，需 D 统一。
3. round3/`change_summary`/`t191-d-review_report` 需在 00/02/05/06/07/08 变更后重建。
4. 生产固件 ARM 构建/map、WCET、目标围栏/epoch 测试、SIL/HIL 均 NOT_RUN；`TEST-V191-GEOFENCE` 未执行。
5. 独立复核与用户 Gate A/ABI 冻结批准 NOT_RUN；本记录不代替 CR 批准。

## 6. 回滚

- 版本回滚：恢复 02/00 的 `0x00010903`、删除 mission_epoch/围栏配置类型，重新 `--emit-lock`；
  旧锁与新头不得混用。
- 文件回滚点：CR 前 02 由 layout `icd_sha256` 旧值标识；07 在 B-R08 前为 `aacf4ca9…`；
  05 的 28KiB 行、06/08 的新增段可整体删除。

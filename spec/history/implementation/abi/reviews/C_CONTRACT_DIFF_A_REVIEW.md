# A 对 C 侧 product_contract.h 契约 diff 的复核（CR-V191-C01/C02）

| 项目 | 内容 |
|---|---|
| 复核人 | Agent A（02/ICD、ABI lock 与生产头 Owner） |
| 被复核对象 | C 提交的 `applications/products/QUAD_X4_H723/product_contract.h` 契约 diff（`mission_epoch` + `PRODUCT_CONTRACT_REVISION=0x00010904`）及重生成的 `abi_lock.h`/ABI 证据 |
| 基线 | `V1.9.1-GLOBAL-AUDIT-R3` / Schema 6.0 / Product-Port `0x00010904` |
| 对照基线 | CR 前 `spec/validation/T191-D-REVIEW/r3_archive/round3_abi_layout.json`（143 类型，`ProductOperationPayload`=44B） |
| 机器证据 | `c_contract_diff_check.json`（20/20 PASS）、`c_contract_diff_check.py` |
| 结论 | **PASS（契约/ABI）**；附 3 项非阻塞观察 |

## 1. 复核范围与声明

- 复核的是**契约与 ABI**：`ProductOperationPayload` 字段/valid bit、版本宏、围栏配置类型、
  `abi_lock.h` 与 Host/ARM 证据。
- 不复核 C 的 FMS 状态机、epoch 行为测试、围栏几何算法和目标/HIL；这些归 `T191-C-FMS`
  与 Gate D/E。
- A 是 02/ABI 的 Owner，C 是产品绑定/实现作者；本意见不代替 D 的合并与用户批准。

## 2. 契约 diff（相对 CR 前基线）

由当前 `abi_arm.json` 与 CR 前归档逐类型/逐 offset 比较：

| 变化 | 结果 |
|---|---|
| 新增类型 | `ProductGeofenceDefinition`、`ProductGeofenceConfigPayload`、`ProductGeofenceConfig`，无其他新增 |
| 删除类型 | 无 |
| size/alignment 变化 | **仅** `ProductOperationPayload`：44→48B，align 4 不变 |
| `ProductOperationPayload` 既有字段 offset | 全部不变；`mission_epoch` 位于 offset 44（末尾） |
| `OperationRequest` | 160B、align 8、全部字段 offset 不变（原 padding 吸收 +4B） |
| `ProductGeofenceDefinition/ConfigPayload/Config` | 280 / 1132 / 1144B，字段与 02 规范块一致 |
| 其他 138 个共享类型 | 无任何 size/align/offset 变化 |

因此 C 的 `mission_epoch` 改动符合 02 §4.2 的“末尾追加、0=不适用”设计，没有顺带改变
其他公共布局。

## 3. 版本与 valid_mask

- `PRODUCT_CONTRACT_REVISION=0x00010904u`，与 `FC_PORT_API_REVISION=0x00010904u` 一致
  （`FC_STATIC_ASSERT` 通过）。
- `PRODUCT_OPERATION_VALID_MISSION_EPOCH = (1u << 8)`，与 02 §1.4.3 注册一致。
- `ProductOperationPayload` 内 `uint32_t mission_epoch;` 仅出现一次；`PRODUCT_PROFILE_ID`
  仍为 `0x00010001`。
- 围栏配置类型与 02 §4.2 同名同序；`ProductGeofenceSnapshot` 未改。

## 4. abi_lock.h（145 类型）

| 项 | 值 |
|---|---|
| `FC_ABI_LOCK_TYPE_COUNT` | **145**（142 旧公共类型 + 3 围栏配置类型） |
| `FC_ABI_LOCK_FIELD_COUNT` | 1003 |
| Schema | 6.0（Major 6 / Minor 0，公共 envelope 未变） |
| Port / Product revision | `0x00010904` / `0x00010904` |
| `ContractHeader` | 88B |
| ARM hash | `588707eb4e722b7d9d885d9ab2c40767a69bfad99f8e214da54d17ba4cc8d9fc` |
| Host hash | `2e8c855d1b6522cccc725cd582007bd141306982fcc69ac68c8bbd7ffe166a4b` |
| lock SHA256 | `4781f28b6e5a20a159097050e60fbb16b043421149e9ece6059845d62662e8e2` |

锁内 ARM/Host hash 与 `abi_arm.json`/`abi_host.json` 一致；消息注册表 SHA 仍为
`e2893aeb…`（55 条未变）。

## 5. ABI 证据确认

| 检查 | 结果 |
|---|---|
| 生产头 ARM/Host × C11/C++14 编译 | PASS |
| 与 `spec/validation/abi_layout.json`（146 类型，含私有 `MpcOutputSnapshot`） | PASS：145 公共类型 / 998 offsets，0 差异 |
| Host↔ARM | PASS：140 相同，5 个指针承载类型按预期不同，0 意外 |
| 消息 ID / 固件 flags / 头文件卫生 / 自包含 | PASS |
| 负向对照（注入 `NavStateCore` 位宽变化） | PASS：检查器 FAIL（14 spec 差异 + lock 失败） |

## 6. 观察（不阻塞契约结论）

1. **OBS-C-DIFF-01（P3，C）**：`applications/core/m4/src/m4_mission.c` 文件头仍写
   “OPEN ICD GAP … ProductOperationPayload has no such field … does not claim epoch
   binding”，但同一文件 214–220 行已实现 epoch 必需/旧 epoch 拒绝。注释与实现矛盾，
   建议 C 更新注释，避免审计误判。
2. **OBS-C-DIFF-02（P3，C/D）**：C 的 `status_request_T191-C-FMS.json` 仍写
   baseline `0x00010903`、`update_sequence=2`、A-ABI `READY_FOR_ASSIGNMENT`；
   实际 HANDOFF 已 `T191-A-ABI ACCEPTED/PASS/seq 7`、baseline `0x00010904`。属提交头
   未刷新，不影响契约本身。
3. **OBS-C-DIFF-03（P3，C/D）**：C02 的编译期入口在 C 侧实现为
   `quad_x4_fms_configure_product_geofence()`（在 `fms_policy_init` 前安装），而 02/06/08
   只写“M4 在 fms_policy_init 校验/持有”，未命名该 composition-root 钩子。建议 C/D 在
   02/06/08 明确该钩子（非公共 Port、非运行时可写），或改由 Profile 静态符号绑定；
   这是文字入口问题，不影响 `ProductGeofenceConfig` 的 ABI。

## 7. 复核结论

- C 的 `product_contract.h` 契约 diff 与 02 规范一致；只引入预期变化（`mission_epoch` +
  3 个围栏配置类型），`OperationRequest` 与其余公共类型布局不变。
- `abi_lock.h` 已按 145 类型、`0x00010904` 重建；Host/ARM 证据与规范布局均 PASS。
- 契约/ABI 层面 **PASS**；上述 3 项观察交 C/D 处理，不改变锁定值，也不构成 ABI 冻结批准。

## 8. 文件哈希

| 文件 | SHA256 |
|---|---|
| `product_contract.h` | `940b94461c6230ff535e00b29f8a82a75f4da0c8f5a02cc47f6537f4f899dcca` |
| `abi_lock.h` | `4781f28b6e5a20a159097050e60fbb16b043421149e9ece6059845d62662e8e2` |
| `abi_arm.json` | `c20185e365c6b796d724e1dbd1e681aca69da587c37119e54b2906db6c45f8af` |
| `abi_host.json` | `c0beff4b4903a1aba7bf848a3535be4946a702e0a940360a0e1d1a49cd2f0816` |
| `abi_compare.json` | `4aff2e60c2b2358d550fcaa45e0f6c76bc7d7d75441bfe3c828493f29927a9d0` |

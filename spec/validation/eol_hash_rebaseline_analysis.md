# Spec/ABI 证据的 CRLF/LF 哈希重基线分析

状态：**BLOCKED / WAITING_OWNER**。本文只做分析，不修改 `middleware/contracts/product_contract/abi_lock.h` 或已提交的 spec 证据。

## 1. 现象

在 LF 工作区（仓库 `.gitattributes` 固定 `* text=auto eol=lf`，Linux CI 也是 LF）执行：

```powershell
python spec/tools/audit_spec.py --round 3 --label diag
python tests/abi/abi_report.py
```

结果：

```text
SPEC_AUDIT_FAILED: spec/validation/dialect_report.json spec_xml_sha256 does not match
spec/contracts/FCU.xml; the committed evidence is stale or was generated from a
different line-ending checkout
```

```json
{"status": "FAIL", "lock": "FAIL", "spec_mismatches": 0, "unexpected_host_arm": 0,
 "arm_hash": "12b64f6368aad4b96b5bdd21f382a1babc810a2df96680af7853273e2e7f66c4",
 "host_hash": "0edc9d8baaca7a826d9e807132f0cf232fd55f7a9b646d4d9a82aab4c288eacf"}
```

## 2. 决定性哈希对照

| 对象 | 已提交 blob（LF）SHA256 | 证据中钉住的值 | 结论 |
|---|---|---|---|
| `spec/contracts/FCU.xml` | `17d6e980731244852e415eb7cb7a124033160ccd7119f751c0dbe52078ce4321` | `1be7adc3b849bc89f812ad66353368ae2c8c6290f2ea97168ee710a7145446ed`（`dialect_report.json`） | 钉住值 = 同一文本的 **CRLF** 哈希 |
| `spec/contracts/message_registry.json` | `dd06554c240567d5f81b51e4aed39336ccf3e469c6c95a952d1a2d766bbc49e7` | `e2893aebd8c1121b1ede80591849730ec722c5f9e0616443507c2ff99179bf41`（`abi_lock.h`） | 钉住值 = 同一文本的 **CRLF** 哈希 |
| ABI 布局（ARM / Host） | `12b64f63…66c4` / `0edc9d8b…eacf` | 与 `abi_lock.h` **完全一致** | **无 ABI 漂移** |

复算命令：

```powershell
python -c "import hashlib,subprocess; d=subprocess.check_output(['git','cat-file','blob','HEAD:spec/contracts/FCU.xml']); print(hashlib.sha256(d).hexdigest()); print(hashlib.sha256(d.replace(b'\n',b'\r\n')).hexdigest())"
```

## 3. 根因

1. 证据文件在 Windows CRLF 工作区生成并提交；Git 以 LF 存入 blob，此后任何 LF 检出（含 `ubuntu-latest` CI）都无法复现钉住的哈希。
2. `assert` 在 `python -O` 下会被剥离，原本可能把这种不一致变成静默通过；本轮已改为 `audit_spec.require()` 并由 `tests/guidance/test_audit_optimized.py` 做 `-O` 负向验证。
3. GitHub Actions 的四个 job 都不执行 `audit_spec.py` 或 `abi_report.py`，因此问题从未在 CI 暴露。

## 4. 影响与非影响

- **非影响**：ABI 结构布局、字段偏移、类型数量与锁文件完全一致（`spec_mismatches: 0`，`unexpected_host_arm: 0`），不存在 ABI 漂移。
- **影响**：spec/ABI 证据不可复现；任何真实的 spec 或 ABI 变更都可能被这个“永远失败”的状态掩盖。

## 5. 重基线步骤（需所有者批准，本轮未执行）

1. 在 LF 工作区生成 spec 方言证据并提交：
   ```powershell
   python spec/tools/check_dialect.py      # 需要 pymavlink
   ```
   该工具会以 LF 内容重写 `spec/validation/dialect_report.json`。
2. 在 LF 工作区重钉 ABI 消息注册表哈希（ABI 布局哈希不变）：
   ```powershell
   python tests/abi/abi_report.py --emit-lock
   ```
   该命令会修改 `middleware/contracts/product_contract/abi_lock.h` 的 `ABI_LOCK_MESSAGE_REGISTRY_SHA256`，属于 ABI 锁变更，必须走 ABI 评审。
3. 在 CI 的 `Source layout and time-service tests` job 增加：
   ```yaml
   - run: python spec/tools/audit_spec.py --round 3 --label ci
   - run: python tests/abi/abi_report.py
   ```
   并在 ARM job 中保留 `check_consumer_fp_contraction.py`。
4. 重基线后必须复核 `spec/09` 与 `spec/validation/*.json` 的对应关系，避免历史报告与新哈希混用。

## 6. 当前状态

- `spec_layout`：FAIL（证据过期，非代码缺陷）。
- `production_abi`：FAIL（`abi_lock.h` 的注册表哈希为 CRLF 值；布局一致）。
- 未修改 `abi_lock.h`、`dialect_report.json` 或任何 spec 证据。
- 该阻塞不阻止 M5-06 Host/SIL、Sanitizer 与 Cortex-M7 编译证据，但阻止“spec/ABI 证据可复现”这一验收项。

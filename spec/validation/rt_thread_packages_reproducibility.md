# 固件链接所需 RT-Thread packages：版本锁定与可重复安装流程

状态：2026-11-12 批准的三项依赖按完整 Git commit 固定，锁文件为 `RESOLVED`。下方 1–7 节保留批准前的实施记录，当前安装及验证流程见第 8 节。

## 1. 为什么需要 packages/

- 根 `SConscript` 无条件执行 `SConscript('packages/SConscript')`；
- 根 `Kconfig` 执行 `osource "$PKGS_DIR/Kconfig"`；
- `packages/` 自首次提交起就在 `.gitignore` 中，且 `packages/SConscript` 从未进入 Git 历史（已用 `git log --all -- packages/` 核实）；
- `libraries/` 只有 `HAL_Drivers`，没有 STM32H7 HAL/CMSIS 源码。

因此干净克隆无法执行固件链接：`check_target_build.py` 现在返回 `BLOCKED`（exit 2）而不是一个含糊的 SCons 错误。

## 2. 当前选择（来自 `.config`）

| 包 | 配置项 | 当前版本 | 可复现 |
|---|---|---|---|
| CMSIS_CORE | `CONFIG_PKG_CMSIS_CORE_VER` | `"latest"` | 否 |
| STM32H7_HAL_DRIVER | `CONFIG_PKG_STM32H7_HAL_DRIVER_VER` | `"latest"` | 否 |
| STM32H7_CMSIS_DRIVER | `CONFIG_PKG_STM32H7_CMSIS_DRIVER_VER` | `"latest"` | 否 |

`latest` 正是 `spec/00 §4` 明确拒绝的做法：同一天的不同时刻可能拉到不同内容。

## 3. 锁定文件格式（已实现）

`tools/rt_packages.lock.json`：

```json
{
  "schema": 1,
  "status": "UNRESOLVED | RESOLVED",
  "resolved_by": "<审批人>",
  "resolved_at_utc": "<ISO-8601>",
  "packages": [
    {
      "symbol": "STM32H7_HAL_DRIVER",
      "config_version_key": "CONFIG_PKG_STM32H7_HAL_DRIVER_VER",
      "selected_version": "latest",
      "pinned_version": "v1.11.0",
      "pinned_commit": "<40-hex>",
      "required_for": "STM32H723 HAL sources linked into the firmware"
    }
  ]
}
```

校验器 `tools/check_rt_packages_lock.py`：

- 解析 `.config` 的 `CONFIG_PKG_USING_*`；
- 要求每个被选中的包在锁文件中有 `pinned_version`，且不等于 `latest`；
- 要求 `.config` 的 `CONFIG_PKG_<NAME>_VER` 与锁文件一致；
- 要求锁文件里没有已取消选择的包；
- 结果写入 `build/m4_m5_implementation/evidence/rt_packages_lock.json`。

## 4. 可重复安装流程（待批准后执行）

1. **选定版本**（需要所有者/架构负责人批准）：对每个包选定 tag（例如 CMSIS Core 与 STM32H7 HAL 的发布版本），记录来源仓库与 tag。
2. **写入锁文件**：填 `pinned_version`、`pinned_commit`、`status: RESOLVED`、审批人与时间。
3. **修改 `.config`**：把 `CONFIG_PKG_<NAME>_VER="latest"` 改为锁定版本，并关闭 `CONFIG_PKG_USING_<NAME>_LATEST_VERSION`。
4. **安装**（Windows PowerShell，RT-Thread env 工具）：
   ```powershell
   # 一次性准备 env 工具（不在本仓库内提交）
   git clone --depth 1 https://github.com/RT-Thread/env.git $env:USERPROFILE\rt-env
   $env:Path = "$env:USERPROFILE\rt-env;$env:USERPROFILE\rt-env\tools\scripts;$env:Path"
   cd E:\Flight\Flight_Test
   pkgs --update
   ```
5. **验证**：
   ```powershell
   python tools/check_rt_packages_lock.py
   Test-Path packages/SConscript
   python tests/guidance/check_target_build.py
   ```
6. **证据**：把 `packages/` 的实际内容哈希（每个包的 commit 与文件树 SHA256）写入 `build/m4_m5_implementation/evidence/`，并在 `spec/09` 记录。

## 5. 为什么本轮不直接锁定

- 变更包版本会改变固件依赖基线（HAL/CMSIS 行为、链接内容、许可证清单），属于需要批准的依赖变更；
- 本地未安装 RT-Thread env 工具，无法在**当前环境**证明某个 tag 能被 `pkgs --update` 拉取并成功链接；
- 在没有真实安装证据前填写 `pinned_version` 会制造“看起来已锁定”的假象，违反“不得猜测”的规则。

## 6. 验收状态

| 项 | 状态 |
|---|---|
| 锁文件格式 | IMPLEMENTED |
| 校验器 | IMPLEMENTED，当前返回 FAIL（3 个包未锁定） |
| 安装流程 | DOCUMENTED |
| 实际锁定与安装 | BLOCKED（等待版本批准与 env 工具） |
| 固件链接证据 | BLOCKED（`packages/SConscript` 缺失） |

## 7. R4 更新：候选版本与更严格校验

- 新增 `spec/validation/rt_thread_packages_candidates.md`：三个包的上游 tag、commit 与文件级兼容性证据（`stm32h723xx.h`、`core_cm7.h` 均以 HTTP 200 核实）。
- `tools/check_rt_packages_lock.py` 现在额外要求：
  - `lock.status == RESOLVED` 且 `resolved_by` / `resolved_at_utc` 非空；
  - `pinned_commit` 为 40 位十六进制；
  - `CONFIG_PKG_USING_<NAME>_LATEST_VERSION` 必须已关闭；
  - `.config` 版本与锁定版本完全一致；
  - 锁文件不得包含已取消选择的包。
- 新增 `tests/guidance/test_rt_packages_lock.py`：10 个负向用例，覆盖上述每条规则。
- 当前三个包仍为 `latest`，校验器按设计继续 FAIL；在所有者批准版本并完成真实 `pkgs --update` 前不得改为 RESOLVED。

## 8. 当前安装与验证

在 RT-Thread Env 4.x 中执行 `pkgs --update`，用 `tools/rt_packages.lock.json` 的 `installed_dir` 和 `pinned_commit` 核对各包。索引仍以 `latest` 选择包。若拉取的 HEAD 变化，在包自身的 Git 检出中取得并检出锁定 commit，然后运行：

```powershell
python tools/check_rt_packages_lock.py --installed
python tests/guidance/check_target_build.py
```

验证入口会核对三个包的实际 HEAD 和跟踪文件状态，验证失败时链接步骤返回 BLOCKED。源目录检查允许 SCons 生成的未跟踪对象和 `.su` 文件。

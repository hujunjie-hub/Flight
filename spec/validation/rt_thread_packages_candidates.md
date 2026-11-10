# RT-Thread package 版本候选与兼容性证据（待审批）

状态：**CANDIDATES_ONLY / NOT_APPROVED**。本文只整理候选方案与证据；`tools/rt_packages.lock.json` 保持 `UNRESOLVED`，未执行 `pkgs --update`，不得据此宣称固件链接可复现。

## 1. 当前选择

`.config` 中三个包均为 `VER="latest"`：

| 包 | 配置项 | 当前值 | `_LATEST_VERSION` |
|---|---|---|---|
| `CMSIS_CORE` | `CONFIG_PKG_CMSIS_CORE_VER` | `"latest"` | 开启 |
| `STM32H7_HAL_DRIVER` | `CONFIG_PKG_STM32H7_HAL_DRIVER_VER` | `"latest"` | 开启 |
| `STM32H7_CMSIS_DRIVER` | `CONFIG_PKG_STM32H7_CMSIS_DRIVER_VER` | `"latest"` | 开启 |

`spec/00 §4` 明确说明 `latest` 标签不足以复现。

## 2. 候选版本（上游 tag 已核实到 commit）

| 包 | 候选 tag | Commit | 上游仓库 | 兼容性证据 |
|---|---|---|---|---|
| `CMSIS_CORE` | `5.9.0` | `61e36449f53c25ef7825c40f7dd93685736f457f` | `ARM-software/CMSIS_5` | 该 tag 下 `CMSIS/Core/Include/core_cm7.h` 存在（HTTP 200 校验） |
| `STM32H7_CMSIS_DRIVER` | `v1.10.7` | `05ea8164ca19102bb5c9eebf0c45f7ac2cd1e35c` | `STMicroelectronics/cmsis_device_h7` | 该 tag 下 `Include/stm32h723xx.h` 存在（HTTP 200 校验），覆盖 STM32H723 器件定义 |
| `STM32H7_HAL_DRIVER` | `v1.11.6` | `33ca46547d40b281a98b5ca497efc89dc628a4a7` | `STMicroelectronics/stm32h7xx_hal_driver` | 该 tag 下 `Inc/stm32h7xx_hal.h` 存在；H7 HAL 由器件头文件选择 H723 外设 |

复核命令：

```powershell
git ls-remote --tags --refs https://github.com/ARM-software/CMSIS_5.git refs/tags/5.9.0
git ls-remote --tags --refs https://github.com/STMicroelectronics/cmsis_device_h7.git refs/tags/v1.10.7
git ls-remote --tags --refs https://github.com/STMicroelectronics/stm32h7xx_hal_driver.git refs/tags/v1.11.6
```

## 3. 为什么这些候选合理（但还不是结论）

1. `board/SConscript` 定义 `STM32H723xx`，器件头文件必须提供 `stm32h723xx.h`；`v1.10.7` 满足。
2. Cortex-M7 内核头 `core_cm7.h` 由 CMSIS Core 提供；`5.9.0` 满足。
3. HAL 版本选择以“器件头 + HAL 驱动都可编译”为准，`v1.11.6` 是当前上游最新稳定 tag。

## 4. 仍未验证的部分（不得跳过）

- **RT-Thread 包索引的实际可选版本**：`packages.rt-thread.org` 的索引接口在当前网络返回 404，无法确认包管理器是否提供上述 tag；必须以 `pkgs --update` 的真实输出为准。
- **实际安装与链接**：本地未安装 RT-Thread env 工具，未执行 `pkgs --update`，因此没有任何“这三个版本能编译并链接 H723 固件”的证据。
- **许可证与体积**：HAL/CMSIS 的文件清单、许可证文本与 flash/RAM 增量未核对。
- **RT-Thread BSP 期望版本**：`rt-thread/bsp` 未随本仓库提供，无法核对 BSP 侧对 HAL 版本的假设。

## 5. 批准与执行步骤

1. 所有者批准上表版本（或给出替代版本）。
2. 修改 `.config`：`CONFIG_PKG_<NAME>_VER="<tag>"`，并关闭对应的 `CONFIG_PKG_USING_<NAME>_LATEST_VERSION`。
3. 更新 `tools/rt_packages.lock.json`：填 `pinned_version`、`pinned_commit`、`status: RESOLVED`、`resolved_by`、`resolved_at_utc`。
4. 执行 `pkgs --update`，记录每个包的实际 commit 与文件树 SHA256。
5. 运行 `python tools/check_rt_packages_lock.py`（必须 PASS）与 `python tests/guidance/check_target_build.py`（必须不再 BLOCKED）。
6. 把安装日志、`packages/` 文件树哈希与链接产物写入证据包。

在第 4、5 步完成前，本文只能作为候选，`check_rt_packages_lock.py` 必须继续 FAIL。

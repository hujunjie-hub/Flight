# Flight 本地 Agent 操作指南

版本：1.0  
适用：Flight 飞控固件仓库的本地 Agent（Codex、IDE Agent 或其他可执行终端的编码 Agent）。

## 1. 工作模式

采用与 Flight_Matlab 相同的闭环方式：

- 用户：拉取代码、启动本地 Agent、提供必要的硬件/环境信息；查看 Agent 报告并决定是否授权 push。
- 本地 Agent：扫描源码、复现问题、修改代码、增加测试、执行本地验证、登记 GitHub Issue 草稿/状态建议、生成证据包。
- ChatGPT：审查架构边界、接口契约、数学推导、测试覆盖和证据；提出问题和下一阶段工作；复核 Agent 交付。

目标是把耗时的源码扫描、编译、测试和调试放在本地完成。不要等待 ChatGPT 通过 GitHub API 逐个文件修改。

## 2. 开始前必须读取

每次新任务都必须读取，不得只看本指南：

1. `AGENTS.md`
2. `spec/00_工程基线与唯一权威规范.md`
3. `spec/09_需求验证实施与发布管理.md`
4. `spec/Flight-验收与结果提交.md`
5. `spec/quality/README.md`
6. 当前任务对应的架构/接口/测试矩阵/历史审查记录。

M5-06 还必须读取：

- `tests/guidance/ADVANCED_PLANNER_VALIDATION_MATRIX.md`
- `spec/history/implementation/guidance/M5_06_2_VALIDATION_MATRIX.md`
- `spec/history/implementation/guidance/M5_06_4_ADVERSARIAL_TESTS.md`
- `spec/history/implementation/guidance/M5_06_5_INDEPENDENT_MATH_REVIEW.md`
- `middleware/guidance/trajectory_planning/m5_trajectory_safety.c`
- `middleware/guidance/trajectory_planning/m5_trajectory_safety.h`
- `tests/guidance/test_m5_main.c`
- `tests/guidance/verify_norm_bounds.py`
- `tests/guidance/verify_dynamic_prediction.py`

冲突时按以下顺序处理：安全/接口规范（spec/00–09）> 当前阶段验收矩阵 > 测试实现 > 代码注释 > Agent 自己的推断。发现规范互相矛盾时停止相关修改，先提交冲突报告。

## 3. 本地拉取代码

### 3.1 首次克隆（Windows PowerShell）

```powershell
git clone https://github.com/hujunjie-hub/Flight.git
cd Flight
git fetch origin --prune
git switch --track origin/opt/stage-02-m5-06-safety-validator
git status --short --branch
git rev-parse HEAD
```

### 3.2 已有本地仓库

```powershell
git status --short --branch
git fetch origin --prune
git switch opt/stage-02-m5-06-safety-validator
git pull --ff-only origin opt/stage-02-m5-06-safety-validator
git rev-parse HEAD
```

如果本地存在未提交改动，先报告文件清单和 diff 摘要；不得自动 reset、clean、stash、覆盖用户文件。若远端分支已变更，先比较提交，不得使用强制覆盖。

本轮基线分支为 `opt/stage-02-m5-06-safety-validator`。截至 2026-11-13，PR #2 为 CLOSED / UNMERGED，目标分支为 `opt/stage-01-host-validation`；本轮只更新 stage-02，不重开或合并该 PR。若后续 PR/分支变化，以 GitHub 当前状态为准，不得硬编码旧 SHA。

### 3.3 Windows 换行修复（每次新检出必须确认）

Windows Git 默认 `core.autocrlf=true`，会把跟踪的 `.sh` 检出为 CRLF。bash 把 CR 当作选项词的一部分，所有测试脚本会在第一行 `set -euo pipefail` 处立即失败，看起来像测试框架损坏：

```text
tests/guidance/run_host_tests.sh: line 3: set: pipefail
: invalid option name
```

这不是 Host 测试失败，也不是算法失败。仓库已用 `.gitattributes` 固定 `eol=lf`；旧检出用下面这一条命令就地修复，它只把这些 `*.sh` 的 CRLF 改成 LF，不执行 `git checkout/reset/clean/stash`，不会覆盖其他未提交改动：

```powershell
python tools/check_line_endings.py --fix
python tools/check_line_endings.py
```

`python tools/check_line_endings.py` 必须输出 `PASS`；CI 的 `Source layout and time-service tests` job 执行同一检查。若该检查失败，先修复换行，再运行任何 Host/Sanitizer/ARM 命令，否则得到的失败没有诊断价值。

不要用 `git checkout -- .` 修复换行：它会覆盖本地未提交的修改。旧的 `core.autocrlf=true` 全局设置可以保留，`.gitattributes` 的 `eol=lf` 优先。


## 4. Windows 本地环境

推荐从 Windows PowerShell 7 启动 Agent，使用 WSL2 Ubuntu-26.04 执行与 GitHub Actions 一致的 GCC Host/Sanitizer/ARM Shell 测试。

必须检查：

```powershell
$PSVersionTable.PSVersion
git --version
python --version
wsl.exe --status
wsl.exe -l -q
wsl.exe -d Ubuntu-26.04 -- bash -lc "gcc --version && g++ --version && python3 --version"
wsl.exe -d Ubuntu-26.04 -- bash -lc "arm-none-eabi-gcc --version"
```

工具要求：
- Windows：Git、Python 3、PowerShell 7（`pwsh`）。
- WSL：Ubuntu-26.04、GCC/G++、Python 3、GNU Arm Embedded Toolchain、make 和基础构建工具。
- Sanitizer：WSL GCC 支持 AddressSanitizer 和 UndefinedBehaviorSanitizer。
- WSL 发行版名称若不同，先确认环境，再按实际名称运行；不得把 Windows MSVC 测试结果冒充 GCC/CI 结果。

## 5. 一键回归和专项命令

### 5.1 首选完整本地回归

从 Flight 仓库根目录运行：

```powershell
python tests/guidance/run_validation.py
```

脚本会运行 M4 Host、M5 Host、M4/M5 integration、M5 ASan/UBSan、integration ASan/UBSan、ARM 检查、spec audit、内存预算、ABI 和目标构建检查。原始日志和汇总写入：

`build/m4_m5_implementation/evidence/`

脚本退出码非零时不得继续标记 PASS。Agent 必须打开失败日志，定位首个真实失败，不得只读取控制台最后一行。

### 5.2 复现 GitHub Actions 的关键命令

以下命令从 PowerShell 在仓库根目录运行，WSL 会继承当前目录映射：

```powershell
wsl.exe -d Ubuntu-26.04 -- bash tests/flight_management/run_host_tests.sh
wsl.exe -d Ubuntu-26.04 -- bash tests/guidance/run_host_tests.sh
wsl.exe -d Ubuntu-26.04 -- bash tests/guidance/run_integration_tests.sh
wsl.exe -d Ubuntu-26.04 -- bash tests/guidance/arm_build_check.sh
wsl.exe -d Ubuntu-26.04 -- bash tests/control/run_host_tests.sh
wsl.exe -d Ubuntu-26.04 -- bash tests/actuation/run_host_tests.sh
wsl.exe -d Ubuntu-26.04 -- bash tests/simulation/run_sil.sh
wsl.exe -d Ubuntu-26.04 -- python3 tests/algorithms/run_tests.py --target host
wsl.exe -d Ubuntu-26.04 -- python3 tests/algorithms/run_tests.py --target arm
```

M5 Sanitizer：

```powershell
wsl.exe -d Ubuntu-26.04 -- bash -lc 'EXTRA_CFLAGS="-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer" bash tests/guidance/run_host_tests.sh'
wsl.exe -d Ubuntu-26.04 -- bash -lc 'EXTRA_CFLAGS="-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer" bash tests/guidance/run_integration_tests.sh'
```

M6/M7 Sanitizer：

```powershell
wsl.exe -d Ubuntu-26.04 -- bash -lc 'EXTRA_CFLAGS="-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer" bash tests/control/run_host_tests.sh'
wsl.exe -d Ubuntu-26.04 -- bash -lc 'EXTRA_CFLAGS="-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer" bash tests/actuation/run_host_tests.sh'
```

`tests/simulation/run_sil.sh` 使用真实 runtime task owner 验证 M4→M7 ground-stop 软件链路；它是 Host/SIL 证据，不是 H723 实机或飞行验收。

注意：`tests/guidance/arm_build_check.sh` 会将 M5-06 Host/SIL-only 验证器编译为 Cortex-M7 object，仅用于编译器/架构兼容性检查，不链接进产品。必须与 MPC RAM 检查、生产固件链接、WCET/RTA、目标板实测分开报告。

### 5.3 基础静态检查

```powershell
python tools/check_layout.py
python spec/tools/audit_spec.py --round 3 --label local-agent
python tests/abi/abi_report.py
python tests/guidance/check_memory_budget.py
python tests/guidance/check_target_build.py
```

## 6. 本地 Agent 的修改流程

1. **基线确认**：记录分支、HEAD、工作区状态、Profile、相关配置、编译器版本。
2. **源码定位**：先从调用链、接口契约和测试入口入手，不得先大范围重构。
3. **问题复现**：使用原始失败命令重跑，记录首个失败用例、完整日志、测试 ID、随机 seed 和输入。
4. **根因分析**：区分测试夹具错误、测试断言错误、编译器/环境问题、接口问题、生产算法问题。不能把测试预期不成立直接当作算法 bug。
5. **先补测试**：新增最小复现用例；随机测试固定 seed；保存失败输入和缩减后的最小反例。
6. **最小修复**：只修改本任务范围；不顺手重构其他模块；不降低安全断言和预算。
7. **分层回归**：先运行单项，再运行 M5 Host、M4/M5 integration、Sanitizer、Cortex-M7 compile，最后跑完整回归。
8. **文档同步**：更新验证矩阵、问题记录、变更摘要和证据清单。
9. **本地提交**：仅在工作区干净、测试结果已记录时 commit；提交信息使用 `fix(m5-06): ...`、`test(m5-06): ...` 或 `docs(validation): ...`。
10. **交接**：运行归档脚本并输出本指南第 8 节的报告。

禁止：
- 为了让测试通过而删除用例、改宽容差、降低预算、忽略 UNKNOWN/STALE/INVALID。
- 使用未记录的随机 seed；随机测试失败后不保存反例。
- 将采样 oracle 写成形式化连续证明。
- 在数学审查未批准前将 M5-06 标记完成。
- 把 Host/SIL 或 Cortex-M7 compile 说成 H723 实机验证。

## 7. Issue 登记要求

发现问题时必须按 `spec/quality/ISSUE_WORKFLOW.md` 登记到 GitHub Issues。若 Agent 没有 GitHub 写权限，先生成完整 Issue Markdown 草稿，放入本次 evidence archive，并在交接报告中明确标记 `ISSUE_DRAFT_NOT_PUBLISHED`。

每个问题至少包含：
- Issue 编号/标题、严重级别、子系统、当前状态；
- 发现 commit、基线 commit、RUN_ID、测试 ID、命令；
- 复现步骤、预期/实际、失败次数/总次数；
- 随机 seed、失败输入、最小反例（适用时）；
- 原始日志路径和 SHA256；
- 根因、修复 commit、回归用例、复测 commit；
- 安全影响、是否 fail-closed、阻塞下一阶段的条件。

## 8. 交付给 ChatGPT 的固定内容

每轮结束，Agent 必须给出：

```text
PROJECT: Flight
TASK / MILESTONE:
BRANCH:
BASE:
HEAD:
WORKTREE: CLEAN | DIRTY
PROFILE:
CHANGED_FILES:
ISSUES: #... (status/severity)
REPRODUCTION: PASS | FAIL | NOT_RUN
TESTS:
  - test_id:
    command:
    checks:
    failures:
    exit_code:
    log:
    seed:
CI:
  - run_url:
    commit:
    conclusion:
MATH_REVIEW: APPROVED | OPEN | BLOCKED | NOT_RUN
TARGET_COMPILE: PASS | FAIL | NOT_RUN
H723_RUNTIME / WCET / HIL / FLIGHT: NOT_RUN unless actual evidence exists
ARCHIVE:
  - path:
  - sha256:
BLOCKERS:
NEXT_ACTION:
```

必须同时提供 `RUN_ID`、ZIP 文件、ZIP SHA256、HEAD SHA。若已 push，提供 PR 链接和远端 HEAD；未 push 则明确说明。

## 9. Git 操作边界

- Agent 默认只在本地当前功能分支修改和 commit。
- 只有用户明确说“推送”时才 push，且只允许推送到指定功能分支。
- 永远不推送 `main`，不合并 PR，不关闭问题，不把 PR 从 Draft 改为 Ready。
- 不使用 `git push --force`、`git reset --hard`、`git clean -fd`。
- 如需 cherry-pick/rebase 或处理冲突，先报告影响和方案，等待用户授权。

## 10. M5-06 特别安全边界

- 输入为 M6 实际消费的 anchor + 24 个 preview intervals；不得只验证 M5-05 内部候选系数。
- 固定内存、固定栈、固定 DFS 深度和 work/cell-check 预算。
- 静态占用、动态障碍相对运动、加速度界、不确定度、预测误差、围栏、导航 epoch/origin/frame/有效位均须纳入。
- 任何未证明净空、预算/容量耗尽、未知/过期/无效输入均拒绝。
- 当前 S06-080..084 的 CI 通过只说明对应自动化测试已执行；不等于 MR-001..MR-008 独立数学审查全部关闭。
- M5-07 只有在 M5-06 验收和独立数学审查明确批准后才可开始。

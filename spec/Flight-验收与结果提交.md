# Flight 验收与结果提交指南

版本：1.0  
适用：Flight 固件、Host/SIL、Cortex-M7 编译、M5-06 独立轨迹安全验证。

## 1. 目的与权威来源

本指南定义本地执行、验收、问题闭环和证据交付。详细设计约束仍以 `spec/00`～`spec/09` 为准，不在本文件复制架构规范。

| 内容 | 唯一/权威文件 |
|---|---|
| 工程基线和安全边界 | `spec/00_工程基线与唯一权威规范.md` |
| 模块和接口契约 | `spec/01`～`spec/08` |
| 正式需求验证状态 | `spec/09_需求验证实施与发布管理.md` |
| 本地 Agent 操作 | `spec/AGENT-操作指南.md` |
| 自动化测试入口和命令清单 | `spec/validation/flight_acceptance_manifest.json` |
| M5 高级规划器验证细节 | `tests/guidance/ADVANCED_PLANNER_VALIDATION_MATRIX.md` |
| M5-06 数学与对抗性验证记录 | `spec/history/implementation/guidance/M5_06_*.md` |
| 问题状态 | GitHub Issues |
| 本地测试证据 | `work/validation/archives/<RUN_ID>/` |

不要维护第二份手工问题总表或第二份验收状态表。GitHub Issues 是问题状态唯一权威来源；spec/09 是需求/验收状态唯一权威来源。

## 2. 验收基本原则

1. 每次验证必须绑定精确 Git commit SHA、分支、Profile、配置、工具链版本和命令。
2. 测试用例必须实际执行，测试数量必须大于零；仅编译成功、测试程序启动成功或测试文件存在都不算测试通过。
3. 测试 PASS 要求退出码为 0、失败数为 0、实际检查数大于 0，且原始日志可复核。
4. 失败结果必须保留。修复后重新运行原失败测试和完整回归，不得删除失败日志或覆盖历史证据。
5. 随机/属性测试必须记录测试 ID、seed、生成器版本、case 数、失败输入和最小反例。
6. CI 结果只对其实际执行的 commit 有效。新 commit 必须重新检查对应 CI；不能把旧 SHA 的 PASS 自动继承到新 SHA。
7. `NOT_RUN`、`IN_PROGRESS`、`PASS`、`FAIL`、`WAIVED` 是唯一验证状态。没有证据一律 NOT_RUN；被阻塞时另填 blocked_reason。
8. Host、SIL、ARM compile、目标板、HIL、实际飞行分别记录，不得相互替代。
9. WAIVED 必须有明确批准人、理由、风险接受和期限，Agent 不得自行豁免。

## 3. 本地验证环境与执行顺序

推荐 Windows PowerShell 7 + WSL2 Ubuntu-26.04。Host、Sanitizer 和 Cortex-M7 shell 编译使用与 CI 一致的 GCC 工具链。

### 3.1 快速完整回归

从仓库根目录执行：

```powershell
python tests/guidance/run_validation.py
```

日志和结构化报告写入：

`build/m4_m5_implementation/evidence/`

此入口覆盖 M4 Host、M5 Host、M4/M5 integration、M5 ASan/UBSan、integration ASan/UBSan、PowerShell ARM 检查、spec audit、内存预算、ABI 和目标构建检查。当前 `tests/guidance/arm_build_check.ps1` 主要编译生产源；M5-06 Host-only 验证器的 Cortex-M7 object 编译证据必须额外执行 §3.2 的 `bash tests/guidance/arm_build_check.sh`，不能只凭一键脚本的 PowerShell ARM 步骤认定完成。

### 3.2 CI 对齐的关键回归

如果需要逐项复现 GitHub Actions，运行：

```powershell
wsl.exe -d Ubuntu-26.04 -- bash tests/flight_management/run_host_tests.sh
wsl.exe -d Ubuntu-26.04 -- bash tests/guidance/run_host_tests.sh
wsl.exe -d Ubuntu-26.04 -- bash tests/guidance/run_integration_tests.sh
wsl.exe -d Ubuntu-26.04 -- python3 tests/algorithms/run_tests.py --target host
wsl.exe -d Ubuntu-26.04 -- python3 tests/algorithms/run_tests.py --target arm
wsl.exe -d Ubuntu-26.04 -- bash tests/guidance/arm_build_check.sh
```

Sanitizer：

```powershell
wsl.exe -d Ubuntu-26.04 -- bash -lc 'EXTRA_CFLAGS="-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer" bash tests/guidance/run_host_tests.sh'
wsl.exe -d Ubuntu-26.04 -- bash -lc 'EXTRA_CFLAGS="-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer" bash tests/guidance/run_integration_tests.sh'
```

`tests/guidance/arm_build_check.sh` 将 M5-06 Host/SIL-only 验证器编译为 Cortex-M7 object，验证目标编译器兼容性；该 object 不进入 H723 产品链接，不构成目标运行、WCET、RTA、实机 RAM/stack 或飞行证据。

### 3.3 基础静态检查

```powershell
python tools/check_layout.py
python tools/check_line_endings.py
python spec/tools/audit_spec.py --round 3 --label flight-local
python tests/abi/abi_report.py
python tests/guidance/check_memory_budget.py
python tests/guidance/check_target_build.py
```

`python tools/check_line_endings.py` 必须在所有 Host/Sanitizer 命令之前通过；CRLF 版的 `*.sh` 会让 bash 在 `set -euo pipefail` 处失败，并把环境问题伪装成测试失败。`check_target_build.py` 在缺少 RT-Thread `packages/` 树时返回退出码 2（BLOCKED），不得记为 PASS 或 FAIL。

执行顺序建议：源码/文档检查 → 单元测试 → M4/M5 集成 → Sanitizer → ARM 编译/资源静态检查 → 目标链接/ABI → 证据归档。

## 4. M5-06 专项验收清单

M5-06 是独立安全验证器。验证输入必须是下游实际消费的 anchor + 24 个 preview intervals，而不是仅验证 M5-05 的内部候选系数。

### 4.1 输入与契约

- [ ] 恰好 24 个连续预览区间，offset 按接口契约严格递增，时间跨度和 reference lease 有效。
- [ ] candidate、navigation、map、directive、geofence 的 frame、origin、state/time epoch、revision、时间戳一致。
- [ ] 导航有效位完整，位置/速度/加速度和不确定度均有限且处于有效范围。
- [ ] map、dynamic track、geofence 的状态、时间、来源和有效期均可验证。
- [ ] 缺失、过期、错 frame/origin/epoch、重复身份或非法元数据必须 fail-closed。

### 4.2 连续几何与动态障碍物

- [ ] 连续检查 24 个消费区间，不能只检查离散预览点。
- [ ] 静态障碍物按车辆包络和安全裕度膨胀；区间内部、voxel 面/棱/角接触均覆盖。
- [ ] 动态障碍物采用相对运动预测，覆盖恒速、加速度上界、速度/位置不确定度和 response delay。
- [ ] 动态预测时间换算、位置乘加和 float 转换误差由 outward interval 包络；溢出/非法区间拒绝。
- [ ] 相邻 20ms preview seam、精确碰撞/相切、内外 1 ULP 和自适应细分边界均有回归。
- [ ] 对采样 oracle 的证明边界明确说明；采样不能替代连续数学证明。

### 4.3 自适应细分和资源预算

- [ ] 速度、加速度上界覆盖完整子区间，区间净空只有被严格证明后才可 PASS。
- [ ] 最大 DFS 深度为 5，work-unit 上限为 256，cell-check 上限为 8192，工作栈固定且不超过 512 bytes（以当前契约为准）。
- [ ] 工作预算、栈容量、cell-check 或数值范围耗尽时明确拒绝。
- [ ] 最大深度仍无法证明净空时返回 clearance-unproven/对应拒绝状态。
- [ ] 不存在 heap 分配、无界循环或输入相关的无界工作量。

### 4.4 当前独立 oracle 与固定 seed

| 测试 ID | 内容 | 固定 seed / 规模 | 验收重点 |
|---|---|---|---|
| S06-080 | binary32 norm upper、binary64 norm lower 的 Fraction 精确平方范数 oracle | `0x6D350680`；每条路径 10,007 cases | upper 不低估、lower 不高估；包含次正规数和极端尺度 |
| S06-081 | 动态相对运动连续最小距离差分 | `0x6D350681` / `0xD1FF0610`；恒速和恒加速度各 64 cases | 恒速解析最近点；恒加速度候选点解析 oracle。long double oracle 不是形式化区间证明 |
| S06-082 | 动态精确相切、内外 1 ULP、20ms seam | deterministic | 相切/内侧拒绝；外侧允许保守拒绝但不得错误 PASS |
| S06-083 | 自适应细分位置精确点及 ±1 float ULP | deterministic | collision、fail-closed，并确认 work_units 实际进入细分 |
| S06-084 | 动态预测时间和位置误差 Fraction oracle | `0x6D350684`；10,007 cases | 精确时间/位置落入 outward prediction envelope；大 epoch 不变性 |

每次运行应从原始日志提取实际 cases、checks、failures、seed 和退出码，不得只抄本表中的设计规模。

### 4.5 安全边界

- M5-05 只输出 candidate。
- M5-06 PASS 只是后续安全门禁的输入，不是 Permit，不是执行器授权。
- M5-07 尚未完成前，M5-06 不能被描述为已接入 M6 的不可绕过门禁。
- Host/SIL 动态障碍物地图为合成/Host 数据，不证明传感器感知性能。
- H723 当前 `OBSTACLE_SOURCE_MODE=NONE`，不得宣称在线避障已放行。
- M5-06 独立数学审查 MR-001..MR-008 未明确 APPROVED 前，不进入 M5-07。

## 5. PASS 的证据要求

每一个 PASS 项至少具备：

| 字段 | 内容 |
|---|---|
| requirement/test ID | 对应 spec/09 和 manifest 中的 ID |
| source baseline | commit SHA、分支、是否有未提交改动 |
| profile/config | Product Profile、参数/配置 hash |
| environment | OS、WSL distro、编译器/工具链版本、Python 版本 |
| command | 完整可复制命令 |
| execution | 开始/结束时间、退出码、实际 checks/cases、failures |
| randomness | seed、生成器版本、失败向量/最小反例 |
| artifacts | 原始日志、报告、map/stack/size、SHA256 |
| reviewer | 执行者与独立复核者 |
| scope | Host/SIL/ARM compile/target/HIL/flight 中的具体层级 |
| status | NOT_RUN / IN_PROGRESS / PASS / FAIL / WAIVED |

若只有截图、手工描述或“脚本执行完成”，不得标记 PASS。测试日志应优先提供原始文本，截图只作为辅助。

## 6. 问题登记与修复闭环

发现任何失败、数值风险、需求歧义、环境阻塞或证据缺失，按 `spec/quality/ISSUE_WORKFLOW.md` 建立 GitHub Issue。

- CI 编译错误与算法失败分开登记。
- 测试夹具/断言问题与生产代码问题分开登记。
- 一个 Issue 对应一个可独立复现和关闭的根因；多个独立根因不得塞进一个 Issue。
- 关联测试 ID、commit SHA、RUN_ID 和证据包。
- 修复后必须添加回归用例并复跑原失败测试及完整回归。
- 状态为 FIXED_WAITING_RETEST 不等于 VERIFIED；只有独立复测通过才可关闭。

## 7. 本地证据归档与提交

完整测试完成或失败后均可归档：

```powershell
pwsh -NoProfile -File .\tools\archive_validation_run.ps1
```

可选指定 CI 运行地址：

```powershell
pwsh -NoProfile -File .\tools\archive_validation_run.ps1 -CiRunUrl "https://github.com/hujunjie-hub/Flight/actions/runs/<RUN_ID>"
```

输出：
- 证据目录：`work/validation/archives/<RUN_ID>/`
- ZIP：`work/validation/archives/<RUN_ID>.zip`
- SHA256：终端输出，并写入归档清单。

归档包含：Git 分支/HEAD/工作区状态、测试日志、结构化验证报告、M5/ARM 构建证据、manifest、Git diff 和未跟踪文件（排除常见密钥文件）。发送前仍须检查 ZIP 内容，不得提交 token、私钥、密码或个人敏感文件。

交付给 ChatGPT：
1. ZIP 文件。
2. ZIP SHA256。
3. HEAD SHA 和分支。
4. GitHub Actions run URL（如有）。
5. 当前打开的 Issue 编号。
6. Agent 的交接摘要（按 `spec/AGENT-操作指南.md` 模板）。

不要只发“测试通过”或几张截图。

## 8. 最近一次完整 CI 基线（2026-11-10）

Host validation #542 对 commit `dafe47b8b17e7f32f676cb258eaa7a407a1dfb90` 的全部 jobs success：

| 项目 | 实际结果 |
|---|---:|
| M4 Host | 927 checks / 0 failures |
| M5 Host | 269,207 checks / 0 failures |
| M4/M5 Integration | 1,000 checks / 0 failures |
| M5 ASan/UBSan | 269,207 checks / 0 failures |
| Integration ASan/UBSan | 1,000 checks / 0 failures |
| S06-080 Fraction oracle | binary32 upper 10,007 + binary64 lower 10,007 cases，seed `0x6D350680` |
| S06-081 动态连续距离 oracle | 恒速 64 + 恒加速度 64 cases；analytic long-double candidate set |
| S06-082 / S06-083 | 精确相切、1 ULP、preview seam、自适应细分边界全部 PASS |
| S06-084 Fraction prediction oracle | 10,007 cases，seed `0x6D350684` |
| Cortex-M7 compile | `M5_CORTEX_M7_COMPILE_OK` |
| MPC 静态资源报告 | RAM 29,252 bytes；worker stack 8,192 bytes；最大静态栈帧 2,024 bytes |

CI：[Host validation #542](https://github.com/hujunjie-hub/Flight/actions/runs/42746417706)。

注意：该证据对列出的 commit 有效。S06-081 的 long-double 解析候选 oracle 只针对当前 1D 恒加速度测试模型，不是一般 3D 区间证明。该结果也不代表 H723 目标运行、WCET/RTA、实机 RAM/stack watermark、HIL 或飞行通过。M5-06 整体仍受独立数学审查和后续 M5-07 门禁约束；后续任何新 commit 均须重新核对 CI。

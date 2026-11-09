# KF-GINS 修改清单（GPL-3.0 要求）

本包在 `middleware/model/ins/kf_gins/upstream/` 下保存锁定 commit
`8291a93e49de513fe9d21f819500d39082ded611` 的**原样**源码（逐文件 SHA-256 见
`UPSTREAM_LOCK.json`，与 `spec/validation/upstream_sources.json` 全部一致，未作任何改写）。

固件侧实现位于 `middleware/model/ins/kf_gins/trimmed_core/`，是上游源码的**受控修改版**（GPL-3.0 §5 要求标注修改）。
以下修改按上游文件逐项列出，`[UPSTREAM]` 表示该函数/公式逐行移植未改变数学，`[QUAD]` 表示本项目新增。

## 0. 未修改的部分（可逐行对照）

| 上游 | 移植位置 | 说明 |
|---|---|---|
| `insmech.cpp::velUpdate/posUpdate/attUpdate/insMech` | `trimmed_core/src/quad_core.cpp`（同名匿名函数） | 双子样划桨/圆锥补偿、速度/位置/姿态更新公式、`midpos` 二次地球参数重算、四元数乘法次序均未改变；仅把 `Eigen::Vector3d/Quaterniond` 换成固定维数类型 |
| `gi_engine.cpp::insPropagation` 的 F 矩阵分块 | `QuadCore::buildPhi` | 14 个 F 分块公式逐项一致；`Phi = I + F·dt` 相同 |
| `gi_engine.cpp::EKFUpdate` 的 Joseph 更新思想 | `QuadCore::applyUpdate` | 保留 Joseph 形式，等价改写为 `P - KHP - (KHP)ᵀ + K S Kᵀ`（S = HPHᵀ+R），避免额外 22×22 临时矩阵 |
| `gi_engine.cpp::stateFeedback` 的误差反馈符号 | `kfg::retract`（`quad_types.hpp`） | 位置/速度减误差，姿态左乘 `Exp(δφ)`，零偏/比例因子加误差，`δx` 归零 |
| 状态/噪声索引 | `quad_types.hpp` 的 `StateIndex`/`NoiseIndex` | 前 21 维状态、18 维噪声顺序与上游 `gi_engine.h` 完全一致 |
| `common/earth.h`、`common/rotation.h` | `quad_earth.hpp` | 公式未变；去掉 Eigen 依赖和 `matrix2euler` 的控制台告警 |

## 1. 逐项修改

| ID | 位置 | 修改 | 理由 / 规范依据 |
|---|---|---|---|
| M01 | `trimmed_core/include/kfgins/quad_types.hpp`、`trimmed_core/src/quad_core.cpp` | 误差状态 21→22，过程噪声 18→19：新增 `δb_h[21]`、`w_bh[18]`；`F22=diag(F21,0)`、`G22=diag(G21,1)`、`Qc19=diag(Qc18,q_bh)`、`Φ22=I+F22Δt`、`Qd22=(Φ22Q0Φ22ᵀ+Q0)/2` | 05 §2.2：气压高度偏置状态；新增项只追加，不改前 21 维顺序 |
| M02 | `trimmed_core/include/kfgins/quad_math.hpp` | 以固定维数 `Mat<R,C>`/`Vec3`/`Mat22` 取代 `Eigen::MatrixXd`；矩阵乘法、转置乘、对称化、有限值检查全部静态循环 | 05 §5.4/§6.2：禁止动态 resize、heap；H723 目标不接受 Eigen 动态路径 |
| M03 | `trimmed_core/src/quad_core.cpp::applyUpdate` | `S.inverse()` 改为 Cholesky 分解 + 回代求解；`K` 逐列求解；`S` 不可分解时拒绝更新 | 05 §5.3/§5.4：SPD 求解、不得显式求逆 |
| M04 | `gi_engine.cpp::checkCov`、`print_options`、`std::exit` | 删除控制台输出与进程退出；改为返回状态码 + `QuadDiagnostics` 计数；协方差有限值/对角正/正定检查失败时回滚 | 05 §1.2.1/§5.4：实时任务不得退出进程或打印；失败原子性 |
| M05 | `trimmed_core/src/quad_models.cpp::gnssVelocityPrediction/Jacobian` | 新增 GNSS 天线速度观测：`v_ant=v+C(ω×l)-ω_in×(Cl)`，解析 Jacobian `H_p/H_v/H_φ/H_bg/H_sg` | 05 §1.2、§3.2、§7：上游只有位置更新，速度更新为本项目新增，不得声称上游已有 |
| M06 | `trimmed_core/src/quad_models.cpp::baroPrediction/baroJacobian`、`QuadCore::updateBaro` | 新增气压相对高度观测：`h_b=(h-h_origin)+b_h`，`H[0,2]=-1`、`H[0,21]=-1`，`dz=h_b-z` | 05 §2.3、§3.3：22 维扩展的观测；符号与 innovation 约定一致 |
| M07 | `QuadCore::applyUpdate` | 新增 NIS 门控：`dz^T S^{-1} dz`，dof1=6.6348966、dof3=11.3448667（99%） | 05 §5.3：固定置信水平，超限拒绝并计数，不破坏当前状态 |
| M08 | `QuadCore::applyUpdate` | 更新前备份协方差，更新后强制对称化并验证有限/正定，失败即恢复；`dx` 反馈后才提交名义状态 | 05 §5.4、§6.1.1 失败原子性 |
| M09 | `QuadCore::propagateCovariance` | `Q0=G Qc GᵀΔt` 按 6 个 3×3 块 + 1 个标量块累加，避免 22×19/22×22 大矩阵；数学上与上游 `G*Qc_*G.transpose()*dt` 等价 | 内存预算：22×22 double 矩阵约 3.8 KiB，工作区须落在 14 KiB arena 内 |
| M10 | `trimmed_core/src/quad_core.cpp` 全部 | 命名空间 `kfg`、无异常/RTTI/iostream、无 `new/delete`；`Status` 返回码取代 assert/exit | 05 §6.2：C++14、禁用异常/RTTI、无动态分配 |
| M11 | `trimmed_core/include/kfgins/quad_earth.hpp` | `Rotation::matrix2euler` 的奇异告警删除（保持“roll 置 0”的上游行为，不打印） | 固件不得输出控制台 |
| M12 | 构建 | 不使用上游 `CMakeLists.txt`；由 `SConscript` 只编译 `trimmed_core/src/*.cpp`；桌面入口、`fileio/`、yaml-cpp、abseil 不进入固件 | 05 §6.2、00 §12.2、03 §0 |

## 2. 与上游的已知数值差异

- 固定的 21 维核心与上游原版在 400 个 200 Hz 历元、20 次 GNSS 位置更新后：
  位置 ≤ 3.7e-8 m、速度 ≤ 4.4e-8 m/s、姿态 ≤ 1.5e-9 rad、协方差 ≤ 8.3e-11
  （Host 实测，见 `build/validation/kf_gins_upstream/test_upstream_compare.log`）。
  残差来源为浮点重结合（不同的遍历顺序、四元数归一化路径、Cholesky 替代求逆）经滤波器反馈放大；
  真实算法改动会在 1e-2 量级暴露。
- 22 维扩展本身无法与上游直接对比（上游无该状态），改由 `test_core`、`test_jacobian` 与 09 的
  REQ-NAV-KFG-011 后续数据集验证覆盖。

## 3. 许可证与分发

- 上游 LICENSE 为 GPL-3.0（`LICENSE` 文件 SHA-256 见 `UPSTREAM_LOCK.json`），本包保留完整许可证文本、
  版权声明与来源；`trimmed_core/` 为衍生作品，同样按 GPL-3.0 分发。
- 本轮未确认最终分发/使用方式，按 05 §6.4 不预填“个人自用、无需审查”；正式交付前由 D 汇总 SBOM 并
  核对 GPL 义务。
- 上游 `ThirdParty/`（Eigen 3.3.9 MPL-2.0、yaml-cpp 0.7.0 MIT、abseil 20220623.1 Apache-2.0）**未导入**本仓库；
  Host 参考构建在使用时按各自许可证处理。

## 4. 2026-11-05 控制/导航 Port 接入扩展

- 新增磁航向 1 维观测，含主值角残差、解析姿态 Jacobian、既有 Joseph/NIS/SPD 守卫。
- bias/scale 四组相关时间可分别指定；全零扩展保持原单一相关时间回归行为。
- 协方差分解与 checkpoint 恢复复用固定 workspace，消除 22×22 临时矩阵对线程栈的占用。
- 未改写 upstream/；本次测试证据位于 build/validation/controller_navigation/。

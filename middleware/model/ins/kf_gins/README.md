# KF-GINS 后端（上游锁定 + 受控 22 维裁剪 + 项目适配）

本目录是 M3 的可替换估计器后端，作为一个整体绑定在 INS 模块内：上游算法、受控裁剪核心、
项目适配层（IMU 增量构造与私有 C 接口）、来源锁和复现工具同目录归档。公共
`estimator_*()` Port 归上级 `middleware/model/ins/`。依赖模型的生命周期、观测和回放编排在本目录。

## 1. 目录

```text
middleware/model/ins/kf_gins/
  LICENSE                 GPL-3.0 原文（与上游逐字节一致）
  UPSTREAM_LOCK.json      仓库/commit/tree/归档 SHA-256/逐文件 SHA-256/第三方清单
  MODIFICATIONS.md        GPL-3.0 §5 要求的修改清单（M01..M12）
  upstream/               锁定 commit 的原样源码（禁止在本目录内修改）
  trimmed_core/                   QUAD 22 维固定维数裁剪（衍生作品，GPL-3.0）
    include/kfgins/quad_math.hpp   固定维数矩阵/SPD 分解
    include/kfgins/quad_earth.hpp  WGS-84 地球模型/四元数（无 Eigen、无控制台）
    include/kfgins/quad_types.hpp  状态索引/配置/名义状态/误差重缩
    include/kfgins/quad_models.hpp 观测预测与解析 Jacobian
    include/kfgins/quad_core.hpp   22 维核心 API
    src/quad_core.cpp              机械编排/传播/更新/反馈
    src/quad_models.cpp            GNSS-P/GNSS-V/Baro 观测模型
  backend_adapter/               IMU 增量构造与插件私有 C ABI
  navigation_estimator/            模型生命周期、状态发布与仿真配置
  observation_replay/                观测门控、日志与延迟回放
  tools/gen_locks.py      重新生成哈希锁（离线）
  tools/                  Host 数据集复现与差分工具
  SConscript              固件源选择（默认关闭，需 INS_BACKEND_ENABLE）
```

公共 Port 入口位于 `../ins_interface.c`。模型初始化和状态发布位于 `navigation_estimator/kf_gins_ins.cpp`，观测门控和延迟回放分别位于 `observation_replay/ins_observation.cpp` 和 `observation_replay/ins_replay.cpp`。

## 2. 上游锁定

- 仓库：<https://github.com/i2Nav-WHU/KF-GINS>
- commit：`8291a93e49de513fe9d21f819500d39082ded611`（tree `6d255a9c…`，2026-06-10，`origin/main`）
- 归档：`git archive` tar.gz，56,254,907 B，SHA-256 `183a86c0…`
- 导入文件 10 个 + LICENSE，逐文件哈希与 `spec/validation/upstream_sources.json` **全部一致**
- 未导入：桌面入口 `src/kf_gins.cpp`、`src/fileio/`、`CMakeLists.txt`、`ThirdParty/`
- 许可证：GPL-3.0-or-later，保留原文与版权声明；修改清单见 `MODIFICATIONS.md`

重新校验：

```powershell
python middleware/model/ins/kf_gins/tools/gen_locks.py
```

该工具只刷新本目录的 `UPSTREAM_LOCK.json`，工程依赖索引由 `packages/deps.lock` 管理。

## 3. 使用边界

- `upstream/` 只用于参考/差分，不进入固件编译。
- `SConscript` 选择 `trimmed_core/` 与 `backend_adapter/` 的固件实现。模型层构建清单选择导航和回放源。使用 C++14 和静态存储。
- 矩阵实现为项目自带的固定维数层，**未采用 Eigen**。`spec/00 §12.2` 仅把 Eigen 5.0.1 列为候选；
  是否在 H723 引入 Eigen 属于 PLATFORM 决策，需要 C++14/ARM/数值/WCET 接纳与 CR（见 `packages/deps.lock`）。
- 上游 21 维核心必须与锁定原版保持可复现的数值等价；任何算法改动都要同步
  `MODIFICATIONS.md` 并重跑 Host 差分（`tests/estimator_backend/`）。

## 4. 验证状态（截至本交付）

| 项目 | 结果 | 证据 |
|---|---|---|
| 上游 10 文件哈希 vs 规范记录 | 一致（0 mismatch） | `UPSTREAM_LOCK.json` |
| 21 维核心 vs 上游原版 Host 差分（400 历元/20 次 GNSS） | 位置 3.7e-8 m、速度 4.3e-8 m/s、姿态 1.4e-9 rad | `build/validation/kf_gins_upstream/test_upstream_compare.log` |
| 解析 Jacobian 有限差分 | 位置 3.8e-5（大地坐标舍入下限）、速度 8.8e-8、Baro 2.0e-13 | `test_jacobian.log` |
| 22 维核心单测/NIS/失败原子性 | 2636 checks / 0 fail | `test_core.log` |
| 项目包内实测内存 | 14,112 B（≤14 KiB active arena 目标） | `test_port.log` |
| 原始官方数据集复现（REQ-NAV-KFG-001 上游黄金结果） | **NOT_RUN**（需上游 `dataset/` 与桌面构建） | 09 台账 |
| H723 目标编译/WCET/map | **NOT_RUN**（T191-A-TARGET） | 09 台账 |

本包**未**获得独立审查或 Gate C 会签；任务状态仍为 `READY_FOR_ASSIGNMENT`，验证状态 `NOT_RUN`。

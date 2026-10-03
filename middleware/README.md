# middleware — 传感器数据处理与组合导航中间件

本目录按数据流向分层组织: 传感器驱动 → 原始数据环形缓冲 → 校准 → KF-GINS
组合导航融合。各子目录只依赖下一层的头文件, 可独立替换。

## 目录结构

| 目录 | 职责 |
| --- | --- |
| `sensor/` | 传感器驱动 (ADIS16505/BMM350/BMP585/W25Q64) |
| `protocol/` | 纯协议解析 (um982_nmea: NMEA RMC/GGA/ZDA → GNSS PVT) |
| `data/` | 原始数据环形缓冲区层 (gnss_raw_data/gnss_data/mag_data/baro_data/imu_data) |
| `timebase/` | 时间同步基座: TIM2 @1MHz T_MCU 时基 + PA0 PPS 输入捕获 + UTC↔T_MCU 映射 |
| `calibration/` | 磁力计椭球校准 + 气压计基准校准 (参数经 param_calib 持久化到 W25Q64) |
| `param_calib/` | 飞控参数分区存储 (W25Q64): param_part 分区引擎 + calib/nav/sys 三参数域 |
| `gins/` | KF-GINS 桥接 (gins 解算线程 1kHz + ginsaux 消费线程) + C++ 对齐 new 堆适配 (aligned_new.cpp) |
| `KF-GINS/` | 上游组合导航算法内核 (C++, 21 状态 EKF, 最小嵌入) |
| `so3/` | SO(3) 姿态误差解算 (姿态控制误差 + 验收测试) |
| `control/` | 飞控控制律 (级联): `position_mpc` 外环位置 MPC (凝结 QP+SCA) + `attitude_so3` 内环 SO(3)/PID 姿态, 见根 README "飞行控制" 章 |

## 数据流架构

```
timebase/ (TIM2 @1MHz T_MCU + PA0 PPS 捕获): 全部事件打戳的统一时基
    PPS 滑窗 (窗口3, 四道门槛+最小二乘+残差剔除) → clock_map UTC↔T_MCU 映射

ADIS16505  1kHz DR中断 (EXTI ISR 捕获 T_event + 启 SPI DMA burst)          ┐
    → adis_dr 处理线程 (校验/解析) → 快照 → imu_data 环 → [imudata 采集线程]  │
      (单位换算+轴映射) → imu_data 结构环 ─────────────────────────────────→ │
                                                                             │
UM982 10Hz ─ USART2 460800 (DMA_RX) → [gnssrx 接收线程] 只搬字节              │
             └→ gnss_raw_data 字节环 (data/, buffer/head/tail/size)          │
                 └→ [gnssdata 解析线程] 组句 → um982_nmea (protocol/ 纯解析)  │
                     ├─ 整秒 UTC (定位有效) → timebase PPS 配对刷新滑窗        │ → KF-GINS EKF
                     └→ UTC→T_MCU 映射 → gnss_data 结构环 (T_event) ──────→ │   (gins 线程)
                                                                             │   统一时轴:
BMM350 100Hz ─ I2C4 → mag_data 采集线程 (µT, 入环前: 校准→轴映射→            │   样本 T_event(T_MCU)
              干扰检查→低通; 环内样本 raw(传感器系)/cal(体系FRD)+quality)     │   经首个有效 GNSS 样本
              ├→ [ginsaux 线程] cal → 磁航向观测 (干扰位降权) ─────────→      │   锚定换算 GPST,
              └→ [magcal 采集线程] raw 字段椭球拟合 → 参数                    │   >50ms 丢弃计数)
BMP585 100Hz ─ I2C2 → baro_data 环 (data/, Pa + 芯片温度)              │
              ├→ [ginsaux 线程] baro_calib 偏移 → 气压观测(Pa) ────────────→  │ (Pa→高度换算在引擎内)
              └→ [barocal 采集线程] 基准偏移 → 参数                          │

校准/导航参数: middleware/param_calib ↔ W25Q64 分区 (calib/nav/sys,
          上电加载; 追加式记录, 擦写不关中断不占用时基)

USART1 调试输出 (applications/out_*.c, 460800, 与 console 同口):
  gins 线程解算快照 ─→ [vofa 线程]  JustFloat 50Hz 二进制帧
                    └→ [gins_fused_data 线程] 带标记文本, 行尾 fused_data
  gnss_data 环 ─────→ [gnssout 线程] UM982 定位解文本 (与 gins 分抢样本!)
  mag_data 环 ──────→ [magout 线程] raw + cal(校准/轴映射/低通后), 行尾 mag_calib_data
  baro_data 环 ─────→ [barout 线程] raw+baro_calib 校正, 行尾 baro_calib_data
```

## 各链路说明

### UM982 (GNSS)

- USART2 460800 8N1 接收链路已接入: `data/gnss_data.c` 的 **gnssrx 接收线程**
  (只搬字节不解析, DMA_RX 空闲线批量指示) 把 USART2 原始字节镜像写入
  `gnss_raw_data` 字节环; 波特率/RX 环 (4KB, `GNSS_RAW_DATA_BUF_SIZE`)
  在链路初始化时配置 (512B 在 ~4KB/s NMEA 流下仅 128ms 余量, gnssrx
  偶发调度延迟被 DMA 套圈后未读区被硬件覆写, 读出"句子缝合+重复"流,
  校验失败率 ~50%; 2026-09-29 扩到 2KB, 后随 DMA 链重整扩到 4KB,
  套圈余量 ~1s)。
- `data/gnss_data.c` 的**解析线程** (gnssdata) 是字节环唯一常驻消费者:
  `wait` 等新字节 → `pop` 批量取 → `'\n'` 组句 → `um982_nmea_feed_line()`
  (协议层内部验校验和) → `update_cnt` 变化时 **UTC 双分流**:
  整秒语句 (定位有效) 调 `timebase_pps_pair()` 刷新 PPS 滑窗; 语句时标经
  `timebase_utc_to_mcu()` 换算 `T_event` (映射未就绪时置 0 并计数,
  `utc_sec/usec` 仍保真入环), 连同 `T_arrival` 组装 `gnss_sample` 入结构环。
- `gins` 线程每毫秒排空 `gnss_data_pop()` 取最新样本作位置观测;
  `T_event=0` 的样本跳过 (不用到达时刻顶替, 避免百 ms 级时戳误差污染 EKF);
  引擎观测时标 = `T_event` 经 T_MCU 锚点换算的 GPST。

### BMM350 / BMP585 (磁力计 / 气压计)

- `data/mag_data.c` / `data/baro_data.c` 各自的采集线程 100Hz 轮询传感器
  设备, 单位换算 (mGauss→µT / Pa) 后入环; `T_event` 在采样触发时刻打戳
  (INT 引脚 PF12/PE13 硬件已预留, 迁移 EXTI 事件源后改为 ISR 捕获)。
  **BMM350 2026-10-02 由 I2C1 (PB6/PB7, INT PB5) 重映射到
  I2C4 (SCL=PF14/SDA=PF15, INT PF12)** —— 驱动头注释/rtconfig.h/
  Kconfig/Flight.ioc 均已同步; 2026-09-29/10-01 修复记录中的
  "I2C1/BMM350" 为当时状态的史实, 未改。
- 磁链路的 量程守卫→校准→轴映射→干扰检查→低通 在 `data/mag_data.c` 采集线程
  **入环前**完成: 先做 **float→double 量程守卫** (`|body| < 1e6 µT`, 拒绝
  NaN/±inf/huge —— 驱动补偿链的除法在系数退化或 I2C 毛刺时产出非有限值,
  且一阶 EMA 会永久闩锁 inf−inf=NaN, 一个坏样本毒死全部后续输出; 坏样本
  置 quality 干扰位并复位 EMA 状态), 再在**传感器框架**内做
  `mag_calib_apply` 椭球硬/软磁校正 (**入参显式 float→double 转换后传入,
  不可 `(const double*)raw` 直接强转** —— 按 double 读 24 字节实际取的是
  12 字节 float 缓冲 + 相邻栈垃圾, 两个 float 位模式拼成的 double 恰为
  ~1e10 量级垃圾, 该缺陷曾使磁链路自编译起全程报废), 再轴映射到体系前右下
  (软磁矩阵与轴重排不可交换, 顺序不能调), 然后
  **干扰检查** (校准后体系模值应落在地磁 20~100µT 量级, 超限置 quality
  bit0; 位于低通之前 —— 低通会抹平瞬时尖峰), 最后一阶 EMA 轻量低通
  (`GINS_MAG_LPF_TAU_S`, 0 旁路); 环内样本同时携带 raw (传感器系, magcal
  拟合用) 与 cal (体系, 引擎用) 两份。`ginsaux` 线程消费时直接取 cal 字段
  作磁航向观测 (观测时标前移低通群延迟 τ; **quality 干扰位经
  `std_scale` 降权**而非丢弃, 见 gi_engine magUpdate); 气压计仍由 ginsaux
  消费时加 `baro_calib_apply` 基准偏移。
- 气压计入环与喂引擎的都是 Pa, 链路上**不做高度换算**: Pa→高度在
  `KF-GINS/src/kf-gins/gi_engine.cpp` 的 `GIEngine::baroUpdate()` 内完成
  (压差式测高 `h = h_ref + R·T/g·ln(p_ref/p)`, 首样本以当前 INS/GNSS 高度
  锚定参考点; GNSS 新鲜时按 `GINS_BARO_ANCHOR_TAU_S` 300s 慢速再锚定偏置
  跟踪天气漂移, 失锁期间偏置保持, 观测继续阻尼垂直通道)。
- 气压链路**无低通滤波**: 芯片 IIR 显式旁路 (`BMP585_IIR_COEFF=0`), 噪声
  抑制由 8x 压力过采样 + 引擎量测噪声 `GINS_BARO_STD_M` 承担; 引擎内的
  `barobias_` EMA 只跟踪锚定偏置, 不平滑高度信号本身。
- `calibration/mag_calib.c` / `baro_calib.c` 的采集线程只在维护校准时存在,
  从同一环形缓冲区取数 (`magcal start` / `barocal ref|refalt + start`)。

### ADIS16505 (IMU)

DR EXTI ISR 内一条 `timebase_now_us()` (TIM2 合成 64 位 T_MCU) 完成打戳
(cnt_event, 按 ping-pong 槽位随帧配对) + 启动 SPI DMA burst; 处理线程
校验解析后入快照, `imudata` 采集线程完成 单位换算+轴映射 并把 16bit
DATA_CNTR 扩展为 32bit 单调 `data_cnt` 后推入 imu_data 环; gins 线程排空
环形缓冲区, **dt 由 data_cnt 差分按名义 ODR 计算** (丢拍期 EKF 按实际间隔
积分, 不误当作均匀 1ms)。引擎时间轴 (自 GPS 纪元累计秒) 在对准完成时由
首个有效 GNSS 样本锚定 (T_MCU 锚点时刻 ↔ 语句 UTC → GPST), 之后所有观测
统一按 T_MCU 差分换算。

### KF-GINS 引擎的堆分配 (gins/aligned_new.cpp)

GIEngine 的固定尺寸 Eigen 成员带 alignas, 构造触发 C++17 对齐
`operator new(size, align_val_t)`; 工具链默认落到 newlib `_memalign_r`,
其簿记与 RT-Thread 小内存分配器块头不兼容, 首次构造引擎即
`rt_smem_free` 断言死循环 (gins 线程优先级 9 自旋, 全系统饿死)。
`aligned_new.cpp` 接管全部对齐 new/delete (rt_malloc 超配 + 回指针),
普通 new 走 `_malloc_r → rt_malloc` 不受影响。

## USART1 调试打印数据链路 (applications/out_*.c, 公共设施 app_out.c)

全部与 console 同口 (uart1, PA9/PA10, 460800 8N1), 纯写不复用配置。
行尾带来源标记, 便于在同口混合流里区分数据:

| 链路 (FinSH 命令) | 数据来源 | 频率 | 默认 | 行尾标记 |
| --- | --- | --- | --- | --- |
| `vofa [on\|off]` | gins_bridge 解算快照 | 50Hz | **off** | (JustFloat 二进制帧, 无文本) |
| `gins_fused_data [on\|off]` | gins_bridge 解算快照 | ready=1: 10Hz / ready=0: 1Hz | **off** | `fused_data` |
| `gnssout [on\|off]` | data/gnss_data 环 (UM982 原始定位解) | 10Hz | off | (无) |
| `magout [on\|off]` | data/mag_data 环 + calibration/mag_calib | 100Hz | off | `mag_calib_data` |
| `barout [on\|off]` | data/baro_data 环 + calibration/baro_calib | 100Hz | off | `baro_calib_data` |
| `imuout` | data/imu_data 环 | (IMUOUT_ENABLE=0, 未启用) | — | — |

**uart1 单写者约定 (2026-09-29 实测教训)**: RT-Thread 串口 INT-TX 路径
(`_serial_int_tx` 的逐字符 putc + completion 协议) **无线程保护** —— 两个
写者并发会打烂 TX 环 (堆损坏 → 随机线程 HardFault, 每次启动 3~6 分钟内
必现) 或丢 completion 唤醒 (解析线程停摆数分钟)。应用层数据链路虽持
`uart1_lock()` 互斥, 但 ulog/console 的 `rt_kprintf` 不经过它 —— 因此
**数据链路默认全部 off, 由 console 作唯一写者**; 需要 vofa/fused 输出时
用 FinSH 命令手动开启 (接受与 console 日志的残余竞态风险)。

`gins_fused_data` 行格式 (输入计数器按数据真实来源路径命名):

```
ready:1 time:<GPST> roll: pitch: yaw: vn: ve: vd: lat: lon: alt: \
imu_data:<n> gnss_data:<n> mag_calib_data:<n> baro_calib_data:<n> fused_data
```

- `imu_data`/`gnss_data`: middleware/data 原始观测计数 (无校准环节);
  `mag_calib_data`/`baro_calib_data`: middleware/calibration 校正后喂入
  引擎的观测计数。
- **样本分流注意**: `gnssdata` 解析线程推入 gnss_data 环的样本由 gins 桥接
  (优先级 9) 与 gnssout 调试线程共同消费, FIFO 每样本只能弹出一次 ——
  gnssout 开启期间会抢走大部分样本 (信号量即时唤醒 vs gins 1ms 轮询),
  **跑融合解算前须 `gnssout off`** 把观测完整交还 gins; `magout`/`barout`
  消费独立环形缓冲区, 与 ginsaux 分抢但影响仅限观测时戳稀疏化。
- CPU 预算: 1kHz EKF + 多路文本打印在 460800 下已接近饱和, 优先级低于
  打印链路的线程会被饿死 (曾发生: 打印线程优先级 16 从未被调度);
  新增打印链路时优先级不得低于 12, 且建议默认 off。

## 关键设计约定

1. **环形缓冲区为单消费方语义**: 字节/样本只能被 pop 一次, 每条链路只有
   一个常驻消费者 (gnssdata 解析线程 / ginsaux 线程); 校准采集是维护窗口
   内的第二消费者, 期间样本分流属预期行为 (gins 侧 stale 计数可观察)。
   要加原始流记录等新消费者时, 须在 push 侧做镜像分发。
2. **统一时间戳 T_event**: 四类样本环的 `T_event` 均为 **T_MCU 时基**
   (TIM2 @1MHz, middleware/timebase): ADIS 为 DR 沿 EXTI ISR 捕获, 磁/气压
   为采样触发时刻打戳 (EXTI 事件源迁移后同 ADIS), GNSS 为语句 UTC 经 PPS
   映射换算 (映射未就绪置 0, gins 跳过)。gins 桥接经首个有效 GNSS 样本
   锚点统一换算引擎 GPST, 过旧样本丢弃并计数 —— 磁/气压门限
   `GINS_AUX_MAX_AGE_S` (50ms, 本地采样打戳无传输延迟); **GNSS 单列
   `GINS_GNSS_MAX_AGE_S` (0.5s)**, 因其时标是语句 UTC 经 PPS 映射回算,
   含接收机 ~100-300ms 输出延迟 + 组句/解析排队, 50ms 门限会把位置更新
   几乎全部判过旧 (实测 gnss_stale 远超 gnss_cnt, EKF 位置辅助断供后
   纯惯性发散)。微小超前钳到当前。日志/导出对齐外部 UTC 用
   `timebase_mcu_to_utc()` 反向换算 (不参与融合)。
3. **磁链路处理链在入环前完成**: mag_data 采集线程内固定执行 校准→轴映射→
   低通, 环内样本同时携带 raw (传感器系原始值) 与 cal (体系处理值); 气压
   校准仍挂喂引擎路径 (baro_calib_apply 在 ginsaux)。校准与导航参数上电
   从 W25Q64 分区加载 (middleware/param_calib, 2026-10-02 迁移, 原
   片内 Flash 扇区 7 记录自动导入一次)。
4. **线程优先级**: gins 解算 (9) > ginsaux (10) ≈ main (10) > data 层采集
   线程 (11) > vofa/gins_fused_data (12) > 校准采集 (13) > gnssout (14) >
   magout (15) > barout (17) > FinSH (20); 例外: gnssrx 接收线程 (8) 与
   imudata 采集线程 (7) 高于解算链 (字节搬运/时戳采集要先行)。
   I2C/UART 阻塞均隔离在 data 层与 ginsaux, 不占 1kHz 解算时间预算;
   EKF 满负荷时低于 12 的线程会被饿死。**含控制链计划线程的全量线程表
   (优先级/周期/职责) 见根 README "飞行控制" 章任务清单, 以那边为准**;
   本条只列既有数据供给链的相对次序。
   **栈预算教训 (2026-09-29)**: gnssdata 1536 曾被 ulog 格式化尖峰击穿
   (溢出 → 调度器死循环), 输出线程 2048 曾被 NaN 的浮点格式化路径打穿
   (HardFault) —— 现 gnssdata=3072, vofa/gins_fused_data/gnssout=4096;
   新增会调用 LOG_x/printf 的线程时按含格式化路径的实测深度给栈。

## FinSH 命令速查

| 命令 | 说明 |
| --- | --- |
| `timebase` | 时间同步状态: T_MCU/PPS 捕获/配对门槛计数/滑窗/映射 scale |
| `imudata` / `magdata` / `barodata` | IMU/磁/气压环形缓冲区状态与最新样本 |
| `gnssraw` / `gnssdata` | GNSS 原始字节环 / 解析样本环状态 (含 ts_zero 计数) |
| `um982` | NMEA 解析结果与统计 |
| `magcal start [sec]` / `show` / `clear` | 磁力计椭球校准 |
| `barocal ref <pa>` / `refalt <m>` / `start [sec]` | 气压计基准校准 |
| `gins` | KF-GINS 解算结果与各观测统计 (含 stale 计数) |
| `so3` | SO(3) 姿态误差 vs 期望姿态 (`so3 target r p y`/`clear`; 姿态控制同拍自动发布目标) |
| `mpc` / `mpc set ...` | 位置 MPC 外环: 状态/调参/设定点 (`mpc pos x y z`)/参考点 (`mpc ref`)/单步 (`mpc step`) |
| `att` / `att set ...` | SO(3)+PID 姿态内环: 状态/调参/航向 (`att yaw <deg>`/`att hold`)/单步 (`att step`) |
| `vofa [on\|off]] [log on\|off]` | KF-GINS JustFloat 50Hz 二进制流开关 (默认 off, 单写者约定) |
| `gins_fused_data [on\|off]` | KF-GINS 带标记文本开关 (fused_data 后缀, 默认 off) |
| `gnssout [on\|off]` | UM982 定位解文本开关 (开启时与 gins 分抢样本) |
| `magout [on\|off]` / `barout [on\|off]` | 磁/气压计 raw+calib 文本开关 |
| `nav` / `nav set ...` / `nav save` | 导航参数镜像查看/修改 (磁偏角/NOGNSS 位置/轴向映射/磁开关) /持久化 (W25Q64 nav 分区) |
| `param` / `param erase <part>` | W25Q64 参数分区状态查看 / 分区擦除 (恢复出厂) |
| `sysinfo` | 系统参数 (启动计数/固件标识/迁移标志) |
| `w25q64 id/read/write/erase` | 外置 SPI Flash 调试 (底层) |

## 2026-09-29 修复记录 (SWD 直读诊断, 静置台架验证)

按发现顺序, 前四项为"启动 3~6 分钟内系统冻结/解算 NaN"级联根因:

1. **gnssdata 栈溢出** (1536→3072): PPS 配对里程碑的 ulog LOG_I 格式化
   栈尖峰击穿栈底踩坏 `#` 魔数, 调度器 `rt_scheduler_stack_check` 溢出
   分支 `while(1)` 冻结全系统 (PC 落在 scheduler_comm.c, 关中断自旋)。
2. **uart1 TX 并发竞态**: vofa(50Hz) + gins_fused_data(10Hz, 当时默认 on)
   + console 并写 —— INT-TX 的 putc+completion 无锁, TX 环被打烂后堆
   损害随机蔓延 (受害线程轮换: gins_fused_data → vofa → 未名线程),
   UNDEFINSTR HardFault。修复: 输出链路栈 2048→4096 + 数据链默认 off
   (单写者), 修复后连续运行 2.2h 无崩溃。
3. **NMEA 行缓冲截断** (UM982_NMEA_LINE_MAX 160→256): UM982 双天线
   `#UNIHEADINGA` 约 190 字符, 溢出丢头后残句进解析器误记 csum_err,
   并干扰 RMC/GGA 统计。
4. **GNSS RX fifo 套圈** (512B→2KB): 见 UM982 节说明。
5. **GNSS 观测过期门限** (`GINS_GNSS_MAX_AGE_S` 0.05→0.5s): UM982 输出
   延迟 ~100-300ms, 50ms 门限实测 781 个观测丢弃 780 个, EKF 位置辅助
   断供后纯惯性发散至 NaN (NaN 再触发第 2 项的 dtoa 爆栈)。
6. **timebase 滑窗陈旧楔死**: 单调性检查在重启判定之前 —— 解析线程停摆
   恢复后滑窗留陈旧条目, 每个新候选都被非单调拒绝, 永远走不到重建路径
   (rej_interval 每秒 +1)。修复: 非单调拒绝前检查条目陈旧性
   (>2s 无新 PPS 捕获即重开滑窗)。
7. **失锁重启门限** (TB_RESTART_UTC_GAP 1.5s→10s): 整秒句解析率 ~50%
   时 2-4s 间隙常见, 1.5s 门限反复清滑窗使映射长期无效 (2.2h 内重启
   2023 次); 间隔容差按 du 比例缩放, 10s 间隙下晶振误差余量 >500x。
8. **磁链路 float→double 类型双关** (见 BMM350 节): `(const double*)raw`
   强转读越界, cal 输出 -4.85e10/NaN 垃圾, 磁航向观测自编译起从未
   生效 (yaw 仅靠陀螺 0.1~0.2°/s 漂移)。
9. **EMA NaN 闩锁守卫** + 引擎门限 NaN 穿透修复: `fabs(x)<1e6` 同时拒绝
   NaN/±inf/huge (注意 `isfinite` 只查 double 域, double 域巨大有限值
   转 float 溢出为 inf 仍会穿透); gi_engine 的模值/压强门限改
   `!(min<=x<=max)` 形式。
10. **观测合理性门禁 + NaN 看门狗**: GNSS 位置/速度、磁场模值入引擎前
    门禁 (异常观测是 EKF 数值发散主源); `g_nan` 快照首次 NaN 时刻的
    各观测年龄/计数 (SWD 可读, 无串口依赖)。

**遗留**: 磁航向 `GINS_MAG_ENABLE=0` —— 链路已修复 (cal 正确跟踪 raw,
52µT 地磁量级), 但 `GINS_MAG_AXIS_SRC/SIGN` 未对倒装安装标定, 启用会
以系统性航向偏差把姿态从加计水平基准拽跑 (实测翻滚发散); 需以已知
航向 (或双天线 UNIHEADINGA) 台架核对后开启。NMEA 校验失败残余 ~40-50%
且随运行缓慢退化 (疑慢泄漏/堆碎片, 限制 GNSS 观测率 ~1.8 obs/s)。
对准窗口被扰动会产生发散解算 (重启即愈)。

## 2026-09-29 BMP585 气压链路修复 (硬件接好后 SWD 手动 I2C 取证)

传感器/I2C2 硬件全部正常 (hwi2c2 就绪、CHIP_ID=0x51、PB10/PB11 AF4
开漏+内部上拉生效、手动配置后压强 1002.90hPa 稳定、温度 ~36°C 正常),
链路 dead 纯属驱动 bug, 三层问题逐个修复 (sensor_bmp585.c 文件头有
完整取证记录):

1. **nvm_rdy 门禁误判**: 本模块 STATUS(0x28) 的 nvm_rdy(bit1) 常态读 0
   仅偶发置 1, 而测量功能完好、nvm_err(bit2) 恒 0; 旧驱动把
   "软复位后 100ms 内 nvm_rdy=1" 当硬门禁 → 开机必失败。
   改: nvm_err 才硬失败, nvm_rdy 尽力等待放行。
2. **软复位有害**: 板复位伴随的 3V3 纹波使传感器欠压复位, 之后 NVM
   重载停摆数分钟到十几分钟才自愈 (期间配置写 ACK 但不生效、压强读
   0x7FFFFF 饱和值、温度正常)。旧驱动开机无条件软复位, 自己制造停摆。
   改: 芯片应答且健康时**跳过软复位** (快速路径: 直接配置+压强合理性
   自检 30~1250hPa, 两轮), 软复位只留给芯片真垃圾的兜底。
3. **迟到无兜底**: boot 期自检必落在停摆窗口内, 失败后无人重试。
   改: 驱动 bmpretry 线程**无界重试** (10s 周期, 只走快速路径);
   baro_data 采集线程设备 find+open 同样**无界迟到重试** (5s 周期,
   原先找不到设备直接放弃)。
4. **baro 线程栈溢出** (修复过程翻车实录): baro_data 线程栈 1024,
   设备迟到上线路径新增的 LOG_I 格式化尖峰直接击穿 —— 传感器自愈
   上线瞬间系统冻结 + 内存踩踏 (rt_tick 写坏), 与 2026-09-29 早前
   gnssdata 1536 栈死于 LOG_I 同款机制。baro 线程栈 1024→4096,
   bmpretry 2048→3072。

**实测验证 (2026-09-29, 静置台架, SWD 直读)**: 传感器自愈后 (复位起
最快一次 <60s, 最慢 ~8min) 驱动自动注册 → baro_data 自动打开 →
**100.0 obs/s 喂入 EKF**, 引擎气压高度输出合理跟踪, 调度器
健康无 NaN。链路断供期 (传感器停摆窗口内) 解算不受影响。
**外部上拉迁移完成 (2026-09-29 12:02 固件)**: PB10/PB11 外部上拉焊装
并验证后, 已删除 `bmp_i2c2_pullup_boot` 应急函数 (I2C1/BMM350 侧本来
就没有内部上拉代码, 一直用外部上拉)。验证: PUPDR PB10/PB11=00 (无
内部上拉), IDR 两线仍高, **baro/mag 均 99.9 obs/s**, 且本次烧录后
零停摆 boot 即上线。遗留观察: 引擎 baro_height 模型输出偶发异常值
(实测 3.8e6 m) 而高度通道本身正常 —— 疑首次锚定/慢再锚定与垃圾
GNSS 高度观测交互所致, 与上拉无关, 待 RX 根因修复后复测。

同场加映修复 (gins_bridge.cpp): GNSS 观测两级新防线 ——
(a) 对准播种路径补合理性门禁 (与观测门禁同限): 混缝句可携带有效
fix 标志但坐标残缺 (实测播种到 16.1N/109.5E/-1946km), 之后观测全被
卡方门限拒绝, 解算永久锁死在错误位置;
(b) 观测新息门禁 `GINS_GNSS_INNOV_MAX_M` (200m, 连续 5 次拒收后
放行一次防真失锁重捕被误杀): 量程门禁挡不住"量程内"垃圾 ——
修复前实测高度 20km 级垃圾观测把 pitch 拽到 52°、解算发散到数千
km; 加门禁后同样 RX 劣化条件下静置位置全程稳定在 <50m 包络。

**硬件遗留 (装机前必办)**: PB10/PB11 无外部上拉, 现靠 ~40k 内部上拉
撑总线 (短线台架可用, Fast 模式偏弱); 正式装机焊 2.2~4.7kΩ 到 3V3
并删 bmp_i2c2_pullup_boot。传感器 VDD 在板复位时欠压 (NVM 停摆根因),
检查模块供电/去耦; 若能保证传感器供电干净, 上述停摆窗口自然消失。

## 2026-10-01 修复: 5.3.0 升级后 I2C 两总线全无响应 (BMM350/BMP585 探测恒失败)

平台升级 (RT-Thread 5.3.0, 已知问题 #5 "未上板回归") 上板兑现的第一个
真实回归: **I2C1/I2C2 上所有传输 100% 失败**, 两个传感器驱动探测超时,
baro/mag 链路整体 dead。构建零警告 (宏条件编译, 缺宏不报错), 纯运行时
路径故障。

根因: 5.3.0 `drv_hard_i2c.c` 的传输后端 (IT/DMA/POLL) 全部由
`BSP_I2C{x}_{TX,RX}_USING_{INT,DMA,POLL}` 宏条件编译, 且多消息传输
(写寄存器地址 + 读数据, 传感器驱动一切读操作的形态) 在 xfer 入口就被
"TX+RX 异步后端齐备" 检查 (`stm32_i2c_seq_backend_available`) 拦截 ——
repeated-start 序列 HAL 只有 Seq_IT/Seq_DMA 两族 API, POLL 无法承接。
平台升级重做 rtconfig.h 时只保留了 `BSP_USING_HARD_I2C1/2` 总线开关,
一个模式宏都没带过来 → 三后端全缺 → `i2c_dma_flag=0` → 一切传输
`HAL_ERROR`/LOG_E 直接拒绝。总线设备照常注册 (hwi2c1/2 能 find 到),
所以症状是 "总线无响应" 而非 "设备不存在"。

修复 (B5 结论 "回 IT/PIO" 的正式落地):

1. `rtconfig.h`: 补 `BSP_I2C1/2_TX/RX_USING_INT` 四宏 (TX+RX 齐备才过
   seq 检查; I2C2 不开 DMA —— B5: DMA 直写调用方栈缓冲, DCache 不可
   维护);
2. `board/Kconfig`: 同名选项补齐 (default y, depends on 对应总线),
   防将来 menuconfig 重生成 rtconfig.h 时配置再次丢空 —— 这正是本次
   事故的入口。

闭环链路: INT 宏 → `BSP_I2C_USING_IRQ` → `stm32_i2c_init` 使能
I2Cx_EV/ER 中断 (优先级走 board_nvic 表 =2) → 传输走
`HAL_I2C_Master_Seq_{Transmit,Receive}_IT` → 完成回调
`rt_completion_done` → 线程唤醒 (阻塞语义与 DMA/POLL 等价, 100kHz
12B 传输 ~1ms, 100Hz 轮询无压力)。

验证 (构建级): 双构建清洁全量零警告 (CMake 与 scons/env-windows
工具链各一次); ELF 符号确认 `I2C1/2_EV/ER_IRQHandler`、
`HAL_I2C_MasterTxCpltCallback`、`HAL_I2C_Master_Seq_Receive_IT` 全部
编入。

**上板验证 (2026-10-01, DAPLink 烧录 + COM9 串口观测)**: 旧固件实测
错误信息与根因逐字吻合 (`I2C[hwi2c2] multi-message transfer requires
both TX and RX sequential DMA/IT capability` / `I2C[hwi2c1] Write
error(1)`, 两传感器探测每 10s 失败一轮)。新固件 boot **首次上电即
上线, 无需重试**: `BMM350 ready (addr=0x14, id=0x33)`@561ms、
`BMP585 ready (addr=0x46, id=0x51, ODR=100Hz, 1001.41hPa)`@598ms;
mag/baro 数据链路 30min 内零 I2C 错误。数据质量 (FinSH `magdata`/
`barodata`): **pushed=popped 零丢失零错误**; 磁场 raw 37.7/-0.9/-15.2
uT → cal 47.8/0.8/0.4 uT (FRD, 椭球标定 radius 48.8uT 从 calib 正常
恢复), quality=0x00; 气压 100139Pa / 芯片温度 35.1°C; T_event 时间戳
us 精度正常 (timebase PPS 映射不受影响)。

**观察记录 (非缺陷)**: mag/baro 实际观测率 **~90.8 obs/s** (增量法
实测), 低于 09-29 基线的 99.9 —— 采集线程为 `mdelay(10) + I2C 读`
串行结构, 周期天然 ~11ms (100kHz 总线 12+2B 两消息序列 ~1.5ms +
mdelay(10) 平均 10.5ms); 5.2 旧驱动/旧基线测量口径下为 99.9。磁/气压为
低带宽观测, 90.8 对 EKF 无实质影响; 若需恢复可把采集线程改为绝对
周期调度 (下拍 = 上拍 + 10ms), 属独立优化项。

**当次台架物理状态 (与固件无关, 待用户确认)**: ADIS16505 探测
PROD_ID=0x0000 (MISO 恒零) + W25Q64 JEDEC 全零 + UM982 无定位
(fix=0 sats=0) —— 两个 SPI 域器件物理无响应 + GNSS 天线未接/室内,
故 EKF 停在 ALIGN/WAIT, imu/gnss/mag/baro 引擎侧计数未开。09-30 十轮
测试时 ADIS/GNSS 均正常, 中间仅发生过本固件改动 (纯 I2C 宏, SPI
传输链路完好), 指向 IMU 模块/天线连接变化。

附注: scons 若用 PATH 里的 STM32CubeCLT gcc 而非 env-windows 工具链
(RTT_EXEC_PATH), -O2 下会多出 5 条推演型警告 (klibc/drv_usart_v2/
gi_engine Dr 系列), 属工具链版本差异, 与本工程代码无关; 质量基线以
env-windows 工具链为准。另: `build_host/flash.ps1` 的 program 命令
不带 reset 参数, 烧完 CPU 继续跑旧映像, 必须 telnet 补发 `reset run`
(本次验证踩过)。

## 2026-10-01 ADIS16505 无响应取证 + 传感器链路无界自愈 + NOGNSS 冗余确认

用户报告 ADIS 物理完好但 `PROD_ID=0x0000` (MISO 全零), GNSS 室内无信号
属预期。取证与修复:

**取证链 (固件侧已穷尽, 结论: 物理层无响应, 非驱动问题)**:
1. 新增失败模式区分日志: 探测失败走 "PROD_ID=0x0000" 分支而非
   "probe xfr failed" —— `rt_spi_transfer` 框架传输成功执行, 是读回
   数据全零 (旧日志 prod_id 初值即 0, 无法区分两种失败, 已修);
2. `adisdbg probe`(框架路径 8 轮)与 `adisdbg raw`(HAL 直连 + 手动
   CS, 绕过框架)读数一致为 0x0000 —— 两条独立路径排除框架/时序问题;
3. SWD 读 GPIO IDR (线上真实电平): SCLK/MOSI 空闲高 (模式3 正常)、
   CS(PC4)/RST(PC5) 线上为高、**MISO(PA6) 恒 0、DR(P?4) 恒 0**
   (8 次采样无脉冲; DR 空闲应为高) —— 芯片零输出;
4. SPI1 寄存器 CFG 全部正常 (MASTER/CPOL/CPHA/8bit/954kHz)。
   → **ADIS 供电未到达模块或排线接触不良** (W25Q64 JEDEC 同样全零,
   可能共因)。请检查模块 3V3/排线; 芯片接好后固件自动上线, 无需重烧。

**2026-10-01 复测升级: 硬件级直证 (应用户要求重测总线)**:
`build_host/spi_hw_bus_test3.ps1` —— OpenOCD halt CPU (固件/驱动完全
脱离) 后 SWD 直控 SPI1 外设, 标准 TSIZE=5 批量模式发 0x72 读命令:
SR=0xB01F (EOT/RXP/TXTF 全置位) 证明 5 字节传输完整执行, SCLK/MOSI/CS
全部按模式 3 正常输出, 而 **RXDR 5 字节全 0x00 (MISO 40 bit 全零)、
MISO 线电平 CS 有效全期恒 0** —— 主机端 (外设+引脚) 硬件级完好,
从机端零输出, 与固件层结论相互独立地吻合。RST 线释放为输入读 1
(`rst_line_test.ps1`, 线上有上拉, 板载或芯片内部不可区分)。
**判定: 总线从机端 (ADIS 模块供电或 MISO/DR 排线) 问题, MCU 侧排除。**
排查指引: 万用表量 ADIS 模块 VDD/GND 是否有 3V3 → 检查排线 MISO/DR
线 → 断电重启; 接好后 `adisretry` 自动上线 (`ADIS16505 online after
deferred retry`)。

**2026-10-01 深夜翻案 (上述"物理层"结论错误, 特此记录完整教训)**:
应用户指正重审时间线 (README #8: 平台升级从未上板回归, 09-30 十轮
测试跑的是升级**前**固件 —— "昨天还好"的推理支柱不成立) 后, 新增
`adisdbg reg` 裸寄存器取证 (固件内 CPU 直接操作 SPI1 寄存器, 微秒级
时序, 完全剥离框架/HAL), 并在新固件烧录后 **ADIS 首次探测即成功**
(`prod_id=0x4079`、DR=1、DR→DMA 链路启动、1000Hz)。此后连续 5 次
硬复位 (`recurrence_test.ps1`) 全部上线, 不复现。
**真相: 芯片从未断线/损坏, 而是陷入跨 MCU 复位持续数小时的粘滞异常
态** (期间对一切固件/SWD 探测零响应; 与 BMP585 2026-09-29 的欠压
NVM 停摆同族), 最终被 "halt CPU + SWD 裸寄存器访问序列 + 全片擦除
重烧" 唤醒。之前 "物理层无响应" 的判定被三个因素带偏: (a) 时间线
误读 (以为升级后验证过); (b) SWD 手搓时序测试的缺陷 (TSIZE 连发无
帧间 stall、SPE=0 清 FIFO、RXDR 读空返回 0, v4-v6 逐步修正仍难
完全消除); (c) 芯片粘滞态对一切探测的"稳定复现"假象。
**诱因未定罪**: 嫌疑集中在 5.3.0 首次上板时的异常交互 (spixfer 探测
路径 DMA 化 —— 5.2 为纯轮询; 或上电时序欠压), 一旦复发用 `adisdbg
probe` 取证。实测恢复后: IMU 精确 1000/s (dr_drop=0), 室内无 GNSS
(fix=0) 下 NOGNSS 播种 → `RUNNING`, mag/baro 各 100 obs/s 入引擎
(stale=0/cov_warn=0), 冗余设计目标达成。观察项: baro 观测 skip
(~47%) 待查 (疑 NOGNSS 时基下 T_event 超龄判据偏紧, 与 GINS_AUX_
MAX_AGE_S 相关); W25Q64 "零响应" **已于 2026-10-01 晚破案**: 非芯片
问题, 是 PF10 CLK 复用号配错 (应为 AF9, 旧驱动配 AF10, 时钟未接到
引脚) + IO2/IO3 (WP/HOLD) 未驱动为高, 已随 QSPI 驱动框架化一并修复,
板上 JEDEC/读写擦全通过 (详见文末 "W25Q64 改走 QSPI 驱动框架" 节)。

**固件自愈补强 (对齐 BMP585/BMM350 模式, 2026-10-01)**:
1. `sensor_adis16505.c`: init 拆分为一次性总线准备 (attach/configure/
   MasterKeepIOState, 幂等) + 可重入探测 (每轮重打 RST 脉冲);
   `adisretry` 线程 10s 周期无界重试;
2. `imu_data.c`: 设备打开改惰性 (线程内 1s 节流 find+open+rx_indicate),
   gyroscope LSB 随取 (迟到上线时量程索引可能未就绪) —— ADIS 迟到
   上线即自动接入, 不再一次性禁用;
3. 新增 FinSH `adisdbg [probe|raw|reset]`: 现场取证命令 (探测/直连
   读/复位重探, 含引脚与 SPI 寄存器 dump)。

**NOGNSS 冗余链确认 (室内无 GNSS 的工作模式, 代码已完备)**:
`GINS_NOGNSS_MODE=1` (gins_config.h): 静止对准 (加计窗口 3s) →
持续无定位 30s (`GINS_NOGNSS_WAIT_S`) → 用配置位置播种 (深圳
22.64N/114.01E/50m, `GINS_NOGNSS_LAT/LON/ALT`) → ALIGNED; 引擎时间
基准 NOGNSS 下为 T_MCU 单调时基, **IMU/磁/气压观测不依赖 GNSS**
(仅 GNSS 样本需要 PPS 映射); GNSS 恢复且定位稳定后自动重新播种转
正常融合。室内验证步骤 (ADIS 接好后): 静置上电 → boot 后 ~35s
FinSH `gins` 应离开 ALIGN/WAIT (NOGNSS 播种) → `imudata`(如有)/
`magdata`/`barodata` 计数增长, vofa 观 r/p/y 收敛 (yaw 靠磁航向,
轴向映射核对属已知问题 #2)。

验证 (构建级): 双构建清洁全量零警告; 新符号 (`adisdbg`/`adisretry`)
编入 ELF 并上板实跑 (probe/raw 取证即由板端新命令产出)。

**2026-10-01 10 轮 × 3min 回归 (test10_3min/, DAPLink/OpenOCD 版 runner
`swd_10rounds_3min.py`)**: 10/10 轮 11/17 PASS, 失败 6 项完全一致且均为
器件缺席类 (zda/rmc/gnss obs/map.valid/INS 位置 ← UM982 室内无输出;
imu ← ADIS 物理不在线, 见已知问题 #7)。**可评估判据全过**: 10 次硬复位
冷启动零复位核验失败/零 warn, tick 走速、csum/timebase restart/stale/
rej/ts_zero/raw lost/NaN/degraded/updfail/reseed 增量全 0; 末轮 230s 时
mag pushed=popped=20843 (90.8 obs/s)、baro 21024 (90.8 obs/s), 均
lost=0 errors=0, 压强 100120Pa/30.8°C、磁场量值正常。测试设施:
`swd_stab_alt.py` 符号地址已按本固件 nm+gdb 复核并改走 OpenOCD telnet
读取 (`ocd_swd_read.py`, 剥 IAC/NUL 的 telnet 流坑已修); tick 判据改按
采样起点间隔 (OpenOCD 采样耗时 ~5-8s, 旧口径误杀)。完整 17 判据回归
待 ADIS/GNSS 就位后重跑。

## 2026-10-01 W25Q64 改走 RT-Thread QSPI 驱动框架

原 `sensor_w25q64.c` 自持 OSPI 句柄 HAL 直连 (GPIO/RCC/命令全自包),
是全工程唯一游离于设备框架之外的驱动, 已重构为与 ADIS(rt_spi) 同一分
层模式:

- **BSP 层** `libraries/HAL_Drivers/drivers/drv_qspi.c`: 上游旧版是
  QUADSPI 代码 (H723 无 `QUADSPI` 符号, 开宏必编译失败), 重写为
  OCTOSPI1 间接模式轮询总线驱动 —— RCC/GPIO(PF6-10+PG6 AF10)/OSPIM
  自含, `rt_qspi_bus_register("qspi1")` + `INIT_BOARD_EXPORT`;
  NCS 由 OCTOSPI Manager 硬件驱动, 无软 CS/DMA 路径。
- **芯片层** `sensor_w25q64.c`: attach `qspi10` 设备后全部命令走框架
  API —— `rt_qspi_send` (写使能/擦除/页编程)、`rt_qspi_send_then_recv`
  (JEDEC/状态/Fast Read 0x0B+8dummy)、`rt_qspi_transfer_message` 自组
  消息 (Unique ID 0x4B 需 32 dummy 时钟, 超 H7 DUMCR 31 上限, 以
  32-bit 备用字节阶段等效; 旧驱动 dummy=4 是错误值)。`w25q64_*` 导出
  API 与 `w25q64` MSH 命令语义不变, 零调用方迁移成本。
- **接线** `RT_USING_QSPI`+`BSP_USING_QSPI` (rtconfig.h +
  drivers/Kconfig + 根 CMakeLists 三处, SCons 侧条目本就存在)。

**排障实录 (最终定位为两处软件移植错误, 芯片一直是好的)**:
1. **HAL_OSPI 寄存器"值-1"语义**: SCK=内核时钟/N 而非旧 QUADSPI 公式的
   /(N+1), 首版实际 SCK 137.5MHz 超规格 (日志值也错); SWD 实读 DCR1/
   DCR2 逐位核对修正, 91.67MHz 合规。
2. **PF10 CLK 复用号配错 (根因)**: 旧驱动假设六脚全为 AF10, CubeMX
   器件库逐脚实证 (GPIO-STM32H72_gpio_v1_0_Modes.xml): PF10=
   OCTOSPIM_P1_CLK 为 **AF9**, 其余数据脚/NCS 才是 AF10 —— 时钟根本
   没接到引脚, 外设状态机照常跑完 (HAL 直连探针: Command OK/IR=0x9F/
   Receive OK) 但线上无时钟, JEDEC 任何速率都全零。
3. **IO2/IO3 (WP/HOLD) 电平**: 1-1-1 下 OCTOSPI 不驱动 IO2/IO3 (AF 态
   实测输出低), W25Q64 要求 /WP /HOLD 高; 本总线不跑 2/4 线, PF6/PF7
   改配 GPIO 推挽输出高。
4. **框架路径自证**: `w25q64 dbg` (寄存器转储 + 双速 JEDEC + BSP HAL
   直连探针 `stm32_qspi_bus_diag`) 完成取证; init 探测曾误调带 ready
   门禁的公开 API 导致根本不碰总线 (msh `command failed -5` 由此来)。

**上板验收 (2026-10-01, 全通过)**: boot 即 `W25Q64 ready` + JEDEC
`EF 40 17` + Unique ID `92572F4A27DF62D0` (32-dummy 备用字节技巧实测
有效) + erase 4k/32k OK + 擦后读全 FF + 写读回逐字节一致。
`build_host/w25q64_onboard_test.ps1` 为验收脚本 (COM9 console +
OpenOCD 4444 复位)。

## 2026-10-02 飞控参数分区存储落地 (middleware/param_calib, W25Q64)

作为成熟飞控的参数持久化基础设施: 标定/导航/系统参数按分区存 W25Q64,
原片内 Flash 方案 (calibration 下的 calib_store.c/.h, 已删) 退出。设计见
`param_calib/README.md`, 要点:

- **分区** (编译期权威 `param_part.c`): ptbl 4KB (分区表自描述副本) /
  calib 16KB (磁椭球+气压偏移+加计零偏) / nav 16KB (磁偏角/NOGNSS 部署
  位置/IMU+磁轴向映射/磁观测开关) / sys 8KB (启动计数/固件标识/迁移
  标志) / ctrl 20KB (控制律预留); 其余 ~7.9MB 未分配 (未来黑匣子)。
- **记录纪律** (沿袭已验证的追加式日志): 128B/64B 槽 + magic/ver/seq/
  CRC32 头, 追加写满整擦回卷, 掉电至多损失最新一条, 磨损均衡随回卷
  获得; 引擎锁串行化跨域并发保存。
- **不关中断**: OCTOSPI 间接模式, 擦写期间 CPU/四链 EXTI/DMA 照常,
  原片内方案的 DCache 开关/关中断窗口/timebase 回绕核对全部不再需要
  (timebase 的窗口工具保留为通用设施)。
- **calib_store_* API 原样保留** (mag_calib/baro_calib/gins_bridge 零
  改动), 首次上电自动把片内扇区 7 旧记录导入 calib 分区一次 (sys 标志
  防重复, `param erase calib` 后不复活旧值)。
- **导航参数运行期化**: 原编译期宏 `GINS_MAG_DECL_DEG` /
  `GINS_NOGNSS_LAT/LON/ALT` / `GINS_AXIS_*` / `GINS_MAG_ENABLE` 改由
  param_nav 镜像提供 (缺省仍来自宏, 无记录行为不变); 消费点 gins_bridge
  (播种/磁初始化/位置守卫) 与 data 层 (imu/mag 轴映射热路径) 直读镜像。
  台架核对轴向映射 (遗留问题 #1) 现可用 `nav set maxis ...` + `nav save`
  现场修正, 不再重编译。
- **降级**: W25Q64 缺失时告警一次, 参数仅本次上电有效, 其余不受影响。
- **构建接线**: SCons 经 middleware/SConscript 自动收录 (DefineGroup
  'ParamCalib'); CMake 新增 rtt_ParamCalib 库 (RT_PARAMCALIB_SOURCES)。

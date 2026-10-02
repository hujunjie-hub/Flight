# middleware/gins — 组合导航桥接层 (ADIS16505 + UM982 -> KF-GINS)

把 `middleware/sensor`（ADIS16505 驱动）与 `middleware/protocol/nmea/um982_nmea`
（GNSS PVT）实时接入 `middleware/KF-GINS` 的 GIEngine（21 状态 EKF，
GNSS 位置观测的宽松耦合）。

## 数据流

```
ADIS16505 --1kHz DR--> adis16505_get_snapshot() --1kHz 轮询--\
                                                              gins 线程
UM982 ---10Hz USART2--> gnss_data.c --> um982_nmea_get_data -/   │
                                                              GIEngine (EKF)
                     timebase PPS 配对 (UTC/GPST 锚定)          │
                                                               ▼
                              gins_bridge_get_solution() / FinSH `gins`
```

## 文件

| 文件 | 说明 |
|------|------|
| `gins_config.h`   | 全部可调参数 (宏, 单位与 kf-gins.yaml 一致); 零依赖, 与主机测试共用 |
| `gins_options.hpp`| GINSOptions 构造 (单位换算, 平台无关); 与上游 loadConfig 同款实现 |
| `gins_bridge.h`   | C 接口: `gins_bridge_init()` / `gins_bridge_get_solution()` |
| `gins_bridge.cpp` | 解算线程: 初始化状态机 + 1kHz 馈入 + GNSS 更新 + 结果发布 |
| `test/tc_gins_engine.cpp` | utest 单元测试 (板上运行, 真实引擎 + 同一份选项代码) |

## 上电初始化流程

1. 探测 ADIS 陀螺量程 (RANG_MDL[3:2]);
2. 等待四条件同时满足: UM982 定位有效 (`pos_valid`) + PPS 已同步 + 静止对准
   窗口满 (`GINS_ALIGN_SAMPLES`, 默认 200ms, |f| 偏离 g 超过容差则重开窗口) +
   播种质量门禁通过 (hdop≤`GINS_SEED_HDOP_MAX` 且静止速度达标, 连续
   `GINS_SEED_STABLE_N` 个 10Hz 样本与窗口基准互差在
   `GINS_SEED_STABLE_H/V_M` 内, 超差重开稳定窗);
3. 构造 GIEngine: 位置/速度取自收敛后的定位, roll/pitch 由加计对准给出,
   yaw=0 (初始 std 180°, 移动后由 GNSS 缓慢拉回);
4. 引擎用当前样本"播种" (对应桌面主循环 `addImuData(imu_cur, true)`), 之后
   每毫秒 `addImuData + newImuProcess`。

## 关键约定 (改代码前必读)

- **坐标系**: NED 导航系, FRD 体坐标系 (前右下)。缺省轴映射 = ADIS 片上
  XYZ 直接对应前/右/下; **静止时加计 Z 读数约为 -g** (比力朝上为负)。
- **IMU 增量**: `dtheta` [rad] / `dvel` [m/s] = 体坐标系角增量/比力增量,
  dt 用快照 `local_us` 差分 (1kHz tick 轮询, 与唤醒抖动无关)。
- **时间**: 引擎用"自 GPS 纪元累计秒"(不回绕), 初始化时由快照 UTC 锚定,
  之后按本地微秒差分推进。GNSS 观测打**处理时刻**时标 (与 IMU 同源)。
  NMEA 报文时刻比处理时刻早一个传输延迟 (~几十 ms), 若按报文时刻打标,
  观测会永远"过期" (引擎 `isToUpdate` 判为不需要更新) 而丢失。
- **KF-GINS 最小改动原则**: 算法文件仅加 `KF_GINS_EMBEDDED` 宏保护
  (去 `print_options` 的 iostream、`checkCov` 的 `std::exit` 换告警钩子
  `kf_gins_cov_warning`, 实现在 gins_bridge.cpp)。

## 参数整定 (`gins_config.h`)

- IMU 噪声 (ARW/VRW/零偏/比例因子 std, 相关时间) 目前按 ADIS16505-2
  资料量级给定, **建议 Allan 方差标定后修改**;
- `GINS_ANT_LEVER` 天线杆臂: 影响位置观测精度与 yaw 可观测性, 安装后实测;
- `GINS_AXIS_SRC/SIGN`: 安装方向改变时修改;
- **GNSS 数据质量三级防线** (2026-09-29, "质量合格才入滤"):
  1. 先验质量门 (绝对拒绝, `g_gq`): 观测 HDOP > `GINS_GNSS_HDOP_MAX`
     (几何崩坏) 或与上一入滤观测跳变 > `GINS_GNSS_JUMP_BASE_M +
     GINS_GNSS_JUMP_VMAX_MPS*dt` (接收机跳变/混缝句, 与 INS 状态无关,
     INS 被污染后仍有效); 长间断后首样本门限按 dt 自动放宽不阻碍重连;
  2. 新息门 (分歧放行): 与 INS 位置差 > `GINS_GNSS_INNOV_MAX_M` 拒收,
     5 连拒放行 1 次保失锁重捕恢复 —— 谁错未知不能全拒;
  3. 坏播种侦测 (兜底, `g_reseed`): "一致性野值"(接收机错锁输出内部
     自洽位置) 能穿透一切先验门 (实测错锁 61km 全绿通过), 只能播种后
     印证 —— 播种后 `GINS_RESEED_GUARD_S` 窗口内质量合格观测的新息
     拒收率 EMA 持续 > `GINS_RESEED_REJ_RATE` 达 `GINS_RESEED_HOLD_S`
     即拆引擎重走对准+稳定窗 (错锁期周期重掷, 引擎重建复位气压锚定)。
  播种侧另有种子门禁 (hdop/静止速度/10 样本稳定窗, `g_seed`)。
  1b. 独立气压高度交叉校验 (`GINS_ALTX_H_M`): 播种样本与运行期观测的
     GNSS 高度都要过 —— 与 BMP585 独立基准 `h_baro=44330·(1-(P_cal/
     101325)^0.1903)` 偏差超门限即拒 (垂直垃圾实测 -8.4km/20km/-1886km
     均 ≥900m, 门限 250m 挡得住且放过 ±80m 天气性偏差)。基准只依赖
     本地气压计, 不被 GNSS 垃圾污染; 气压链路未运行时 fail-open 跳过。
- GNSS 观测方差由 HDOP 粗估 (`um982_nmea_pos_std`), 精确标定后可在
  `gins_bridge.cpp` 里直接替换。

## 固件构建要点 (CMakeLists.txt)

- C++14 + `-fno-exceptions -fno-rtti`, ELF 用 C++ 链接器 (自动带 libstdc++);
- `-DKF_GINS_EMBEDDED`; `-DM_PI=...`: 工程的 `-D_POSIX_C_SOURCE=1` 会让
  newlib 隐藏 M_PI (KF-GINS 依赖), 命令行直接补 (勿用 `_DEFAULT_SOURCE`,
  会与 RT-Thread sys/time.h 垫片冲突);
- `rtconfig.h` 开 `RT_USING_CPLUSPLUS`: 跑全局构造 (link.lds 的
  `__ctors_start/end` 已就绪) 并把 `operator new` 接到 rt_malloc;
- 当前 -O0 调试编译 ROM 约 73%; 发布可切 -O2/--gc-sections 显著缩小。

## 验证

单元测试 (utest, 板上运行, 与固件同一份选项代码 + 同一编译宏;
耗时约 1~3 分钟; 运行中会暂停实时解算线程, 跑完建议复位):

```bash
msh> utest_run middleware.gins.engine
```

场景: 静止 60s (GNSS 10Hz) → 断观测惯性加速 10s → 恢复观测 10s
→ 磁航向收敛 30s (倒装 roll=178°, 真值 yaw=137°, 回归 magUpdate 的
倾角保护与去-yaw 倾角补偿公式; 2026-09 实测曾因完整 cbn 旋转使观测
失效, 33424 个观测拒 33423 个)。

上板串口数据质量分析 (USART1 抓包 → 速率/格式/计数器/静态噪声/双链
路一致性):

```bash
python middleware/gins/test/analyze_gins_uart.py <capture.bin> --expect-sec 120
```

2026-09-16 实测 (台架静止 120s, 深圳室外天线): fused 文本 9.46Hz
(mdelay 调度拉伸, 引擎 time 为真时标) / JustFloat 47.4Hz 帧完整
99.98% / IMU 984Hz GNSS 10Hz 磁 98Hz / yaw 漂移 -0.07°/min (磁锚定)
/ 水平游走 N0.61m E0.43m / 静态速度 |mean|<0.04m/s —— 全部通过。

## 2026-09-16 修复记录 (上板验证发现)

1. **data 层断言挂死**: baro/mag/imu_data_init 设备缺失提前 return,
   信号量未初始化, ginsaux 的 wait 触发 RT_ASSERT 系统挂死 —— 改为
   IPC 对象无条件初始化 (设备缺失时必现)。
2. **ginsaux 磁观测饿死**: 阻塞在永不释放的 baro 信号量上 10ms 超时
   期间, magout 的 off态排水把 100Hz 磁样本全部抢走 —— 未运行链路跳过
   等待; magout/barout off 时不再消费 (对齐 gnssout 模式, 环形缓冲满
   自丢旧无需排水)。
3. **magUpdate 两处算法缺陷**: (a) 倾角保护按 |roll|>60° 拒绝把倒装
   (roll≈180°) 的磁观测全灭 —— 只保护 pitch 万向节锁; (b) 磁航向用
   完整 cbn 旋转, 水平方位恒等于磁偏角与 yaw 误差无关 —— 改去-yaw 的
   Rz(-psi)*Cbn 倾角补偿 (主机测试场景 4 回归)。
4. **GINS_MAG_DECL_DEG** 0 → -2.4 (深圳 WMM 2026)。
5. **USART1 STREAM 插 CR**: finsh 线程 finsh_set_device 带
   RT_DEVICE_FLAG_STREAM open 串口, 数据中每个 0x0a 前插 0x0d ——
   ~5% JustFloat 帧含 0x0a 被打坏、文本行尾双 CR; uart1_lock() 每次
   数据写出前清除该位 (console/FinSH 走 nano HAL 直写不受影响)。

## 2026-09-29 修复记录 (SWD 直读诊断; 详见 middleware/README.md)

1. **GNSS 观测过期门限**: `GINS_GNSS_MAX_AGE_S` 0.5s (磁/气压维持
   `GINS_AUX_MAX_AGE_S` 50ms) —— 50ms 门限把位置更新几乎全部判过旧
   (781 丢 780), EKF 位置辅助断供后纯惯性发散到 NaN。
2. **观测合理性门禁**: GNSS 位置/速度 (纬度±90/经度±180/高程±1km~
   30km/速度±500m/s) 与磁场模值 (5~300µT) 入引擎前检查, 异常计数。
3. **NaN 看门狗** (`g_nan`): 发布快照时检测 roll/pitch/yaw/vel/pos,
   首次 NaN 冻结各观测年龄与计数现场 (SWD 可读)。
4. **磁航向 `GINS_MAG_ENABLE=0`**: mag_data 的 float→double 类型双关
   修复后磁链路首次真正生效, 但 AXIS_SRC/SIGN 未对倒装安装标定,
   系统性航向偏差把姿态拽飞 —— 需台架已知航向核对后开启
   (标定步骤见 gins_config.h 注释)。
5. 引擎门限 NaN 穿透修复 (gi_engine magUpdate/baroUpdate,
   `!(min<=x<=max)` 形式)。

**静置实测 (修复后, SWD 1Hz 采样 240s)**: roll -179.24°±0.069 /
pitch +4.05°±0.076 / 速度 |mean|<0.3 m/s (σ<0.2) / 高度 101.0±1.4 m
(GNSS 102.7) / IMU 精确 1000/s / GNSS 1.84 obs/s (受残余校验失败限制,
应为 10) / yaw 漂移 ~0.13°/s (磁禁用中) / 无 NaN 无崩溃。

遗留观察项: ADIS DR 丢拍 ~1.7% (dr_drop/imu_cnt, data_cntr 跳变统计),
增量不可恢复, 建议后续提 gins 线程 SPI 读优先级或改 DMA 链; BMM350
硬磁未标定 (实测场强 ~70µT vs 预期 ~58µT), yaw 绝对精度受限, 需转动
板子跑 `magcal` 椭球标定; NMEA 校验失败残余 ~40-50% 且随运行退化,
为 GNSS 观测率的主要限制。

## 许可提醒

KF-GINS 为 GPLv3, 链接进固件后固件整体需按 GPLv3 义务处理。

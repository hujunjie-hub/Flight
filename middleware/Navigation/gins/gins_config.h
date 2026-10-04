/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * GINS 组合导航参数配置 (ADIS16505 + UM982 -> KF-GINS)
 *
 * 单位约定与 KF-GINS 的 kf-gins.yaml 完全一致 (换算在 gins_options.hpp 的
 * gins_build_options() 里完成, 与上游 loadConfig 相同)。
 *
 * 纯宏定义, 无任何依赖: 固件与主机仿真测试共用。
 */

#ifndef __GINS_CONFIG_H__
#define __GINS_CONFIG_H__

/* ---------------- 初始状态标准差 ---------------- */

/* 初始位置标准差 [n, e, u], m (首次定位时位置精度很好, 可取紧) */
#define GINS_INIT_POS_STD           { 1.0, 1.0, 2.0 }
/* 初始速度标准差 [n, e, d], m/s (来自 RMC 速度, 无 STD 只能给经验值) */
#define GINS_INIT_VEL_STD           { 0.2, 0.2, 0.2 }
/* 初始姿态标准差 [roll, pitch, yaw], deg.
 * roll/pitch 由静止加计对准: 误差为加计零偏主导 (~0.7deg ≈ 120mGal
 * 未建模零偏/g), 加长平均窗只压噪声不改偏置 (2026-09-29 实测 0.5deg
 * 与 1.0deg 收敛行为无差异); 取 1deg 给滤波器修正零偏等效失准的余量;
 * yaw=0 纯猜测, 给半圆 */
#define GINS_INIT_ATT_STD           { 1.0, 1.0, 180.0 }

/* ---------------- IMU 噪声参数 (ADIS16505-2 量级) ----------------
 * 建议 Allan 方差标定后修改! 仿真/数据复现时直接沿用 kf-gins.yaml 的习惯 */
#define GINS_IMU_ARW                0.2     /* 角度随机游走, deg/sqrt(h) */
#define GINS_IMU_VRW                0.05    /* 速度随机游走, m/s/sqrt(h) */
#define GINS_IMU_GB_STD             5.0     /* 陀螺零偏标准差, deg/h */
#define GINS_IMU_AB_STD             50.0    /* 加计零偏标准差, mGal */
#define GINS_IMU_GS_STD             300.0   /* 陀螺比例因子标准差, ppm */
/* 加计比例因子先验: 2026-10-02 实测本机 | ‖f‖-g | 随姿态 0.02~0.18 m/s²
 * (±1.8%, 轴失准+比例因子量级), 旧值 300ppm 低估真值 30~70 倍 —— EKF
 * 的 SA 状态被过紧先验锁死无法吸收, 加计误差全挤进零偏/姿态状态。
 * 放宽到 10000ppm (观测性弱, 仅给吸收余地, 不追求收敛) */
#define GINS_IMU_AS_STD             10000.0 /* 加计比例因子标准差, ppm */
#define GINS_IMU_CORR_TIME          4.0     /* 一阶高斯-马尔科夫相关时间, h */

/* 初始 IMU 误差 (标准差取噪声参数同值; 置零偏初值为 0 即上电不预设) */
#define GINS_INIT_GYR_BIAS          { 0.0, 0.0, 0.0 }   /* deg/h */
#define GINS_INIT_ACC_BIAS          { 0.0, 0.0, 0.0 }   /* mGal */

/* ---------------- 安装参数 ---------------- */

/* GNSS 天线相位中心在 IMU 体坐标系(前右下)中的杆臂, m.
 * 影响 yaw 可观测性与位置观测, 实际安装后务必测量修改 */
#define GINS_ANT_LEVER              { 0.0, 0.0, 0.0 }

/* 体坐标系(前右下)各轴取 ADIS 的哪根轴 (0=X, 1=Y, 2=Z) 及符号.
 * 缺省片上 X/Y/Z = 前/右/下 (右手系): 静止时加计 Z 读数约为 -g,
 * X 向北时前向加速读数为正.
 * 部署现场改用运行期参数: FinSH `nav set iaxis` (W25Q64 nav 分区,
 * 本宏为缺省值), 见 middleware/Sensor_Preprocessing/param_calib/param_nav.h */
#define GINS_AXIS_SRC               { 0, 1, 2 }
#define GINS_AXIS_SIGN              { 1.0, 1.0, 1.0 }

/* ---------------- 磁力计航向观测 (BMM350) ---------------- */

/* 是否启用磁航向观测 (0=禁用, 引擎不使用磁力计数据) */
/* 磁航向观测总开关: 2026-09-30 起 ENABLE (无 GNSS 部署形态: yaw 唯一
 * 绝对基准)。GINS_MAG_AXIS_SRC/SIGN 当前为直通缺省, **轴向映射未经
 * 台架核对** (倒装安装): 首次上板请按 mag_data.c 头注释/middleware
 * README 流程用 magout 流指北静置核对体坐标磁场, 错误映射表现为
 * 磁航向系统性偏差或姿态被拽跑 (新息门禁会拒绝大部分错映射观测,
 * 观察 gins 快照 mag_rej_cnt 飙升即映射错的信号)。 */
#define GINS_MAG_ENABLE             1

/* 当地磁偏角, deg, 东偏为正 (磁北偏东为正).
 * 磁航向修正必需! 查 WMM 模型或 NOAA 地磁图:
 *   https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
 * 国内常见值: 武汉约 -5 deg, 北京约 -7.5 deg, 广州约 -2.5 deg (2025 年代)
 * 当前部署地: 深圳 (22.64N, 114.01E, 2026-09), WMM 约 -2.4 deg (西偏) */
#define GINS_MAG_DECL_DEG           -2.4

/* 磁航向观测噪声标准差, deg (含硬铁残差与倾角补偿误差, 经验值 1~3) */
#define GINS_MAG_STD_DEG            2.0

/* 磁干扰样本降权系数 (README 异常值处理-观测级: 降权而非丢弃):
 * mag_data quality bit0 (模值超限) 置位时, 观测 std x 该系数 (方差 x 平方) */
#define GINS_MAG_INTERF_STD_SCALE   3.0

/* 磁航向新息门限, deg: 收敛后超过此值判为磁干扰/硬铁异常, 拒绝本次观测。
 * 收敛期 (yaw 方差大) 门限自动按 3σ 新息放宽, 不阻碍初始对准 */
#define GINS_MAG_GATE_DEG           15.0

/* 磁航向观测入滤最小间隔, s (0=不限速): 航向计算/门限每样本照常, EKF
 * 融合限速。BMM350 ~100Hz 全量入滤时样本经 τ=20ms EMA 相邻相关 ~0.6,
 * 按独立定权 yaw 信息量虚增 ~10 倍 (baro 2026-09-29 事故同款模式);
 * 限速到 10Hz 后相邻融合样本间隔 100ms >> τ, 独立性假设重新成立 */
#define GINS_MAG_FUSE_DT_S          0.1

/* 磁航向入滤等效观测 std, deg: 信息量守恒配平
 * std × sqrt(入滤前速率/入滤后速率) = 2.0 × sqrt(100/10),
 * 稳态 yaw 方差与限速前一致, 仅削减 CPU (21 状态 Joseph 更新 ~0.7ms/次
 * x 100Hz ≈ 7% CPU → ~0.7%)。需与 GINS_MAG_FUSE_DT_S 联动调整 */
#define GINS_MAG_FUSED_STD_DEG      6.3

/* 体坐标系(前右下)各轴取 BMM350 芯片的哪根轴 (0=X, 1=Y, 2=Z) 及符号.
 * BMM350 是独立芯片, 安装方向与 ADIS 可能不同;
 * 注意右手系一致性: 映射后必须是 前右下 右手系, 否则航向符号翻转.
 * 部署现场改用运行期参数: FinSH `nav set maxis` (本宏为缺省值).
 * 该映射在 process_data 层采集线程入环前生效 (middleware/Sensor_Preprocessing/process_data/mag_data.c) */
#define GINS_MAG_AXIS_SRC           { 0, 1, 2 }
#define GINS_MAG_AXIS_SIGN          { 1.0, 1.0, 1.0 }

/* 磁力计轻量低通时间常数, s (一阶 EMA, 轴映射之后、入环之前生效).
 * 0 = 旁路直通. 默认 0.02: 100Hz 下 α=1/3, -3dB 约 8Hz, 群延迟 20ms
 * (观测时标在 gins 桥接侧按 τ 补偿). τ 过大会在快速转弯时抹平航向变化 */
#define GINS_MAG_LPF_TAU_S          0.02

/* ---------------- 无 GNSS 部署模式 ---------------- */

/* 不接 GNSS (无天线/室内台架) 时的运行形态: 静止对准完成后持续
 * GINS_NOGNSS_WAIT_S 无有效定位, 则用下方配置位置播种引擎 ——
 * 纯 IMU + 磁航向 + 气压高度解算 (位置开环漂移, yaw 由磁观测约束,
 * 垂直由气压锚定)。GNSS 后续接入且定位稳定 (播种稳定窗满) 时自动
 * 拆引擎重走正常融合路径 (时间锚点随之重建)。
 * 引擎时基: 无 GNSS 无 PPS, 锚点 GPST 取 0 (T_MCU 起点对齐), EKF
 * 只依赖单调相对时间, 无 GNSS 观测时不涉绝对时刻换算, 安全。 */
#define GINS_NOGNSS_MODE            1
#define GINS_NOGNSS_WAIT_S          30.0    /* 静止对准后等 GNSS 超时, s */
/* 播种磁就绪门: WAIT_S 到点但无新鲜磁样本 (I2C 恢复中/传感器掉线) 时
 * 不立即播种 —— yaw 只能退半圆先验, 而半圆先验播种后 0.8s 内状态爆
 * 炸 (2026-10-02 实测位置 23088 km 被位置守卫拆除)。最多再等此时长
 * 让磁流恢复; 超时仍无磁才退半圆照播 (真无磁部署, 罕见路径) */
#define GINS_NOGNSS_MAG_WAIT_S      60.0    /* 磁未就绪的额外有界等待, s */
#define GINS_NOGNSS_LAT_DEG         22.64   /* 部署地纬度 (与磁偏角同源: 深圳) */
#define GINS_NOGNSS_LON_DEG         114.01  /* 经度 */
#define GINS_NOGNSS_ALT_M           50.0    /* 椭球高 m (气压计随后锚定垂直) */

/* NOGNSS 播种先验 (2026-10-01 实测事故修正, 旧值 pos std=1e5m/yaw 半圆):
 *   - 位置: 配置坐标是城市级先验 (km 级误差)。旧值 1e5 m 使 P 对角
 *     (1e10 m^2) 与零偏状态 (~1e-11) 跨 21 个数量级, 传播/更新丢精度
 *     -> 播种后 6.5s 即现负对角; 且巨大位置先验 x yaw 半圆先验经 F 阵
 *     传播建立巨大交叉协方差, 磁航向更新的新息把位置/速度随机拖飞
 *     (实测位置 5min 被拉扯至 1190km, 速度虚假涨至 562m/s)。
 *   - yaw: 播种时用磁样本倾角补偿直接初始化 (levelMagHeading, 与 yaw
 *     无关), 先验收紧到 10deg —— 磁观测从第一步就低新息工作。
 *     无磁样本时退半圆先验 (180deg) 等磁收敛 (罕见, ginsaux 先于解算)。 */
#define GINS_NOGNSS_POS_STD_M       { 2000.0, 2000.0, 500.0 }  /* [n,e,u] m */
#define GINS_NOGNSS_YAW_STD_DEG     10.0    /* 磁初始化 yaw 的先验 std, deg */

/* 静止粗对准 (NOGNSS 等待窗 ≥10s 静止时启用, SHAKE 重开): 陀螺均值
 * 扣除地球自转 (配置纬度 + 对准姿态换算到体系) = 陀螺零偏直接测量,
 * 消除冷启动 "零偏先验 0 vs 真值+地转 ~15deg/h" 的最大先验失配;
 * roll/pitch 30s 平均噪声 ~0.01deg, 先验收紧至 0.3deg (加计零偏主导) */
#define GINS_NOGNSS_COARSE_MIN_N    10000   /* 粗对准最少静止样本数 (10s) */
/* 2026-10-01 实测: 本机 ADIS16505 稳态陀螺零偏 ~1900 deg/h (0.5deg/s,
 * 三轴分布, 随热漂移 1200->1900), EKF 先验 0±5 deg/h 偏离真值 240σ —
 * 零偏状态靠 ZUPT 硬拽收敛 (~1min), 是 NOGNSS 收敛慢的主导项。
 * 粗对准把等待窗均值作零偏先验直接种进去: 合理界按实测量级放宽,
 * 先验 std 自适应 (0.3x|测量|, 覆盖热漂移, 下限 = GINS_IMU_GB_STD) */
#define GINS_NOGNSS_GB_SANITY_DEGH  3000.0  /* 测量零偏合理性界, deg/h */
#define GINS_NOGNSS_PHI_STD_DEG     0.3     /* 粗对准 roll/pitch 先验 std, deg */
#define GINS_NOGNSS_VEL_STD_MPS     0.05    /* 静止播种速度先验 std, m/s */
/* 播种位置守卫 (偏发镜像瞬态自愈, 见 gins_bridge.cpp): 播种后侦测窗内
 * 偏离配置点超 KM 门限 (静止速度积分不可达) 拆引擎重播种 */
#define GINS_NOGNSS_POSGUARD_S      60.0    /* 侦测窗, s */
#define GINS_NOGNSS_POSGUARD_KM     100.0   /* 偏离门限, km */

/* ---------------- ZUPT 零速修正 (无 GNSS 静止部署) ----------------
 * 无 GNSS 时速度/水平姿态/陀螺零偏无可观测基准 (2026-10-01 实测: 静止板
 * 姿态 5min 漂 ~10deg -> 重力误投影 -> 速度积分爆炸)。桥接层静止检测
 * (原始 IMU 陀螺能量 + 加计模值偏差, EMA 平滑) 确认后周期注入 3 维零速
 * 观测: 钉住速度, 并使零偏/水平姿态可观 (经典静态对准可观测性)。
 * 仅 NOGNSS 模式注入 (GNSS 有效时 RMC 速度观测已提供 |v|≈0 强先验)。 */
#define GINS_ZUPT_ENABLE            1       /* 总开关 (0=桥接不注入) */
#define GINS_ZUPT_STD_MPS           0.05    /* 零速观测噪声 std, m/s (逐轴)。
                                             * 与 NOGNSS 播种速度先验一致; 收紧
                                             * 加速速度钉扎与零偏/姿态分离, 依赖
                                             * 静止粗对准的紧先验保证一致性 */
#define GINS_ZUPT_DT_S              0.1     /* 注入间隔, s (10Hz: 与磁/气压
                                             * 观测率匹配, 加速零偏可观测性
                                             * 建立; 3 维更新 ~0.85ms, 占空
                                             * 比 <1%; 0.5s 间隔时速度收敛
                                             * 需 ~120s, 10Hz 实测大幅缩短) */
/* 门限依据 2026-10-01 两轮实测静止台架 (EMA tau 0.5s): 上午环境
 * w²_ema≈1.7e-4~4.2e-4, 11:54 起环境振动源出现后 ≈6.3e-4 且瞬时
 * 越过 1e-3 —— 旧门限贴边导致 ZUPT 间歇暂停/全程失效 (10 轮回归的
 * 后 4 轮发散的直接原因)。取 4e-3: 对高振动环境 6x 裕量; 真实拿起/
 * 转动 w²≥3e-2 仍可靠检出; 盲区为 <3.6deg/s 的持续缓慢运动 —— 仅
 * NOGNSS 部署 (台架/地面静止场景) 生效, 飞行态 GNSS 正常不走此路径 */
#define GINS_ZUPT_GYRO_W2_MAX       4.0e-3  /* 陀螺能量 |w|^2 EMA 门限, (rad/s)^2 */
/* 加计模值偏差门限: 静止台架 | ‖f‖-g | 实测 0.02~0.18 随摆放姿态大幅
 * 变化 (2026-10-02: 本机 |f| 偏差 ±1.8%, 轴失准/比例因子量级 —— 远
 * 大于加计零偏 ~340mGal=0.0034 m/s², 零偏补偿救不了; 姿态不利时超
 * 旧门限 0.15, ZUPT 整轮冻结, 无 GNSS 速度自由积分实测发散至
 * 140/2673 m/s)。取 0.30 = 实测最坏 0.18 + 1.7x 裕量; 真实拿起/搬
 * 动线加速度 >> 0.5, 配合陀螺能量判据仍可靠判运动 */
#define GINS_ZUPT_ACC_DEV_MAX       0.30    /* 加计模值偏差 | ‖f‖-g | EMA 门限, m/s^2 */
#define GINS_ZUPT_EMA_TAU_S         0.5     /* 检测量 EMA 时间常数, s */
#define GINS_ZUPT_CONFIRM_MS        2000    /* 静止确认持续时长, ms (运动立即退出) */
/* 迟滞 (2026-10-01 运动注入实测): 判运动的退出门限取确认门限的 3 倍,
 * 防阈值边缘震荡反复暂停/确认; 真实搬动 |w|≥10deg/s (w²≥3e-2) 仍可靠
 * 检出 */
#define GINS_ZUPT_GYRO_W2_MOTION    1.2e-2  /* 判运动陀螺能量门限, (rad/s)^2 */
#define GINS_ZUPT_ACC_DEV_MOTION    0.60    /* 判运动加计模值偏差门限, m/s^2
                                             * (与确认门限 0.30 保持 2x 迟滞比) */
/* 运动恢复发散重播种: 再确认静止时 |v| 超此值 = 运动期间状态已发散
 * (NOGNSS 无观测可跟踪运动), ZUPT 3m/s 新息门 + 5连拒放1 拖回太慢
 * (实测 893 m/s 拖约半小时) —— 直接拆引擎重对准, 此刻静止, 粗对准
 * ~35s 内以当前朝向/当前零偏重建, 有界恢复 */
#define GINS_ZUPT_RESEED_V_MPS      1.0

/* ---------------- 气压高度观测 (BMP585) ---------------- */

/* 是否启用气压高度观测 (0=禁用, 引擎不使用气压计数据) */
#define GINS_BARO_ENABLE            1

/* 气压高度观测噪声标准差, m (BMP585 OSR8 噪声 ~0.1m, 计入测高模型误差)。
 * 与入滤频率配对定权: 10Hz 融合时样本相关性高, std 按采样数放大保持
 * 总信息量 (2Hz@1.5m ≈ 10Hz@3.0m), GNSS 垂直权威约 5:1 不变 */
#define GINS_BARO_STD_M             3.0

/* 气压高度新息门限, m: 超过判为气流冲击/模型失效, 拒绝本次观测 */
#define GINS_BARO_GATE_M            50.0

/* GNSS 有效时气压基准再锚定时间常数, s (跟踪天气漂移, 越大越惰性) */
#define GINS_BARO_ANCHOR_TAU_S      300.0

/* 气压观测入滤最小间隔, s (0=不限速): 气压模型每样本照常更新, 但 EKF
 * 融合限速。BMP585 ~100Hz 全量入滤会以信息速率压倒 GNSS 垂直通道
 * (10Hz), 塌缩垂直方差使 GNSS 失去绝对高度修正权 (2026-09-29 实测
 * 事故); 0.1s=10Hz 时锚定更频繁, 需与 GINS_BARO_STD_M 联动定权 */
#define GINS_BARO_FUSE_DT_S         0.1

/* ---------------- 辅助传感器采样 ---------------- */

/* 磁力计/气压计环形缓冲区 (middleware/Sensor_Preprocessing/process_data) 等待超时, ms (100Hz 生产者) */
#define GINS_AUX_POLL_MS            10

/* 观测样本采样时刻距解算时刻的最大允许滞后, s (超过判过旧丢弃并计数):
 * 磁/气压为本地 T_MCU 时标 (采样即打戳), 50ms 足够;
 * GNSS 观测时标是语句 UTC 经 PPS 映射回算, 含接收机 ~100-300ms 输出
 * 延迟 + 组句/解析排队 —— 50ms 门限会把位置更新几乎全部判过旧
 * (实测 gnss_stale 远超 gnss_cnt, EKF 位置辅助断供后纯惯性发散),
 * 引擎按观测时序入队处理, 0.5s 滞后无害 */
#define GINS_AUX_MAX_AGE_S          0.05
#define GINS_GNSS_MAX_AGE_S         0.5

/* GNSS 观测新息门禁, m: 观测位置与 INS 当前位置的偏差超过此值判为
 * 混缝句/异常定位, 丢弃并计数 (量程门禁只挡越界值, 挡不住"量程内"
 * 的缝合垃圾 —— 2026-09-29 实测高度 20km 级垃圾观测通过量程门禁把
 * pitch 拽到 52°, 解算发散到数千 km)。连续 5 次拒收后放行一次:
 * 真 GNSS 失锁重捕时 INS 已漂远, 不能把回来的好观测全拒门外 */
#define GINS_GNSS_INNOV_MAX_M       200.0

/* GNSS 水平速度观测 (RMC 地速/航迹角 -> N/E) 噪声标准差, m/s (0=不融合)。
 * RMC 地速分辨率 ~0.05m/s, 静止时 "速度≈0" 是强先验: 直接钉住启动速度
 * 暂态 (|v| 峰 ~4m/s), 加速 姿态/加计零偏/yaw 可观测性分离 (roll/pitch
 * 收敛 ~3min -> <1min, 2026-09-29 方案A)。低速时航迹角无效无害:
 * 地速≈0 时 N/E 分量误差以地速噪声为界 */
#define GINS_GNSS_VEL_STD_M         0.2

/* ---------------- GNSS 观测质量门禁 ----------------
 * 观测入滤前先验数据质量, 质量合格才进 EKF (与播种门禁同思路推广到
 * 运行期)。关键区分两类门禁:
 *   - 新息门 (上): 观测 vs INS 分歧 —— 谁错未知, 保留 5 连拒放行 1 次
 *     的恢复通道 (GNSS 失锁重捕/INS 漂远场景);
 *   - 质量门 (下): 数据本身的先验质量, 与 INS 状态无关 —— 绝对拒绝,
 *     不存在"放行一次垃圾"的理由。INS 已被污染时新息门失效, 质量门
 *     仍然有效; 真恢复场景接收机侧质量必然先回到门内, 无需放行垃圾。
 * 拒绝计数在 g_gq (SWD 取证), 同时计入 g_run.gnss_rej 全局口径 */
/*   1) HDOP 绝对上限: 正常 ~0.8-1.5, 半遮挡可用 ~3-5, >8 几何崩坏
 *      (定位连 Speed 野值都伴随), 整样本拒绝 */
#define GINS_GNSS_HDOP_MAX          8.0f
/*   2) 相邻有效观测跳变门限 = BASE + VMAX*dt (dt 为两样本 T_event 差):
 *      10Hz 相邻 fix 位置差超平台机动能力 = 接收机跳变/混缝句。
 *      dt 自然覆盖观测间断 (长间断后首样本门限自动放宽, 不阻碍重连) */
#define GINS_GNSS_JUMP_BASE_M       15.0
#define GINS_GNSS_JUMP_VMAX_MPS     60.0    /* 平台最大机动速度, m/s */

/* ---------------- 坏播种侦测与重对准 (种子门禁兜底) ----------------
 * "一致性野值"(接收机错锁, 输出内部自洽的错误位置) 能通过一切先验
 * 质量门 —— hdop/速度/稳定窗全绿 (2026-09-29 实测: 错锁 61km, 种子
 * 门禁零重置直接放行)。此类垃圾只能在播种后用后续观测印证: 播种后
 * 侦测窗内, 质量合格观测被新息门拒绝的比率 (EMA) 持续超阈即判坏
 * 播种, 拆引擎重开对准+稳定窗。错锁未恢复时周期重掷 (引擎重建顺带
 * 复位气压锚定, 遏制 baro 串扰垂直跑飞), 接收机恢复后即落好种 */
#define GINS_RESEED_GUARD_S         120.0   /* 播种后侦测窗, s */
#define GINS_RESEED_REJ_RATE        0.8     /* 新息拒收率 EMA 阈值 */
#define GINS_RESEED_EMA_TAU         5.0     /* 拒收率 EMA 时间常数, s */
#define GINS_RESEED_HOLD_S          10.0    /* 拒收率持续超阈时长阈值, s */
#define GINS_RESEED_MIN_N           30      /* 侦测窗内最少统计样本数 */

/* ---------------- 独立气压高度交叉校验 ----------------
 * 用 BMP585 气压高度做 GNSS 高度的独立基准 (与引擎/观测完全独立,
 * 不会被 GNSS 垃圾污染): h_baro = 44330·(1-(P_cal/101325)^0.1903),
 * P_cal 经 barocal 偏移校正 (未校准直通, 传感器本身绝对误差小)。
 * 基准含天气性偏差 (QNH 相对标准大气 ±10hPa ≈ ±80m) 与静压源误差,
 * 门限必须大于天气量级、小于实发故障量级 —— 实测种子 -8.4km、观测
 * 20km 垃圾、垂直跑飞 -1886km 全部 ≥900m, 250m 挡得住前者放得过
 * 后者。播种侧与运行期观测侧都校验; 气压链路未运行/样本过旧/量程
 * 异常时跳过校验 (fail-open, 不阻碍播种与观测) */
#define GINS_ALTX_H_M               250.0   /* 交叉校验门限, m */
#define GINS_ALTX_STALE_S           2.0     /* 气压基准最大允许时延, s */

/* INS 垂直失控看门狗: 垂直通道可在所有门禁"绿灯"下缓跑飞 (2026-09-29
 * 实测: 滤波协方差过度收敛 -> 增益趋零 -> 加计偏置以 ~0.17m/s 恒速拖走
 * 高度 +139m; GNSS 106m/baro 模型一致, 桥接新息 139m<200m 不触发,
 * reseed 守卫零拒收不触发 —— 三门全绿)。用同一独立气压参考监视
 * INS 自身: GNSS 新鲜时 |INS高度 - h_baro| 持续超 GINS_ALTX_H_M 达
 * GINS_VWATCH_HOLD_S 即拆引擎重对准。GNSS 过旧不判 (失锁滑行期
 * INS 高度靠 baro 阻尼是预期行为, 拆了就无法失锁重捕) */
#define GINS_VWATCH_HOLD_S          30.0    /* 超限持续时长阈值, s */

/* ---------------- NaN/Inf 看门狗处置 ----------------
 * 解出现非有限值时: (a) 发布侧消毒 —— 保留上一拍有限导航值并置 degraded,
 * 保证 gins_bridge_get_solution() 永不吐 NaN/Inf (输出线程格式化 NaN 曾
 * 爆栈 HardFault 冻结全系统); (b) 恢复 —— NaN 使新息门/vwatch/cov_warn
 * 的比较全部判假 (对 NaN 失明), 引擎状态 NaN 后无自愈路径, 连续
 * GINS_NAN_RESEED_N 拍 (1kHz 下 ~50ms) 非有限即拆引擎重对准。
 * 也覆盖启动竞态的初始 PVA NaN (第 3 拍全 NaN, 原需硬复位救) */
#define GINS_NAN_RESEED_N            50      /* 连续非有限拍数阈值 */

/* ---------------- Cholesky 失败恢复链 ----------------
 * P 阵失去正定后, 所有观测源 (GNSS 位置/速度/磁/气压) 的更新在
 * kf_math::update 的 Cholesky 处失败被静默丢弃 —— EKF 退化为纯惯导
 * 盲推, 与 cov_warn 同为"只有间接信号"且无自愈路径 (vreset 只治垂直
 * 新息不治根因)。自上次成功更新起连续失败 GINS_UPDFAIL_RESEED_N 次
 * (观测 ~20-30 次/s, 折合 ~4s) 即拆引擎重对准。偶发单次失败 (一次坏
 * 观测) 被下一次成功自然复位, 不会误触发 */
#define GINS_UPDFAIL_RESEED_N        100     /* 连续失败观测次数阈值 */

/* ---------------- reseed 风暴限幅 (A3) ----------------
 * 质量类自动重对准 (坏播种侦测/垂直失控看门狗) 在坏条件持续时会周期
 * 重掷 (拆->重播种->又坏->再拆), 解算可用性震荡。限幅策略: 两次实际
 * reseed 至少间隔 GINS_RESEED_MIN_IV_S x 退避倍数; 距上次 reseed 不足
 * GINS_RESEED_HEALTHY_RESET_S (同一事件没治好) 则退避倍数加倍至上限;
 * 退避期压制请求、保持当前解算 (引擎仍出有限数值)。毒化类触发
 * (NaN/Cholesky 失败) 不限幅 —— 引擎已死, 压制只会永久输出降级值 */
#define GINS_RESEED_MIN_IV_S         60.0    /* 两次 reseed 最小间隔, s */
#define GINS_RESEED_BACKOFF_MAX      8u      /* 指数退避倍数上限 (x8 = 480s) */
#define GINS_RESEED_HEALTHY_RESET_S  600.0   /* 距上次 reseed 超此值 = 独立事件, 退避复位 */

/* ---------------- 传感器标度 ---------------- */

/* RTK 固定解水平观测噪声 (C7): UM982 RTK FIX 标称 1cm+1ppm, 取保守
 * 5cm (含杆臂/对流层残余); 垂向按 2xUERE 惯例自动翻倍。仅 FIX 生效,
 * 浮点解维持 hdop*UERE 定权 */
#define GINS_RTK_FIX_STD_H_M        0.05

/* ---------------- 加计零偏持久化 (C8) ----------------
 * EKF 收敛后的零偏估值存 param_calib 镜像并保存 (W25Q64 calib 分区
 * 追加式记录, 温度标签), 下次上电
 * 作引擎初值+收紧先验: 初始对准误差从零偏主导的 ~0.7° 压到 ~0.2°,
 * 姿态/速度收敛同步提速。自动快照: 引擎就绪 GINS_ACCBIAS_SAVE_S 且
 * 静止 (|v|<门限) 时每上电一次; FinSH `accbias save|clear` 手动管理 */
#define GINS_ACCBIAS_SAVE_S         1800.0  /* 自动快照距引擎就绪的最短时长, s */
/* NOGNSS 台架部署的快照提前: 静止台架上零偏经 ZUPT 快速可观, 就绪后
 * 2min 快照 (下次上电作紧先验 15mGal, 压缩冷启动暂态); 快照仍受静止
 * 速度门限与合理性界约束 */
#define GINS_ACCBIAS_SAVE_NOGNSS_S  120.0
#define GINS_ACCBIAS_PRIOR_STD_MGAL 15.0    /* 有记录时的零偏先验 std (残余不确定度) */
#define GINS_ACCBIAS_BOUND_MGAL     500.0   /* 快照合理性界 (2026-10-01 实测本机
                                             * 收敛估值 ~-210mGal, 旧界 200 拒存) */
#define GINS_ACCBIAS_SPEED_MAX      0.5     /* 快照要求的静止水平速度, m/s */

/* IMU 名义输出数据率, Hz: dt = data_cnt 差分 / ODR (DEC_RATE 默认 1000Hz) */
#define GINS_IMU_ODR_HZ             1000

/* 陀螺灵敏度 (mdps/LSB) 探测失败时的缺省值 (ADIS16505-2 = ±500dps)
 * 正常运行时从 RANG_MDL(0x5E)[3:2] 自动探测, 见 gins_bridge.cpp */
#define GINS_GYRO_LSB_DEFAULT_MDPS  25.0
/* 加速度计标度: raw -> m/s^2, 与工程 main.c/VOFA 链路同源
 * (假设 ACC_RANGE 为缺省量程; 改过量程的必须同步修改) */
#define GINS_ACCEL_LSB_MSS          (78.0 / 32000.0)

/* ---------------- 独立看门狗 (IWDG1) ---------------- */

/* 看门狗超时, ms: gins 解算线程卡死/跑飞超过此时长硬件复位。
 * 复位后 ~7s 出解 / ~60s 全稳定 (收敛实测), 15s 留足恢复裕量;
 * LSI 偏差 ~±5%, 上限 32760 (RLR 12 位 @125Hz) */
#define GINS_WDT_TIMEOUT_MS         15000

/* ---------------- 静止对准 ---------------- */

/* 上电初始化时用加计平均对准 roll/pitch 的采样数 (1kHz)。
 * 3s 平均: 加计噪声平均后初始姿态误差 ~0.2deg (200 样本时实测 0.6deg);
 * 对准窗与等 GNSS 定位并发 (GINS_ALIGN_WAIT), 不增加出解时间 */
#define GINS_ALIGN_SAMPLES          3000
/* 对准期间允许的加计波动 (均值偏离 |g| 的最大幅度, m/s^2):
 * 超过视为不静止, 继续等待 */
#define GINS_ALIGN_ACC_TOL          0.5
/* 窗口满后均值的滑动跟踪时间常数, s: 等定位期可达数分钟 (冷启动),
 * 冻结在上电头 3s 的均值会把等待期的搬动/旋转仍判"静止"、以过时姿态
 * 播种; 滑动后搬动的加计瞬态立即触发 SHAKE 重开窗口, 静止后收敛到
 * 当前姿态。3s 与窗口长度同级 (平滑效果近似固定窗, 但始终"新鲜") */
#define GINS_ALIGN_TRACK_TAU_S      3.0

/* ---------------- 播种质量门禁 ----------------
 * "定位有效"不等于"定位收敛": UM982 冷启动首个 fix 可能是未收敛野值
 * (2026-09-29 两次实测: 播种偏真值 49km / 高度 -8.4km), 引擎种歪后
 * 新息门 (GINS_GNSS_INNOV_MAX_M) 持续拒收好观测, 每 6 次才放行 1 次,
 * 解算长时间锁死在错误位置。对准满窗后还须同时满足:
 *   1) hdop <= GINS_SEED_HDOP_MAX (几何精度达标, 缺省 fallback 2.0 可过);
 *   2) 静止速度一致: |v_h| <= GINS_SEED_SPEED_MAX (对准已判静止,
 *      GNSS 报出有感速度即收敛期噪声/野值);
 *   3) 连续 GINS_SEED_STABLE_N 个 10Hz 样本与窗口基准位置互差在
 *      GINS_SEED_STABLE_H/V_M 内 (首定位收敛漂移/跳变过不了稳定窗)。
 * 任一不满足持续 WAIT, 样本超差重开稳定窗; 重启前慢漂移兜底由
 * 新息门+放行通道自愈, 门禁只需拦住"种歪"量级的野值 */

/* 播种要求的最大 HDOP (UM982 开阔环境收敛后 ~0.8-1.5, 留裕量) */
#define GINS_SEED_HDOP_MAX          2.5f
/* 稳定窗样本数 (10Hz -> 1s 收敛确认) */
#define GINS_SEED_STABLE_N          10
/* 稳定窗内相对窗口基准的水平互差门限, m */
#define GINS_SEED_STABLE_H_M        10.0
/* 稳定窗内相对窗口基准的垂直互差门限, m (VDOP 差约 2 倍, 放宽) */
#define GINS_SEED_STABLE_V_M        15.0
/* 稳定期允许的最大 GNSS 水平速度, m/s (静止对准已过, 报出有感速度即异常) */
#define GINS_SEED_SPEED_MAX         1.5f

/* ---------------- 时间基准 ---------------- */

/* GPST 纪元 (1980-01-06 00:00:00 UTC) 与闰秒, 与 Protocol/nmea/um982_nmea 一致
 * 1980-01-06 的 Unix 秒 = 315964800 (此前误用 318729600 = 1980-02-07,
 * 恰好晚 32 天, 引擎时基整体偏移 2764800s) */
#define GINS_GPS_EPOCH_SEC          315964800u
#define GINS_UTC_LEAP_SEC           18u

/* 引擎时间不回绕: 用自 GPS 纪元起的累计秒 (week*604800 + sow) */
#define GINS_GPST_OF_UTC(utc_sec) \
    ((double)(unsigned long long)((utc_sec) - GINS_GPS_EPOCH_SEC + GINS_UTC_LEAP_SEC))

#endif /* __GINS_CONFIG_H__ */

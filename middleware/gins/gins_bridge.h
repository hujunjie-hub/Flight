/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * GINS 组合导航桥接层 (ADIS16505 + UM982 + BMM350 + BMP585 -> KF-GINS GIEngine)
 *
 * 硬件/数据流 (见 doc/Flight.xlsx 接口配置页):
 *   ADIS16505  1kHz DR 中断 (EXTI ISR 捕获 T_MCU 时戳) -> imu_data 环
 *   UM982      10Hz USART2 -> gnssrx 接收线程 -> gnss_raw_data 字节环
 *                             -> gnss_data 解析线程 (组句 + PPS 配对 +
 *                             UTC->T_MCU 换算) -> gnss_data 结构环
 *                             -> 本模块消费 (GNSS 位置观测)
 *   BMM350     100Hz I2C4   -> mag_data 环形缓冲区 (middleware/data,
 *                             入环前完成 校准->轴映射->干扰检查->低通,
 *                             cal 字段 + quality 干扰位)
 *                             -> ginsaux 线程消费 -> 磁航向观测 (干扰降权)
 *   BMP585     100Hz I2C2   -> baro_data 环形缓冲区 (middleware/data)
 *                             -> ginsaux 线程消费 + baro_calib 校准 -> 气压观测(Pa)
 *                                (Pa->高度换算在 GIEngine::baroUpdate 内压差式测高)
 *   本模块     1kHz 轮询快照 -> GIEngine (EKF) -> gins_bridge_get_solution()
 *
 * ---------------------------------------------------------------------------
 * 设计要点
 * ---------------------------------------------------------------------------
 * 1. 时间基准 (根 README "KF-GINS 融合约定"): 四类样本环 T_event 一律为
 *    T_MCU (TIM2 @1MHz, middleware/timebase), GNSS 样本由解析线程按 PPS
 *    映射 (clock_map) 把语句 UTC 换算为 T_event 后入环。引擎用"自 GPS
 *    纪元累计秒"(不回绕): 首个 T_event!=0 的 GNSS 样本构成锚点
 *    (T_MCU 锚点时刻 <-> utc -> GPST), 之后全部观测统一按 T_MCU 差分换算
 *    (单一单调时基, IMU dt 由 data_cnt 差分按名义 ODR 计算, 丢拍期 EKF
 *    按实际间隔积分)。映射未就绪 (GNSS 未定位/PPS 断接/失锁超时) 时
 *    gnss 样本 T_event=0, 本侧跳过 —— 不用"到达时刻"顶替; IMU/磁/气压
 *    观测不受影响 (T_MCU 独立走时)。滞后超过 GINS_AUX_MAX_AGE_S (50ms)
 *    的过旧样本丢弃并计数, 微小超前钳到当前时刻。
 *
 * 2. 初始化: 等 UM982 定位有效 + PPS 已同步 + 加计静止对准窗口满, 才构造
 *    GIEngine (heap new)。roll/pitch 由加计对准给出, yaw=0 (大初始方差),
 *    磁航向观测随后将 yaw 收敛到真值 (需配置当地磁偏角 GINS_MAG_DECL_DEG)。
 *
 * 3. 实时性: 1kHz 轮询 (tick=1ms), dt 由 DATA_CNTR 增量按名义 ODR 换算
 *    (本模块按样本 data_cnt 差分计算), 与线程唤醒抖动无关; 偶发丢拍
 *    (data_cntr 跳变) 只记统计。引擎内部 21x21 MatrixXd
 *    每毫秒定长分配/释放, 属常量尺寸模式, 对 RT-Thread 堆无碎片风险。
 *    磁/气压的 I2C 读取隔离在 middleware/data 的采集线程里, ginsaux 线程
 *    只消费环形缓冲区 (wait+pop; 磁校准/轴映射/低通已在 data 层入环前完成),
 *    均不占 1kHz 解算时间预算。
 *
 * 4. KF-GINS 上游仅做最小嵌入 (KF_GINS_EMBEDDED 宏): 去 iostream 打印与
 *    std::exit, 协方差异常改为告警钩子。滤波状态仍为 21 维, 磁/气压为
 *    1 维标量观测 (磁航向 -> yaw 误差; 气压高度 -> 天向位置误差)。
 *
 * FinSH 命令: gins  查看解算结果与状态。
 */

#ifndef __GINS_BRIDGE_H__
#define __GINS_BRIDGE_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 是否启用 (不跑组合导航时置 0, 不编入固件) */
#define GINS_BRIDGE_ENABLE          1

/* 解算线程 */
#define GINS_THREAD_PRIO            9       /* 低于 IMU DR 采样(6), 高于 VOFA(12) */
#define GINS_THREAD_STACK           8192    /* Eigen 局部对象较大 */
#define GINS_THREAD_TICK            1

/* 磁/气压数据线程 (消费 mag_data/baro_data 环形缓冲区 + 校准) */
#define GINS_AUX_THREAD_PRIO        10      /* 低于解算线程(9): 阻塞等待不占解算预算 */
#define GINS_AUX_THREAD_STACK       2048
#define GINS_AUX_THREAD_TICK        1

/* ---------------- 解算结果 (C POD, 供飞行控制/日志等消费) ---------------- */

struct gins_solution
{
    rt_bool_t   ready;          /* 引擎已初始化, 正在解算 */
    double      time;           /* GPST 累计秒 (自 GPS 纪元, 不回绕) */

    double      latitude;       /* deg */
    double      longitude;      /* deg */
    double      altitude;       /* m, 椭球高 */

    double      vn;             /* m/s, 北向 */
    double      ve;             /* m/s, 东向 */
    double      vd;             /* m/s, 地向 (NED, D 轴向下为正) */

    double      roll;           /* deg */
    double      pitch;          /* deg */
    double      yaw;            /* deg, KF-GINS 输出 [0,360), 非连续 (so3 误差解算对缠绕不敏感) */

    rt_uint32_t imu_cnt;        /* 已喂入引擎的 IMU 样本数 */
    rt_uint32_t gnss_cnt;       /* 已喂入引擎的 GNSS 观测数 */
    rt_uint32_t gnss_stale_cnt; /* 桥接侧因过旧丢弃的 GNSS 样本数 (>50ms) */
    rt_uint32_t gnss_age_ms;    /* 距最近一次 GNSS 观测 (按引擎时间) */
    rt_uint32_t cov_warn_cnt;   /* 协方差异常告警次数 */
    rt_uint32_t drop_cnt;       /* 检测到的 DR 丢拍数 (data_cntr) */
    double      gyro_lsb_mdps;  /* 探测到的陀螺灵敏度 */

    rt_uint32_t mag_cnt;        /* 已喂入引擎的磁力计观测数 */
    rt_uint32_t mag_rej_cnt;    /* 磁航向观测被拒绝次数 (门限/倾角/场强/过旧) */
    rt_uint32_t mag_stale_cnt;  /* 桥接侧因过旧丢弃的磁力计样本数 (>50ms) */
    rt_uint32_t baro_cnt;       /* 已喂入引擎的气压计观测数 */
    rt_uint32_t baro_rej_cnt;   /* 气压高度观测被拒绝次数 (门限/过旧) */
    rt_uint32_t baro_stale_cnt; /* 桥接侧因过旧丢弃的气压计样本数 (>50ms) */
    double      baro_height;    /* 引擎气压高度模型输出, m (与 altitude 对比看健康度) */
    double      mag_decl_deg;   /* 当前配置的磁偏角, deg */
    rt_uint32_t baro_skip_cnt;  /* 气压观测因入滤限速被跳过次数 (模型仍更新) */
    rt_uint32_t vreset_cnt;     /* 垂直新息持续超限触发协方差膨胀次数 */
    /* 追加字段 (末尾追加保持既有 SWD 读取偏移不变) */
    rt_uint32_t nan_cnt;        /* 引擎解出现过非有限值的累计次数 */
    rt_bool_t   degraded;       /* 1 = 当前导航字段为上一拍有限值 (引擎输出非有限, 已消毒) */
    rt_uint32_t updfail_cnt;    /* 观测更新 Cholesky 失败次数 (P 失健康, 持续失败触发重对准) */
    rt_uint32_t gnss_engdrop_cnt;   /* 引擎侧过旧静默丢弃的 GNSS 观测数 (与桥接侧 stale 分口径) */
    rt_uint32_t covheal_cnt;        /* 协方差对角自愈复位次数 (A4) */
    rt_uint32_t reseed_sup_cnt;     /* reseed 退避压制次数 (A3, 质量类触发被限幅) */
    /* 追加字段 (末尾追加保持既有 SWD 读取偏移不变) */
    rt_bool_t   nognss;         /* 1 = 无 GNSS 部署模式运行 (配置位置播种, 位置开环; GNSS 恢复稳定后自动转正常融合) */
    rt_uint32_t zupt_cnt;       /* 已注入的 ZUPT 零速观测数 (静止确认期计数; 更新/拒绝数看 FinSH `gins`) */
};

/* 启动解算线程 (INIT_ENV_EXPORT 自动执行; 失败原因看日志) */
rt_err_t gins_bridge_init(void);

/* 取最新解算结果快照 (任意线程/中断可用) */
void gins_bridge_get_solution(struct gins_solution *out);

/*
 * 暂停/恢复实时解算线程 (测试注入用): kf_math 的 EKF 工作矩阵为全局静态,
 * 外部构造第二个测试引擎前必须暂停实时引擎, 防止两引擎交替踩踏 P/Qc
 * 状态。暂停期间观测丢弃, 解算冻结; 恢复后继续 (协方差已被测试引擎
 * 复写, 导航精度需复位恢复)。
 * 现状核注 (2026-10-03): 原消费者为板上 utest 用例 tc_gins_engine,
 * utest 框架 2026-09-30 移除后暂无调用方, 机制保留备用。
 */
void gins_bridge_set_pause(rt_bool_t on);

#ifdef __cplusplus
}
#endif

#endif /* __GINS_BRIDGE_H__ */

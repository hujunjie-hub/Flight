/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * GINS 组合导航桥接实现 (设计要点见 gins_bridge.h 头注释)
 */

#include "gins_bridge.h"

#include <rtthread.h>
#include <rtdevice.h>
#include <math.h>
#include <string.h>

#include "gins_config.h"
#include "gins_options.hpp"

#include "um982_nmea.h"                     /* middleware/Protocol (FinSH 显示/宏) */
#include "gnss_data.h"                      /* middleware/Sensor_Preprocessing/process_data GNSS 环形缓冲区 */
#include "imu_data.h"                       /* middleware/Sensor_Preprocessing/process_data IMU 环形缓冲区 */
#include "sensor_adis16505.h"               /* middleware/Sensor_Drivers (陀螺灵敏度接口) */
#include "mag_data.h"                       /* middleware/Sensor_Preprocessing/process_data 环形缓冲区 */
#include "baro_data.h"                      /* middleware/Sensor_Preprocessing/process_data 环形缓冲区 */
#include "baro_calib.h"                     /* middleware/Sensor_Preprocessing/filter_calib */
#include "param_calib.h"                    /* middleware/Sensor_Preprocessing/param_calib C8 零偏持久化 (W25Q64) */
#include "param_nav.h"                      /* middleware/Sensor_Preprocessing/param_calib 导航参数镜像 (磁偏角/部署位置/观测开关) */
#include "timebase.h"                       /* middleware/Sensor_Drivers T_MCU 时基 */
#include "board.h"                          /* DWT */

#include "kf-gins/gi_engine.h"              /* KF-GINS 算法内核 (C++) */
#include "common/earth.h"                   /* WGS84_WIE (静止粗对准扣地转) */
#include "common/rotation.h"                /* euler2matrix (粗对准姿态换算) */
#include "kf_math.h"                        /* float32 EKF 内核耗时统计 */
#include "gins_wdt.h"                           /* IWDG1 独立看门狗 (飞控安全兜底) */

#define LOG_TAG "gins"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

#if GINS_BRIDGE_ENABLE

/* 对准检查结果 */
enum gins_align_res
{
    GINS_ALIGN_WAIT = 0,                    /* 窗口未满 / 静止但等定位: 保持 */
    GINS_ALIGN_SHAKE,                       /* 不静止: 重开窗口 */
    GINS_ALIGNED,                           /* 引擎已构造 */
};

static GIEngine *s_engine = RT_NULL;        /* 运行期构造, 不销毁 */

/* 引擎生命周期互斥: 只串行化 FinSH 读端与 reseed 拆建两侧。解算线程
 * 热路径不加锁 (它自身就是 reseed 执行者, 判空+解引用在同一线程内
 * 天然一致); FinSH 判空后解引用与 reseed delete 之间的 use-after-free
 * 窗口 (P2 优先级高于 FinSH, 随时可拆引擎) 由此关闭 */
static struct rt_mutex s_engine_lk;
static rt_bool_t s_engine_lk_ready = RT_FALSE;

static struct
{
    rt_uint32_t imu_cnt;
    rt_uint32_t gnss_cnt;
    rt_uint32_t cov_warn;
    rt_uint32_t drop_cnt;
    rt_uint32_t gnss_stale;                  /* 过旧被丢弃的 GNSS 样本数 */
    rt_uint32_t mag_cnt;                     /* 已喂入引擎的磁力计观测数 */
    rt_uint32_t baro_cnt;                    /* 已喂入引擎的气压计观测数 */
    rt_uint32_t mag_seq;                     /* 已消费的磁力计样本序号 */
    rt_uint32_t baro_seq;                    /* 已消费的气压计样本序号 */
    rt_uint32_t mag_stale;                   /* 过旧被丢弃的磁力计样本数 */
    rt_uint32_t baro_stale;                  /* 过旧被丢弃的气压计样本数 */
    double      gyro_lsb_mdps;
    double      now_gpst;                   /* 引擎时间 = 最近喂入 IMU 时刻 */
    double      gnss_time;                  /* 最近一次 GNSS 观测的引擎时间 */
    float       step_us_avg;                /* 单步解算耗时 (DWT 统计, EMA) */
    float       step_us_max;                /* 单步解算最大耗时 */
    /* 追加字段 (末尾追加保持既有字段 SWD 读取偏移不变) */
    rt_uint32_t gnss_rej;                   /* 合理性门禁拒绝的 GNSS 观测数 */
    float       dt_last;                    /* 最近一次 IMU dt, s */
    double      mag_time;                   /* 最近一次磁观测的引擎时间 */
    rt_uint32_t mag_rej;                    /* 模值门禁拒绝的磁样本数 */
    /* 追加字段 (末尾追加保持既有字段 SWD 读取偏移不变) */
    rt_uint32_t gnss_ts_zero;               /* T_event==0 被跳过的 GNSS 样本数 */
    rt_uint32_t gnss_late_us_max;           /* 已收样本传输延迟峰值 (T_arrival-T_event) */
    rt_uint32_t gnss_skip_mid;              /* 单拍多样本时被跳过的中间有效样本数 */
    /* 追加字段 (末尾追加保持既有字段 SWD 读取偏移不变) */
    rt_uint32_t zupt_cnt;                   /* 已注入引擎的 ZUPT 零速观测数 */
} g_run;

/* NaN 看门狗诊断: 首次 NaN 时刻冻结现场 (SWD 读取, 无串口依赖) */
static struct
{
    rt_uint32_t cnt;                        /* NaN 出现次数 */
    rt_uint32_t nan_mask;                   /* bit0..6: roll pitch yaw vn ve vd pos 先 NaN */
    rt_uint32_t imu_cnt;                    /* 首次 NaN 时 */
    rt_uint32_t gnss_cnt, mag_cnt, baro_cnt;
    rt_uint32_t drop_cnt, cov_warn, gnss_rej;
    double      gpst;                       /* 首次 NaN 引擎时间 */
    double      gnss_age_s, mag_age_s;
    float       dt_last;
    rt_uint32_t streak;                     /* 当前连续非有限拍数 (达阈值拆引擎) */
} g_nan;

/* 引擎时间锚点: 首个 T_event != 0 的 GNSS 样本 (PPS 映射就绪) 的
 * utc_sec/usec -> GPST 与其 T_event (T_MCU) 构成仿射锚点; 之后全部
 * 观测统一经 gins_gpst_of_tmcu() 换算, EKF 只面对单一单调时基
 * (根 README "KF-GINS 融合约定": 四类样本环的 T_event 一律为 T_MCU) */
static struct
{
    rt_uint64_t tmcu;                       /* 锚点 T_MCU, us */
    double      gpst;                       /* 锚点 GPST, s */
    rt_bool_t   valid;
} g_anchor;

static double gins_gpst_of_tmcu(rt_uint64_t tmcu)
{
    return g_anchor.gpst + (double)(rt_int64_t)(tmcu - g_anchor.tmcu) * 1e-6;
}

static struct gins_solution g_sol;          /* 对外快照 (关中断拷贝) */

/* ------------------------- 协方差告警钩子 ------------------------- */

/* gi_engine.h 在 KF_GINS_EMBEDDED 下调用, 代替 std::cout/std::exit */
extern "C" void kf_gins_cov_warning(double timestamp)
{
    g_run.cov_warn++;

    if (g_run.cov_warn == 1u || g_run.cov_warn % 1000u == 0u)
        LOG_W("KF-GINS 协方差对角出现负值 @%d.%03d s (累计 %u 次), 检查 IMU 噪声参数",
              (int)timestamp, (int)(timestamp * 1000.0) % 1000, g_run.cov_warn);
}

/* ------------------------- 大修正黑匣子钩子 ------------------------- */

/* stateFeedback 单拍位置修正 >100m 时由引擎调用 (播种后数值爆炸取证) */
extern "C" void kf_gins_dx_surge(double t, int src, double dpos,
                                 double dvel, double dphi)
{
    static const char *name[] = { "NONE", "GNSS", "MAG", "BARO", "ZUPT" };

    LOG_W("DX-SURGE t=%.3f src=%s |dpos|=%.0fm |dvel|=%.3fm/s |dphi|=%.4frad",
          t, name[src & 3], dpos, dvel, dphi);
}

/* ------------------------- 陀螺灵敏度 ------------------------- */
/* 从 ADIS 驱动读取 (RANG_MDL 上电时已自动识别)。驱动独占 SPI1 总线
 * (DR->DMA burst 链路), 本模块不再直接访问总线 */

static void gins_probe_gyro_range(void)
{
    g_run.gyro_lsb_mdps = adis16505_gyro_lsb_mdps();
    LOG_I("陀螺灵敏度 %.2f mdps/LSB (驱动 RANG_MDL 自动识别)",
          g_run.gyro_lsb_mdps);
}

/* ------------------------- 引擎初始化 ------------------------- */

/* 量程合理性: 解析残缺/异常值 (经纬度越界、高度/速度超物理量程)。
 * 观测门禁与播种门禁共用同一限值 */
static rt_bool_t gnss_sample_plausible(const struct gnss_sample *gs)
{
    return gs->latitude_deg > -90.1 && gs->latitude_deg < 90.1 &&
           gs->longitude_deg > -180.1 && gs->longitude_deg < 180.1 &&
           gs->altitude_m > -1e3 && gs->altitude_m < 3e4 &&
           (double)gs->vn > -5e2 && (double)gs->vn < 5e2 &&
           (double)gs->ve > -5e2 && (double)gs->ve < 5e2;
}

/*
 * 播种质量稳定窗 (gins_config.h "播种质量门禁", 遗留 #5):
 * 对准期逐个新 GNSS 样本评估收敛性, hdop/静止速度/窗内位置互差
 * 任一超差重开窗口; 窗口凑满 GINS_SEED_STABLE_N 才允许播种。
 * 计数器供 SWD/FinSH 取证: seq=进度, rej_*=各原因重开累计次数 */
static rt_bool_t gins_baro_alt_ref(rt_uint64_t tmcu_us, double *alt_out);

/*
 * 无 GNSS 部署模式状态 (gins_config.h "无 GNSS 部署模式"):
 * wait_s 为静止对准后无有效定位的累计等待 (有效 GNSS 到达即清零);
 * active 标记当前引擎为配置位置播种 (GNSS 恢复稳定后 reseed 转正常融合)。
 */
static struct
{
    double    wait_s;
    rt_bool_t active;
} g_nognss;

#if GINS_NOGNSS_MODE
/*
 * 静止粗对准陀螺累计 (gins_config.h "静止粗对准"): 对准窗满后 (等待
 * 定位/NOGNSS 超时期) 逐样本累计陀螺角增量均值, SHAKE 重开清零,
 * reseed 保留 (板仍静止, 均值仍有效)。NOGNSS 播种时以配置纬度+对准
 * 姿态扣除地球自转得到陀螺零偏先验 (静止平台免费的方向基准测量)。
 */
static struct
{
    double      sum[3];                     /* 角增量累计, rad */
    rt_uint32_t n;                          /* 累计样本数 */
} g_align_w;

/*
 * 播种位置守卫 (偏发镜像瞬态的自愈兜底): NOGNSS 播种后侦测窗
 * (GINS_NOGNSS_POSGUARD_S) 内, 位置偏离配置播种点超
 * GINS_NOGNSS_POSGUARD_KM —— 静止平台速度积分不可达的量级, 是状态
 * 被确定性搬走 (两次实测首行即镜像至 -2x 播种位置, 偏发未复现) 的
 * 特征 —— 即拆引擎重播种自愈; 走 reseed_request 限幅防重掷风暴。
 * GNSS 恢复转正常融合 (g_nognss.active 撤销) 后守卫自然失效。
 */
static struct
{
    rt_bool_t   armed;                      /* 侦测窗激活 (播种时武装) */
    double      t0;                         /* 武装时刻 (引擎时间) */
} g_posguard;
#endif /* GINS_NOGNSS_MODE */

#if GINS_ZUPT_ENABLE && GINS_NOGNSS_MODE
/*
 * ZUPT 静止检测 (gins_config.h "ZUPT 零速修正"): 逐 IMU 样本 (1kHz) 维护
 * 陀螺能量 |w|^2 与加计模值偏差 | ‖f‖-g | 的 EMA (tau=GINS_ZUPT_EMA_TAU_S),
 * 双判据连续满足 GINS_ZUPT_CONFIRM_MS 确认静止, 任一样本破坏立即退出。
 * EMA 初值取门限值: 上电起步视为"未知"不判静止, 静止时 ~1-2s 内降入门限。
 * 判据用原始传感器 (与解算速度无关 —— 解算速度正在发散正是要修的问题)。
 */
static struct
{
    double      w2_ema;                     /* |w|^2 EMA, (rad/s)^2 */
    double      fdev_ema;                   /* | ‖f‖-g | EMA, m/s^2 */
    rt_bool_t   active;                     /* 静止已确认 (破坏即撤) */
    rt_uint32_t hold_ms;                    /* 连续满足累计, ms */
    double      last_t;                     /* 上次注入的引擎时间, s */
} g_zupt =
{
    (double)GINS_ZUPT_GYRO_W2_MAX,
    (double)GINS_ZUPT_ACC_DEV_MAX,
    RT_FALSE, 0u, -1.0e9
};
#endif /* GINS_ZUPT_ENABLE && GINS_NOGNSS_MODE */

static struct
{
    rt_uint32_t seq;                        /* 稳定窗进度 0..GINS_SEED_STABLE_N */
    double      lat0, lon0, alt0;           /* 窗口基准位置 (窗内首样本) */
    rt_uint32_t rej_hdop;                   /* hdop 超限重开次数 */
    rt_uint32_t rej_speed;                  /* 静止速度超限重开次数 */
    rt_uint32_t rej_jump;                   /* 位置互差超限重开次数 */
    rt_uint32_t rej_sanity;                 /* 量程/有效性不通过次数 */
    rt_uint32_t rej_altx;                   /* 气压高度交叉校验不通过次数 */
} g_seed;

static void seed_track(const struct gnss_sample *gs)
{    double hdop, vh, dn, de;

    if (gs->fix_type == GNSS_FIX_INVALID || gs->utc_sec == 0u ||
        !gnss_sample_plausible(gs))
    {
        g_seed.rej_sanity++;
        g_seed.seq = 0;
        return;
    }

    hdop = (gs->hdop > 0.1f) ? (double)gs->hdop
                             : (double)UM982_NMEA_HDOP_FALLBACK;
    if (hdop > (double)GINS_SEED_HDOP_MAX)
    {
        g_seed.rej_hdop++;
        g_seed.seq = 0;
        return;
    }

    vh = sqrt((double)gs->vn * (double)gs->vn + (double)gs->ve * (double)gs->ve);
    if (vh > (double)GINS_SEED_SPEED_MAX)
    {
        g_seed.rej_speed++;                 /* 对准期静止, GNSS 报有感速度 */
        g_seed.seq = 0;
        return;
    }

    /* 独立气压高度交叉校验: GNSS 高度偏离 baro 基准 > 门限 = 垂直
     * 方向一致性垃圾 (慢收敛的高度假收敛稳定窗挡不住), 拒种 */
    {
        double ha;

        if (gins_baro_alt_ref(gs->T_event, &ha) &&
            fabs(gs->altitude_m - ha) > (double)GINS_ALTX_H_M)
        {
            g_seed.rej_altx++;
            g_seed.seq = 0;
            return;
        }
    }

    if (g_seed.seq == 0u)
    {
        g_seed.lat0 = gs->latitude_deg;
        g_seed.lon0 = gs->longitude_deg;
        g_seed.alt0 = gs->altitude_m;
        g_seed.seq  = 1u;
        return;
    }

    dn = (gs->latitude_deg - g_seed.lat0) * 111320.0;
    de = (gs->longitude_deg - g_seed.lon0) * 111320.0 * cos(g_seed.lat0 * D2R);
    if (dn * dn + de * de > GINS_SEED_STABLE_H_M * GINS_SEED_STABLE_H_M ||
        fabs(gs->altitude_m - g_seed.alt0) > GINS_SEED_STABLE_V_M)
    {
        g_seed.rej_jump++;                  /* 仍在收敛: 重开稳定窗 */
        g_seed.seq = 0;
        return;
    }

    g_seed.seq++;
}

/*
 * 观测质量门禁状态 (gins_config.h "GNSS 观测质量门禁"): 运行期入滤前
 * 先验数据质量 —— hdop 几何 + 相邻有效观测跳变。跳变基准只取"已入滤"
 * 样本 (垃圾样本不当基准); dt 缩放门限使长间断后首样本自动放行。
 * 与新息门的分工见配置注释: 质量不合格绝对拒绝, 新息分歧保留放行通道 */
static struct
{
    rt_bool_t   have_prev;                  /* 已有入滤基准 */
    double      lat, lon, alt;              /* 上一个入滤观测位置 */
    rt_uint64_t tmcu;                       /* 其 T_event (算 dt) */
    rt_uint32_t rej_hdop;                   /* hdop 超限拒绝累计 */
    rt_uint32_t rej_jump;                   /* 跳变超限拒绝累计 */
    rt_uint32_t rej_altx;                   /* 气压高度交叉校验拒绝累计 */
} g_gq;

/* 新息门连续拒收计数 (放行通道, 见观测门禁): file 作用域以便坏播种
 * 拆引擎时一并复位 */
static rt_uint8_t g_innov_rej;

/* 坏播种侦测 (gins_config.h "坏播种侦测与重对准"): 播种后侦测窗内
 * 统计"质量合格观测被新息门拒绝"的比率, EMA 持续超阈判坏播种 */
static struct
{
    rt_bool_t   armed;                      /* 侦测窗激活 (播种时武装) */
    double      t0;                         /* 武装时刻 (引擎时间) */
    double      ema;                        /* 新息拒收率 EMA */
    double      ema_t;                      /* 上次 EMA 更新时刻 */
    double      hold;                       /* 超阈持续时长, s */
    rt_uint32_t seen;                       /* 窗口内质量合格样本数 */
} g_guard;

static struct
{
    rt_uint32_t cnt;                        /* 坏播种拆引擎重对准次数 */
} g_reseed;

/* reseed 风暴限幅 (A3, gins_config.h "reseed 风暴限幅"): 坏条件持续时
 * 侦测-重对准会周期重掷 (拆引擎->重播种->又坏->再拆), 解算可用性震荡。
 * 质量类触发 (坏播种/垂直失控) 走 gins_reseed_request: 周期下限 + 指数
 * 退避, 退避期压制请求保持当前解算 (数值仍有限, 只是质量存疑);
 * 毒化类触发 (NaN/Cholesky) 不限幅直拆 —— 引擎已死时压制只会永久输出
 * 降级值, 重对准循环虽震荡但永不发布垃圾 */
static struct
{
    rt_tick_t   last_t;                     /* 上次实际 reseed 的 tick */
    rt_uint32_t backoff;                    /* 当前退避倍数 1..MAX */
    rt_uint32_t suppressed;                 /* 退避期被压制的请求数 */
} g_reseed_ctl;

/* 垂直失控看门狗 (gins_config.h "INS 垂直失控看门狗"): GNSS 新鲜时
 * 监视 INS 高度与独立气压参考的偏差, 持续超限拆引擎 —— 垂直通道
 * 滤波失健康 (协方差塌缩+加计偏置拖走) 时所有观测级门禁都绿灯,
 * 只有独立参考能裁判 */
static struct
{
    double      over_t;                     /* 当前超限累计时长, s */
    rt_uint32_t cnt;                        /* 触发次数 */
} g_vwatch;

/* Cholesky 失败恢复链状态 (gins_config.h "Cholesky 失败恢复链"):
 * 自上次成功观测更新起连续失败的更新次数 */
static struct
{
    rt_uint32_t streak;                     /* 连续失败次数 */
    rt_uint32_t last_ok;                    /* 上拍 updok 基准 */
    rt_uint32_t last_fail;                  /* 上拍 updfail 基准 */
} g_updfail;

/* 加计零偏快照状态 (C8): 引擎就绪计时 + 本上电是否已自动快照 */
static struct
{
    rt_tick_t   engine_tick;                /* 引擎就绪时刻 (0 = 未就绪) */
    rt_bool_t   saved;                      /* 本上电已自动快照 */
    rt_uint32_t auto_cnt;                   /* 自动快照累计 (跨上电) */
} g_accsnap;

/* 坏播种拆引擎复位: 引擎/时间锚点/观测跳变基准/种子稳定窗/放行通道
 * 全部回到上电态, 对准窗均值保留 (静止均值仍有效), 稳定窗在后续
 * GNSS 到达侧自动重走。引擎重建同时复位引擎内气压锚定, 遏制坏种子
 * 引发的 baro 串扰垂直跑飞 */
static void gins_reseed(void)
{
    if (s_engine != RT_NULL)
    {
        if (s_engine_lk_ready)
            rt_mutex_take(&s_engine_lk, RT_WAITING_FOREVER);
        delete s_engine;
        s_engine = RT_NULL;
        if (s_engine_lk_ready)
            rt_mutex_release(&s_engine_lk);
    }
    g_anchor.valid = RT_FALSE;
    memset(&g_gq, 0, sizeof(g_gq));
    g_seed.seq    = 0;
    g_innov_rej   = 0;
    g_sol.ready   = RT_FALSE;
    g_vwatch.over_t = 0.0;
    g_nan.streak  = 0;
    memset(&g_updfail, 0, sizeof(g_updfail));
    g_reseed.cnt++;
    g_nognss.active = RT_FALSE;
    g_nognss.wait_s = 0.0;
}

/*
 * 带风暴限幅的 reseed 入口 (质量类触发用): 两次实际拆引擎之间须留
 * GINS_RESEED_MIN_IV_S x 退避倍数; 距上次 reseed 不足健康判定时长
 * (说明上一次没治好, 属同一事件) 则退避加倍。压制期间计数 + 低频
 * 提示, 不拆不重启 (ready 维持, 解算照旧 —— 质量类触发引擎仍在出
 * 有限数值, 震荡式重对准并不更好)。距上次 reseed 足够久视为独立事件,
 * 退避复位。毒化类 (NaN/updfail) 不走此入口, 直接 gins_reseed()。
 *
 * force=TRUE 跳过压制窗口 (保留记账): 用于"引擎已被证明是垃圾"的
 * 触发 —— 2026-10-02 实测播种位置守卫在播种后 0.8s 内检出位置被
 * 搬 23088 km, 但距上次 reseed 30s < 60s 退避窗, 请求被压制, 带毒
 * 引擎 (位置滞留南太平洋) 跑完整个 330s 测试。压制的前提"引擎仍在
 * 出有限数值, 震荡不更好"对已炸引擎不成立; 且守卫每次播种只武装
 * 一次, 自然限幅 = 播种周期 (~35s)。
 */
static void gins_reseed_request(const char *why, rt_bool_t force)
{
    rt_tick_t now = rt_tick_get();
    rt_tick_t min_iv = (rt_tick_t)((double)GINS_RESEED_MIN_IV_S *
                                   (double)g_reseed_ctl.backoff * RT_TICK_PER_SECOND);

    if (!force && g_reseed_ctl.last_t != 0 &&
        (rt_tick_t)(now - g_reseed_ctl.last_t) < min_iv)
    {
        g_reseed_ctl.suppressed++;
        if ((g_reseed_ctl.suppressed & 0x7u) == 1u)
            LOG_W("reseed 请求被退避压制 (%s, 窗口 %.0fs, 累计 %u 次)",
                  why, (double)min_iv / RT_TICK_PER_SECOND,
                  g_reseed_ctl.suppressed);
        return;
    }

    if (g_reseed_ctl.last_t != 0 &&
        (rt_tick_t)(now - g_reseed_ctl.last_t) <
            (rt_tick_t)((double)GINS_RESEED_HEALTHY_RESET_S * RT_TICK_PER_SECOND))
        g_reseed_ctl.backoff <<= 1;         /* 短周期连发: 同一事件没治好 */
    else
        g_reseed_ctl.backoff = 1;           /* 距上次够久: 独立事件 */
    if (g_reseed_ctl.backoff > GINS_RESEED_BACKOFF_MAX)
        g_reseed_ctl.backoff = GINS_RESEED_BACKOFF_MAX;
    g_reseed_ctl.suppressed = 0;
    g_reseed_ctl.last_t = now;

    LOG_W("reseed: %s (第 %u 次, 退避 x%u)",
          why, g_reseed.cnt + 1u, g_reseed_ctl.backoff);
    gins_reseed();
}

/*
 * 加计零偏快照 (C8): 取引擎 EKF 当前零偏估值, 过合理性界后写入
 * param_calib 镜像并保存 (W25Q64 calib 分区追加式记录)。temp 为 IMU 内部温度标签 (°C)。
 * force=TRUE 跳过静止/时长条件 (FinSH 手动路径, 仍受合理性界约束)。
 */
static rt_err_t gins_accbias_snapshot(double temp_c, rt_bool_t force)
{
    struct calib_data *cal;
    NavState ns;
    double mgal[3];
    int i;

    {
        /* FinSH 侧 (accbias save) 与解算线程 (自动快照) 都会进入本函数;
         * 引擎生命周期锁关闭 FinSH 判空后 reseed 拆引擎的 UAF 窗口 */
        if (s_engine_lk_ready)
            rt_mutex_take(&s_engine_lk, RT_WAITING_FOREVER);
        if (s_engine == RT_NULL)
        {
            if (s_engine_lk_ready)
                rt_mutex_release(&s_engine_lk);
            return -RT_EEMPTY;
        }
        ns = s_engine->getNavState();
        if (s_engine_lk_ready)
            rt_mutex_release(&s_engine_lk);
    }

    if (!force)
    {
        double need_s = g_nognss.active ? (double)GINS_ACCBIAS_SAVE_NOGNSS_S
                                        : (double)GINS_ACCBIAS_SAVE_S;

        if (g_accsnap.engine_tick == 0 || g_accsnap.saved)
            return -RT_EBUSY;
        if ((rt_tick_t)(rt_tick_get() - g_accsnap.engine_tick) <
            (rt_tick_t)(need_s * RT_TICK_PER_SECOND))
            return -RT_ETIMEOUT;
        if (fabs(g_sol.vn) > (double)GINS_ACCBIAS_SPEED_MAX ||
            fabs(g_sol.ve) > (double)GINS_ACCBIAS_SPEED_MAX ||
            fabs(g_sol.vd) > (double)GINS_ACCBIAS_SPEED_MAX)
            return -RT_EPERM;                  /* 未静止: 下拍再试 */
    }

    ns = s_engine->getNavState();
    for (i = 0; i < 3; i++)
    {
        mgal[i] = ns.imuerror.accbias[i] * 1e5;      /* m/s^2 -> mGal */
        if (!(fabs(mgal[i]) < (double)GINS_ACCBIAS_BOUND_MGAL))
        {
            LOG_W("加计零偏快照拒绝: 轴 %d = %.1f mGal 超界 (未收敛?)",
                  i, mgal[i]);
            return -RT_EINVAL;
        }
    }

    cal = calib_store_ram();
    for (i = 0; i < 3; i++)
        cal->acc_bias_mgal[i] = (float)mgal[i];
    cal->acc_cal_temp = (float)temp_c;
    cal->acc_valid = RT_TRUE;

    {
        rt_err_t err = calib_store_save();

        if (err != RT_EOK)
            return err;
    }
    g_accsnap.saved = RT_TRUE;
    g_accsnap.auto_cnt++;
    LOG_I("加计零偏快照已保存: (%.1f, %.1f, %.1f) mGal @%.1f°C "
          "(下上电作引擎初值, 先验 std %.0f mGal)",
          mgal[0], mgal[1], mgal[2], temp_c,
          (double)GINS_ACCBIAS_PRIOR_STD_MGAL);
    return RT_EOK;
}

/*
 * 最新磁样本槽 (关中断保护的单槽快照): 定义前置 —— gins_seed_engine 的
 * NOGNSS 磁航向播种段与 gins_aux_thread_entry (生产) 都访问它。
 */
static struct gins_aux_mag_slot
{
    rt_uint32_t seq;                        /* 0=无样本, 每次发布 +1 */
    rt_uint64_t tmcu_us;                    /* 采样时刻 (T_MCU µs, mag_data 打戳) */
    rt_uint8_t  quality;                    /* 质量标志 (bit0 干扰 -> 降权) */
    float mag_ut[3];                        /* µT, 体坐标系 FRD, 校准+轴映射+低通后 */
} s_aux_mag;

/*
 * 引擎播种 (对准完成后的公共构造段): 位置/速度来源可参数化 ——
 * 正常路径取 UM982 收敛定位, 无 GNSS 部署取 gins_config.h 配置值;
 * roll/pitch <- 加计对准, yaw = 0 (180° 先验, 磁航向观测收敛拉正)。
 * anchor_gpst: 锚点 GPST (正常路径 = 首个有效 GNSS 样本 UTC 换算,
 * NOGNSS 路径 = 0, EKF 只依赖单调相对时间)。
 * 返回 1 = 播种成功; 0 = 引擎构造失败 (调用方 WAIT 下拍重试);
 * -1 = 初始 PVA 终检不通过 (调用方 SHAKE 重开窗口)。
 */
static rt_int8_t gins_seed_engine(double lat_deg, double lon_deg, double alt_m,
                                  double vn, double ve, double vd,
                                  double anchor_gpst,
                                  const struct imu_sample *s0,
                                  const double f_sum[3], rt_uint32_t n,
                                  const double w_mean[3], rt_uint32_t w_n,
                                  double dt, rt_bool_t nognss)
{
    double f_mean[3], roll_deg, pitch_deg;

    for (int i = 0; i < 3; i++)
        f_mean[i] = f_sum[i] / (double)n;

    {
        /* C8: 持久化的加计零偏作引擎初值 (收敛估值快照, 先验收紧),
         * 无记录时 build_options 内部用缺省 0 + 冷启动先验 */
        struct calib_data *cal = calib_store_ram();
        double acc_mgal[3] = { (double)cal->acc_bias_mgal[0],
                               (double)cal->acc_bias_mgal[1],
                               (double)cal->acc_bias_mgal[2] };
        const double *acc_ptr = cal->acc_valid ? acc_mgal : 0;

        GINSOptions opt = gins_build_options(lat_deg, lon_deg, alt_m,
                                             vn, ve, vd,
                                             f_mean,
                                             acc_ptr,
                                             (double)GINS_ACCBIAS_PRIOR_STD_MGAL);

        if (nognss)
        {
            /* 配置位置是城市级先验 (km 级误差, 无 GNSS 不可观): 位置先验取
             * km 级宽 (防小 std 制造虚假位置-姿态耦合), 但不可过宽 —— 旧值
             * 1e5 m 使 P 对角跨 21 个数量级, 传播/更新丢精度出负对角, 且
             * 与 yaw 半圆先验的交叉项把磁航向新息传导到位置/速度
             * (2026-10-01 实测位置被拉扯至 1190km), 详见 gins_config.h
             * "NOGNSS 播种先验" 注释 */
            double pstd[3] = GINS_NOGNSS_POS_STD_M;
            opt.initstate_std.pos << pstd[0], pstd[1], pstd[2];

            /* 静止粗对准生效 (SHAKE 门控, 窗内持续静止 ≥10s):
             * roll/pitch 30s 加计平均噪声 ~0.01deg (振动余量后 0.3deg),
             * 速度 0 强先验收紧至与 ZUPT 同级 —— 先验一致才能快速收敛 */
            if (w_n >= GINS_NOGNSS_COARSE_MIN_N)
            {
                opt.initstate_std.euler[0] = (double)GINS_NOGNSS_PHI_STD_DEG * D2R;
                opt.initstate_std.euler[1] = (double)GINS_NOGNSS_PHI_STD_DEG * D2R;
                opt.initstate_std.vel      = Eigen::Vector3d::Constant(
                    (double)GINS_NOGNSS_VEL_STD_MPS);
            }
        }

        rt_bool_t yaw_seeded = RT_FALSE;

        if (nognss && param_nav()->mag_enable)
        {
            /* yaw 磁航向倾角补偿直接初始化 (levelMagHeading 只用 roll/pitch,
             * 与 yaw 无关): 消除半圆先验及其与位置先验的巨大交叉项, 磁观测
             * 从第一步就低新息工作。样本要求: 有磁样本、距播种时刻 < 2s、
             * 模值在物理量程 (同 gins_aux_feed 磁门禁); 不满足退半圆先验
             * (引擎内 magUpdate 会拉收敛, 罕见路径)。 */
            struct gins_aux_mag_slot m;
            double mnorm, roll_rad, pitch_rad, ps;
            rt_int64_t d_us;
            rt_base_t level;

            level = rt_hw_interrupt_disable();
            m = s_aux_mag;
            rt_hw_interrupt_enable(level);

            mnorm = sqrt((double)m.mag_ut[0] * (double)m.mag_ut[0] +
                         (double)m.mag_ut[1] * (double)m.mag_ut[1] +
                         (double)m.mag_ut[2] * (double)m.mag_ut[2]);
            d_us = (rt_int64_t)(s0->T_event - m.tmcu_us);

            ps = f_mean[0] / 9.80665;                 /* pitch = asin(fx/g) */
            if (ps > 1.0) ps = 1.0;
            else if (ps < -1.0) ps = -1.0;
            roll_rad  = atan2(-f_mean[1], -f_mean[2]);
            pitch_rad = asin(ps);

            if (m.seq != 0u && d_us >= -100000 && d_us <= 2000000 &&
                mnorm > 5.0 && mnorm < 300.0)
            {
                Eigen::Vector3d magv((double)m.mag_ut[0], (double)m.mag_ut[1],
                                     (double)m.mag_ut[2]);

                opt.initstate.euler[2]     = GIEngine::levelMagHeading(
                    magv, roll_rad, pitch_rad,
                    (double)param_nav()->mag_decl_deg * D2R);
                opt.initstate_std.euler[2] = (double)GINS_NOGNSS_YAW_STD_DEG * D2R;
                yaw_seeded = RT_TRUE;
            }
        }

        rt_bool_t gb_seeded = RT_FALSE;

#if GINS_NOGNSS_MODE
        if (nognss && yaw_seeded && w_n >= GINS_NOGNSS_COARSE_MIN_N)
        {
            /* 静止粗对准陀螺零偏先验: 等待窗陀螺均值 = 零偏 + 地球自转
             * 体系投影; 用配置纬度与对准姿态 (加计 roll/pitch + 磁 yaw,
             * yaw 误差 ~10deg 只影响水平地转分量 ~2.4deg/h, 计入先验
             * std 3deg/h) 扣除地转即得零偏测量。消除冷启动 "零偏先验 0
             * vs 真值+地转 ~15deg/h" 的先验失配 —— 该失配正是 ZUPT 期
             * 速度暂态与深收敛慢的主导项 */
            Eigen::Vector3d wmean(w_mean[0], w_mean[1], w_mean[2]);
            Eigen::Vector3d wie_n(WGS84_WIE * cos(lat_deg * D2R), 0.0,
                                  -WGS84_WIE * sin(lat_deg * D2R));
            Eigen::Vector3d gb = wmean -
                Rotation::euler2matrix(opt.initstate.euler).transpose() * wie_n;

            if (gb.norm() < (double)GINS_NOGNSS_GB_SANITY_DEGH * D2R / 3600.0)
            {
                double gb_norm_degh = gb.norm() * R2D * 3600.0;
                double gb_std_degh  = 0.3 * gb_norm_degh;

                if (gb_std_degh < (double)GINS_IMU_GB_STD)
                    gb_std_degh = (double)GINS_IMU_GB_STD;

                opt.initstate.imuerror.gyrbias = gb;
                opt.initstate_std.imuerror.gyrbias = Eigen::Vector3d::Constant(
                    gb_std_degh * D2R / 3600.0);
                gb_seeded = RT_TRUE;
            }
        }
#endif /* GINS_NOGNSS_MODE */

        IMU imu0;

        /* 初始 PVA 终检 (有限+量程; NaN/Inf 与任何比较均为假, 一并拒):
         * 上游各门禁的最后一道兜底 —— 偶发启动竞态曾让引擎拿到带
         * NaN/垃圾的初值, 第 3 拍即全 NaN (mask=0x7f), 现由 NaN 自愈
         * 兜底但每次损失一次对准+播种周期, 在源头拦掉最省 */
        if (!(fabs(opt.initstate.pos[0]) < 1.6 &&
              fabs(opt.initstate.pos[1]) < 3.2 &&
              fabs(opt.initstate.pos[2]) < 20000.0 &&
              fabs(opt.initstate.vel[0]) < 500.0 &&
              fabs(opt.initstate.vel[1]) < 500.0 &&
              fabs(opt.initstate.vel[2]) < 500.0 &&
              fabs(opt.initstate.euler[0]) < 4.0 &&
              fabs(opt.initstate.euler[1]) < 4.0 &&
              fabs(opt.initstate.euler[2]) < 7.0))
        {
            LOG_W("初始 PVA 终检不通过, 重开对准 (lat=%.4f lon=%.4f alt=%.1f)",
                  opt.initstate.pos[0] * 57.2958, opt.initstate.pos[1] * 57.2958,
                  opt.initstate.pos[2]);
            return -1;                           /* 调用方按 SHAKE 处理 */
        }

        s_engine = new GIEngine(opt);
        if (s_engine == RT_NULL)
        {
            /* -fno-exceptions 下 aligned_new 的 OOM 语义是返回 NULL,
             * 不判空则下一行解引用直接 HardFault; 返回让调用方下拍重试 */
            LOG_E("KF-GINS 引擎构造失败 (堆耗尽?), 对准保持, 下拍重试");
            return 0;
        }

        /* 引擎时间锚定: 正常路径 = 首个有效 GNSS 样本的 UTC -> GPST 与其
         * T_event (T_MCU) 构成仿射锚点; NOGNSS = GPST 0 起点 (EKF 只依赖
         * 单调相对时间, 无 GNSS 观测不涉绝对时刻换算) */
        g_anchor.tmcu  = s0->T_event;
        g_anchor.gpst  = anchor_gpst;
        g_anchor.valid = RT_TRUE;

        g_run.now_gpst  = gins_gpst_of_tmcu(s0->T_event);
        if (!nognss)
            g_run.gnss_time = g_run.now_gpst;

        /* 播种当前样本 (桌面主循环 addImuData(imu_cur, true) 同款):
         * 使 imupre_ 合法, 下一拍才开始传播。
         * s0 的角增量直接用样本值 (imu_data 已完成单位换算+轴映射) */
        imu0.time   = g_run.now_gpst;
        imu0.dt     = dt;
        imu0.dtheta << (double)s0->gyro[0] * dt, (double)s0->gyro[1] * dt, (double)s0->gyro[2] * dt;
        imu0.dvel   << f_mean[0] * dt, f_mean[1] * dt, f_mean[2] * dt;
        imu0.odovel = 0.0;
        s_engine->addImuData(imu0, true);

        g_sol.ready         = RT_TRUE;
        g_sol.gyro_lsb_mdps = g_run.gyro_lsb_mdps;
        g_accsnap.engine_tick = rt_tick_get();
        g_nognss.active       = nognss;
#if GINS_ZUPT_ENABLE && GINS_NOGNSS_MODE
        if (nognss)
            g_zupt.last_t = -1.0e9;    /* 引擎重建: 立即允许 ZUPT 注入 */
#endif

#if GINS_NOGNSS_MODE
        /* 播种位置守卫武装: NOGNSS 播种后侦测窗内位置偏离配置点超
         * 100 km (速度积分不可达, 镜像/跳变特征) 即拆引擎重播种 */
        if (nognss)
        {
            g_posguard.armed = RT_TRUE;
            g_posguard.t0    = g_run.now_gpst;
        }
#endif

        /* 武装坏播种侦测窗 (拆引擎重对准的兜底, 见 gins_config.h):
         * 引擎时间刚锚定, t0 与后续样本同源 */
        g_guard.armed  = RT_TRUE;
        g_guard.t0     = g_run.now_gpst;
        g_guard.ema    = 0.0;
        g_guard.ema_t  = g_run.now_gpst;
        g_guard.hold   = 0.0;
        g_guard.seen   = 0;

        roll_deg  = atan2(-f_mean[1], -f_mean[2]) * (180.0 / M_PI);
        pitch_deg = asin(f_mean[0] / 9.80665) * (180.0 / M_PI);

        if (nognss)
        {
            LOG_I("KF-GINS 初始化 (无 GNSS 部署): 配置位置 %.4f/%.4f/%.1fm, "
                  "r=%.2f p=%.2f y=%.2f deg (%s), pos std {%g,%g,%g} m, "
                  "时基锚定 T_MCU (GPST 0 起点)",
                  lat_deg, lon_deg, alt_m, roll_deg, pitch_deg,
                  opt.initstate.euler[2] * R2D,
                  yaw_seeded ? "磁航向初始化" : "半圆先验, 待磁收敛",
                  opt.initstate_std.pos[0], opt.initstate_std.pos[1],
                  opt.initstate_std.pos[2]);
            if (gb_seeded)
                LOG_I("静止粗对准: 陀螺均值 %us, 零偏先验 "
                      "(%.1f, %.1f, %.1f) deg/h (std %.0f), "
                      "r/p 先验 %.1fdeg, v 先验 %.2f m/s",
                      w_n / 1000u,
                      opt.initstate.imuerror.gyrbias[0] * R2D * 3600.0,
                      opt.initstate.imuerror.gyrbias[1] * R2D * 3600.0,
                      opt.initstate.imuerror.gyrbias[2] * R2D * 3600.0,
                      opt.initstate_std.imuerror.gyrbias[0] * R2D * 3600.0,
                      (double)GINS_NOGNSS_PHI_STD_DEG,
                      (double)GINS_NOGNSS_VEL_STD_MPS);
        }
        else
        {
            LOG_I("KF-GINS 初始化: r=%.2f p=%.2f deg, 引擎时间锚定 GPST",
                  roll_deg, pitch_deg);
        }
    }
    return 1;
}

/*
 * 对准窗口满时检查: 不静止 -> SHAKE (重开窗口);
 * 静止且定位收敛稳定 -> 构造引擎 ALIGNED; 静止但等定位/稳定窗 -> WAIT (保持窗口)。
 *   位置/速度 <- UM982 收敛定位, roll/pitch <- 加计对准, yaw = 0。
 * 无 GNSS 部署 (GINS_NOGNSS_MODE): 静止后等定位超时 -> 配置位置播种。
 */
static enum gins_align_res gins_try_init_engine(const struct gnss_sample *gs,
                                                const struct imu_sample *s0,
                                                const double f_sum[3], rt_uint32_t n,
                                                double dt)
{
    double f_mean[3], norm;

    for (int i = 0; i < 3; i++)
        f_mean[i] = f_sum[i] / (double)n;

    /* NaN 防护: 窗口内出现 NaN 样本时 norm=NaN, 下方比较 NaN>TOL 为假
     * 会被误判"静止"继续播种 -> 引擎输出全 NaN (实测 mask=0x7f)。
     * NaN 比较仅对自身成立 (x!=x), 显式检查并重开对准窗口。 */
    for (int i = 0; i < 3; i++)
        if (f_mean[i] != f_mean[i])
            return GINS_ALIGN_SHAKE;

    norm = sqrt(f_mean[0] * f_mean[0] + f_mean[1] * f_mean[1] + f_mean[2] * f_mean[2]);
    if (!(norm > 0.0) || fabs(norm - 9.80665) > GINS_ALIGN_ACC_TOL)
        return GINS_ALIGN_SHAKE;                /* 不静止 */

    /* gnss_data 环内样本均为有效位置更新 (协议层 update_cnt 门控),
     * fix_type 再兜底; utc_sec==0 表示尚未建立日期基准;
     * T_event==0 表示时间映射未就绪 (PPS 滑窗未满/作废) */
    if (gs->fix_type == GNSS_FIX_INVALID || gs->utc_sec == 0u || gs->T_event == 0u)
    {
#if GINS_NOGNSS_MODE
        /* 无 GNSS 部署: 静止但持续无定位, 超时后用配置位置播种
         * (位置开环, yaw 靠磁航向; GNSS 恢复稳定后自动 reseed 转正常融合) */
        g_nognss.wait_s += dt;
        if (g_nognss.wait_s >= (double)GINS_NOGNSS_WAIT_S)
        {
            /* 磁就绪门 (有界): 无新鲜磁样本不播种 —— yaw 半圆先验是
             * 播种后状态爆炸源 (2026-10-02 实测, 见 gins_config.h
             * GINS_NOGNSS_MAG_WAIT_S 注释)。新鲜判据与播种 yaw 采样
             * 相同 (±2s 窗); 超时仍无磁才退半圆照播 */
            {
                struct gins_aux_mag_slot m;
                rt_int64_t mag_age_us;
                rt_base_t level;

                level = rt_hw_interrupt_disable();
                m = s_aux_mag;
                rt_hw_interrupt_enable(level);
                mag_age_us = (rt_int64_t)s0->T_event - (rt_int64_t)m.tmcu_us;

                if (m.seq == 0u || mag_age_us < -100000 || mag_age_us > 2000000)
                {
                    if (g_nognss.wait_s < (double)GINS_NOGNSS_WAIT_S +
                                          (double)GINS_NOGNSS_MAG_WAIT_S)
                    {
                        return GINS_ALIGN_WAIT;     /* 磁未就绪, 有界再等 */
                    }
                }
            }

            /* 静止粗对准均值 (窗内持续静止 ≥GINS_NOGNSS_COARSE_MIN_N 才有效) */
            double w_mean[3] = { 0.0, 0.0, 0.0 };

#if GINS_NOGNSS_MODE
            if (g_align_w.n >= GINS_NOGNSS_COARSE_MIN_N)
            {
                w_mean[0] = g_align_w.sum[0] / (double)g_align_w.n;
                w_mean[1] = g_align_w.sum[1] / (double)g_align_w.n;
                w_mean[2] = g_align_w.sum[2] / (double)g_align_w.n;
            }
#endif
            rt_int8_t r = gins_seed_engine(param_nav()->nognss_lat,
                                           param_nav()->nognss_lon,
                                           param_nav()->nognss_alt,
                                           0.0, 0.0, 0.0,
                                           0.0, s0, f_sum, n,
                                           w_mean, g_align_w.n, dt, RT_TRUE);

            if (r == 1)
                return GINS_ALIGNED;
            if (r < 0)
                return GINS_ALIGN_SHAKE;    /* 配置 PVA 异常, 重开窗口 */
            return GINS_ALIGN_WAIT;         /* 构造失败, 下拍重试 */
        }
#endif /* GINS_NOGNSS_MODE */
        return GINS_ALIGN_WAIT;             /* 静止, 等 GNSS/时间映射 */
    }

#if GINS_NOGNSS_MODE
    g_nognss.wait_s = 0.0;                  /* 有效定位到达: 撤销 NOGNSS 计时 */
#endif

    /* 播种样本合理性门禁 (与观测门禁同限): RX 劣化期的混缝句可能带
     * 有效 fix 标志但坐标残缺 (2026-09-29 实测播种到 16.1N/109.5E/
     * -1946km, 之后观测全被卡方门限拒绝, 解算永久锁死在错误位置) */
    if (!gnss_sample_plausible(gs))
        return GINS_ALIGN_WAIT;                 /* 等下一个干净样本 */

    /* 播种质量门禁: "定位有效"不等于"定位收敛", 冷启动首定位野值种歪
     * 引擎后新息门解不回来 (实测偏 49km/高度 -8.4km)。稳定窗由
     * seed_track() 在 GNSS 到达侧逐样本评估, 这里只查进度 */
    if (g_seed.seq < GINS_SEED_STABLE_N)
        return GINS_ALIGN_WAIT;                 /* 等定位收敛稳定 */

    {
        double w_zero[3] = { 0.0, 0.0, 0.0 };
        rt_int8_t r = gins_seed_engine(gs->latitude_deg, gs->longitude_deg,
                                       gs->altitude_m,
                                       gs->vn, gs->ve, 0.0,
                                       GINS_GPST_OF_UTC(gs->utc_sec) +
                                       (double)gs->utc_usec * 1e-6,
                                       s0, f_sum, n,
                                       w_zero, 0u, dt, RT_FALSE);

        if (r < 0)
            return GINS_ALIGN_SHAKE;            /* 初始 PVA 终检不通过 */
        if (r == 0)
            return GINS_ALIGN_WAIT;             /* 引擎构造失败, 下拍重试 */
    }
    return GINS_ALIGNED;
}

/* ------------------------- 解算结果发布 ------------------------- */

static void gins_publish(void)
{
    struct gins_solution s = g_sol;             /* 保留统计字段 */

    if (s_engine != RT_NULL)
    {
        /* 调试黑匣子: 引擎重建后前 3 次发布 (重生引擎首拍 alt=-50 瞬态取证) */
        static rt_uint8_t pub3 = 0;

        if (!g_sol.ready)
            pub3 = 0;
        if (pub3 < 3u)
        {
            NavState ns0 = s_engine->getNavState();

            LOG_I("首发[%u]: pos=%.6f/%.6f/%.3f vel=%.4f/%.4f/%.4f "
                  "att=%.2f/%.2f/%.2f", pub3 + 1u,
                  ns0.pos[0] * R2D, ns0.pos[1] * R2D, ns0.pos[2],
                  ns0.vel[0], ns0.vel[1], ns0.vel[2],
                  ns0.euler[0] * R2D, ns0.euler[1] * R2D, ns0.euler[2] * R2D);
            pub3++;
        }
        NavState ns = s_engine->getNavState();
        const GIEngine::UpdateStat &us = s_engine->updateStat();

        s.time      = g_run.now_gpst;
        s.latitude  = ns.pos[0] * R2D;
        s.longitude = ns.pos[1] * R2D;
        s.altitude  = ns.pos[2];
        s.vn        = ns.vel[0];
        s.ve        = ns.vel[1];
        s.vd        = ns.vel[2];
        s.roll      = ns.euler[0] * R2D;
        s.pitch     = ns.euler[1] * R2D;
        s.yaw       = ns.euler[2] * R2D;
        s.gnss_age_ms = g_nognss.active ? 0u :
                        (rt_uint32_t)((g_run.now_gpst - g_run.gnss_time) * 1000.0);

        s.mag_rej_cnt  = us.magrej + us.magdrop;
        s.baro_rej_cnt = us.barorej + us.barodrop;
        s.baro_height  = s_engine->baroHeight();
        s.mag_decl_deg = param_nav()->mag_decl_deg;
        s.baro_skip_cnt = us.baroskip;
        s.vreset_cnt    = us.vreset;
        s.updfail_cnt   = us.updfail;
        s.gnss_engdrop_cnt = us.gnssdrop;
        s.covheal_cnt      = us.covheal;
        s.reseed_sup_cnt   = g_reseed_ctl.suppressed;
        s.degraded     = RT_FALSE;
        s.nognss       = g_nognss.active;

        /* NaN/Inf 看门狗: 解算出现非有限值时 (a) 冻结首发现场 (b) 发布侧
         * 消毒 —— 保留上一拍有限导航值并置 degraded, 保证本接口永不吐
         * NaN/Inf (输出线程格式化 NaN 会爆栈 HardFault 冻结全系统)
         * (c) 恢复 —— NaN 使新息门/vwatch/cov_warn 的比较全部判假
         * (失明), 引擎无自愈路径, 连续达阈值即拆引擎重对准 */
        {
            rt_uint32_t mask = 0;

            if (!isfinite(s.roll))      mask |= 1u << 0;
            if (!isfinite(s.pitch))     mask |= 1u << 1;
            if (!isfinite(s.yaw))       mask |= 1u << 2;
            if (!isfinite(s.vn))        mask |= 1u << 3;
            if (!isfinite(s.ve))        mask |= 1u << 4;
            if (!isfinite(s.vd))        mask |= 1u << 5;
            if (!isfinite(s.latitude) || !isfinite(s.longitude) ||
                !isfinite(s.altitude))
                mask |= 1u << 6;
            if (mask != 0u)
            {
                if (g_nan.cnt == 0u)
                {
                    g_nan.nan_mask   = mask;
                    g_nan.imu_cnt    = g_run.imu_cnt;
                    g_nan.gnss_cnt   = g_run.gnss_cnt;
                    g_nan.mag_cnt    = g_run.mag_cnt;
                    g_nan.baro_cnt   = g_run.baro_cnt;
                    g_nan.drop_cnt   = g_run.drop_cnt;
                    g_nan.cov_warn   = g_run.cov_warn;
                    g_nan.gnss_rej   = g_run.gnss_rej;
                    g_nan.gpst       = g_run.now_gpst;
                    g_nan.gnss_age_s = g_run.now_gpst - g_run.gnss_time;
                    g_nan.mag_age_s  = g_run.now_gpst - g_run.mag_time;
                    g_nan.dt_last    = g_run.dt_last;
                }
                g_nan.cnt++;
                g_nan.streak++;

                if (g_nan.streak >= GINS_NAN_RESEED_N)
                {
                    LOG_W("解连续 %u 拍非有限 (mask=0x%x, imu=%u gnss=%u "
                          "rej=%u), 拆引擎重对准 (第 %u 次)",
                          g_nan.streak, mask, g_run.imu_cnt, g_run.gnss_cnt,
                          g_run.gnss_rej, g_reseed.cnt + 1u);
                    gins_reseed();
                    return;             /* 本拍不发布: g_sol 保持有限值且 ready=FALSE */
                }

                /* 消毒: 回退到上一拍有限导航字段 (统计字段随后刷新) */
                s = g_sol;
                s.degraded = RT_TRUE;
            }
            else
            {
                g_nan.streak = 0;
            }
        }

        /* Cholesky 失败恢复链: P 非正定后所有观测更新在 kf_math::update
         * 内被静默丢弃 (EKF 退化为纯惯导盲推, cov_warn 只是间接信号)。
         * 自上次成功更新起连续失败达 GINS_UPDFAIL_RESEED_N 次拆引擎
         * 重对准; 单次偶发失败 (一次坏观测) 被下次成功复位, 不误触发 */
        if (us.updfail != g_updfail.last_fail || us.updok != g_updfail.last_ok)
        {
            if (us.updfail > g_updfail.last_fail)
                g_updfail.streak += us.updfail - g_updfail.last_fail;
            if (us.updok > g_updfail.last_ok)
                g_updfail.streak = 0;
            g_updfail.last_ok   = us.updok;
            g_updfail.last_fail = us.updfail;

            if (g_updfail.streak >= GINS_UPDFAIL_RESEED_N)
            {
                LOG_W("观测更新连续 %u 次 Cholesky 失败 (P 非正定), "
                      "拆引擎重对准 (第 %u 次)",
                      g_updfail.streak, g_reseed.cnt + 1u);
                gins_reseed();
                return;
            }
        }
    }

    s.nan_cnt      = g_nan.cnt;
    s.imu_cnt      = g_run.imu_cnt;
    s.gnss_cnt     = g_run.gnss_cnt;
    s.gnss_stale_cnt = g_run.gnss_stale;
    s.cov_warn_cnt = g_run.cov_warn;
    s.drop_cnt     = g_run.drop_cnt;
    s.mag_cnt      = g_run.mag_cnt;
    s.baro_cnt     = g_run.baro_cnt;
    s.mag_stale_cnt = g_run.mag_stale;
    s.baro_stale_cnt = g_run.baro_stale;
    s.zupt_cnt     = g_run.zupt_cnt;

    {
        rt_base_t level = rt_hw_interrupt_disable();

        g_sol = s;
        rt_hw_interrupt_enable(level);
    }
}

/* ------------------------- 辅助传感器采样 (BMM350/BMP585) ------------------------- */

/*
 * ginsaux 线程消费 middleware/Sensor_Preprocessing/process_data 的磁/气压环形缓冲区 (100Hz 生产者,
 * 样本自带 T_MCU 采样时刻): wait 阻塞等新样本, pop 排空积压并把最新
 * 样本连同序号发布到关中断保护的槽里; 解算线程每毫秒检查序号变化喂入
 * 引擎。磁样本在 process_data 层入环前已完成 校准->轴映射->干扰检查->低通
 * (见 mag_data.c), 这里直接取 cal 字段 (quality 干扰位经 std_scale
 * 降权喂入); 气压观测仍在此做 baro_calib 基准偏移校正。I2C 读取耗时
 * 隔离在数据层采集线程, 不占 1kHz 解算时间预算。观测时标用样本自带
 * 采样时刻经 T_MCU 锚点换算 (见 gins_aux_feed; 磁观测再前移低通群延迟
 * GINS_MAG_LPF_TAU_S), 滞后超过 GINS_AUX_MAX_AGE_S 的过旧样本丢弃并计数。
 * 注: 环形缓冲区为单消费方语义, 校准采集 (magcal/barocal) 运行期间
 * 样本在两消费方间分流, 属维护窗口内的预期行为。
 */
static struct gins_aux_baro_slot
{
    rt_uint32_t seq;
    rt_uint64_t tmcu_us;                    /* 采样时刻 (T_MCU µs, baro_data 打戳) */
    float pressure_pa;                      /* Pa, 未校准 */
    float temp_dc;                          /* degC, 无效时 < -100 (引擎退回 ISA) */
} s_aux_baro;

/*
 * 独立气压高度基准 (gins_config.h "独立气压高度交叉校验"):
 * h_baro = 44330·(1-(P_cal/101325)^0.1903), P_cal 经 barocal 偏移校正。
 * 基准只依赖 BMP585 本地传感器, 与 GNSS/引擎完全独立, 不会被观测垃圾
 * 污染 —— 这是它能当裁判的原因。时延比对在 T_MCU 域做 (播种前引擎
 * 锚点未建立, 不能用引擎时间); 链路未运行/样本过旧/量程外一律返回
 * 失败, 调用方跳过校验 (fail-open) */
static rt_bool_t gins_baro_alt_ref(rt_uint64_t tmcu_us, double *alt_out)
{
    struct gins_aux_baro_slot b;
    rt_int64_t d_us;
    double p;
    rt_base_t level;

    level = rt_hw_interrupt_disable();
    b = s_aux_baro;
    rt_hw_interrupt_enable(level);

    if (b.seq == 0u)
        return RT_FALSE;
    d_us = (rt_int64_t)(tmcu_us - b.tmcu_us);
    if (d_us < -100000 || d_us > (rt_int64_t)(GINS_ALTX_STALE_S * 1e6))
        return RT_FALSE;
    p = baro_calib_apply((double)b.pressure_pa);
    if (!(p > 30000.0 && p < 120000.0))     /* 物理量程 (NaN 也在此拦截) */
        return RT_FALSE;
    *alt_out = 44330.0 * (1.0 - pow(p / 101325.0, 0.190295));
    return RT_TRUE;
}

static void gins_aux_thread_entry(void *parameter)
{
    struct mag_data_status mst;
    struct baro_data_status bst;
    rt_uint32_t wait_s = 0;

    RT_UNUSED(parameter);

    /* 本线程随 gins 桥接 (INIT_ENV) 启动, 早于 mag_data/baro_data 的
     * INIT_APP —— 数据链未 running 时轮询等待而不是退出, 否则磁/气压
     * 融合自上电起永远不接入 (实测 mag_cnt 恒 0 的根因) */
    while (1)
    {
        mag_data_get_status(&mst);
        baro_data_get_status(&bst);
        if (mst.running || bst.running)
            break;
        if (wait_s % 10u == 0u)
            LOG_W("ginsaux: mag_data/baro_data 链路均未运行, 等待中 (%us) "
                  "(查 FinSH `magdata`/`barodata` 与传感器接线)", wait_s);
        rt_thread_mdelay(1000u);
        wait_s++;
    }

    LOG_I("ginsaux: mag_data%s + baro_data%s 环形缓冲区, %dms 等待超时",
          mst.running ? "" : "(缺失)", bst.running ? "" : "(缺失)",
          GINS_AUX_POLL_MS);

    while (1)
    {
        struct mag_sample ms;
        struct baro_sample bs;

        /* 未运行的链路跳过等待: 阻塞在永不释放的信号量上直到固定超时,
         * 期间 magout 等排水消费方会把另一通道的样本全部抢走
         * (实测气压链路未运行时 mag 观测被饿死, mag_cnt≈0 的根因) */
        mag_data_get_status(&mst);
        if (mst.running && mag_data_wait(GINS_AUX_POLL_MS) == RT_EOK)
        {
            while (mag_data_pop(&ms) == RT_EOK)
            {
                rt_base_t level = rt_hw_interrupt_disable();

                s_aux_mag.tmcu_us   = ms.T_event;
                s_aux_mag.quality  = ms.quality;
                s_aux_mag.mag_ut[0] = ms.cal[0];    /* 已校准+轴映射+低通 (FRD) */
                s_aux_mag.mag_ut[1] = ms.cal[1];
                s_aux_mag.mag_ut[2] = ms.cal[2];
                s_aux_mag.seq++;
                rt_hw_interrupt_enable(level);
            }
        }

        baro_data_get_status(&bst);
        if (bst.running && baro_data_wait(GINS_AUX_POLL_MS) == RT_EOK)
        {
            while (baro_data_pop(&bs) == RT_EOK)
            {
                rt_base_t level = rt_hw_interrupt_disable();

                s_aux_baro.tmcu_us    = bs.T_event;
                s_aux_baro.pressure_pa = bs.pressure_pa;
                s_aux_baro.temp_dc     = bs.temperature_c;
                s_aux_baro.seq++;
                rt_hw_interrupt_enable(level);
            }
        }
    }
}

/* 解算线程侧: 消费新样本并喂入引擎 (样本采样时刻时标) */
static void gins_aux_feed(GIEngine *eng)
{
    struct gins_aux_mag_slot m;
    struct gins_aux_baro_slot b;
    rt_base_t level;

    level = rt_hw_interrupt_disable();
    m = s_aux_mag;
    b = s_aux_baro;
    rt_hw_interrupt_enable(level);

    if (m.seq != 0u && m.seq != g_run.mag_seq)
    {
        /* 观测时标 = 采样时刻前移低通群延迟: EMA 输出代表 τ 之前的磁场,
         * 一阶低通 DC 群延迟 ≈ τ (τ=0 旁路时补偿为零) */
        double t_obs = gins_gpst_of_tmcu(m.tmcu_us) - GINS_MAG_LPF_TAU_S;
        double age = g_run.now_gpst - t_obs;

        g_run.mag_seq = m.seq;
        g_run.mag_cnt++;
        g_run.mag_time = t_obs;

        if (age > GINS_AUX_MAX_AGE_S)
        {
            g_run.mag_stale++;             /* 消费被延迟/校准分流, 丢弃 */
        }
        else
        {
            /* 磁场模值硬门禁: 校准参数异常/极值样本直接丢弃
             * (地磁 20~100µT 量级, 超 10 倍即判坏样本) */
            double mnorm = sqrt((double)m.mag_ut[0] * (double)m.mag_ut[0] +
                                (double)m.mag_ut[1] * (double)m.mag_ut[1] +
                                (double)m.mag_ut[2] * (double)m.mag_ut[2]);

            if (!(mnorm > 5.0 && mnorm < 300.0))
            {
                g_run.mag_rej++;
            }
            else
            {
                MAG mg;

                /* 微小超前 (样本晚于最近 IMU) 钳到当前时刻;
                 * quality 干扰位 (模值超限) -> std 放大降权, 而非丢弃 */
                mg.time      = (age < 0) ? g_run.now_gpst : t_obs;
                mg.std_scale = (m.quality & 0x01u) ? GINS_MAG_INTERF_STD_SCALE : 1.0;
                mg.mag    << (double)m.mag_ut[0], (double)m.mag_ut[1],
                             (double)m.mag_ut[2];
                eng->addMagData(mg);
            }
        }
    }

    if (b.seq != 0u && b.seq != g_run.baro_seq)
    {
        double t_obs = gins_gpst_of_tmcu(b.tmcu_us);
        double age = g_run.now_gpst - t_obs;

        g_run.baro_seq = b.seq;
        g_run.baro_cnt++;

        if (age > GINS_AUX_MAX_AGE_S)
        {
            g_run.baro_stale++;
        }
        else
        {
            BARO br;

            br.time     = (age < 0) ? g_run.now_gpst : t_obs;
            br.pressure = baro_calib_apply((double)b.pressure_pa);   /* 基准偏移 (barocal) */
            br.temp     = (double)b.temp_dc;     /* < -100 时引擎锚定退回 ISA 温度 */
            eng->addBaroData(br);
        }
    }
}

/* ------------------------- 解算线程 ------------------------- */

/* 单次唤醒最多追赶的 IMU 样本数 (1ms tick): 消费延迟恢复时缓冲区内
 * 样本逐拍消化而不丢弃; 常态下每拍恰好 1 个 */
#define GINS_IMU_CATCHUP_MAX        10

/* 测试暂停开关 (gins_bridge_set_pause): 见 gins_bridge.h 注释 */
static volatile rt_bool_t s_thread_pause;

void gins_bridge_set_pause(rt_bool_t on)
{
    s_thread_pause = on;
}

static void gins_thread_entry(void *parameter)
{
    struct imu_sample smp;
    struct gnss_sample gs;
    rt_uint64_t last_gnss_ts = 0;
    rt_uint32_t loops = 0;
    rt_uint32_t wait_log_ms = 0;
    double f_sum[3] = { 0.0, 0.0, 0.0 };
    rt_uint32_t align_n = 0;
    rt_bool_t have_dc = RT_FALSE;          /* data_cnt 差分基准 (dt 计算) */
    rt_uint32_t last_dc = 0;
    rt_bool_t have_tev = RT_FALSE;         /* T_event 差分基准 (dt/引擎时钟) */
    rt_uint64_t last_tev = 0;

    RT_UNUSED(parameter);
    memset(&gs, 0, sizeof(gs));

    while (1)
    {
        rt_uint32_t n = 0;

        gins_wdt_feed();                        /* 线程活着即喂狗 */
        /* IMU 事件驱动等待: 样本入环 (rx_indicate → 信号量) 即醒, 消费
         * 延迟比 1ms 轮询少一拍且免去 1kHz 无效唤醒。dt 由 T_event 差分
         * 计算不受唤醒相位影响; 断流时按 2ms 超时醒来维持喂狗与 aux
         * 排空节拍。数据链未运行时 wait 立即返回错误, 退回 1ms 节拍。 */
        if (imu_data_wait(2) != RT_EOK)
            rt_thread_mdelay(1);
        loops++;

        /* 测试暂停: 睡眠等待恢复 (kf_math 静态态让给外部测试引擎) */
        while (s_thread_pause)
        {
            rt_thread_mdelay(20);
            loops++;
        }

        /* 1. GNSS 观测优先入队 (与桌面主循环同序): 引擎按时间先后处理。
         *    消费 middleware/Sensor_Preprocessing/process_data 的 gnss_data 环形缓冲区 (10Hz 定位解),
         *    每毫秒排空取最新; T_event 为语句 UTC 经 PPS 映射换算的
         *    T_MCU 时刻 (映射未就绪时为 0, 本侧跳过, 不用到达时刻顶替
         *    —— 避免百 ms 级时戳误差污染 EKF)。
         *    T_event==0 的样本不覆盖已到手的有效样本 (旧实现有效样本被
         *    其顶掉、还误计成 ts_zero); 单拍排空多个有效样本时中间样本
         *    计入 gnss_skip_mid (10Hz 下常态为 0, 非 0 = 消费延迟)。 */
        {
            struct gnss_sample keep;
            rt_bool_t have_keep = RT_FALSE;

            while (gnss_data_pop(&gs) == RT_EOK)
            {
                if (gs.T_event != 0u)
                {
                    if (have_keep && gs.T_event != keep.T_event)
                        g_run.gnss_skip_mid++;
                    keep = gs;
                    have_keep = RT_TRUE;
                }
                else
                {
                    g_run.gnss_ts_zero++;   /* 映射未就绪: 样本被跳过 */
                }
            }
            if (have_keep)
                gs = keep;
        }
        if (gs.T_event != 0u && gs.T_event != last_gnss_ts)
        {
            last_gnss_ts = gs.T_event;
            if (s_engine != RT_NULL &&
                gs.fix_type != GNSS_FIX_INVALID && gs.utc_sec != 0u)
            {
                double t_obs = gins_gpst_of_tmcu(gs.T_event);
                double age = g_run.now_gpst - t_obs;
                rt_uint32_t late_us =
                    (gs.T_arrival > gs.T_event)
                        ? (rt_uint32_t)(gs.T_arrival - gs.T_event) : 0u;

                if (late_us > g_run.gnss_late_us_max)
                    g_run.gnss_late_us_max = late_us;

                if (age > GINS_GNSS_MAX_AGE_S)
                {
                    g_run.gnss_stale++;         /* 消费被延迟, 丢弃 */
                }
                else
                {
                    GNSS g;
                    double hdop = (gs.hdop > 0.1f) ? (double)gs.hdop
                                                  : (double)UM982_NMEA_HDOP_FALLBACK;
                    double std_h = hdop * UM982_NMEA_UERE_M;

                    /* RTK 固定解观测噪声收紧 (C7): 米级 UERE 定权对 RTK
                     * 是 20 倍悲观, 收紧后水平分米级; 浮点解保守不收
                     * (质量不稳定)。hdop 几何门/跳变门照常生效, 定权只
                     * 影响 R 阵 */
                    if (gs.rtk_status == GNSS_RTK_FIX)
                        std_h = GINS_RTK_FIX_STD_H_M;

                    /* 观测合理性门禁: 解析残缺/异常值直接丢弃并计数
                     * (异常观测进 EKF 是数值发散 -> NaN 的主要来源) */
                    if (!gnss_sample_plausible(&gs))
                    {
                        g_run.gnss_rej++;
                    }
                    else
                    {
                        /* 观测质量门禁 (先验, 绝对拒绝): hdop 几何崩坏或
                         * 相邻观测跳变超机动能力 = 数据本身是垃圾, 与
                         * INS 状态无关 —— INS 被污染后新息门失效, 此门
                         * 仍有效 (见 gins_config.h 质量门禁段注释) */
                        rt_bool_t q_ok = RT_TRUE;

                        if (hdop > (double)GINS_GNSS_HDOP_MAX)
                        {
                            g_gq.rej_hdop++;
                            q_ok = RT_FALSE;
                        }
                        else if (g_gq.have_prev)
                        {
                            double ddt = (double)(rt_int64_t)
                                             (gs.T_event - g_gq.tmcu) * 1e-6;
                            double jn = (gs.latitude_deg - g_gq.lat) * 111320.0;
                            double je = (gs.longitude_deg - g_gq.lon) * 111320.0 *
                                        cos(g_gq.lat * D2R);
                            double ju = gs.altitude_m - g_gq.alt;
                            double allow = GINS_GNSS_JUMP_BASE_M +
                                GINS_GNSS_JUMP_VMAX_MPS * (ddt > 0.0 ? ddt : 0.0);

                            if (jn * jn + je * je + ju * ju > allow * allow)
                            {
                                g_gq.rej_jump++;
                                q_ok = RT_FALSE;
                            }
                        }

                        /* 独立气压高度交叉校验: 观测高度偏离 baro 基准
                         * 超门限 = 垂直垃圾 (20km 级高度垃圾曾把 pitch
                         * 拽到 52°), baro 基准独立于 GNSS 不被污染 */
                        if (q_ok)
                        {
                            double ha;

                            if (gins_baro_alt_ref(gs.T_event, &ha) &&
                                fabs(gs.altitude_m - ha) > (double)GINS_ALTX_H_M)
                            {
                                g_gq.rej_altx++;
                                q_ok = RT_FALSE;
                            }
                        }

                        /* 新息门禁: 量程内的混缝句垃圾 (与 INS 位置差
                         * 千米级) 同样致命; 连续拒收 5 次后放行一次,
                         * 防 GNSS 真失锁重捕时 INS 已漂远被全拒。
                         * 仅质量合格样本参与 (质量门已绝对拒绝的不再看新息) */
                        rt_bool_t innov_ok = RT_TRUE;
                        double dn = (gs.latitude_deg - g_sol.latitude) * 111320.0;
                        double de = (gs.longitude_deg - g_sol.longitude) * 111320.0 *
                                    cos(g_sol.latitude * D2R);
                        double du = gs.altitude_m - g_sol.altitude;

                        if (!q_ok)
                        {
                            g_run.gnss_rej++;       /* 计入全局拒收口径 */
                        }
                        else if (dn * dn + de * de + du * du >
                                GINS_GNSS_INNOV_MAX_M * GINS_GNSS_INNOV_MAX_M &&
                            g_innov_rej < 5u)
                        {
                            g_run.gnss_rej++;
                            g_innov_rej++;
                            innov_ok = RT_FALSE;
                        }
                        else
                        {
                            g_innov_rej = 0;

                            /* 跳变基准更新 (只记已入滤样本, 含放行通道) */
                            g_gq.have_prev = RT_TRUE;
                            g_gq.lat  = gs.latitude_deg;
                            g_gq.lon  = gs.longitude_deg;
                            g_gq.alt  = gs.altitude_m;
                            g_gq.tmcu = gs.T_event;

                            /* 观测方差 N/E/U (同 um982_nmea_pos_std: UERE 粗估) */
                            g.time = (age < 0) ? g_run.now_gpst : t_obs;
                            g.blh << gs.latitude_deg * D2R, gs.longitude_deg * D2R,
                                     gs.altitude_m;
                            g.std << std_h, std_h, 2.0 * std_h;
                            /* RMC 水平速度 (方案A): 静止时 ≈0 的强先验,
                             * 加速姿态/零偏/yaw 分离, NMEA 无垂向速度 */
                            if (gs.vel_valid)
                            {
                                g.velne << gs.vn, gs.ve;
                                g.velvalid = true;
                            }
                            else
                            {
                                g.velvalid = false;
                            }
                            g.isvalid = false;
                            s_engine->addGnssData(g);

                            g_run.gnss_time = g.time;
                            g_run.gnss_cnt++;
                        }

                        /* 坏播种侦测 (种子先验门禁的兜底): 只统计质量合格
                         * 样本的新息拒收率, EMA 持续超阈 -> 拆引擎重对准 */
                        if (g_guard.armed)
                        {
                            if (g_run.now_gpst - g_guard.t0 > (double)GINS_RESEED_GUARD_S)
                            {
                                g_guard.armed = RT_FALSE;   /* 平安渡过侦测窗 */
                            }
                            else if (q_ok)
                            {
                                double dt_g = g_run.now_gpst - g_guard.ema_t;
                                double x = innov_ok ? 0.0 : 1.0;

                                if (dt_g > 0.0 && dt_g < 2.0)
                                    g_guard.ema += (x - g_guard.ema) *
                                        (dt_g / (double)GINS_RESEED_EMA_TAU);
                                else
                                    g_guard.ema = x;
                                g_guard.ema_t = g_run.now_gpst;
                                g_guard.seen++;

                                if (g_guard.seen >= GINS_RESEED_MIN_N &&
                                    g_guard.ema > (double)GINS_RESEED_REJ_RATE)
                                {
                                    g_guard.hold += (dt_g > 0.0 ? dt_g : 0.0);

                                    if (g_guard.hold > (double)GINS_RESEED_HOLD_S)
                                    {
                                        gins_reseed_request("坏播种侦测", RT_FALSE);
                                        g_guard.armed = RT_FALSE;
                                        /* 引擎已 NULL: 对准窗均值保留仍有效,
                                         * 种子稳定窗在 GNSS 到达侧重走 */
                                    }
                                }
                                else
                                {
                                    g_guard.hold = 0.0;
                                }
                            }
                        }
                    }
                }
            }
            else if (s_engine == RT_NULL || g_nognss.active)
            {
                /* 对准期: 播种质量稳定窗逐样本评估 (遗留 #5)。
                 * 对准窗未满也照常累计——定位收敛与静止判据相互独立,
                 * 收敛确认先凑好可缩短上电初始化时间。
                 * NOGNSS 运行期同样累计: GNSS 接入且定位稳定 (稳定窗满)
                 * 即拆引擎重走正常融合路径 (锚点/播种由 GNSS 重建)。 */
                seed_track(&gs);
                if (g_nognss.active && g_seed.seq >= GINS_SEED_STABLE_N)
                    gins_reseed_request("GNSS 恢复稳定, 转正常融合", RT_FALSE);
            }
        }

        /* 1.5 磁/气压观测入队 (ginsaux 线程 100Hz 采样, 样本自带采样时刻时标;
         *    磁样本已在 process_data 层完成 校准->轴映射->低通) */
        if (s_engine != RT_NULL)
            gins_aux_feed(s_engine);

        /*
         * 2. IMU: 排空环形缓冲区 (样本在 imu_process_data 层已完成 校验+单位换算+
         *    轴映射校准, 直接是体坐标系物理单位)。dt 由样本 data_cnt
         *    (DATA_CNTR 32bit 扩展) 差分按名义 ODR 换算, 与线程唤醒抖动
         *    无关; 丢拍期 dt 相应放大 (EKF 按实际间隔积分, 不误当作均匀
         *    1ms); 单拍最多追赶 GINS_IMU_CATCHUP_MAX 个, 其余留待下拍
         *    (缓冲区吸收抖动)。
         */
        while (n < GINS_IMU_CATCHUP_MAX && imu_data_pop(&smp) == RT_EOK)
        {
            double dt;
            double f_b[3], w_b[3];
            int i;

            for (i = 0; i < 3; i++)
            {
                f_b[i] = (double)smp.accel[i];          /* m/s^2, 体系 FRD */
                w_b[i] = (double)smp.gyro[i];           /* rad/s, 体系 FRD */
            }

            /* 逐样本 NaN/Inf 防护: 单个 NaN 样本会即时毒化机械编排
             * (对准期还会穿透静止门禁, 见 gins_try_init_engine);
             * Inf 同样致命 (积分发散为 Inf, sin/cos(Inf)=NaN 穿透)。
             * isfinite 同时拦二者 (x!=x 只拦 NaN), 丢弃并计数, 不进引擎 */
            if (!isfinite(f_b[0]) || !isfinite(f_b[1]) || !isfinite(f_b[2]) ||
                !isfinite(w_b[0]) || !isfinite(w_b[1]) || !isfinite(w_b[2]))
            {
                g_run.drop_cnt++;
                n++;
                continue;
            }

            /* dt: 优先 T_event 差分 (TIM2 晶振 ~5ppm, 与 GNSS 观测/锚点
             * 同一时基)。ADIS 内部时钟对名义 ODR 存在 ±0.1% 级失配
             * (实测 +1200ppm): 旧实现 dt=data_cnt/ODR 且引擎时钟用
             * Σdt 累计, 引擎时间相对锚点漂移 ~1.2ms/s, 约 420s 后
             * age 越过 0.5s 门限 -> GNSS 全部判 stale -> EKF 失明 ->
             * 惯导盲推发散。时标异常 (回绕/野值) 回退 data_cnt 差分。 */
            {
                rt_uint32_t d;
                double dt_tev = -1.0;

                if (have_tev && smp.T_event > last_tev)
                {
                    dt_tev = (double)(smp.T_event - last_tev) * 1e-6;
                    if (dt_tev > 0.1)                  /* >100ms: 时标跳变 */
                        dt_tev = -1.0;
                }
                have_tev = RT_TRUE;
                last_tev = smp.T_event;

                if (dt_tev > 0.0)
                {
                    dt = dt_tev;
                }
                else if (!have_dc)
                {
                    have_dc = RT_TRUE;
                    dt = 1.0 / (double)GINS_IMU_ODR_HZ;
                }
                else
                {
                    d = smp.data_cnt - last_dc;
                    if (d == 0u || d > 100u)
                    {
                        dt = 1.0 / (double)GINS_IMU_ODR_HZ;
                        g_run.drop_cnt++;
                    }
                    else
                    {
                        dt = (double)d / (double)GINS_IMU_ODR_HZ;
                        if (d > 1u)
                            g_run.drop_cnt += d - 1u;   /* DR 丢拍统计 */
                    }
                }
                last_dc = smp.data_cnt;
                g_run.dt_last = (float)dt;
            }

#if GINS_ZUPT_ENABLE && GINS_NOGNSS_MODE
            /* ZUPT 静止检测 (逐样本, 对准期照常跟踪 —— 播种时 EMA 已收敛,
             * 引擎就绪即可注入)。加计判据扣除引擎当前零偏估值 (本机零偏
             * ~340mGal=0.0034 m/s², 量级小但白扣白不扣; 主导项是轴失准/
             * 比例因子的姿态相关 |f| 偏差 ±0.18, 由 GINS_ZUPT_ACC_DEV_MAX
             * =0.30 门限覆盖, 见 gins_config.h 注释)。零偏 1Hz 缓存 (相关
             * 时间 4h, 无需逐拍取); 引擎未就绪期间 0 (对准期仅跟踪 EMA) */
            {
                static double   ab_cache[3];        /* 引擎零偏估值缓存, m/s² */
                static rt_tick_t ab_tick;
                double alpha = dt / ((double)GINS_ZUPT_EMA_TAU_S + dt);
                double fc[3];
                double w2    = w_b[0] * w_b[0] + w_b[1] * w_b[1] + w_b[2] * w_b[2];
                double fnorm, fdev;

                if (s_engine != RT_NULL &&
                    (rt_tick_get() - ab_tick) > (rt_tick_t)RT_TICK_PER_SECOND)
                {
                    NavState ns = s_engine->getNavState();

                    ab_cache[0] = ns.imuerror.accbias[0];
                    ab_cache[1] = ns.imuerror.accbias[1];
                    ab_cache[2] = ns.imuerror.accbias[2];
                    ab_tick = rt_tick_get();
                }
                fc[0] = f_b[0] - ab_cache[0];
                fc[1] = f_b[1] - ab_cache[1];
                fc[2] = f_b[2] - ab_cache[2];
                fnorm = sqrt(fc[0] * fc[0] + fc[1] * fc[1] + fc[2] * fc[2]);
                fdev  = fabs(fnorm - 9.80665);

                g_zupt.w2_ema   += (w2 - g_zupt.w2_ema) * alpha;
                g_zupt.fdev_ema += (fdev - g_zupt.fdev_ema) * alpha;

                /* 迟滞状态机: 确认 = w²/fdev 低于 CONFIRM 门限持续 2s;
                 * 判运动 = 超过 MOTION 门限 (3x CONFIRM, 防边缘震荡);
                 * 两门限之间为迟滞带 —— hold 保持、状态不变 */
                if (g_zupt.w2_ema < (double)GINS_ZUPT_GYRO_W2_MAX &&
                    g_zupt.fdev_ema < (double)GINS_ZUPT_ACC_DEV_MAX)
                {
                    g_zupt.hold_ms += (rt_uint32_t)(dt * 1000.0);
                    if (!g_zupt.active && g_zupt.hold_ms >= GINS_ZUPT_CONFIRM_MS)
                    {
                        g_zupt.active = RT_TRUE;
                        LOG_I("ZUPT: 静止确认 (w²=%.1f n(rad/s)², |f|-g=%.3f m/s²), "
                              "开始零速修正",
                              g_zupt.w2_ema * 1e9, g_zupt.fdev_ema);

                        /* 运动恢复发散检测: NOGNSS 无观测跟踪运动, 运动期
                         * 状态必然漂移; 速度已发散时 ZUPT 新息门 + 5连拒
                         * 放1 拖回极慢 (实测 893 m/s 拖约半小时) —— 直接
                         * 拆引擎重对准 (此刻静止, 粗对准 ~35s 重建) */
                        if (g_nognss.active)
                        {
                            double v2 = g_sol.vn * g_sol.vn + g_sol.ve * g_sol.ve +
                                        g_sol.vd * g_sol.vd;

                            if (v2 > (double)GINS_ZUPT_RESEED_V_MPS *
                                     (double)GINS_ZUPT_RESEED_V_MPS)
                            {
                                LOG_W("ZUPT: 静止恢复但速度已发散 (%.1f m/s), "
                                      "拆引擎重对准",
                                      sqrt(v2));
                                gins_reseed_request("ZUPT 恢复发散重对准", RT_FALSE);
                            }
                        }
                    }
                }
                else if (g_zupt.w2_ema >= (double)GINS_ZUPT_GYRO_W2_MOTION ||
                         g_zupt.fdev_ema >= (double)GINS_ZUPT_ACC_DEV_MOTION)
                {
                    /* 超运动门限: 真实运动, 暂停 + 清确认累计 */
                    if (g_zupt.active)
                    {
                        g_zupt.active = RT_FALSE;
                        LOG_I("ZUPT: 检测到运动, 零速修正暂停 (w²=%.1f n(rad/s)², "
                              "|f|-g=%.3f m/s²)",
                              g_zupt.w2_ema * 1e9, g_zupt.fdev_ema);
                    }
                    g_zupt.hold_ms = 0u;
                }
                /* else: 迟滞带内, hold 与状态保持 */
                else
                {
                    g_zupt.hold_ms = 0u;
                }
            }
#endif /* GINS_ZUPT_ENABLE && GINS_NOGNSS_MODE */

            /* 3. 未初始化: 静止对准 + 等待首次定位 */
            if (s_engine == RT_NULL)
            {
                enum gins_align_res res;

                if (align_n < GINS_ALIGN_SAMPLES)
                {
                    f_sum[0] += f_b[0];
                    f_sum[1] += f_b[1];
                    f_sum[2] += f_b[2];
                    align_n++;
                }
                else
                {
                    /* 窗口满后转滑动均值 (f_sum 始终 ~ n*EMA(f), 时间常数
                     * GINS_ALIGN_TRACK_TAU_S): 等定位期可达数分钟 (冷启动),
                     * 冻结在上电头 3s 的均值会把等待期的搬动/旋转仍判
                     * "静止"、以过时姿态播种 (f_sum/n 既是静止判据也是
                     * roll/pitch 初值)。滑动后搬动的加计瞬态立即推高
                     * |f_mean|-g 偏差触发 SHAKE 重开, 静止后收敛到当前姿态 */
                    double alpha = dt / ((double)GINS_ALIGN_TRACK_TAU_S + dt);

                    f_sum[0] += ((double)align_n * f_b[0] - f_sum[0]) * alpha;
                    f_sum[1] += ((double)align_n * f_b[1] - f_sum[1]) * alpha;
                    f_sum[2] += ((double)align_n * f_b[2] - f_sum[2]) * alpha;
#if GINS_NOGNSS_MODE
                    /* 静止粗对准累计: 等待窗陀螺角速率均值 (SHAKE 重开清零)。
                     * 逐样本静态门 |w|<0.1 rad/s (±5.7deg/s): 剔除上电头几拍
                     * ADIS 未就绪的满量程野值 (实测 ~20 拍把 30s 均值毒化到
                     * 1208 deg/h), 静态台架真实角速率远低于此门 */
                    if (fabs(w_b[0]) < 0.1 && fabs(w_b[1]) < 0.1 &&
                        fabs(w_b[2]) < 0.1)
                    {
                        g_align_w.sum[0] += w_b[0];
                        g_align_w.sum[1] += w_b[1];
                        g_align_w.sum[2] += w_b[2];
                        g_align_w.n++;
                    }
#endif
                }
                if (align_n < GINS_ALIGN_SAMPLES)
                {
                    n++;
                    continue;
                }

                res = gins_try_init_engine(&gs, &smp, f_sum, align_n, dt);
                if (res == GINS_ALIGNED)
                {
                    n++;
                    continue;                   /* 下拍开始正常解算 */
                }
                if (res == GINS_ALIGN_SHAKE)
                {
                    align_n = 0;                /* 不静止: 重开窗口 */
                    f_sum[0] = f_sum[1] = f_sum[2] = 0.0;
#if GINS_NOGNSS_MODE
                    g_align_w.sum[0] = g_align_w.sum[1] = g_align_w.sum[2] = 0.0;
                    g_align_w.n = 0u;
#endif
                }
            else if (loops - wait_log_ms >= 30000u)
            {
                /* 静止但等定位: 保持窗口, 30s 提示一次 */
                wait_log_ms = loops;
                LOG_I("对准完成, 等待定位收敛稳定 (fix=%d sats=%u hdop=%.1f "
                      "稳定窗 %u/%u 映射=%s%s)",
                      gs.fix_type, gs.satellites, (double)gs.hdop,
                      g_seed.seq, (unsigned)GINS_SEED_STABLE_N,
                      timebase_map_valid() ? "ready" : "wait",
#if GINS_NOGNSS_MODE
                      g_nognss.wait_s < (double)GINS_NOGNSS_WAIT_S
                          ? ", 无定位超时转配置位置播种" : ""
#else
                      ""
#endif
                );
            }
                n++;
                continue;
            }

            /* 4. 正常解算: addImuData + newImuProcess (上游主循环同款)
             *    引擎时钟 = 锚点映射的样本 T_event (与 GNSS t_obs 同源,
             *    消除 Σdt 累计漂移; 见上 dt 注释) */
            g_run.now_gpst = gins_gpst_of_tmcu(smp.T_event);
            {
                IMU imu;
                rt_uint32_t cyc0 = DWT->CYCCNT;

                imu.time   = g_run.now_gpst;
                imu.dt     = dt;
                imu.dtheta << w_b[0] * dt, w_b[1] * dt, w_b[2] * dt;
                imu.dvel   << f_b[0] * dt, f_b[1] * dt, f_b[2] * dt;
                imu.odovel = 0.0;

                s_engine->addImuData(imu);
                s_engine->newImuProcess();

                /* 单步耗时 = 机械编排 + EKF (P 传播 ~100Hz 节流冲刷), DWT 周期差分 */
                {
                    float us = (float)(DWT->CYCCNT - cyc0) /
                               ((float)SystemCoreClock / 1000000.0f);

                    if (g_run.step_us_avg == 0.0f)
                        g_run.step_us_avg = us;
                    else
                        g_run.step_us_avg += (us - g_run.step_us_avg) * 0.05f;
                    if (us > g_run.step_us_max)
                        g_run.step_us_max = us;
                }
            }
            g_run.imu_cnt++;

#if GINS_ZUPT_ENABLE && GINS_NOGNSS_MODE
            /* ZUPT 注入 (NOGNSS 运行期 + 静止确认): 周期性零速观测,
             * 钉住速度并使零偏/水平姿态可观; 观测时标 = 本拍 IMU 时刻,
             * 引擎下一拍 newImuProcess 按 res==1 调度更新 */
            if (g_nognss.active && g_zupt.active &&
                g_run.now_gpst - g_zupt.last_t >= (double)GINS_ZUPT_DT_S)
            {
                ZUPT z;

                z.time = g_run.now_gpst;
                z.std  = (double)GINS_ZUPT_STD_MPS;
                s_engine->addZuptData(z);
                g_zupt.last_t = g_run.now_gpst;
                g_run.zupt_cnt++;
            }
#endif /* GINS_ZUPT_ENABLE && GINS_NOGNSS_MODE */

            n++;
        }

        /* 5. 有新样本才发布解算结果快照 */
        if (n != 0u)
            gins_publish();

        /* 5.5 加计零偏自动快照 (~1Hz 查, C8): 就绪满 GINS_ACCBIAS_SAVE_S
         *    且静止时每上电存一次; 条件不满足静默返回下拍再试 */
        if (s_engine != RT_NULL && !g_accsnap.saved && (loops & 0x3FFu) == 2u)
            (void)gins_accbias_snapshot((double)smp.temperature, RT_FALSE);

        /* 5.8 播种位置守卫 (~1Hz): NOGNSS 播种后侦测窗内位置偏离配置点
         *     超限 (速度积分不可达) = 状态被确定性搬走, 拆引擎重播种 */
#if GINS_NOGNSS_MODE
        if (g_posguard.armed && g_nognss.active && (loops & 0x3FFu) == 3u)
        {
            if (g_run.now_gpst - g_posguard.t0 > (double)GINS_NOGNSS_POSGUARD_S)
            {
                g_posguard.armed = RT_FALSE;    /* 平安渡过侦测窗 */
            }
            else
            {
                double dn = (g_sol.latitude - param_nav()->nognss_lat) * 111320.0;
                double de = (g_sol.longitude - param_nav()->nognss_lon) * 111320.0 *
                            cos(param_nav()->nognss_lat * D2R);

                if (dn * dn + de * de >
                    (double)GINS_NOGNSS_POSGUARD_KM * 1000.0 * (double)GINS_NOGNSS_POSGUARD_KM * 1000.0)
                {
                    LOG_W("播种位置守卫: 偏离配置点 %.0f km (镜像/跳变特征), "
                          "拆引擎重播种 (第 %u 次)",
                          sqrt(dn * dn + de * de) / 1000.0, g_reseed.cnt + 1u);
                    if (s_engine != RT_NULL)
                    {
                        NavState ns = s_engine->getNavState();
                        const GIEngine::UpdateStat &us = s_engine->updateStat();

                        LOG_W("守卫转储: pos=%.6f/%.6f/%.2f vel=%.3f/%.3f/%.3f "
                              "att=%.2f/%.2f/%.2f",
                              ns.pos[0] * R2D, ns.pos[1] * R2D, ns.pos[2],
                              ns.vel[0], ns.vel[1], ns.vel[2],
                              ns.euler[0] * R2D, ns.euler[1] * R2D,
                              ns.euler[2] * R2D);
                        LOG_W("守卫转储: upd ok=%u fail=%u covheal=%u | mag u/r/d=%u/%u/%u "
                              "baro u/r/d/s=%u/%u/%u/%u zupt u/r=%u/%u vreset=%u",
                              us.updok, us.updfail, us.covheal,
                              us.magupd, us.magrej, us.magdrop,
                              us.baroupd, us.barorej, us.barodrop, us.baroskip,
                              us.zuptupd, us.zuptrej, us.vreset);
                    }
                    g_posguard.armed = RT_FALSE;
                    gins_reseed_request("播种位置守卫", RT_TRUE);
                }
            }
        }
#endif /* GINS_NOGNSS_MODE */

        /* 6. 垂直失控看门狗 (~1Hz): GNSS 新鲜时 INS 高度偏离独立气压
         *    参考持续超限 -> 拆引擎重对准 (观测级门禁对滤波失健康全盲,
         *    见 gins_config.h "INS 垂直失控看门狗") */
        if (s_engine != RT_NULL && (loops & 0x3FFu) == 1u)
        {
            double ha;

            if (g_run.now_gpst - g_run.gnss_time < 2.0 &&
                gins_baro_alt_ref(timebase_now_us(), &ha))
            {
                if (fabs(g_sol.altitude - ha) > (double)GINS_ALTX_H_M)
                {
                    g_vwatch.over_t += 1024.0 / 1000.0;

                    if (g_vwatch.over_t > (double)GINS_VWATCH_HOLD_S)
                    {
                        LOG_W("垂直失控看门狗触发: INS 高度 %.0fm 偏离气压基准 "
                              "%.0fm 达 %.0fs (GNSS 正常)",
                              g_sol.altitude, ha, g_vwatch.over_t);
                        gins_reseed_request("垂直失控看门狗", RT_FALSE);
                        g_guard.armed = RT_FALSE;
                        g_vwatch.cnt++;
                    }
                }
                else
                {
                    g_vwatch.over_t = 0.0;
                }
            }
        }
    }
}

/* ------------------------- 对外接口 ------------------------- */

void gins_bridge_get_solution(struct gins_solution *out)
{
    rt_base_t level;

    if (out == RT_NULL)
        return;

    level = rt_hw_interrupt_disable();
    *out = g_sol;
    rt_hw_interrupt_enable(level);
}

rt_err_t gins_bridge_init(void)
{
    rt_thread_t tid;

    /* 引擎生命周期互斥 (FinSH 读端 vs reseed 拆建), 见 s_engine_lk 注释;
     * init 幂等 (autoinit 与可能的显式调用), 已就绪则跳过 */
    if (!s_engine_lk_ready)
    {
        if (rt_mutex_init(&s_engine_lk, "ginsegn", RT_IPC_FLAG_PRIO) == RT_EOK)
            s_engine_lk_ready = RT_TRUE;
        else
            LOG_W("engine lifecycle mutex init failed, FinSH 读端无保护");
    }

    /* 看门狗已由 gins_wdt.c 的 INIT_BOARD_EXPORT 更早启动 (早于任何
     * 可能卡死的组件初始化), 此处只保留喂狗 (解算线程循环) */
    gins_probe_gyro_range();

    /* 磁/气压采样线程: 100Hz I2C 轮询, 优先级低于解算线程 */
    tid = rt_thread_create("ginsaux", gins_aux_thread_entry, RT_NULL,
                           GINS_AUX_THREAD_STACK, GINS_AUX_THREAD_PRIO, GINS_AUX_THREAD_TICK);
    if (tid != RT_NULL)
        rt_thread_startup(tid);
    else
        LOG_W("ginsaux 线程创建失败, 磁/气压融合未接入");

    tid = rt_thread_create("gins", gins_thread_entry, RT_NULL,
                           GINS_THREAD_STACK, GINS_THREAD_PRIO, GINS_THREAD_TICK);
    if (tid == RT_NULL)
    {
        LOG_E("gins 线程创建失败");
        return -RT_ENOMEM;
    }
    rt_thread_startup(tid);

    LOG_I("GINS bridge: ADIS16505 + UM982 + BMM350(磁航向) + BMP585(气压高) "
          "-> KF-GINS (EKF 21 状态), 初始化需静止 %ums + GNSS 定位收敛稳定 "
          "(hdop<=%.1f 且连续 %u 样本互差达标) + PPS 时间映射就绪",
          (unsigned)(GINS_ALIGN_SAMPLES),
          (double)GINS_SEED_HDOP_MAX, (unsigned)GINS_SEED_STABLE_N);
    return RT_EOK;
}

static int gins_bridge_autoinit(void)
{
    gins_bridge_init();
    return 0;
}
INIT_ENV_EXPORT(gins_bridge_autoinit);

/* ------------------------- FinSH 调试命令 ------------------------- */

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>

static void gins(int argc, char **argv)
{
    struct gins_solution s;
    struct gnss_data gd;

    RT_UNUSED(argc);
    RT_UNUSED(argv);

    gins_bridge_get_solution(&s);
    um982_nmea_get_data(&gd);

    LOG_I("=== GINS (ADIS16505 + UM982 + BMM350 + BMP585 -> KF-GINS EKF) ===");
    LOG_I("status  : %s, gyro=%.2f mdps/LSB, fix=%d sats=%u",
          s.ready ? "RUNNING" : "ALIGN/WAIT",
          s.gyro_lsb_mdps, gd.status.fix_type, gd.status.satellites);
    if (s.ready)
    {
        LOG_I("position: lat=%.7f lon=%.7f alt=%.1f m",
              s.latitude, s.longitude, s.altitude);
        LOG_I("velocity: vn=%.3f ve=%.3f vd=%.3f m/s",
              s.vn, s.ve, s.vd);
        LOG_I("attitude: r=%.3f p=%.3f y=%.3f deg",
              s.roll, s.pitch, s.yaw);
        LOG_I("gnss age: %u ms", s.gnss_age_ms);
    }
    LOG_I("stats   : imu=%u gnss=%u stale=%u dr_drop=%u cov_warn=%u",
          s.imu_cnt, s.gnss_cnt, s.gnss_stale_cnt, s.drop_cnt, s.cov_warn_cnt);
    LOG_I("quality : obs rej hdop=%u jump=%u altx=%u | seed rej hdop=%u speed=%u jump=%u sanity=%u altx=%u (窗 %u/%u) | reseed=%u vwatch=%u",
          g_gq.rej_hdop, g_gq.rej_jump, g_gq.rej_altx,
          g_seed.rej_hdop, g_seed.rej_speed, g_seed.rej_jump, g_seed.rej_sanity,
          g_seed.rej_altx, g_seed.seq, (unsigned)GINS_SEED_STABLE_N,
          g_reseed.cnt, g_vwatch.cnt);
    LOG_I("mag/baro: mag=%u rej=%u stale=%u, baro=%u rej=%u stale=%u skip=%u, "
          "baro_h=%.1f m (alt=%.1f), decl=%.1f deg",
          s.mag_cnt, s.mag_rej_cnt, s.mag_stale_cnt,
          s.baro_cnt, s.baro_rej_cnt, s.baro_stale_cnt, s.baro_skip_cnt,
          s.baro_height, s.altitude, s.mag_decl_deg);
#if GINS_ZUPT_ENABLE && GINS_NOGNSS_MODE
    if (s.nognss || g_run.zupt_cnt != 0u)
    {
        unsigned int zupd = 0, zrej = 0;

        /* FinSH 线程读引擎: 引擎生命周期锁关闭与 reseed delete 的竞态 */
        if (s_engine_lk_ready)
            rt_mutex_take(&s_engine_lk, RT_WAITING_FOREVER);
        if (s_engine != RT_NULL)
        {
            const GIEngine::UpdateStat &us2 = s_engine->updateStat();
            zupd = us2.zuptupd;
            zrej = us2.zuptrej;
        }
        if (s_engine_lk_ready)
            rt_mutex_release(&s_engine_lk);
        LOG_I("zupt    : inject=%u (eng upd=%u rej=%u), static=%s "
              "(w²=%.1f n |f|-g=%.3f)",
              s.zupt_cnt, zupd, zrej,
              g_zupt.active ? "YES" : "no",
              g_zupt.w2_ema * 1e9, g_zupt.fdev_ema);
    }
#endif
    if (s.vreset_cnt != 0u)
        LOG_W("gins: 垂直新息持续超限, 已膨胀垂直协方差 %u 次 "
              "(查垂直观测权重/传感器健康)", s.vreset_cnt);
    if (s.nan_cnt != 0u)
        LOG_W("gins: 解出现过非有限值 %u 次, 当前%s", s.nan_cnt,
              s.degraded ? "降级中 (输出为上一拍有限值)" : "已恢复");
    if (s.updfail_cnt != 0u)
        LOG_W("gins: 观测更新 Cholesky 失败 %u 次 (P 非正定征兆, "
              "持续失败会自动重对准)", s.updfail_cnt);
    if (s.covheal_cnt != 0u)
        LOG_W("gins: 协方差自愈复位 %u 次 (负对角/NaN, P 已回初始方差)",
              s.covheal_cnt);
    if (s.reseed_sup_cnt != 0u)
        LOG_W("gins: reseed 退避压制 %u 次 (风暴限幅, 退避 x%u)",
              s.reseed_sup_cnt, g_reseed_ctl.backoff);
    if (s.ready)
    {
        float pd_avg, pd_max, up_avg, up_max;

        /* 单步 = 机械编排(1kHz) + EKF 更新; kfm = kf_math 内核耗时
         * (predict 为 ~100Hz 节流冲刷, update 为观测历元) */
        kf_math::stats(&pd_avg, &pd_max, &up_avg, &up_max);
        LOG_I("compute : step avg=%.3f ms max=%.3f ms | "
              "kfm pred %u/%u us, update %u/%u us (avg/max)",
              (double)(g_run.step_us_avg / 1000.0f), (double)(g_run.step_us_max / 1000.0f),
              (unsigned)(pd_avg + 0.5f), (unsigned)(pd_max + 0.5f),
              (unsigned)(up_avg + 0.5f), (unsigned)(up_max + 0.5f));
    }
    if (!s.ready)
        LOG_W("hint    : 初始化需同时满足: UM982 定位有效且收敛稳定 "
              "(hdop<=%.1f 稳定窗 %u/%u rej: hdop=%u speed=%u jump=%u sanity=%u altx=%u) + "
              "时间映射就绪 (FinSH `timebase` map=VALID) + 静止 %ums "
              "(保持设备静止, 用 `um982`/`timebase` 检查链路)",
              (double)GINS_SEED_HDOP_MAX, g_seed.seq,
              (unsigned)GINS_SEED_STABLE_N,
              g_seed.rej_hdop, g_seed.rej_speed, g_seed.rej_jump,
              g_seed.rej_sanity, g_seed.rej_altx,
              (unsigned)(GINS_ALIGN_SAMPLES));
}
MSH_CMD_EXPORT(gins, GINS 组合导航解算状态与结果);

/* C8 加计零偏管理: accbias [save|clear] */
static void accbias(int argc, char **argv)
{
    struct calib_data *cal = calib_store_ram();

    if (argc >= 2 && !rt_strcmp(argv[1], "save"))
    {
        rt_err_t err = gins_accbias_snapshot(0.0, RT_TRUE);

        LOG_I("accbias save: %s (%d)", err == RT_EOK ? "OK" : "failed", (int)err);
        return;
    }
    if (argc >= 2 && !rt_strcmp(argv[1], "clear"))
    {
        cal->acc_valid = RT_FALSE;
        rt_err_t err = calib_store_save();

        LOG_I("accbias clear: %s (%d)", err == RT_EOK ? "OK" : "failed", (int)err);
        return;
    }

    LOG_I("=== 加计零偏持久化 (C8) ===");
    LOG_I("stored  : %s (%.1f, %.1f, %.1f) mGal @%.1f°C",
          cal->acc_valid ? "VALID" : "none",
          (double)cal->acc_bias_mgal[0], (double)cal->acc_bias_mgal[1],
          (double)cal->acc_bias_mgal[2], (double)cal->acc_cal_temp);
    if (s_engine_lk_ready)
        rt_mutex_take(&s_engine_lk, RT_WAITING_FOREVER);
    if (s_engine != RT_NULL)
    {
        NavState ns = s_engine->getNavState();

        LOG_I("engine  : estimate (%.1f, %.1f, %.1f) mGal",
              ns.imuerror.accbias[0] * 1e5, ns.imuerror.accbias[1] * 1e5,
              ns.imuerror.accbias[2] * 1e5);
    }
    if (s_engine_lk_ready)
        rt_mutex_release(&s_engine_lk);
    LOG_I("autosnap: %s (引擎就绪 %us 后静止自动, 累计 %u 次)",
          g_accsnap.saved ? "本上电已存" : "等待条件",
          (unsigned)GINS_ACCBIAS_SAVE_S, g_accsnap.auto_cnt);
    LOG_I("usage   : accbias save | accbias clear");
}
MSH_CMD_EXPORT(accbias, accel bias persistence: accbias [save|clear]);
#endif

#endif /* GINS_BRIDGE_ENABLE */

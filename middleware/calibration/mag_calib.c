/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BMM350 磁力计椭球校准实现, 流程与数据链见 mag_calib.h。
 *
 * 拟合质量门限 (不满足任一条则拒绝采用, 保留旧参数):
 *   样本数/各轴覆盖/等效半径/主轴半径比/偏置模长/代数残差 各有上下限,
 *   数值按地磁场 25~65 µT 常见范围与工程经验取, 详见下方宏。
 */

#include <rtthread.h>
#include <string.h>
#include <math.h>

#include "mag_calib.h"
#include "param_calib.h"                 /* middleware/param_calib 持久化 (W25Q64) */
#include "ellipsoid_fit.h"
#include "mag_data.h"                   /* middleware/data 环形缓冲区 */

#define MAGCAL_TAG              "magcal"
#define LOG_TAG                 MAGCAL_TAG
#define LOG_LVL                 LOG_LVL_INFO
#include <ulog.h>

/* ------------------------- 配置 ------------------------- */

#define MAGCAL_WAIT_MS          200     /* 单次等新样本超时 */
#define MAGCAL_STARVE_MS        3000    /* 连续无样本判数据链路失效 */
#define MAGCAL_DEF_SECONDS      45      /* 默认采集时长 */
#define MAGCAL_THREAD_PRIO      13      /* 低于 gins(9)/magdata(11) */
#define MAGCAL_THREAD_STACK     4096    /* 9x9 法方程求解栈开销 */

#define MAGCAL_MIN_STEP_UT      2.0     /* 相邻采纳样本的最小位移 (µT) */
#define MAGCAL_MIN_SAMPLES      400     /* 最少采纳样本数 */
#define MAGCAL_MIN_AXIS_UT      15.0    /* 各轴最小覆盖量程 (µT), 保证姿态覆盖 */

#define MAGCAL_RADIUS_MIN_UT    15.0    /* 等效球半径合理范围 (地磁场) */
#define MAGCAL_RADIUS_MAX_UT    110.0
#define MAGCAL_MAX_RATIO        3.0     /* 主轴半径最大/最小比 (软磁畸变上限) */
#define MAGCAL_MAX_BIAS_UT      150.0   /* 硬磁偏置模长上限 */
#define MAGCAL_MAX_RESID        0.08    /* 代数残差 rms 上限 (相对量) */

/* ------------------------- 生效参数 (apply 热路径) ------------------------- */

static struct
{
    rt_bool_t enabled;
    rt_bool_t valid;
    float     bias_ut[3];
    float     softiron[3][3];
} s_par;

/* ------------------------- 采集状态 ------------------------- */

static struct
{
    rt_thread_t thread;
    volatile rt_bool_t cancel;
    struct ell_fit_acc acc;
    rt_uint32_t seconds;
} s_cal;

/* ------------------------- 应用接口 ------------------------- */

void mag_calib_apply(const double raw_ut[3], double out_ut[3])
{
    rt_base_t level;
    rt_bool_t use;
    float bias[3], m[3][3];

    level = rt_hw_interrupt_disable();
    use = (rt_bool_t)(s_par.enabled && s_par.valid);
    if (use)
    {
        memcpy(bias, s_par.bias_ut, sizeof(bias));
        memcpy(m, s_par.softiron, sizeof(m));
    }
    rt_hw_interrupt_enable(level);

    if (!use)
    {
        out_ut[0] = raw_ut[0];
        out_ut[1] = raw_ut[1];
        out_ut[2] = raw_ut[2];
        return;
    }

    /* 先整体升 double 再运算, 避免混合表达式逐次隐式提升 (m/bias 均为 float) */
    const double r0 = raw_ut[0], r1 = raw_ut[1], r2 = raw_ut[2];
    const double b0 = bias[0], b1 = bias[1], b2 = bias[2];

    for (int i = 0; i < 3; i++)
    {
        const double m0 = m[i][0], m1 = m[i][1], m2 = m[i][2];

        out_ut[i] = m0 * (r0 - b0)
                  + m1 * (r1 - b1)
                  + m2 * (r2 - b2);
    }
}

rt_bool_t mag_calib_active(void)
{
    return (rt_bool_t)(s_par.enabled && s_par.valid);
}

void mag_calib_get_status(struct mag_calib_status *st)
{
    rt_base_t level;

    memset(st, 0, sizeof(*st));

    level = rt_hw_interrupt_disable();
    st->valid    = s_par.valid;
    st->enabled  = s_par.enabled;
    st->busy     = (s_cal.thread != RT_NULL);
    if (s_par.valid)
    {
        for (int i = 0; i < 3; i++)
        {
            st->bias_ut[i] = s_par.bias_ut[i];
            for (int j = 0; j < 3; j++)
                st->softiron[i][j] = s_par.softiron[i][j];
        }
    }
    rt_hw_interrupt_enable(level);

    {
        struct calib_data *d = calib_store_ram();

        st->radius_ut = d->mag_radius_ut;
        st->resid     = d->mag_resid;
        st->maxratio  = d->mag_maxratio;
        st->samples   = d->mag_samples;
    }
}

/* 把 calib_store 镜像中的磁力计参数装进生效参数 */
static void mag_calib_load_from_store(void)
{
    struct calib_data *d = calib_store_ram();
    rt_base_t level;

    level = rt_hw_interrupt_disable();
    s_par.valid = d->mag_valid;
    if (d->mag_valid)
    {
        memcpy(s_par.bias_ut, d->mag_bias_ut, sizeof(s_par.bias_ut));
        memcpy(s_par.softiron, d->mag_softiron, sizeof(s_par.softiron));
    }
    rt_hw_interrupt_enable(level);
}

int mag_calib_init(void)
{
    (void)calib_store_init();
    mag_calib_load_from_store();
    s_par.enabled = RT_TRUE;

    if (s_par.valid)
        LOG_I("已加载椭球校准 (radius=%.1f uT, samples=%d)",
              (double)calib_store_ram()->mag_radius_ut,
              (int)calib_store_ram()->mag_samples);
    else
        LOG_I("无校准参数, 磁力计直通 (执行 magcal start 开始校准)");

    return 0;
}
INIT_COMPONENT_EXPORT(mag_calib_init);

/* ------------------------- 拟合与校验 ------------------------- */

static const char *fit_errstr(int rc)
{
    switch (rc)
    {
    case ELL_FIT_ERR_SAMPLES:  return "样本不足或姿态覆盖差";
    case ELL_FIT_ERR_SINGULAR: return "法方程病态";
    case ELL_FIT_ERR_SHAPE:    return "数据退化, 不是有效椭球";
    default:                   return "未知错误";
    }
}

/* 采集体 + 拟合结果 -> 门限校验 -> 写 store 并保存 -> 激活 */
static rt_err_t mag_calib_finish(void)
{
    struct ell_fit_result r;
    struct calib_data *d = calib_store_ram();
    double bias_norm, rmin, rmax;
    int rc;

    rc = ell_fit_solve(&s_cal.acc, &r);
    if (rc != ELL_FIT_OK)
    {
        LOG_E("椭球拟合失败: %s (samples=%d)", fit_errstr(rc), (int)r.samples);
        return -RT_ERROR;
    }

    bias_norm = sqrt(r.bias[0] * r.bias[0] + r.bias[1] * r.bias[1] +
                     r.bias[2] * r.bias[2]);
    rmin = rmax = r.radii[0];
    for (int i = 1; i < 3; i++)
    {
        if (r.radii[i] < rmin) rmin = r.radii[i];
        if (r.radii[i] > rmax) rmax = r.radii[i];
    }

    /* 门限逐项检查, 失败给出原因, 保留旧参数 */
    if (r.samples < MAGCAL_MIN_SAMPLES)
    {
        LOG_E("样本不足: %d < %d", (int)r.samples, MAGCAL_MIN_SAMPLES);
        return -RT_ERROR;
    }
    for (int i = 0; i < 3; i++)
    {
        double cover = s_cal.acc.mx[i] - s_cal.acc.mn[i];

        if (cover < MAGCAL_MIN_AXIS_UT)
        {
            LOG_E("第 %d 轴覆盖 %.1f uT 不足 (%d uT), 请更多姿态旋转",
                  i, cover, (int)MAGCAL_MIN_AXIS_UT);
            return -RT_ERROR;
        }
    }
    if (r.radius < MAGCAL_RADIUS_MIN_UT || r.radius > MAGCAL_RADIUS_MAX_UT)
    {
        LOG_E("等效半径 %.1f uT 超出 [%d, %d]", r.radius,
              (int)MAGCAL_RADIUS_MIN_UT, (int)MAGCAL_RADIUS_MAX_UT);
        return -RT_ERROR;
    }
    if (rmax / rmin > MAGCAL_MAX_RATIO)
    {
        LOG_E("主轴半径比 %.2f > %.2f (软磁畸变过大或数据差)",
              rmax / rmin, MAGCAL_MAX_RATIO);
        return -RT_ERROR;
    }
    if (bias_norm > MAGCAL_MAX_BIAS_UT)
    {
        LOG_E("硬磁偏置模长 %.1f uT > %d", bias_norm, (int)MAGCAL_MAX_BIAS_UT);
        return -RT_ERROR;
    }
    if (r.resid_rms > MAGCAL_MAX_RESID)
    {
        LOG_E("拟合残差 %.3f > %.2f (数据噪声或磁干扰大)",
              r.resid_rms, MAGCAL_MAX_RESID);
        return -RT_ERROR;
    }

    LOG_I("拟合通过: bias=(%.1f %.1f %.1f) uT, radius=%.1f uT, "
          "ratio=%.2f, resid=%.4f, samples=%d",
          r.bias[0], r.bias[1], r.bias[2], r.radius,
          rmax / rmin, r.resid_rms, (int)r.samples);

    d->mag_valid = RT_TRUE;
    for (int i = 0; i < 3; i++)
    {
        d->mag_bias_ut[i] = (float)r.bias[i];
        for (int j = 0; j < 3; j++)
            d->mag_softiron[i][j] = (float)r.softiron[i][j];
    }
    d->mag_radius_ut = (float)r.radius;
    d->mag_resid     = (float)r.resid_rms;
    d->mag_maxratio  = (float)(rmax / rmin);
    d->mag_samples   = (r.samples > 65535u) ? 65535u : (rt_uint16_t)r.samples;

    if (calib_store_save() != RT_EOK)
    {
        d->mag_valid = RT_FALSE;
        return -RT_ERROR;
    }

    mag_calib_load_from_store();        /* 失败会回退上面的 valid=FALSE */
    if (!d->mag_valid)
        return -RT_ERROR;

    LOG_I("校准参数已保存并生效");
    return RT_EOK;
}

/* ------------------------- 采集线程 ------------------------- */

static void magcal_thread_entry(void *parameter)
{
    struct mag_sample s;
    double last[3] = { 0, 0, 0 };
    int have_last = 0;
    rt_tick_t start;
    rt_uint32_t accepted = 0, report = 0, starve_ms = 0;

    RT_UNUSED(parameter);

    {
        struct mag_data_status st;

        mag_data_get_status(&st);
        if (!st.running)
        {
            LOG_E("mag_data 数据链路未运行 (查 FinSH `magdata` 与 BMM350 接线), "
                  "校准中止");
            goto _exit;
        }
    }

    /* 丢弃积压旧样本, 从当前时刻开始采集 */
    mag_data_flush();

    LOG_I("开始采集 %u 秒: 请手持整机缓慢旋转/翻转, 覆盖所有姿态 "
          "(画 8 字 + 各轴朝天/朝地)", (unsigned)s_cal.seconds);

    start = rt_tick_get();

    while (!s_cal.cancel &&
           (rt_tick_get() - start) < rt_tick_from_millisecond(s_cal.seconds * 1000u))
    {
        /* 阻塞等新样本 (100Hz 生产者), 取一个即处理 */
        if (mag_data_wait(MAGCAL_WAIT_MS) != RT_EOK ||
            mag_data_pop(&s) != RT_EOK)
        {
            starve_ms += MAGCAL_WAIT_MS;
            if (starve_ms >= MAGCAL_STARVE_MS)
            {
                LOG_E("连续 %d ms 无磁力计样本, 数据链路中断, 校准中止",
                      (int)MAGCAL_STARVE_MS);
                goto _exit;
            }
            continue;
        }
        starve_ms = 0;

        {
            double dx = (double)s.mag[0] - last[0];
            double dy = (double)s.mag[1] - last[1];
            double dz = (double)s.mag[2] - last[2];

            if (have_last && sqrt(dx * dx + dy * dy + dz * dz) < MAGCAL_MIN_STEP_UT)
                continue;

            ell_fit_add(&s_cal.acc, (double)s.mag[0], (double)s.mag[1],
                        (double)s.mag[2]);
            last[0] = s.mag[0];
            last[1] = s.mag[1];
            last[2] = s.mag[2];
            have_last = 1;
            accepted++;
        }

        if (accepted >= (report + 100u))        /* 每采纳 100 点汇报一次 */
        {
            report = accepted;
            LOG_I("采集进行中: %d 点, 轴覆盖 [%d %d %d] uT", (int)accepted,
                  (int)(s_cal.acc.mx[0] - s_cal.acc.mn[0]),
                  (int)(s_cal.acc.mx[1] - s_cal.acc.mn[1]),
                  (int)(s_cal.acc.mx[2] - s_cal.acc.mn[2]));
        }
    }

    if (s_cal.cancel)
    {
        LOG_W("校准已取消, 参数未改动");
        goto _exit;
    }

    LOG_I("采集结束: 共 %d 点, 开始椭球拟合", (int)accepted);
    (void)mag_calib_finish();

_exit:
    s_cal.thread = RT_NULL;
}

/* ------------------------- 采集控制 ------------------------- */

rt_err_t mag_calib_start(rt_uint32_t seconds)
{
    if (s_cal.thread != RT_NULL)
        return -RT_EBUSY;

    if (seconds == 0)
        seconds = MAGCAL_DEF_SECONDS;

    ell_fit_reset(&s_cal.acc);
    s_cal.cancel = RT_FALSE;
    s_cal.seconds = seconds;

    s_cal.thread = rt_thread_create(MAGCAL_TAG, magcal_thread_entry, RT_NULL,
                                    MAGCAL_THREAD_STACK, MAGCAL_THREAD_PRIO, 10);
    if (s_cal.thread == RT_NULL)
        return -RT_ENOMEM;
    rt_thread_startup(s_cal.thread);

    return RT_EOK;
}

void mag_calib_cancel(void)
{
    if (s_cal.thread != RT_NULL)
        s_cal.cancel = RT_TRUE;
}

rt_err_t mag_calib_set_enable(rt_bool_t on)
{
    if (on && !s_par.valid)
        return -RT_ERROR;

    s_par.enabled = on;
    return RT_EOK;
}

/* ------------------------- SWD 触发 (无串口环境) ------------------------- */

/* 本机调试不接串口控制台, FinSH 命令不可达, 采集由 SWD 写此变量触发:
 *   SWD 写采集秒数 (1~65535) -> 启动采集 (等效 magcal start <sec>)
 *   SWD 写 0xFFFFFFFF        -> 取消进行中的采集 (等效 magcal stop)
 *   SWD 写 0xFFFFFFF1        -> 把 calib RAM 镜像原样存盘 (等效手动
 *                               save; 用于 SWD 回填镜像后持久化恢复)
 * 固件消费后清零; 采集进度/结果经 SWD 直读 s_cal / calib_store 镜像。 */
volatile rt_uint32_t g_magcal_swd_req;

static void magcal_swd_thread_entry(void *parameter)
{
    RT_UNUSED(parameter);

    while (1)
    {
        rt_uint32_t req = g_magcal_swd_req;

        if (req != 0u)
        {
            g_magcal_swd_req = 0u;
            if (req == 0xFFFFFFFFu)
                mag_calib_cancel();
            else if (req == 0xFFFFFFF1u)
                (void)calib_store_save();
            else
                (void)mag_calib_start(req);
        }
        rt_thread_mdelay(200);
    }
}

static int magcal_swd_init(void)
{
    rt_thread_t t = rt_thread_create("magswd", magcal_swd_thread_entry, RT_NULL,
                                     1024, MAGCAL_THREAD_PRIO, 10);

    if (t != RT_NULL)
        rt_thread_startup(t);
    return 0;
}
INIT_APP_EXPORT(magcal_swd_init);

/* ------------------------- FinSH 命令 ------------------------- */

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>

static void magcal(int argc, char **argv)
{
    if (argc >= 2)
    {
        if (!rt_strcmp(argv[1], "start"))
        {
            rt_uint32_t sec = (argc >= 3) ? (rt_uint32_t)calib_parse_num(argv[2]) : 0;

            if (mag_calib_start(sec) == RT_EOK)
                LOG_I("magcal: 采集中, 期间请覆盖所有姿态");
            else
                LOG_W("magcal: 已有校准在进行");
            return;
        }
        if (!rt_strcmp(argv[1], "stop"))
        {
            mag_calib_cancel();
            LOG_I("magcal: 取消请求已发出");
            return;
        }
        if (!rt_strcmp(argv[1], "on") || !rt_strcmp(argv[1], "off"))
        {
            rt_bool_t on = (argv[1][1] == 'n');

            if (mag_calib_set_enable(on) == RT_EOK)
                LOG_I("magcal: %s", on ? "已启用" : "已停用(直通)");
            else
                LOG_W("magcal: 无有效参数, 无法启用");
            return;
        }
        if (!rt_strcmp(argv[1], "clear"))
        {
            struct calib_data *d = calib_store_ram();

            d->mag_valid = RT_FALSE;
            if (calib_store_save() == RT_EOK)
            {
                s_par.valid = RT_FALSE;
                s_par.enabled = RT_TRUE;
                LOG_I("magcal: 已清除磁力计校准");
            }
            else
                LOG_E("magcal: flash 写入失败");
            return;
        }
    }

    {
        struct mag_calib_status st;

        mag_calib_get_status(&st);

        LOG_I("=== magcal (BMM350 椭球校准) ===");
        LOG_I("state : %s%s",
              st.busy ? "COLLECTING" :
              (st.valid && st.enabled ? "ACTIVE" :
               (st.valid ? "VALID, disabled" : "NO PARAMS (直通)")),
              st.busy ? ", magcal stop 可取消" : "");
        if (st.valid)
        {
            LOG_I("bias  : (%.2f %.2f %.2f) uT",
                  st.bias_ut[0], st.bias_ut[1], st.bias_ut[2]);
            LOG_I("radius: %.1f uT, axis ratio %.2f, resid %.4f, samples %d",
                  st.radius_ut, st.maxratio, st.resid, (int)st.samples);
            LOG_I("softiron S (row-major):");
            for (int i = 0; i < 3; i++)
                LOG_I("  %.4f %.4f %.4f",
                      st.softiron[i][0], st.softiron[i][1], st.softiron[i][2]);
        }
        LOG_I("usage : magcal start [sec] | stop | on/off | clear");
    }
}
MSH_CMD_EXPORT(magcal, BMM350 ellipsoid calib: start [sec] / stop / show / on / off / clear);
#endif /* RT_USING_FINSH && FINSH_USING_MSH */

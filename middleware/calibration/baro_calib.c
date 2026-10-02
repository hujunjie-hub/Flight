/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BMP585 气压计基准偏移校准实现, 流程见 baro_calib.h。
 *
 * 校准门限:
 *   采集窗口气压标准差 <= 12 Pa (约 1 m, 超出判为未静置/风扰/天气突变);
 *   |偏移| <= 2500 Pa (25 hPa, 超出说明参考值给错或器件故障)。
 */

#include <rtthread.h>
#include <string.h>
#include <math.h>

#include "baro_calib.h"
#include "calib_store.h"
#include "baro_data.h"                 /* middleware/data 环形缓冲区 */

#define BAROCAL_TAG             "barocal"
#define LOG_TAG                 BAROCAL_TAG
#define LOG_LVL                 LOG_LVL_INFO
#include <ulog.h>

/* ------------------------- 配置 ------------------------- */

#define BAROCAL_WAIT_MS         200     /* 单次等新样本超时 */
#define BAROCAL_STARVE_MS       3000    /* 连续无样本判数据链路失效 */
#define BAROCAL_DEF_SECONDS     30      /* 默认采集时长 */
#define BAROCAL_THREAD_PRIO     13
#define BAROCAL_THREAD_STACK    2048

#define BAROCAL_MAX_STD_PA      12.0    /* 静置判据: 窗口内气压标准差上限 */
#define BAROCAL_MAX_OFF_PA      2500.0  /* 偏移绝对值上限 */

/* ------------------------- 生效参数 ------------------------- */

static struct
{
    rt_bool_t enabled;
    rt_bool_t valid;
    float     offset_pa;                /* 32 位读取原子, apply 无需加锁 */
} s_par;

/* 参考值 (FinSH 设定, 只在标定线程读) */
static struct
{
    rt_bool_t set;
    double    pa;
} s_ref;

/* ------------------------- 采集状态 ------------------------- */

static struct
{
    rt_thread_t thread;
    volatile rt_bool_t cancel;
    rt_uint32_t seconds;
} s_cal;

/* ------------------------- 应用接口 ------------------------- */

double baro_calib_apply(double pa_raw)
{
    if (s_par.enabled && s_par.valid)
        return pa_raw + (double)s_par.offset_pa;

    return pa_raw;
}

rt_bool_t baro_calib_active(void)
{
    return (rt_bool_t)(s_par.enabled && s_par.valid);
}

void baro_calib_get_status(struct baro_calib_status *st)
{
    struct calib_data *d = calib_store_ram();

    memset(st, 0, sizeof(*st));

    st->valid     = s_par.valid;
    st->enabled   = s_par.enabled;
    st->busy      = (s_cal.thread != RT_NULL);
    st->offset_pa = s_par.valid ? (double)d->baro_offset_pa : 0.0;
    st->cal_temp  = (double)d->baro_cal_temp;
    st->mean_pa   = (double)d->baro_mean_pa;
}

static void baro_calib_load_from_store(void)
{
    struct calib_data *d = calib_store_ram();

    s_par.valid     = d->baro_valid;
    s_par.offset_pa = d->baro_offset_pa;
}

int baro_calib_init(void)
{
    (void)calib_store_init();
    baro_calib_load_from_store();
    s_par.enabled = RT_TRUE;

    if (s_par.valid)
        LOG_I("已加载偏移 %.1f Pa (标定温度 %.1f C)",
              (double)calib_store_ram()->baro_offset_pa,
              (double)calib_store_ram()->baro_cal_temp);
    else
        LOG_I("无校准参数, 气压计直通 (barocal ref/refalt + start 开始校准)");

    return 0;
}
INIT_COMPONENT_EXPORT(baro_calib_init);

/* ------------------------- 参考值 ------------------------- */

void baro_calib_set_ref(double ref_pa)
{
    s_ref.pa = ref_pa;
    s_ref.set = RT_TRUE;
}

double baro_calib_ref_from_alt(double alt_m)
{
    double t;

    /* ISA: p = p0 (1 - 2.25577e-5 h)^5.25588, 对流层内 (<11 km) 有效 */
    t = 1.0 - 2.25577e-5 * alt_m;
    if (t < 0.1)
        t = 0.1;

    return 101325.0 * pow(t, 5.25588);
}

/* ------------------------- 采集与标定 ------------------------- */

static void barocal_thread_entry(void *parameter)
{
    struct baro_sample s;
    rt_tick_t start;
    rt_uint32_t n = 0, starve_ms = 0;
    double sum = 0.0, sumsq = 0.0, tsum = 0.0;

    RT_UNUSED(parameter);

    {
        struct baro_data_status st;

        baro_data_get_status(&st);
        if (!st.running)
        {
            LOG_E("baro_data 数据链路未运行 (查 FinSH `barodata` 与 BMP585 接线), "
                  "校准中止");
            goto _exit;
        }
    }

    /* 丢弃积压旧样本, 从当前时刻开始采集 */
    baro_data_flush();

    LOG_I("开始静置采集 %u 秒 (请保持设备静止、避风口)",
          (unsigned)s_cal.seconds);

    start = rt_tick_get();

    while (!s_cal.cancel &&
           (rt_tick_get() - start) < rt_tick_from_millisecond(s_cal.seconds * 1000u))
    {
        /* 阻塞等新样本 (100Hz 生产者), 逐个累计气压均值/方差与芯片温度 */
        if (baro_data_wait(BAROCAL_WAIT_MS) != RT_EOK ||
            baro_data_pop(&s) != RT_EOK)
        {
            starve_ms += BAROCAL_WAIT_MS;
            if (starve_ms >= BAROCAL_STARVE_MS)
            {
                LOG_E("连续 %d ms 无气压样本, 数据链路中断, 校准中止",
                      (int)BAROCAL_STARVE_MS);
                goto _exit;
            }
            continue;
        }
        starve_ms = 0;

        sum += (double)s.pressure_pa;
        sumsq += (double)s.pressure_pa * (double)s.pressure_pa;
        tsum += (double)s.temperature_c;    /* 随气压样本带出 (baro_data 设计) */
        n++;
    }

    if (s_cal.cancel)
    {
        LOG_W("校准已取消, 参数未改动");
        goto _exit;
    }

    if (n < 20)
    {
        LOG_E("有效样本不足 (%d)", (int)n);
        goto _exit;
    }

    {
        double mean = sum / n;
        double std = sqrt(sumsq / n - mean * mean);

        if (std > BAROCAL_MAX_STD_PA)
        {
            LOG_E("窗口气压标准差 %.1f Pa > %.1f, 未静置或环境突变",
                  std, BAROCAL_MAX_STD_PA);
            goto _exit;
        }

        LOG_I("采集结束: 均值 %.1f Pa, 标准差 %.1f Pa, 芯片温度 %.1f C, %d 点",
              mean, std, tsum / n, (int)n);

        if (!s_ref.set)
        {
            LOG_W("未设参考值 (barocal ref <pa> 或 barocal refalt <m>), "
                  "本次只打印统计, 不计算偏移");
            goto _exit;
        }

        {
            double off = s_ref.pa - mean;

            if (fabs(off) > BAROCAL_MAX_OFF_PA)
            {
                LOG_E("偏移 %.1f Pa 超出 ±%.1f, 检查参考值是否正确",
                      off, BAROCAL_MAX_OFF_PA);
                goto _exit;
            }

            {
                struct calib_data *d = calib_store_ram();

                d->baro_valid     = RT_TRUE;
                d->baro_offset_pa = (float)off;
                d->baro_cal_temp  = (float)(tsum / n);
                d->baro_mean_pa   = (float)mean;

                if (calib_store_save() != RT_EOK)
                {
                    d->baro_valid = RT_FALSE;
                    goto _exit;
                }

                baro_calib_load_from_store();
                LOG_I("气压偏移 %.1f Pa 已保存并生效 (参考 %.1f Pa)",
                      off, s_ref.pa);
            }
        }
    }

_exit:
    s_cal.thread = RT_NULL;
}

/* ------------------------- 采集控制 ------------------------- */

rt_err_t baro_calib_start(rt_uint32_t seconds)
{
    if (s_cal.thread != RT_NULL)
        return -RT_EBUSY;

    if (seconds == 0)
        seconds = BAROCAL_DEF_SECONDS;

    s_cal.cancel = RT_FALSE;
    s_cal.seconds = seconds;

    s_cal.thread = rt_thread_create(BAROCAL_TAG, barocal_thread_entry, RT_NULL,
                                    BAROCAL_THREAD_STACK, BAROCAL_THREAD_PRIO, 10);
    if (s_cal.thread == RT_NULL)
        return -RT_ENOMEM;
    rt_thread_startup(s_cal.thread);

    return RT_EOK;
}

void baro_calib_cancel(void)
{
    if (s_cal.thread != RT_NULL)
        s_cal.cancel = RT_TRUE;
}

rt_err_t baro_calib_set_enable(rt_bool_t on)
{
    if (on && !s_par.valid)
        return -RT_ERROR;

    s_par.enabled = on;
    return RT_EOK;
}

/* ------------------------- FinSH 命令 ------------------------- */

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>

static void barocal(int argc, char **argv)
{
    if (argc >= 3 && !rt_strcmp(argv[1], "ref"))
    {
        baro_calib_set_ref(calib_parse_num(argv[2]));
        LOG_I("barocal: 参考气压 %d Pa 已设定", (int)calib_parse_num(argv[2]));
        return;
    }
    if (argc >= 3 && !rt_strcmp(argv[1], "refalt"))
    {
        double pa = baro_calib_ref_from_alt(calib_parse_num(argv[2]));

        baro_calib_set_ref(pa);
        LOG_I("barocal: 海拔 %d m -> ISA 参考气压 %d Pa",
              (int)calib_parse_num(argv[2]), (int)pa);
        return;
    }
    if (argc >= 2)
    {
        if (!rt_strcmp(argv[1], "start"))
        {
            rt_uint32_t sec = (argc >= 3) ? (rt_uint32_t)calib_parse_num(argv[2]) : 0;

            if (baro_calib_start(sec) == RT_EOK)
                LOG_I("barocal: 采集中, 请保持静止");
            else
                LOG_W("barocal: 已有校准在进行");
            return;
        }
        if (!rt_strcmp(argv[1], "stop"))
        {
            baro_calib_cancel();
            LOG_I("barocal: 取消请求已发出");
            return;
        }
        if (!rt_strcmp(argv[1], "on") || !rt_strcmp(argv[1], "off"))
        {
            rt_bool_t on = (argv[1][1] == 'n');

            if (baro_calib_set_enable(on) == RT_EOK)
                LOG_I("barocal: %s", on ? "已启用" : "已停用(直通)");
            else
                LOG_W("barocal: 无有效参数, 无法启用");
            return;
        }
        if (!rt_strcmp(argv[1], "clear"))
        {
            struct calib_data *d = calib_store_ram();

            d->baro_valid = RT_FALSE;
            if (calib_store_save() == RT_EOK)
            {
                s_par.valid = RT_FALSE;
                s_par.enabled = RT_TRUE;
                LOG_I("barocal: 已清除气压计校准");
            }
            else
                LOG_E("barocal: flash 写入失败");
            return;
        }
    }

    {
        struct baro_calib_status st;

        baro_calib_get_status(&st);

        LOG_I("=== barocal (BMP585 基准偏移) ===");
        LOG_I("state : %s%s, ref %s",
              st.busy ? "COLLECTING" :
              (st.valid && st.enabled ? "ACTIVE" :
               (st.valid ? "VALID, disabled" : "NO PARAMS (直通)")),
              st.busy ? ", barocal stop 可取消" : "",
              s_ref.set ? "已设" : "未设");
        if (st.valid)
            LOG_I("offset: %.1f Pa, 标定均值 %.1f Pa @ %.1f C",
                  st.offset_pa, st.mean_pa, st.cal_temp);
        LOG_I("usage : barocal ref <pa> | refalt <m> | start [sec] | "
              "stop | on/off | clear");
    }
}
MSH_CMD_EXPORT(barocal, BMP585 baro calib: ref/refalt/start/stop/on/off/clear);
#endif /* RT_USING_FINSH && FINSH_USING_MSH */

/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ADIS16505 环形缓冲区原始数据 -> 带标记文本打印 (USART1 调试口)
 *
 * 数据来源: middleware/data 的 imu_data 环形缓冲区 (1kHz 未校准原始样本,
 *   快照打 UTC 微秒时间标签并完成单位换算), imu_data_wait/pop 取数,
 *   与 gins 桥接消费同一个缓冲区。
 *
 * 输出 (USART1, 与 console/VOFA 同口 460800 8N1), 每块一行带标记文本 (~125B):
 *   time:111.428619 dt:0.000999 gx:0.010471 gy:-0.002617 gz:-0.003054 \
 *   ax:0.782437 ay:-0.092625 az:9.542812 temp:27.200000
 *     time    块首样本 T_event (T_MCU 秒), 6 位小数 = µs
 *     dt      块内采样间隔均值 s (由 data_cnt 差分换算)
 *     gx..gz  陀螺 rad/s 块均值 (未校准)  ax..az 加速度 m/s^2 块均值 (未校准)
 *     temp    IMU 内部温度 °C 块均值
 *   块均值: 每弹出的 dec 个样本求平均后打印一行, 白噪声按 sqrt(dec) 缩小
 *   (dec=8 -> /2.8, 微振动显示也被抹平), 1kHz 原始流 (imu_data -> gins
 *   EKF) 不受影响。实测 X/Y 陀螺噪声主要是桌面微振动 (Z 轴 0.10°/s@1kHz
 *   已达器件标称), 想再降显示噪声用 `imuout dec 20|40`。
 *   默认 dec=8 = 125 行/s, 约 35% 带宽含 VOFA; dec=1 全速 1kHz 超 460800
 *   带宽, 阻塞写反压, imu_data 缓冲区会挤掉最旧样本。
 *
 * 纯调试输出口: 只写不复用配置、不开 RX 中断/回调 (console 自身的中断
 *   接收不受影响)。浮点打印走 app_out_cat_fx 定点拆分 (见 app_out.c)。
 *
 * FinSH 命令: imuout [on|off|dec N]
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <math.h>

#include "imu_data.h"              /* imu_data_wait/pop(): ADIS16505 环形缓冲区原始样本 */
#include "app_out.h"               /* app_out_write()/app_out_cat_fx(): 共享写与定点格式化 */

#define LOG_TAG "imuout"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

#define IMUOUT_ENABLE           0               /* 0=不启动 IMU 打印链路 (当前只测 UM982) */
#define IMUOUT_DEV_NAME         "uart1"         /* 与 console/VOFA 同口 */
#define IMUOUT_DECIMATE         8               /* 块均值窗口: N 个样本平均成 1 行 (125Hz) */
#define IMUOUT_WAIT_MS          1000            /* 无数据等待超时 */
#define IMUOUT_THREAD_PRIO      13              /* 低于 vofa(12), 高于 FinSH(20) */
#define IMUOUT_THREAD_STACK     4096
#define IMUOUT_THREAD_TICK      10

static struct
{
    rt_device_t uart;
    rt_thread_t thread;

    rt_bool_t   on;             /* FinSH 开关 */
    rt_uint32_t dec;            /* 当前块均值窗口 (样本数) */
    rt_uint32_t lines;          /* 累计打印行数 */
    rt_uint32_t samples;        /* 累计消费样本数 */
} imuout_ctx;

#if IMUOUT_ENABLE
/* 整行一次写出: 串口框架发送路径有锁, 行不会被日志插断 */
static void imuout_line(const struct imu_sample *s, float dt)
{
    static const char *gyro_lab[3] = { "gx", "gy", "gz" };
    static const char *acce_lab[3] = { "ax", "ay", "az" };
    char buf[192];
    rt_size_t off;
    rt_uint8_t i;
    int len;

    /* time: T_MCU 微秒直接拆 秒.微秒 */
    len = rt_snprintf(buf, sizeof(buf), "time:%u.%06u",
                      (rt_uint32_t)(s->T_event / 1000000u),
                      (rt_uint32_t)(s->T_event % 1000000u));
    off = (len > 0) ? (rt_size_t)len : 0u;

    app_out_cat_fx(buf, sizeof(buf), &off, "dt", dt);
    for (i = 0; i < 3; i++)
        app_out_cat_fx(buf, sizeof(buf), &off, gyro_lab[i], s->gyro[i]);
    for (i = 0; i < 3; i++)
        app_out_cat_fx(buf, sizeof(buf), &off, acce_lab[i], s->accel[i]);
    app_out_cat_fx(buf, sizeof(buf), &off, "temp", s->temperature);

    buf[off++] = '\r';
    buf[off++] = '\n';
    app_out_write(imuout_ctx.uart, buf, off);
}

static void imuout_thread_entry(void *parameter)
{
    struct imu_sample s;
    struct imu_sample acc;                /* 块累加和 */
    float dtsum;                          /* 块内 dt 累加 (data_cnt 差分) */
    rt_uint64_t t0;                       /* 块首样本时间戳 */
    rt_uint32_t n = 0;
    rt_bool_t dc_init = RT_FALSE;         /* data_cnt 差分基准 (dt 换算) */
    rt_uint32_t last_dc = 0;

    RT_UNUSED(parameter);

    /* imu_data 采集链路就绪等待 (IMU 未接线/禁用时在此空转, 不占带宽) */
    while (1)
    {
        struct imu_data_status st;

        imu_data_get_status(&st);
        if (st.running)
            break;
        rt_thread_mdelay(50);
    }

    /*
     * 块均值输出: 每弹出 dec 个样本求一次平均再打印一行, 白噪声按
     * sqrt(dec) 缩小 (dec=8 -> /2.8), 桌面微振动在显示上也被抹平;
     * 1kHz 原始流 (imu_data 缓冲区 -> gins) 不受影响。行时间戳取块首
     * 样本, dt 为块内采样间隔均值 (data_cnt 差分按名义 ODR 换算)。
     */
    while (1)
    {
        imu_data_wait(IMUOUT_WAIT_MS);

        while (imu_data_pop(&s) == RT_EOK)
        {
            rt_uint8_t i;
            rt_uint32_t dc;

            if (!imuout_ctx.on)
            {
                n = 0;                     /* 关闭时只排水, 丢弃半块 */
                continue;
            }

            /* dt 由 data_cnt 差分换算 (样本不再携带 dt) */
            dc = dc_init ? (s.data_cnt - last_dc) : 1u;
            if (!dc_init)
                dc_init = RT_TRUE;
            if (dc == 0u || dc > 100u)
                dc = 1u;
            last_dc = s.data_cnt;

            if (n == 0)
            {
                acc = s;
                dtsum = 0.0f;
                t0 = s.T_event;
            }
            else
            {
                for (i = 0; i < 3; i++)
                {
                    acc.gyro[i]  += s.gyro[i];
                    acc.accel[i] += s.accel[i];
                }
                acc.temperature += s.temperature;
            }
            dtsum += (float)dc * 0.001f;
            n++;
            imuout_ctx.samples++;

            if (n < imuout_ctx.dec)
                continue;

            {
                float inv = 1.0f / (float)n;

                for (i = 0; i < 3; i++)
                {
                    acc.gyro[i]  *= inv;
                    acc.accel[i] *= inv;
                }
                acc.temperature *= inv;
                acc.T_event      = t0;

                imuout_line(&acc, dtsum * inv);
                imuout_ctx.lines++;
            }
            n = 0;
        }
    }
}
#endif /* IMUOUT_ENABLE */

/* ---------------------------- 初始化 ---------------------------- */

int imuout_link_init(void)
{
#if IMUOUT_ENABLE
    imuout_ctx.uart = rt_device_find(IMUOUT_DEV_NAME);
    if (imuout_ctx.uart == RT_NULL)
    {
        LOG_E("imuout: uart \"%s\" not found", IMUOUT_DEV_NAME);
        return -RT_ERROR;
    }
    /* 纯调试输出口: 只写, 不带任何中断/接收标志; console 已打开则复用 */
    if (!(imuout_ctx.uart->open_flag & RT_DEVICE_OFLAG_OPEN))
    {
        if (rt_device_open(imuout_ctx.uart, RT_DEVICE_OFLAG_WRONLY) != RT_EOK)
        {
            LOG_E("imuout: open uart \"%s\" failed", IMUOUT_DEV_NAME);
            imuout_ctx.uart = RT_NULL;
            return -RT_ERROR;
        }
    }

    imuout_ctx.on = RT_TRUE;
    imuout_ctx.dec = IMUOUT_DECIMATE;

    imuout_ctx.thread = rt_thread_create("imuout", imuout_thread_entry, RT_NULL,
                                         IMUOUT_THREAD_STACK, IMUOUT_THREAD_PRIO,
                                         IMUOUT_THREAD_TICK);
    if (imuout_ctx.thread == RT_NULL)
        return -RT_ENOMEM;
    rt_thread_startup(imuout_ctx.thread);

    LOG_I("imuout: IMU raw tagged text %d Hz (dec=%d) on %s @%s "
          "(time/dt/gx../ax../temp)",
          1000 / IMUOUT_DECIMATE, IMUOUT_DECIMATE, IMUOUT_DEV_NAME,
          APP_OUT_BAUD_TEXT);
#endif /* IMUOUT_ENABLE */
    return RT_EOK;
}

/* ---------------------------- FinSH 命令 ---------------------------- */

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>

static void imuout(int argc, char **argv)
{
    if (argc >= 3 && !rt_strcmp(argv[1], "dec"))
    {
        const char *p = argv[2];
        rt_uint32_t d = 0;

        while (*p >= '0' && *p <= '9')
            d = d * 10u + (rt_uint32_t)(*p++ - '0');
        if (d >= 1u && *p == '\0')
        {
            imuout_ctx.dec = d;
            /* ulog 已开浮点支持 (标准版 rt_vsnprintf), 直接 %f */
            LOG_I("imuout dec = %d (%d 行/s @1kHz ODR, 噪声/%.1f)",
                  (int)d, 1000 / (int)d, sqrt((double)d));
            return;
        }
        LOG_W("usage: imuout dec N (N >= 1)");
        return;
    }
    if (argc >= 2 && !rt_strcmp(argv[1], "on"))
        imuout_ctx.on = RT_TRUE;
    else if (argc >= 2 && !rt_strcmp(argv[1], "off"))
        imuout_ctx.on = RT_FALSE;

    {
        struct imu_data_status st;

        imu_data_get_status(&st);
        LOG_I("=== IMU raw tagged print (USART1) ===");
        LOG_I("run     : %s, uart %s @ %s 8N1, %s",
              imuout_ctx.thread ? "yes" : "no", IMUOUT_DEV_NAME,
              APP_OUT_BAUD_TEXT, imuout_ctx.on ? "ON" : "OFF");
        LOG_I("dec     : %d (%d Hz @1kHz ODR), ~%d B/s",
              (int)imuout_ctx.dec, 1000 / (int)imuout_ctx.dec,
              125 * 1000 / (int)imuout_ctx.dec);
        LOG_I("stats   : lines=%u samples=%u (块均值 dec=%d, 白噪声/√%d)",
              imuout_ctx.lines, imuout_ctx.samples,
              (int)imuout_ctx.dec, (int)imuout_ctx.dec);
        LOG_I("imu     : pushed=%u popped=%u lost=%u errors=%u",
              st.pushed, st.popped, st.lost, st.errors);
        LOG_I("columns : time:.. dt:.. gx:.. gy:.. gz:.. ax:.. ay:.. az:.. temp:..");
        LOG_I("hint    : dec=1 全速会超 %s 带宽; 单样本细节看 `imudata`",
              APP_OUT_BAUD_TEXT);
    }
}
MSH_CMD_EXPORT(imuout, IMU raw tagged text on USART1: imuout [on|off|dec N]);

#endif /* RT_USING_FINSH && FINSH_USING_MSH */

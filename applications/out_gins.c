/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * KF-GINS 融合解算结果 -> 带标记文本打印 (USART1 调试口)
 *
 * 数据来源: middleware/gins 的 gins_bridge_get_solution() 解算快照
 *   (ADIS16505 1kHz + UM982 10Hz -> GIEngine EKF, 快照任意线程可读)。
 *   与 vofa JustFloat 二进制帧 (50Hz) 同源, 本链路输出可读文本便于调试。
 *
 * 输出 (USART1, 与 console/VOFA 同口 460800 8N1), 每行 (~186B):
 *   ready:1 time:1418000.123 roll:1.234568 pitch:-0.567890 yaw:89.012345 \
 *   vn:0.012014 ve:0.005036 vd:-0.001022 lat:22.6441457 lon:114.0135623 \
 *   alt:93.607 imu_data:1234567 gnss_data:78 mag_calib_data:901 \
 *   baro_calib_data:0 fused_data
 *     ready   0 = 对准/等定位中 (引擎未构造), 1 = 解算中
 *     time    引擎 GPST 累计秒 (自 GPS 纪元, 3 位小数 = ms)
 *     roll/pitch/yaw 姿态 deg (yaw KF-GINS 输出 [0,360) 非连续)
 *     vn/ve/vd NED 速度 m/s (D 轴向下为正)  lat/lon deg (7 位 ≈1cm)  alt 椭球高 m
 *     imu_data/gnss_data 已喂入引擎的原始观测计数, mag_calib_data/
 *     baro_calib_data 为 middleware/calibration 校正后的观测计数
 *     (imu=0 说明 ADIS16505 未出数)
 *   ready=1 时 10Hz (~1.75KB/s), ready=0 时降为 1Hz 心跳 (对齐/等定位
 *   期间不刷屏, 仍能看到 imu/gnss 计数判断卡在哪一环)。
 *
 * FinSH 命令: gins_fused_data [on|off]
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>

#include "gins_bridge.h"            /* gins_bridge_get_solution(): KF-GINS 融合解算快照 */
#include "app_out.h"                /* app_out_write()/app_out_cat_*(): 共享写与定点格式化 */

#define LOG_TAG "ginsout"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

#define GINSOUT_DEV_NAME        "uart1"         /* 与 console/VOFA 同口 */
#define GINSOUT_RATE_HZ         10              /* 解算中输出频率 */
#define GINSOUT_IDLE_PERIOD_MS  1000            /* ready=0 心跳周期 */
#define GINSOUT_THREAD_PRIO     12              /* 与 vofa(12) 同级: EKF(9) 满负荷时低于 14/16 会被饿死 */
#define GINSOUT_THREAD_STACK    2048
#define GINSOUT_THREAD_TICK     10

static struct
{
    rt_device_t uart;
    rt_thread_t thread;

    rt_bool_t   on;             /* FinSH 开关 */
    rt_uint32_t lines;          /* 累计打印行数 */
} gins_fused_data_ctx;

/* 整行一次写出并用共享互斥保护, 行不会被其他数据链插断 */
static void gins_fused_data_line(const struct gins_solution *s)
{
    char buf[320];
    rt_size_t off;
    rt_uint32_t sec = (rt_uint32_t)s->time;
    rt_uint32_t ms  = (rt_uint32_t)((s->time - (double)sec) * 1000.0);
    int len;

    len = rt_snprintf(buf, sizeof(buf), "ready:%u time:%u.%03u",
                      s->ready ? 1u : 0u, sec, ms);
    off = (len > 0) ? (rt_size_t)len : 0u;

    app_out_cat_fx(buf, sizeof(buf), &off, "roll",  (float)s->roll);
    app_out_cat_fx(buf, sizeof(buf), &off, "pitch", (float)s->pitch);
    app_out_cat_fx(buf, sizeof(buf), &off, "yaw",   (float)s->yaw);
    app_out_cat_fx(buf, sizeof(buf), &off, "vn",    (float)s->vn);
    app_out_cat_fx(buf, sizeof(buf), &off, "ve",    (float)s->ve);
    app_out_cat_fx(buf, sizeof(buf), &off, "vd",    (float)s->vd);
    app_out_cat_d7(buf, sizeof(buf), &off, "lat", s->latitude);
    app_out_cat_d7(buf, sizeof(buf), &off, "lon", s->longitude);
    app_out_cat_d3(buf, sizeof(buf), &off, "alt", s->altitude);

    len = rt_snprintf(buf + off, sizeof(buf) - off,
                      " imu_data:%u gnss_data:%u mag_calib_data:%u "
                      "baro_calib_data:%u fused_data",
                      s->imu_cnt, s->gnss_cnt, s->mag_cnt, s->baro_cnt);
    if (len > 0)
        off += (rt_size_t)len;
    if (off > sizeof(buf) - 2u)
        off = sizeof(buf) - 2u;

    buf[off++] = '\r';
    buf[off++] = '\n';
    app_out_write(gins_fused_data_ctx.uart, buf, off);
}

static void gins_fused_data_thread_entry(void *parameter)
{
    struct gins_solution sol;

    RT_UNUSED(parameter);
    memset(&sol, 0, sizeof(sol));

    while (1)
    {
        /* ready=1: 10Hz 全速; ready=0: 1Hz 心跳; off: 只休眠不打印 */
        rt_thread_mdelay(gins_fused_data_ctx.on
                         ? (sol.ready ? 1000u / GINSOUT_RATE_HZ : GINSOUT_IDLE_PERIOD_MS)
                         : 500u);

        if (!gins_fused_data_ctx.on)
            continue;

        gins_bridge_get_solution(&sol);
        gins_fused_data_line(&sol);
        gins_fused_data_ctx.lines++;
    }
}

/* ---------------------------- 初始化 ---------------------------- */

int gins_fused_data_link_init(void)
{
    gins_fused_data_ctx.uart = rt_device_find(GINSOUT_DEV_NAME);
    if (gins_fused_data_ctx.uart == RT_NULL)
    {
        LOG_E("gins_fused_data: uart \"%s\" not found", GINSOUT_DEV_NAME);
        return -RT_ERROR;
    }
    /* 纯调试输出口: 只写, 不带任何中断/接收标志; console 已打开则复用 */
    if (!(gins_fused_data_ctx.uart->open_flag & RT_DEVICE_OFLAG_OPEN))
    {
        if (rt_device_open(gins_fused_data_ctx.uart, RT_DEVICE_OFLAG_WRONLY) != RT_EOK)
        {
            LOG_E("gins_fused_data: open uart \"%s\" failed", GINSOUT_DEV_NAME);
            gins_fused_data_ctx.uart = RT_NULL;
            return -RT_ERROR;
        }
    }

    /* 默认关: uart1 TX 框架层无线程保护 (putc+completion 竞态会打烂
     * TX 环 -> 堆损坏 -> 随机线程 HardFault; 须单写者运行,
     * 需要时 FinSH `gins_fused_data on` 手动开启 */
    gins_fused_data_ctx.on = RT_FALSE;

    gins_fused_data_ctx.thread = rt_thread_create("gins_fused_data", gins_fused_data_thread_entry, RT_NULL,
                                          GINSOUT_THREAD_STACK, GINSOUT_THREAD_PRIO,
                                          GINSOUT_THREAD_TICK);
    if (gins_fused_data_ctx.thread == RT_NULL)
        return -RT_ENOMEM;
    rt_thread_startup(gins_fused_data_ctx.thread);

    LOG_I("gins_fused_data: KF-GINS tagged text %d Hz on %s @%s "
          "(ready/time/roll/pitch/yaw/vn/ve/vd/lat/lon/alt/"
          "imu_data/gnss_data/mag_calib_data/baro_calib_data)",
          GINSOUT_RATE_HZ, GINSOUT_DEV_NAME, APP_OUT_BAUD_TEXT);
    return RT_EOK;
}

/* ---------------------------- FinSH 命令 ---------------------------- */

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>

static void gins_fused_data(int argc, char **argv)
{
    if (argc >= 2 && !rt_strcmp(argv[1], "on"))
        gins_fused_data_ctx.on = RT_TRUE;
    else if (argc >= 2 && !rt_strcmp(argv[1], "off"))
        gins_fused_data_ctx.on = RT_FALSE;

    {
        struct gins_solution sol;

        gins_bridge_get_solution(&sol);
        LOG_I("=== KF-GINS tagged print (USART1) ===");
        LOG_I("run     : %s, uart %s @ %s 8N1, %s",
              gins_fused_data_ctx.thread ? "yes" : "no", GINSOUT_DEV_NAME,
              APP_OUT_BAUD_TEXT, gins_fused_data_ctx.on ? "ON" : "OFF");
        LOG_I("stats   : lines=%u", gins_fused_data_ctx.lines);
        LOG_I("gins    : %s, imu=%u gnss=%u mag=%u stale=%u drop=%u",
              sol.ready ? "RUNNING" : "ALIGN/WAIT",
              sol.imu_cnt, sol.gnss_cnt, sol.mag_cnt,
              sol.gnss_stale_cnt, sol.drop_cnt);
        LOG_I("hint    : ready=0 时看 imu/gnss 计数定位卡点; 细节看 `gins`");
    }
}
MSH_CMD_EXPORT(gins_fused_data, KF-GINS tagged text on USART1: gins_fused_data [on|off]);

#endif /* RT_USING_FINSH && FINSH_USING_MSH */

/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * UM982 GNSS 数据 -> 带标记文本打印 (USART1 调试口)
 *
 * 数据来源: middleware/data 的 gnss_data 环形缓冲区 (gnss_data_wait/pop,
 *   10Hz 结构化定位解, 样本自带统一 UTC 时间戳), 验证 UART4 INT_RX ->
 *   gnss_raw_data -> um982_nmea -> gnss_data 整条解析链路。
 *   注意: 环形缓冲区是 FIFO, 每个样本只能弹出一次, 本调试线程与 gins
 *   桥接线程 (优先级 9, 1ms 轮询排空) 消费同一个缓冲区 —— 本线程由信号量
 *   即时唤醒, 通常先于 gins 取走样本, 开启期间 GINS 的 GNSS 观测基本被
 *   分流 (解算退化为纯惯导递推, 属测试期预期); `gnssout off` 后本线程
 *   不再取信号量/弹样本, 缓冲区完整交还 gins 链路。
 *
 * 输出 (USART1, 与 console/VOFA 同口 460800 8N1), 每定位解一行 (~165B):
 *   time:1234.567890 utc:1760000000.123456 fix:4 sats:30 rtk:3 \
 *   lat:30.1111111 lon:120.1111111 alt:12.345 vn:0.123456 ve:0.123456 \
 *   vu:0.000000 hdop:0.800000
 *     time    样本 T_event (T_MCU 时基, PPS 映射换算; 映射未就绪时 0)
 *     utc     UM982 报文自身 UTC (无日期基准时 0, 与 time 对比可看映射偏差)
 *     fix     0无定位 1单点 2差分 3PPS 4RTK固定 5RTK浮点 6推算
 *     rtk     0无差分 1码差分 2RTK浮点 3RTK固定
 *     lat/lon deg (7 位小数 ≈ 1cm)  alt 椭球高 m (3 位)
 *     vn/ve/vu NED 速度 m/s (NMEA 无天向, vu 恒 0)  hdop 水平精度因子
 *   10Hz x ~165B 约 1.7KB/s, 连 VOFA 帧共约 10% 带宽, 无需抽样/均值。
 *
 * 浮点打印走 app_out_cat_fx/d7/d3 定点拆分 (见 app_out.c); lat/lon 用
 *   double x 1e7 (|lon|<=180 -> <2^31), alt 用 x 1e3 (高度大时 1e6 会溢出
 *   int32)。
 *
 * FinSH 命令: gnssout [on|off]
 */

#include <rtthread.h>
#include <rtdevice.h>

#include "gnss_data.h"             /* gnss_data_wait/pop(): UM982 结构化定位解环形缓冲区 */
#include "app_out.h"               /* app_out_write()/app_out_cat_*(): 共享写与定点格式化 */

#define LOG_TAG "gnssout"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

#define GNSSOUT_DEV_NAME        "uart1"         /* 与 console/VOFA 同口 */
#define GNSSOUT_PERIOD_MS       10              /* 镜像快照轮询周期 (seq 变化才打印) */
#define GNSSOUT_THREAD_PRIO     14              /* 低于 imuout(13), 高于 FinSH(20) */
#define GNSSOUT_THREAD_STACK    4096
#define GNSSOUT_THREAD_TICK     10

static struct
{
    rt_device_t uart;
    rt_thread_t thread;

    rt_bool_t   on;             /* FinSH 开关 */
    rt_uint32_t last_seq;       /* 已打印的镜像序号 (判新) */
    rt_uint32_t lines;          /* 累计打印行数 */
} gnssout_ctx;

/* 整行一次写出: 串口框架发送路径有锁, 行不会被日志插断 */
static void gnssout_line(const struct gnss_sample *s)
{
    char buf[224];
    rt_size_t off;
    int len;

    len = rt_snprintf(buf, sizeof(buf),
                      "time:%u.%06u utc:%u.%06u fix:%u sats:%u rtk:%u",
                      (rt_uint32_t)(s->T_event / 1000000u),
                      (rt_uint32_t)(s->T_event % 1000000u),
                      s->utc_sec, s->utc_usec,
                      s->fix_type, s->satellites, s->rtk_status);
    off = (len > 0) ? (rt_size_t)len : 0u;

    app_out_cat_d7(buf, sizeof(buf), &off, "lat", s->latitude_deg);
    app_out_cat_d7(buf, sizeof(buf), &off, "lon", s->longitude_deg);
    app_out_cat_d3(buf, sizeof(buf), &off, "alt", s->altitude_m);
    app_out_cat_fx(buf, sizeof(buf), &off, "vn", s->vn);
    app_out_cat_fx(buf, sizeof(buf), &off, "ve", s->ve);
    app_out_cat_fx(buf, sizeof(buf), &off, "vu", s->vu);
    app_out_cat_fx(buf, sizeof(buf), &off, "hdop", s->hdop);

    buf[off++] = '\r';
    buf[off++] = '\n';
    app_out_write(gnssout_ctx.uart, buf, off);
}

static void gnssout_thread_entry(void *parameter)
{
    struct gnss_sample s;
    rt_uint32_t seq;

    RT_UNUSED(parameter);

    /* gnss_data 采集链路就绪等待 (UM982 未接/链路断时在此空转, 不占带宽) */
    while (1)
    {
        struct gnss_data_status st;

        gnss_data_get_status(&st);
        if (st.running)
            break;
        rt_thread_mdelay(50);
    }

    while (1)
    {
        if (!gnssout_ctx.on)
        {
            rt_thread_mdelay(200);
            continue;
        }

        /* 镜像快照轮询 (seq 变化才打印): 不弹 FIFO, 与 gins 桥接线程
         * 融合消费方共存 (原排空式 pop 会在开启期抢走 GNSS 观测) */
        rt_thread_mdelay(GNSSOUT_PERIOD_MS);

        if (gnss_data_peek_latest(&s, &seq) == RT_EOK &&
            seq != gnssout_ctx.last_seq)
        {
            gnssout_ctx.last_seq = seq;
            gnssout_line(&s);
            gnssout_ctx.lines++;
        }
    }
}


/* ---------------------------- 初始化 ---------------------------- */

int gnssout_link_init(void)
{
    gnssout_ctx.uart = rt_device_find(GNSSOUT_DEV_NAME);
    if (gnssout_ctx.uart == RT_NULL)
    {
        LOG_E("gnssout: uart \"%s\" not found", GNSSOUT_DEV_NAME);
        return -RT_ERROR;
    }
    /* 纯调试输出口: 只写, 不带任何中断/接收标志; console 已打开则复用 */
    if (!(gnssout_ctx.uart->open_flag & RT_DEVICE_OFLAG_OPEN))
    {
        if (rt_device_open(gnssout_ctx.uart, RT_DEVICE_OFLAG_WRONLY) != RT_EOK)
        {
            LOG_E("gnssout: open uart \"%s\" failed", GNSSOUT_DEV_NAME);
            gnssout_ctx.uart = RT_NULL;
            return -RT_ERROR;
        }
    }

    gnssout_ctx.on = RT_FALSE;  /* 默认关: 开启时与 gins 桥接分抢 gnss 样本 */

    gnssout_ctx.thread = rt_thread_create("gnssout", gnssout_thread_entry, RT_NULL,
                                          GNSSOUT_THREAD_STACK, GNSSOUT_THREAD_PRIO,
                                          GNSSOUT_THREAD_TICK);
    if (gnssout_ctx.thread == RT_NULL)
        return -RT_ENOMEM;
    rt_thread_startup(gnssout_ctx.thread);

    LOG_I("gnssout: UM982 gnss_data tagged text 10Hz on %s @%s "
          "(time/utc/fix/sats/rtk/lat/lon/alt/vn/ve/vu/hdop)",
          GNSSOUT_DEV_NAME, APP_OUT_BAUD_TEXT);
    return RT_EOK;
}

/* ---------------------------- FinSH 命令 ---------------------------- */

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>

static void gnssout(int argc, char **argv)
{
    if (argc >= 2 && !rt_strcmp(argv[1], "on"))
        gnssout_ctx.on = RT_TRUE;
    else if (argc >= 2 && !rt_strcmp(argv[1], "off"))
        gnssout_ctx.on = RT_FALSE;

    {
        struct gnss_data_status st;

        gnss_data_get_status(&st);
        LOG_I("=== UM982 tagged print (USART1) ===");
        LOG_I("run     : %s, uart %s @ %s 8N1, %s",
              gnssout_ctx.thread ? "yes" : "no", GNSSOUT_DEV_NAME,
              APP_OUT_BAUD_TEXT, gnssout_ctx.on ? "ON" : "OFF");
        LOG_I("stats   : lines=%u", gnssout_ctx.lines);
        LOG_I("gnss    : running=%d pushed=%u popped=%u lost=%u",
              (int)st.running, st.pushed, st.popped, st.lost);
        LOG_I("hint    : 开启时与 gins 桥接分抢样本, 测完 `gnssout off` 交还; "
              "缓冲细节看 `gnssdata`, 原始语句看 `gnssraw`");
    }
}
MSH_CMD_EXPORT(gnssout, UM982 tagged text on USART1: gnssout [on|off]);

#endif /* RT_USING_FINSH && FINSH_USING_MSH */

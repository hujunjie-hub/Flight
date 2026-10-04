/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BMM350 磁力计数据 (原始 + 入环前处理结果) -> 带标记文本打印 (USART1 调试口)
 *
 * 数据来源: middleware/Sensor_Preprocessing/process_data 的 mag_data 最新样本镜像 (100Hz, 非消费读),
 *   seq 变化才打印 —— 不弹 FIFO, 与 ginsaux 融合消费方共存; 每个样本同时携带:
 *     mag[]  原始磁强 µT (传感器坐标系, 仅单位换算)
 *     cal[]  校准 -> 轴映射(前右下) -> 低通 后的磁强 µT (process_data 层采集线程
 *            入环前完成, 喂 KF-GINS 的同一份数据)
 *   原始/处理两组同行打印, 便于检验校准与滤波效果。
 *
 * 输出 (USART1, 与 console/VOFA 同口 460800 8N1), 每样本一行 (~125B, 100Hz):
 *   time:123.456789 act:1 rx:12.3 ry:-45.6 rz:30.1 cx:11.2 cy:-44.8 cz:29.9 \
 *   rmag:56.7 cmag:48.9 mag_calib_data
 *     time    统一时基 UTC 秒 (PPS 未同步 = 本地单调秒)
 *     act     1 = 校准参数生效中 (mag_calib_active), 0 = 直通 (未校准)
 *     rx..rz  原始磁强 µT (传感器坐标系)
 *     cx..cz  处理后磁强 µT (体坐标系 FRD, 含低通群延迟)
 *     rmag/cmag 原始/处理后模长 µT —— 校准良好时 cmag 应近似恒定 (地磁场),
 *             rmag 的波动即硬/软磁畸变量; 模长球检验也是 magcal 采集的
 *             质量指标
 *   100Hz x ~125B 约 12.5KB/s, 连 VOFA 帧共约 32% 带宽, 无需抽样。
 *
 * FinSH 命令: magout [on|off]    校准参数管理看 `magcal` (采集/使能/清除)
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <math.h>

#include "mag_data.h"              /* mag_data_wait/pop(): BMM350 磁力计原始样本 (µT) */
#include "mag_calib.h"             /* mag_calib_active()/get_status(): 椭球硬/软磁校正 */
#include "app_out.h"               /* app_out_write()/app_out_cat_fx(): 共享写与定点格式化 */

#define LOG_TAG "magout"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

#define MAGOUT_DEV_NAME         "uart1"         /* 与 console/VOFA 同口 */
#define MAGOUT_PERIOD_MS        10              /* 镜像快照轮询周期 (seq 变化才打印) */
#define MAGOUT_THREAD_PRIO      15              /* 低于 gnssout(14), 高于 FinSH(20) */
#define MAGOUT_THREAD_STACK     2048
#define MAGOUT_THREAD_TICK      10

static struct
{
    rt_device_t uart;
    rt_thread_t thread;

    rt_bool_t   on;             /* FinSH 开关 */
    rt_uint32_t last_seq;       /* 已打印的镜像序号 (判新) */
    rt_uint32_t lines;          /* 累计打印行数 */
} magout_ctx;

/* 整行一次写出: 串口框架发送路径有锁, 行不会被日志插断 */
static void magout_line(const struct mag_sample *m)
{
    static const char *raw_lab[3] = { "rx", "ry", "rz" };
    static const char *cal_lab[3] = { "cx", "cy", "cz" };
    float raw[3];
    char buf[192];
    rt_size_t off;
    int len;
    rt_uint8_t i;

    for (i = 0; i < 3; i++)
        raw[i] = m->mag[i];             /* 传感器系原始值, 与 cal 对比用 */

    len = rt_snprintf(buf, sizeof(buf), "time:%u.%06u act:%u q:%u",
                      (rt_uint32_t)(m->T_event / 1000000u),
                      (rt_uint32_t)(m->T_event % 1000000u),
                      mag_calib_active() ? 1u : 0u,
                      (m->quality & 0x01u) ? 1u : 0u);
    off = (len > 0) ? (rt_size_t)len : 0u;

    for (i = 0; i < 3; i++)
        app_out_cat_fx(buf, sizeof(buf), &off, raw_lab[i], m->mag[i]);
    for (i = 0; i < 3; i++)
        app_out_cat_fx(buf, sizeof(buf), &off, cal_lab[i], m->cal[i]);
    app_out_cat_fx(buf, sizeof(buf), &off, "rmag",
                  (float)sqrt(raw[0] * raw[0] + raw[1] * raw[1] + raw[2] * raw[2]));
    app_out_cat_fx(buf, sizeof(buf), &off, "cmag",
                  (float)sqrt(m->cal[0] * m->cal[0] + m->cal[1] * m->cal[1] +
                              m->cal[2] * m->cal[2]));

    /* 行尾标记: 本行含 middleware/Sensor_Preprocessing/filter_calib 校正输出 (cx..cz/cmag) */
    {
        int n2 = rt_snprintf(buf + off, sizeof(buf) - off, " mag_calib_data");

        if (n2 > 0)
            off += (rt_size_t)n2;
        if (off > sizeof(buf) - 2u)
            off = sizeof(buf) - 2u;
    }

    buf[off++] = '\r';
    buf[off++] = '\n';
    app_out_write(magout_ctx.uart, buf, off);
}

static void magout_thread_entry(void *parameter)
{
    struct mag_sample m;
    rt_uint32_t seq;

    RT_UNUSED(parameter);

    /* mag_data 就绪等待 (BMM350 未接/链路断时在此空转, 不占带宽) */
    while (1)
    {
        struct mag_data_status st;

        mag_data_get_status(&st);
        if (st.running)
            break;
        rt_thread_mdelay(100);
    }

    while (1)
    {
        if (!magout_ctx.on)
        {
            rt_thread_mdelay(200);
            continue;
        }

        /* 镜像快照轮询 (seq 变化才打印): 不弹 FIFO, 与 ginsaux 融合
         * 消费方共存 —— 原排空式 pop 会在开启期以不确定比例抢走磁
         * 观测 (EKF 更新率随机下降), 2026-10-02 改旁路读 */
        rt_thread_mdelay(MAGOUT_PERIOD_MS);

        if (mag_data_peek_latest(&m, &seq) == RT_EOK &&
            seq != magout_ctx.last_seq)
        {
            magout_ctx.last_seq = seq;
            magout_line(&m);
            magout_ctx.lines++;
        }
    }
}

/* ---------------------------- 初始化 ---------------------------- */

int magout_link_init(void)
{
    magout_ctx.uart = rt_device_find(MAGOUT_DEV_NAME);
    if (magout_ctx.uart == RT_NULL)
    {
        LOG_E("magout: uart \"%s\" not found", MAGOUT_DEV_NAME);
        return -RT_ERROR;
    }
    /* 纯调试输出口: 只写, 不带任何中断/接收标志; console 已打开则复用 */
    if (!(magout_ctx.uart->open_flag & RT_DEVICE_OFLAG_OPEN))
    {
        if (rt_device_open(magout_ctx.uart, RT_DEVICE_OFLAG_WRONLY) != RT_EOK)
        {
            LOG_E("magout: open uart \"%s\" failed", MAGOUT_DEV_NAME);
            magout_ctx.uart = RT_NULL;
            return -RT_ERROR;
        }
    }

    magout_ctx.on = RT_FALSE;   /* 默认关 */

    magout_ctx.thread = rt_thread_create("magout", magout_thread_entry, RT_NULL,
                                         MAGOUT_THREAD_STACK, MAGOUT_THREAD_PRIO,
                                         MAGOUT_THREAD_TICK);
    if (magout_ctx.thread == RT_NULL)
        return -RT_ENOMEM;
    rt_thread_startup(magout_ctx.thread);

    LOG_I("magout: BMM350 raw+calib tagged text 100Hz on %s @%s "
          "(time/act/rx../cx../rmag/cmag)",
          MAGOUT_DEV_NAME, APP_OUT_BAUD_TEXT);
    return RT_EOK;
}

/* ---------------------------- FinSH 命令 ---------------------------- */

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>

static void magout(int argc, char **argv)
{
    if (argc >= 2 && !rt_strcmp(argv[1], "on"))
        magout_ctx.on = RT_TRUE;
    else if (argc >= 2 && !rt_strcmp(argv[1], "off"))
        magout_ctx.on = RT_FALSE;

    {
        struct mag_data_status ds;
        struct mag_calib_status cs;

        mag_data_get_status(&ds);
        mag_calib_get_status(&cs);
        LOG_I("=== MAG raw+calib print (USART1) ===");
        LOG_I("run     : %s, uart %s @ %s 8N1, %s",
              magout_ctx.thread ? "yes" : "no", MAGOUT_DEV_NAME,
              APP_OUT_BAUD_TEXT, magout_ctx.on ? "ON" : "OFF");
        LOG_I("stats   : lines=%u", magout_ctx.lines);
        LOG_I("magdata : running=%d pushed=%u popped=%u lost=%u errors=%u",
              (int)ds.running, ds.pushed, ds.popped, ds.lost, ds.errors);
        LOG_I("calib   : valid=%d enabled=%d busy=%d samples=%u radius=%.2f ut",
              (int)cs.valid, (int)cs.enabled, (int)cs.busy, cs.samples,
              cs.radius_ut);
        LOG_I("hint    : cmag 恒定 = 校准好; 参数采集/管理看 `magcal`");
    }
}
MSH_CMD_EXPORT(magout, MAG raw+calib tagged text on USART1: magout [on|off]);

#endif /* RT_USING_FINSH && FINSH_USING_MSH */

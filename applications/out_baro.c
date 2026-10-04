/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BMP585 气压计数据 (含校准输出) -> 带标记文本打印 (USART1 调试口)
 *
 * 数据来源: middleware/Sensor_Preprocessing/process_data 的 baro_data 环形缓冲区 (100Hz, Pa/°C),
 *   baro_data_wait/pop 取数; 每个样本经 middleware/Sensor_Preprocessing/filter_calib 的
 *   baro_calib_apply() 基准偏移校正 (无效/关闭时原样直通),
 *   原始/校准两组同行打印, 行尾 baro_calib_data 标记。
 *   采集链路未运行 (running=FALSE) 时, 本线程静默空转。
 *
 * 输出 (USART1, 与 console/VOFA 同口 460800 8N1), 每样本一行 (~90B, 100Hz):
 *   time:123.456789 pa:101325.123 pac:101325.123 temp:27.500 baro_calib_data
 *     time    统一时基 UTC 秒   pa/pac  原始/校准后气压 Pa (3 位小数)
 *     temp    气压计芯片温度 °C
 *   100Hz x ~90B 约 9KB/s, 若带宽紧张可再抽样。
 *
 * FinSH 命令: barout [on|off]    校准参数管理看 `barocal`
 */

#include <rtthread.h>
#include <rtdevice.h>

#include "baro_data.h"             /* baro_data_wait/pop(): BMP585 气压计样本 (Pa/°C) */
#include "baro_calib.h"            /* baro_calib_apply()/get_status(): 气压基准偏移校正 */
#include "app_out.h"               /* app_out_write()/app_out_cat_d3(): 共享写与定点格式化 */

#define LOG_TAG "barout"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

#define BAROUT_DEV_NAME         "uart1"         /* 与 console/VOFA 同口 */
#define BAROUT_PERIOD_MS        10              /* 镜像快照轮询周期 (seq 变化才打印) */
#define BAROUT_THREAD_PRIO      17              /* 低于 gins_fused_data(12)/magout(15) */
#define BAROUT_THREAD_STACK     2048
#define BAROUT_THREAD_TICK      10

static struct
{
    rt_device_t uart;
    rt_thread_t thread;

    rt_bool_t   on;             /* FinSH 开关 */
    rt_uint32_t last_seq;       /* 已打印的镜像序号 (判新) */
    rt_uint32_t lines;          /* 累计打印行数 */
} barout_ctx;

/* 整行一次写出: 串口框架发送路径有锁, 行不会被日志插断 */
static void barout_line(const struct baro_sample *b)
{
    char buf[128];
    rt_size_t off;
    int len;

    len = rt_snprintf(buf, sizeof(buf), "time:%u.%06u",
                      (rt_uint32_t)(b->T_event / 1000000u),
                      (rt_uint32_t)(b->T_event % 1000000u));
    off = (len > 0) ? (rt_size_t)len : 0u;

    app_out_cat_d3(buf, sizeof(buf), &off, "pa", (double)b->pressure_pa);
    app_out_cat_d3(buf, sizeof(buf), &off, "pac", baro_calib_apply((double)b->pressure_pa));
    app_out_cat_d3(buf, sizeof(buf), &off, "temp", (double)b->temperature_c);

    {
        int n2 = rt_snprintf(buf + off, sizeof(buf) - off, " baro_calib_data");

        if (n2 > 0)
            off += (rt_size_t)n2;
        if (off > sizeof(buf) - 2u)
            off = sizeof(buf) - 2u;
    }

    buf[off++] = '\r';
    buf[off++] = '\n';
    app_out_write(barout_ctx.uart, buf, off);
}

static void barout_thread_entry(void *parameter)
{
    struct baro_sample b;
    rt_uint32_t seq;

    RT_UNUSED(parameter);

    /* baro_data 就绪等待 (BMP585 未接时在此空转, 不占带宽) */
    while (1)
    {
        struct baro_data_status st;

        baro_data_get_status(&st);
        if (st.running)
            break;
        rt_thread_mdelay(100);
    }

    while (1)
    {
        if (!barout_ctx.on)
        {
            rt_thread_mdelay(200);
            continue;
        }

        /* 镜像快照轮询 (seq 变化才打印): 不弹 FIFO, 与 ginsaux 融合
         * 消费方共存 (原排空式 pop 会在开启期抢走气压观测) */
        rt_thread_mdelay(BAROUT_PERIOD_MS);

        if (baro_data_peek_latest(&b, &seq) == RT_EOK &&
            seq != barout_ctx.last_seq)
        {
            barout_ctx.last_seq = seq;
            barout_line(&b);
            barout_ctx.lines++;
        }
    }
}

/* ---------------------------- 初始化 ---------------------------- */

int barout_link_init(void)
{
    barout_ctx.uart = rt_device_find(BAROUT_DEV_NAME);
    if (barout_ctx.uart == RT_NULL)
    {
        LOG_E("barout: uart \"%s\" not found", BAROUT_DEV_NAME);
        return -RT_ERROR;
    }
    /* 纯调试输出口: 只写, 不带任何中断/接收标志; console 已打开则复用 */
    if (!(barout_ctx.uart->open_flag & RT_DEVICE_OFLAG_OPEN))
    {
        if (rt_device_open(barout_ctx.uart, RT_DEVICE_OFLAG_WRONLY) != RT_EOK)
        {
            LOG_E("barout: open uart \"%s\" failed", BAROUT_DEV_NAME);
            barout_ctx.uart = RT_NULL;
            return -RT_ERROR;
        }
    }

    barout_ctx.on = RT_FALSE;   /* 默认关: 开启时与 ginsaux 分抢 baro 样本 */

    barout_ctx.thread = rt_thread_create("barout", barout_thread_entry, RT_NULL,
                                         BAROUT_THREAD_STACK, BAROUT_THREAD_PRIO,
                                         BAROUT_THREAD_TICK);
    if (barout_ctx.thread == RT_NULL)
        return -RT_ENOMEM;
    rt_thread_startup(barout_ctx.thread);

    LOG_I("barout: BMP585 raw+calib tagged text 100Hz on %s @%s "
          "(time/pa/pac/temp, baro_calib_data)",
          BAROUT_DEV_NAME, APP_OUT_BAUD_TEXT);
    return RT_EOK;
}

/* ---------------------------- FinSH 命令 ---------------------------- */

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>

static void barout(int argc, char **argv)
{
    if (argc >= 2 && !rt_strcmp(argv[1], "on"))
        barout_ctx.on = RT_TRUE;
    else if (argc >= 2 && !rt_strcmp(argv[1], "off"))
        barout_ctx.on = RT_FALSE;

    {
        struct baro_data_status ds;
        struct baro_calib_status cs;

        baro_data_get_status(&ds);
        baro_calib_get_status(&cs);
        LOG_I("=== BARO raw+calib print (USART1) ===");
        LOG_I("run     : %s, uart %s @ %s 8N1, %s",
              barout_ctx.thread ? "yes" : "no", BAROUT_DEV_NAME,
              APP_OUT_BAUD_TEXT, barout_ctx.on ? "ON" : "OFF");
        LOG_I("stats   : lines=%u", barout_ctx.lines);
        LOG_I("barodata: running=%d pushed=%u popped=%u lost=%u errors=%u",
              (int)ds.running, ds.pushed, ds.popped, ds.lost, ds.errors);
        LOG_I("calib   : valid=%d enabled=%d offset=%.2f pa",
              (int)cs.valid, (int)cs.enabled, cs.offset_pa);
        LOG_I("hint    : pac 恒定偏移 = 校准好; 参数采集/管理看 `barocal`");
    }
}
MSH_CMD_EXPORT(barout, BARO raw+calib tagged text on USART1: barout [on|off]);

#endif /* RT_USING_FINSH && FINSH_USING_MSH */

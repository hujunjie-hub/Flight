/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * USART1 调试输出端口公共设施: 共享写互斥 + STREAM 标志兜底清除 +
 * tiny klibc 定点字段格式化助手 (实现说明见 app_out.h 与各 out_*.c 头注)
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>                       /* rt_interrupt_get_nest (ISR 上下文放行判断) */
#include "app_out.h"

#define APP_OUT_UART_DEV        "uart1"         /* 与 console 同口 (PA9/PA10) */
#define CAT_FX_SCALE            1000000u        /* 定点小数 = 6 位 (µ) */

/* ------------------------- USART1 共享写互斥 -------------------------
 * 串口驱动 (DMA TX) 无跨线程写保护, 多输出线程 (vofa 二进制帧 / 各文本
 * 链 / console) 并发写在字节级交错, 实测文本行被插花、VOFA 帧夹在行中。
 * 各链路整行/整帧 rt_device_write 由本模块统一加锁; ulog 日志行经
 * console_be 的 ulog_console_tx_lock/unlock 弱钩子共用本互斥 (2026-10-02,
 * 替换原 "日志默认关闭避冲突" 的权宜), 高频日志与高频帧同开时行/帧
 * 完整不插花 (UTF-8 多字节序列不再被拆断)。ISR/调度器未起上下文放行。 */
static rt_mutex_t s_uart1_lock = RT_NULL;

/* console_be.c 弱钩子的强实现: 与六条输出链路同一把 uart1wr 互斥
 * (rt_mutex 递归持有, 链路锁内打日志不死锁)。锁不存在 (main() 之前
 * 的启动期日志) 或 ISR/调度器未起上下文时放行 —— 此时帧链路必然未
 * 运行, 无冲突对象。 */
void ulog_console_tx_lock(void)
{
    if (s_uart1_lock == RT_NULL ||
        rt_interrupt_get_nest() != 0u ||
        rt_thread_self() == RT_NULL)
        return;
    rt_mutex_take(s_uart1_lock, RT_WAITING_FOREVER);
}

void ulog_console_tx_unlock(void)
{
    if (s_uart1_lock == RT_NULL ||
        rt_interrupt_get_nest() != 0u ||
        rt_thread_self() == RT_NULL)
        return;
    rt_mutex_release(s_uart1_lock);
}

void app_out_init(void)
{
    rt_device_t uart1;

    s_uart1_lock = rt_mutex_create("uart1wr", RT_IPC_FLAG_PRIO);

    /* 去掉 console/FinSH 打上的 RT_DEVICE_FLAG_STREAM: 该模式会在写出数据
     * 的每个 0x0a 字节前插入 0x0d, 与同口传输的 VOFA JustFloat 二进制帧
     * 冲突 (~5% 的帧含 0x0a 字节, 被插入 CR 后错位损坏, 文本行尾也多出
     * 一个 \r)。各链路文本输出均显式写 \r\n, 去除后仅内核/FinSH 日志
     * 变 LF 结尾。标志在设备上跨 open/close 粘滞, 须显式清除。 */
    uart1 = rt_device_find(APP_OUT_UART_DEV);
    if (uart1 != RT_NULL)
        uart1->open_flag &= ~RT_DEVICE_FLAG_STREAM;   /* finsh 线程还会再置回, 写前兜底见 app_out_write() */
}

void app_out_write(rt_device_t dev, const void *buf, rt_size_t len)
{
    if (dev == RT_NULL)
        return;

    if (s_uart1_lock != RT_NULL)
        rt_mutex_take(s_uart1_lock, RT_WAITING_FOREVER);

    /* finsh 线程首次调度时 finsh_set_device 会带 RT_DEVICE_FLAG_STREAM 重新
     * open 本串口 (晚于 app_out_init() 的启动期清除), 每次写出前兜底清除;
     * console/FinSH 输出走 nano drv_console 的 HAL 直写路径 (自带 \n 转换,
     * 不查设备标志), 清除无副作用。 */
    if (dev->open_flag & RT_DEVICE_FLAG_STREAM)
        dev->open_flag &= ~RT_DEVICE_FLAG_STREAM;

    rt_device_write(dev, 0, buf, len);

    if (s_uart1_lock != RT_NULL)
        rt_mutex_release(s_uart1_lock);
}

/* ---------------------------- 定点格式化助手 ---------------------------- */

void app_out_cat_fx(char *buf, rt_size_t size, rt_size_t *off,
                    const char *label, float v)
{
    rt_int32_t  s = (rt_int32_t)(v * (float)CAT_FX_SCALE);
    rt_uint32_t mag = (s < 0) ? (rt_uint32_t)(-s) : (rt_uint32_t)s;
    int len;

    if (*off >= size - 2u)                  /* 至少留 " x" 与 \r\n */
        return;

    len = rt_snprintf(buf + *off, size - *off, " %s:%s%u.%06u",
                      label, (s < 0) ? "-" : "",
                      mag / CAT_FX_SCALE, mag % CAT_FX_SCALE);
    if (len > 0)
        *off += (rt_size_t)len;
    if (*off > size - 2u)
        *off = size - 2u;
}

void app_out_cat_d7(char *buf, rt_size_t size, rt_size_t *off,
                    const char *label, double v)
{
    rt_int32_t  s = (rt_int32_t)(v * 10000000.0);
    rt_uint32_t mag = (s < 0) ? (rt_uint32_t)(-s) : (rt_uint32_t)s;
    int len;

    if (*off >= size - 2u)
        return;

    len = rt_snprintf(buf + *off, size - *off, " %s:%s%u.%07u",
                      label, (s < 0) ? "-" : "",
                      mag / 10000000u, mag % 10000000u);
    if (len > 0)
        *off += (rt_size_t)len;
    if (*off > size - 2u)
        *off = size - 2u;
}

void app_out_cat_d3(char *buf, rt_size_t size, rt_size_t *off,
                    const char *label, double v)
{
    rt_int32_t  s = (rt_int32_t)(v * 1000.0);
    rt_uint32_t mag = (s < 0) ? (rt_uint32_t)(-s) : (rt_uint32_t)s;
    int len;

    if (*off >= size - 2u)
        return;

    len = rt_snprintf(buf + *off, size - *off, " %s:%s%u.%03u",
                      label, (s < 0) ? "-" : "",
                      mag / 1000u, mag % 1000u);
    if (len > 0)
        *off += (rt_size_t)len;
    if (*off > size - 2u)
        *off = size - 2u;
}

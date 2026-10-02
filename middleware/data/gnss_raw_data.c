/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * UM982 UART 原始字节环形缓冲区实现 (接口说明见 gnss_raw_data.h)
 *
 * 环结构即经典 head/tail 字节环:
 *   - head: 写位置 (只在 push 侧推进)
 *   - tail: 读位置 (只在 pop 侧推进, 溢出挤旧时由 push 侧代推)
 *   - 空: head == tail;  满: (head + 1) % size == tail (保留一格区分空满)
 * push/pop 均在关中断临界区内完成 (push 在 USART2 接收线程,
 * pop 在任意消费线程, 单写多读由临界区串行化)。
 */

#include "gnss_raw_data.h"
#include <rtdevice.h>
#include <string.h>

#define LOG_TAG "data.gnssraw"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

/* ------------------------- 配置 ------------------------- */

/* 缓冲区大小 (字节): 460800 波特率下 4KB ≈ 90ms 历史 */
#define GNSS_RAW_DATA_BUF_SIZE     4096u

/* ------------------------- 环结构 ------------------------- */

/*
 * 原始字节环 (对外只暴露 head/tail/size/buffer 语义, 实例为本模块私有):
 *   buffer[]  UART 原始字节
 *   head      写位置
 *   tail      读位置
 *   size      缓冲区大小
 */
struct gnss_raw_data
{
    volatile rt_uint32_t head;              /* 写位置 */
    volatile rt_uint32_t tail;              /* 读位置 */
    rt_uint32_t          size;              /* 缓冲区大小 (字节) */
    rt_uint8_t          *buffer;            /* UART 原始字节 */
};

static struct gnss_raw_data g_raw =
{
    0u, 0u, GNSS_RAW_DATA_BUF_SIZE, RT_NULL
};
static rt_uint8_t g_raw_pool[GNSS_RAW_DATA_BUF_SIZE];

/* 环占用高水位 (字节, SWD 直读; 接近 4096 说明解析线程跟不上) */
static volatile rt_uint32_t g_raw_hiwat;

/* ------------------------- 运行统计 ------------------------- */

static struct
{
    struct gnss_raw_data_status st;
    struct rt_semaphore data_sem;           /* 消费等待: 每次 push 释放一次 */
    rt_uint8_t inited;
} ctx;

/* ------------------------- 对外接口 ------------------------- */

void gnss_raw_data_push(const rt_uint8_t *data, rt_size_t len)
{
    if (data == RT_NULL || len == 0)
        return;

    if (!ctx.inited)
    {
        /* 静态零值 + 编译期缓冲即空环, 首次写入前补挂 buffer/信号量 */
        g_raw.buffer = g_raw_pool;
        rt_sem_init(&ctx.data_sem, "gnssrsem", 0, RT_IPC_FLAG_FIFO);
        ctx.inited = 1u;
    }

    rt_base_t level = rt_hw_interrupt_disable();

    for (rt_size_t i = 0; i < len; i++)
    {
        rt_uint32_t next = (g_raw.head + 1u) % g_raw.size;

        if (next == g_raw.tail)             /* 满: 挤掉最旧一字节 */
        {
            g_raw.tail = (g_raw.tail + 1u) % g_raw.size;
            ctx.st.lost++;
        }
        g_raw.buffer[g_raw.head] = data[i];
        g_raw.head = next;
    }
    ctx.st.pushed += (rt_uint32_t)len;

    /* 高水位 (SWD 诊断: gnssdata 消费是否跟得上) */
    {
        rt_uint32_t fill = (g_raw.head + g_raw.size - g_raw.tail) % g_raw.size;

        if (fill > g_raw_hiwat)
            g_raw_hiwat = fill;
    }

    rt_hw_interrupt_enable(level);

    rt_sem_release(&ctx.data_sem);
}

rt_size_t gnss_raw_data_pop(rt_uint8_t *out, rt_size_t len)
{
    rt_size_t n = 0;

    if (out == RT_NULL || len == 0 || !ctx.inited)
        return 0;

    rt_base_t level = rt_hw_interrupt_disable();

    while (n < len && g_raw.tail != g_raw.head)
    {
        out[n++] = g_raw.buffer[g_raw.tail];
        g_raw.tail = (g_raw.tail + 1u) % g_raw.size;
    }
    ctx.st.popped += (rt_uint32_t)n;

    rt_hw_interrupt_enable(level);
    return n;
}

rt_err_t gnss_raw_data_wait(rt_int32_t timeout_ms)
{
    if (!ctx.inited)
        return -RT_EEMPTY;

    return rt_sem_take(&ctx.data_sem,
                       rt_tick_from_millisecond((rt_int32_t)timeout_ms));
}

rt_uint32_t gnss_raw_data_count(void)
{
    rt_base_t level = rt_hw_interrupt_disable();
    rt_uint32_t n =
        (g_raw.head + g_raw.size - g_raw.tail) % g_raw.size;

    rt_hw_interrupt_enable(level);
    return n;
}

void gnss_raw_data_flush(void)
{
    /* 先排空信号量再清环 (同其余 data 模块: 反序会把 reset 后新推入
     * 字节的通知吃掉, 消费方多等一个完整超时) */
    while (rt_sem_take(&ctx.data_sem, 0) == RT_EOK)
        ;

    {
        rt_base_t level = rt_hw_interrupt_disable();

        g_raw.tail = g_raw.head;
        rt_hw_interrupt_enable(level);
    }
}

void gnss_raw_data_get_status(struct gnss_raw_data_status *st)
{
    if (st == RT_NULL)
        return;

    rt_base_t level = rt_hw_interrupt_disable();

    *st = ctx.st;
    st->inited = (rt_bool_t)ctx.inited;
    rt_hw_interrupt_enable(level);
}

/* ------------------------- 初始化 ------------------------- */

static int gnss_raw_data_init(void)
{
    g_raw.buffer = g_raw_pool;
    rt_sem_init(&ctx.data_sem, "gnssrsem", 0, RT_IPC_FLAG_FIFO);
    ctx.inited = 1u;

    LOG_I("ready: %u bytes raw UART buffer (460800 baud ~90ms)",
          (unsigned)GNSS_RAW_DATA_BUF_SIZE);
    return 0;
}
INIT_COMPONENT_EXPORT(gnss_raw_data_init);

/* ------------------------- FinSH 命令 ------------------------- */

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>

static void gnssraw(void)
{
    struct gnss_raw_data_status st;

    gnss_raw_data_get_status(&st);

    LOG_I("=== GNSS raw UART buffer ===");
    LOG_I("inited  : %s, count=%u/%u bytes",
          st.inited ? "yes" : "no",
          gnss_raw_data_count(), (unsigned)GNSS_RAW_DATA_BUF_SIZE);
    LOG_I("stats   : pushed=%u popped=%u lost=%u",
          st.pushed, st.popped, st.lost);
    LOG_I("hint    : push 由 gnssrx 接收线程镜像写入 (USART2); "
          "pop 出的字节由解析线程组句解析");
}
MSH_CMD_EXPORT(gnssraw, GNSS raw UART ring buffer status);
#endif /* RT_USING_FINSH && FINSH_USING_MSH */

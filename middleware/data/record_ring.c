/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 定长记录环形缓冲区公共层实现 (语义说明见 record_ring.h)
 */

#include "record_ring.h"
#include <ipc/ringbuffer.h>

/* push 挤掉最旧记录的临时缓冲上限; 各样本结构均须小于此值
 * (init 校验 rec_size, 超限直接报错防止静默截断) */
#define RECORD_RING_MAX_REC_SIZE   128

rt_err_t record_ring_init(struct record_ring *rr, void *pool, rt_size_t pool_size,
                          rt_size_t rec_size, const char *sem_name)
{
    rt_err_t ret;

    rt_ringbuffer_init(&rr->rb, (rt_uint8_t *)pool, pool_size);
    rr->rec_size = (rt_uint16_t)rec_size;
    rr->lost = 0;

    if (rec_size == 0 || rec_size > RECORD_RING_MAX_REC_SIZE)
        return -RT_EINVAL;

    /* 信号量无条件初始化: 设备缺失链路提前 return 后 wait/pop 仍安全
     * (信号量存在但永不释放 -> 消费方超时返回), 否则 rt_sem_take 断言挂死 */
    ret = rt_sem_init(&rr->data_sem, sem_name, 0, RT_IPC_FLAG_FIFO);

    return ret;
}

void record_ring_push(struct record_ring *rr, const void *rec)
{
    rt_base_t level = rt_hw_interrupt_disable();

    if (rt_ringbuffer_space_len(&rr->rb) < rr->rec_size)
    {
        rt_uint8_t drop[RECORD_RING_MAX_REC_SIZE];

        rt_ringbuffer_get(&rr->rb, drop, rr->rec_size);
        rr->lost++;
    }
    rt_ringbuffer_put(&rr->rb, (const rt_uint8_t *)rec, rr->rec_size);
    rt_hw_interrupt_enable(level);

    rt_sem_release(&rr->data_sem);
}

rt_err_t record_ring_pop(struct record_ring *rr, void *out)
{
    rt_base_t level;
    rt_size_t n;

    /* 未初始化防护: init 前的并发访问 (gins 线程优先级高于 INIT_APP 所在
     * main 线程, 上电窗口内可能先 pop)。rec_size=0 时 rt_ringbuffer_get
     * 返回 0 会与 rec_size 比较相等, 误报 RT_EOK 使消费方排空循环
     * 无限弹出幽灵样本 (2026-09-30 实测 IWDG 复位循环根因) */
    if (out == RT_NULL || rr->rec_size == 0)
        return -RT_EEMPTY;

    level = rt_hw_interrupt_disable();
    n = rt_ringbuffer_get(&rr->rb, (rt_uint8_t *)out, rr->rec_size);
    rt_hw_interrupt_enable(level);

    if (n != rr->rec_size)
        return -RT_EEMPTY;

    return RT_EOK;
}

rt_err_t record_ring_wait(struct record_ring *rr, rt_int32_t timeout_ms)
{
    return rt_sem_take(&rr->data_sem,
                       rt_tick_from_millisecond(timeout_ms));
}

rt_uint32_t record_ring_count(const struct record_ring *rr)
{
    rt_base_t level;
    rt_uint32_t n;

    level = rt_hw_interrupt_disable();
    n = rt_ringbuffer_data_len(&((struct record_ring *)rr)->rb) / rr->rec_size;

    rt_hw_interrupt_enable(level);
    return n;
}

void record_ring_flush(struct record_ring *rr)
{
    /* 先排空信号量再清环: 反序会在 reset 与 drain 之间把新推入样本的
     * 通知吃掉, 消费方要多等一个完整超时才发现数据 (flush 语义是
     * "丢弃当前积压", 先 drain 后 reset 才能一致达成) */
    while (rt_sem_take(&rr->data_sem, 0) == RT_EOK)
        ;

    {
        rt_base_t level = rt_hw_interrupt_disable();

        rt_ringbuffer_reset(&rr->rb);
        rt_hw_interrupt_enable(level);
    }
}

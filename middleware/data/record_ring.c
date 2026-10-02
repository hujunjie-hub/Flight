/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 定长记录环形缓冲区公共层实现 (语义说明见 record_ring.h)
 */

#include "record_ring.h"
#include <ipc/ringbuffer.h>
#include <string.h>

rt_err_t record_ring_init(struct record_ring *rr, void *pool, rt_size_t pool_size,
                          rt_size_t rec_size, const char *sem_name)
{
    rt_err_t ret;

    if (rec_size == 0 || rec_size > RECORD_RING_MAX_REC_SIZE)
        return -RT_EINVAL;

    /* 信号量无条件初始化: 设备缺失链路提前 return 后 wait/pop 仍安全
     * (信号量存在但永不释放 -> 消费方超时返回), 否则 rt_sem_take 断言挂死。
     * 顺序关键: rec_size (wait/pop/count 的就绪标志) 必须最后置位 —— 先
     * 置标志后 init sem 的话, 消费方越过防护 take 到零值对象仍会断言
     * (2026-10-02 实测 gins 线程抢先 wait 命中) */
    ret = rt_sem_init(&rr->data_sem, sem_name, 0, RT_IPC_FLAG_FIFO);
    if (ret != RT_EOK)
        return ret;

    rt_ringbuffer_init(&rr->rb, (rt_uint8_t *)pool, pool_size);
    rr->lost = 0;
    rr->rec_size = (rt_uint16_t)rec_size;

    return RT_EOK;
}

void record_ring_push(struct record_ring *rr, const void *rec)
{
    rt_base_t level = rt_hw_interrupt_disable();

    /* 最新样本镜像: 与入环同一临界区 (读侧 peek_latest 关中断拷贝,
     * 双侧关中断下无撕裂)。开销 = 一次 rec_size memcpy, imu 1kHz x ~50B
     * ≈ 50KB/s, 可忽略; 打印类旁路读者由此免于抢融合消费方的环 */
    rt_memcpy(rr->mirror, rec, rr->rec_size);
    rr->mirror_seq++;

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

rt_err_t record_ring_peek_latest(struct record_ring *rr, void *out,
                                 rt_uint32_t *seq)
{
    rt_base_t level;
    rt_uint32_t s;

    if (out == RT_NULL || rr->rec_size == 0)
        return -RT_EEMPTY;

    level = rt_hw_interrupt_disable();
    s = rr->mirror_seq;
    if (s != 0u)
        rt_memcpy(out, rr->mirror, rr->rec_size);
    rt_hw_interrupt_enable(level);

    if (seq != RT_NULL)
        *seq = s;
    return (s != 0u) ? RT_EOK : -RT_EEMPTY;
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
    /* 未初始化防护: 与 record_ring_pop 同因 —— gins/ginsaux 线程 (INIT_ENV
     * 创建, 优先级高) 会先于 INIT_APP 的 init 抢先 wait, 裸 take 零值
     * 信号量直接断言挂死 (2026-10-02 实测 IWDG 复位循环根因)。返回
     * -RT_EEMPTY, 调用方按 "无数据" 走 1ms 退避, init 完成后自然收敛 */
    if (rr->rec_size == 0)
        return -RT_EEMPTY;

    return rt_sem_take(&rr->data_sem,
                       rt_tick_from_millisecond(timeout_ms));
}

rt_uint32_t record_ring_count(const struct record_ring *rr)
{
    rt_base_t level;
    rt_uint32_t n;

    if (rr->rec_size == 0)
        return 0;                       /* 防 init 前除零 */

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
    if (rr->rec_size == 0)
        return;                         /* 未初始化: 无积压可清 */

    while (rt_sem_take(&rr->data_sem, 0) == RT_EOK)
        ;

    {
        rt_base_t level = rt_hw_interrupt_disable();

        rt_ringbuffer_reset(&rr->rb);
        rt_hw_interrupt_enable(level);
    }
}

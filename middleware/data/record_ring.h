/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 定长记录环形缓冲区公共层 — data 各模块 (imu/mag/baro/gnss) 共用
 *
 * 语义 (与原各模块逐字重复的实现严格一致):
 *   - 底层 RT-Thread rt_ringbuffer (字节流), 本层按定长记录读写;
 *     缓冲区大小须为记录长度整数倍。
 *   - push 满时先弹出最旧一条再写入 (覆盖旧数据), 并对 lost 计数。
 *   - put/get 均在关中断临界区内完成 (单条 ~40 字节 memcpy, 1kHz 下
 *     对中断延迟影响可忽略); rt_ringbuffer 自身无锁。
 *   - data_sem 每推送一条释放一次, 消费方 wait; flush 语义是
 *     "丢弃当前积压" — 先排空信号量再清环 (反序会在 reset 与 drain
 *     之间把新推入样本的通知吃掉, 消费方要多等一个完整超时)。
 * 字节流语义的缓冲区 (如 gnss_raw_data) 不适用本层。
 */

#ifndef __RECORD_RING_H__
#define __RECORD_RING_H__

#include <rtthread.h>
#include <ipc/ringbuffer.h>

#define RECORD_RING_MAX_REC_SIZE   128

struct record_ring
{
    struct rt_ringbuffer rb;    /* 底层字节流环 (关中断保护) */
    struct rt_semaphore  data_sem;  /* 消费等待: 每推送一条释放 */
    rt_uint16_t          rec_size;  /* 定长记录字节数 */
    rt_uint32_t          lost;      /* 满时挤掉的记录数 (诊断) */
    /* 最新样本镜像 (调试快照): push 时在同一关中断临界区内顺带更新,
     * 供打印类"旁路读者"取最新值而不与融合消费方抢环 (2026-10-02) */
    rt_uint8_t           mirror[RECORD_RING_MAX_REC_SIZE];
    volatile rt_uint32_t mirror_seq;    /* 镜像更新序号, 0 = 尚无样本 */
};

/*
 * 初始化。pool 大小须为 rec_size 整数倍; sem_name 限 4 字符 (RT-Thread
 * 对象名截断约定, 各模块已按此命名)。失败仅可能来自信号量 (RT_ENOMEM)。
 */
rt_err_t record_ring_init(struct record_ring *rr, void *pool, rt_size_t pool_size,
                          rt_size_t rec_size, const char *sem_name);

/* 推入一条记录; 满时挤掉最旧一条并 lost++ (中断/线程上下文均可调用) */
void record_ring_push(struct record_ring *rr, const void *rec);

/* 取最新样本镜像 (非消费, 不动环/信号量): seq 返回镜像序号, 读侧比对
 * seq 变化判新。无样本 (从未推送) 返回 -RT_EEMPTY */
rt_err_t record_ring_peek_latest(struct record_ring *rr, void *out,
                                 rt_uint32_t *seq);

/* 弹出一条; 空或环未初始化返回 -RT_EEMPTY, out 为空返回 -RT_EEMPTY */
rt_err_t record_ring_pop(struct record_ring *rr, void *out);

/* 等待新记录 (超时 ms; 返回 rt_sem_take 的码) */
rt_err_t record_ring_wait(struct record_ring *rr, rt_int32_t timeout_ms);

/* 当前记录条数 */
rt_uint32_t record_ring_count(const struct record_ring *rr);

/* 丢弃积压: 先排空信号量再清环 (见文件头说明) */
void record_ring_flush(struct record_ring *rr);

#endif /* __RECORD_RING_H__ */

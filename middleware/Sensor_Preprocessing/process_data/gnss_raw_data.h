/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * UM982 UART 原始字节环形缓冲区
 *
 * 数据链路 (生产者为 gnss_data.c 的接收线程 gnssrx):
 *   UM982 --460800--> UART4 接收线程读取 (只搬字节, 不解析)
 *         -> gnss_raw_data_push() 原始字节环形缓冲区 (本模块)
 *   消费方 (middleware/Sensor_Preprocessing/process_data/gnss_data.c 解析线程) gnss_raw_data_wait()
 *   阻塞等新字节, gnss_raw_data_pop() 取出后组句喂 um982_nmea 解析,
 *   结构化结果进 gnss_data 环形缓冲区。
 *
 * 注意: 本缓冲区为单消费方语义 (字节只能被 pop 一次), 解析线程是
 * 唯一常驻消费者。
 *
 * 溢出策略: 缓冲区满时挤掉最旧字节 (新数据永远进得来), 计入 lost。
 * 460800 波特率下约 46 B/ms, 4KB 缓冲 ≈ 90ms 历史。
 *
 * FinSH: gnssraw  查看缓冲区状态
 */

#ifndef __GNSS_RAW_DATA_H__
#define __GNSS_RAW_DATA_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------- 状态 ------------------------- */

struct gnss_raw_data_status
{
    rt_bool_t   inited;         /* 已初始化 (静态零值即空环, 恒可写) */
    rt_uint32_t pushed;         /* 累计推入字节数 */
    rt_uint32_t popped;         /* 累计读出字节数 */
    rt_uint32_t lost;           /* 缓冲区满被挤掉的字节数 */
};

/* ------------------------- 接口 ------------------------- */

/* 推入一批 UART 原始字节 (UART4 接收方调用; 满时挤掉最旧) */
void gnss_raw_data_push(const rt_uint8_t *data, rt_size_t len);

/* 读出最多 len 字节 (FIFO), 返回实际读出数 (0 = 空) */
rt_size_t gnss_raw_data_pop(rt_uint8_t *out, rt_size_t len);

/* 阻塞等待新字节 (每次 push 释放一次), 超时返回 -RT_ETIMEOUT */
rt_err_t gnss_raw_data_wait(rt_int32_t timeout_ms);

/* 当前缓冲字节数 */
rt_uint32_t gnss_raw_data_count(void);

/* 清空缓冲区 */
void gnss_raw_data_flush(void);

void gnss_raw_data_get_status(struct gnss_raw_data_status *st);

#ifdef __cplusplus
}
#endif

#endif /* __GNSS_RAW_DATA_H__ */

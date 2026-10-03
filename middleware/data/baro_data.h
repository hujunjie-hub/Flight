/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BARO (BMP585) 原始数据环形缓冲区
 *
 * 数据链路 (只打时间标签 + 单位换算, 不做校准):
 *   本模块采集线程 100Hz 轮询 baro_bmp585 + temp_bmp585 设备 (I2C4)
 *     -> 时间标签 (T_MCU 时基, timebase; BMP585 INT 引脚 PE1 硬件已
 *        预留, 迁移到 EXTI 事件源后时戳改为 EXTI ISR 捕获)
 *     + 单位换算 -> rt_ringbuffer 环形缓冲区
 *   消费方 baro_data_pop() 读出后再自行校准 (基准偏移: baro_calib_apply)。
 *
 * 采样元素 (结构见下, 根 README bmp585_data_t):
 *   T_event       T_MCU 时戳, us (采样触发时刻)
 *   pressure_pa   Pa (传感器框架整型 Pa, 分辨率 1 Pa ≈ 8cm 高度)
 *   temperature_c °C, 气压计芯片温度 (0.1°C 换算)
 *
 * 注意: 温度取自 temp_bmp585 的紧后一次读取, 与气压相差一次 I2C 传输
 * (~ms 级), 对惯性温度漂移而言可忽略。
 *
 * 溢出策略: 缓冲区满时挤掉最旧样本 (新数据永远进得来), 计入 status.lost。
 *
 * FinSH: barodata  查看缓冲区状态与最新样本
 */

#ifndef __BARO_DATA_H__
#define __BARO_DATA_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------- 采样结构 ------------------------- */

struct baro_sample
{
    rt_uint64_t T_event;        /* 时间戳, us; T_MCU 时基, 采样触发时刻 */
    float       pressure_pa;    /* 气压, Pa, 未校准 */
    float       temperature_c;  /* 温度, °C, 气压计芯片温度 */
};

/* ------------------------- 状态 ------------------------- */

struct baro_data_status
{
    rt_bool_t   running;        /* 采集线程已启动 */
    rt_uint32_t pushed;         /* 累计推送样本数 */
    rt_uint32_t popped;         /* 累计读出样本数 */
    rt_uint32_t lost;           /* 缓冲区满被挤掉的样本数 */
    rt_uint32_t errors;         /* 设备读取失败次数 */
};

/* ------------------------- 接口 ------------------------- */

/* 初始化并启动采集线程 (INIT_APP_EXPORT 自动执行) */
int baro_data_init(void);

/* 读出一个样本 (FIFO); 空时返回 -RT_EEMPTY */
rt_err_t baro_data_pop(struct baro_sample *out);

/* 取最新样本镜像 (非消费, 不动 FIFO/信号量): 调试打印读者专用; *seq 比对变化判新 */
rt_err_t baro_data_peek_latest(struct baro_sample *out, rt_uint32_t *seq);

/* 阻塞等待新样本 (每个推送释放一次), 超时返回 -RT_ETIMEOUT */
rt_err_t baro_data_wait(rt_int32_t timeout_ms);

/* 当前缓冲样本数 */
rt_uint32_t baro_data_count(void);

/* 清空缓冲区 */
void baro_data_flush(void);

void baro_data_get_status(struct baro_data_status *st);

#ifdef __cplusplus
}
#endif

#endif /* __BARO_DATA_H__ */

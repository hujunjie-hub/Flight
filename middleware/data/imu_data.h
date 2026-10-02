/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * IMU (ADIS16505) 环形缓冲区 (接口说明见 imu_data.h)
 */

#ifndef __IMU_DATA_H__
#define __IMU_DATA_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------- 采样结构 (根 README adis16505_data_t) ------------------------- */

struct imu_sample
{
    rt_uint64_t T_event;        /* 时间戳, us; T_MCU 时基, DR 上升沿时刻
                                 * (EXTI ISR 读 TIM2->CNT + 溢出累计合成) */
    float       gyro[3];        /* rad/s, 体坐标系(前右下), 系统级校准后 */
    float       accel[3];       /* m/s^2, 体坐标系(前右下), 系统级校准后 */
    float       temperature;    /* °C, IMU 内部温度 (调试输出用) */
    rt_uint32_t data_cnt;       /* ADIS 数据计数器 (16bit 芯片计数已扩展为
                                 * 32bit 单调值), 用于检测丢帧/数据连续性;
                                 * dt 由 gins 按 DATA_CNTR 差分计算, 丢拍期
                                 * EKF 按实际间隔积分 */
};

/* ------------------------- 状态 ------------------------- */

struct imu_data_status
{
    rt_bool_t   running;        /* 采集线程已启动 */
    rt_uint32_t pushed;         /* 累计推送样本数 */
    rt_uint32_t popped;         /* 累计读出样本数 */
    rt_uint32_t lost;           /* 缓冲区满被挤掉的样本数 */
    rt_uint32_t errors;         /* 驱动读取失败次数 */
};

/* ------------------------- 接口 ------------------------- */

/* 初始化并启动采集线程 (INIT_APP_EXPORT 自动执行) */
int imu_data_init(void);

/* 读出一个样本 (FIFO); 空时返回 -RT_EEMPTY */
rt_err_t imu_data_pop(struct imu_sample *out);

/* 取最新样本镜像 (非消费, 不动 FIFO/信号量): 调试打印读者专用; *seq 比对变化判新 */
rt_err_t imu_data_peek_latest(struct imu_sample *out, rt_uint32_t *seq);

/* 阻塞等待新样本 (每个推送释放一次), 超时返回 -RT_ETIMEOUT;
 * 与 imu_data_pop 配合: imu_data_wait(...) 后循环 pop 直到 -RT_EEMPTY */
rt_err_t imu_data_wait(rt_int32_t timeout_ms);

/* 当前缓冲样本数 */
rt_uint32_t imu_data_count(void);

/* 清空缓冲区 */
void imu_data_flush(void);

void imu_data_get_status(struct imu_data_status *st);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_DATA_H__ */

/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MAG (BMM350) 数据环形缓冲区
 *
 * 数据链路 (采集线程入环前完成 校准 -> 轴映射 -> 低通):
 *   BMM350 驱动 (解析 + Bosch OTP 出厂补偿, mGauss)
 *     -> 本模块采集线程 100Hz 轮询 mag_bmm350 设备 (I2C1)
 *     -> 单位换算 (mGauss->µT)
 *     -> mag_calib_apply  椭球硬/软磁校正 (传感器坐标系内, middleware/Sensor_Preprocessing/filter_calib)
 *     -> 轴映射到体系前右下 (gins_config.h 的 GINS_MAG_AXIS_SRC/SIGN;
 *        软磁矩阵与轴重排不可交换, 必须先校准后映射)
 *     -> 一阶 EMA 轻量低通 (GINS_MAG_LPF_TAU_S, 0 旁路)
 *     -> rt_ringbuffer 环形缓冲区
 *   消费方: ginsaux 线程取 processed 直接作磁航向观测; magcal 采集线程
 *   取 raw 做椭球拟合 (拟合必须在传感器坐标系的原始数据上进行)。
 *
 * 采样元素 (结构见下, 在根 README bmm350_data_t 基础上额外保留传感器系
 * 原始值: 磁力计椭球校准采集 magcal 作为维护窗口内的第二消费者, 从同一
 * 缓冲区取原始样本拟合, README "单消费方+挤旧留新"约定):
 *   T_event      T_MCU 时戳, us (采样触发时刻)
 *   mag[3]       µT, 传感器坐标系, 原始 (仅单位换算, 供椭球拟合/对比)
 *   cal[3]       µT, 体坐标系(前右下), 校准+轴映射+低通后 (喂引擎)
 *   quality      bit0 = 模值超限干扰 (低通之前判定, ginsaux 据此降权)
 *
 * 单位换算: 传感器框架 mGauss × 0.1 = µT (1 µT = 10 mGauss)。
 *
 * 溢出策略: 缓冲区满时挤掉最旧样本 (新数据永远进得来), 计入 status.lost。
 *
 * FinSH: magdata  查看缓冲区状态与最新样本
 */

#ifndef __MAG_DATA_H__
#define __MAG_DATA_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------- 采样结构 ------------------------- */

struct mag_sample
{
    rt_uint64_t T_event;        /* 时间戳, us; T_MCU 时基, 采样触发时刻
                                 * (INT 沿事件源接入后为 EXTI ISR 捕获) */
    float       mag[3];         /* µT, 传感器坐标系, 原始未校准 */
    float       cal[3];         /* µT, 体坐标系(前右下), 校准+轴映射+低通后 */
    rt_uint8_t  quality;        /* bit0 = 模值超限干扰 (20~100µT 之外, 低通
                                 * 之前判定; ginsaux 据此降权而非丢弃) */
};

/* ------------------------- 状态 ------------------------- */

struct mag_data_status
{
    rt_bool_t   running;        /* 采集线程已启动 */
    rt_uint32_t pushed;         /* 累计推送样本数 */
    rt_uint32_t popped;         /* 累计读出样本数 */
    rt_uint32_t lost;           /* 缓冲区满被挤掉的样本数 */
    rt_uint32_t errors;         /* 设备读取失败次数 */
};

/* ------------------------- 接口 ------------------------- */

/* 初始化并启动采集线程 (INIT_APP_EXPORT 自动执行) */
int mag_data_init(void);

/* 读出一个样本 (FIFO); 空时返回 -RT_EEMPTY */
rt_err_t mag_data_pop(struct mag_sample *out);

/* 取最新样本镜像 (非消费, 不动 FIFO/信号量): 调试打印读者专用, 与
 * ginsaux 融合消费方共存; *seq 返回镜像序号, 比对变化判新 */
rt_err_t mag_data_peek_latest(struct mag_sample *out, rt_uint32_t *seq);

/* 阻塞等待新样本 (每个推送释放一次), 超时返回 -RT_ETIMEOUT */
rt_err_t mag_data_wait(rt_int32_t timeout_ms);

/* 当前缓冲样本数 */
rt_uint32_t mag_data_count(void);

/* 清空缓冲区 */
void mag_data_flush(void);

void mag_data_get_status(struct mag_data_status *st);

#ifdef __cplusplus
}
#endif

#endif /* __MAG_DATA_H__ */

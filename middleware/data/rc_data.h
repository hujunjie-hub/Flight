/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 遥控输入数据层 (RC/虚拟摇杆统一快照, FMT_README §13.5 任务 #4)
 *
 * ---------------------------------------------------------------------------
 * 定位与轴约定
 * ---------------------------------------------------------------------------
 * 源无关的遥控快照层: CRSF 接收线程 (本文件 "rcrx", USART3) 与 QGC
 * 虚拟摇杆 (applications/mavgcs.c 的 MANUAL_CONTROL handler) 都往
 * rc_data_publish() 发布; 消费方 (quad_model 摇杆叠加/失效保护,
 * mavgcs RC_CHANNELS 下行) 只读 rc_data_get_latest()。
 * 双源并存策略: 真实 CRSF 在线 (500ms 内有更新) 时虚拟摇杆发布被忽略
 * (mavgcs 侧门控), 与 FMT "真实 RC 失联 1s 后虚拟摇杆才接管" 同义。
 *
 * 统一轴约定 (发布方负责换算, [-1,1]):
 *   axis[0] pitch   前推 +1 (前飞)
 *   axis[1] roll    右压 +1 (右飞)
 *   axis[2] yaw     右转 +1 (航向速率正, NED yaw 增大方向)
 *   axis[3] throttle/climb  上 +1 (NED 下轴负向)
 *   aux[0..3]  ch5-8 开关 (CRSF 两端 ±1; 虚拟摇杆按钮按下 +1/释放 0)
 * kill 语义: aux[0] < -0.5 (真实 RC ch5 低) -> quad_model 立即 disarm。
 *
 * ---------------------------------------------------------------------------
 * CRSF 硬件 (2026-10-04 定案)
 * ---------------------------------------------------------------------------
 * RC_CRSF_DEV = "uart3" (USART3, RX=PD9 / TX=PD8, RX DMA1_Stream6 ping 环),
 * BSP_USING_UART3 已启用, 引脚见 Flight.ioc。波特率 420000 (ELRS 标准,
 * 8N1) 在 rc_data_crsf_start 打开设备后经 RT_SERIAL_CTRL_CONFIG 显式下发
 * (驱动注册默认 115200)。接收机半双工: 只接 MCU RX 亦可正常收通道帧。
 *
 * FinSH 命令: rc   查看源/轴量/失联状态
 */
#ifndef __RC_DATA_H__
#define __RC_DATA_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------- 配置 ---------------------------- */

/* 有效窗口: 超过视为失联 (ELRS 150Hz / QGC 摇杆 ~10Hz 都远小于此) */
#define RC_DATA_TIMEOUT_MS      500

/* CRSF 接收 UART (USART3: RX=PD9/TX=PD8, 见头注) */
#define RC_CRSF_DEV             "uart3"
#define RC_CRSF_BAUD_TEXT       "420000"
#define RC_CRSF_BAUD            420000u

/* ---------------------------- 快照 ---------------------------- */

enum rc_data_src
{
    RC_SRC_NONE = 0,
    RC_SRC_CRSF,        /* 真实接收机 (rcrx 线程) */
    RC_SRC_VIRTUAL,     /* QGC 虚拟摇杆 (mavgcs MANUAL_CONTROL) */
};

struct rc_data
{
    rt_uint8_t  src;        /* enum rc_data_src, 最近一次发布者 */
    rt_uint8_t  valid;      /* 1 = 距最近发布 < RC_DATA_TIMEOUT_MS */
    float       axis[4];    /* 统一轴约定 [-1,1], 见头注 */
    float       aux[4];     /* ch5-8 / 按钮, 见头注 */
    rt_uint32_t stamp_ms;   /* 最近发布时刻 (系统 ms) */
    rt_uint32_t seq;        /* 发布计数 */
};

/* ---------------------------- 接口 ---------------------------- */

/* 发布一帧遥控输入 (CRSF 线程 / 虚拟摇杆 handler 调用, 关中断保护) */
void rc_data_publish(enum rc_data_src src, const float axis[4],
                     const float aux[4]);

/* 最新快照 (含 valid 判定; 关中断拷贝, 任意线程) */
void rc_data_get_latest(struct rc_data *out);

/* 距最近发布的毫秒数 (无发布返回 UINT32_MAX) */
rt_uint32_t rc_data_age_ms(void);

/* 启动 CRSF 接收线程 (uart3 不存在时返回 -RT_ERROR 且不启动;
 * 由 quad_model_init 顺带调用, 无需单独触发) */
int rc_data_crsf_start(void);

#ifdef __cplusplus
}
#endif

#endif /* __RC_DATA_H__ */

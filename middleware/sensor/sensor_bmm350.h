/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BMM350 三轴磁力计 RT-Thread 传感器驱动
 *
 * 硬件连接 (2026-10-02 由 I2C1/PB6/PB7/PB5 重映射):
 *   I2C4: SCL=PF14, SDA=PF15 (AF4 开漏), 400 kHz
 *   INT = PF12 (EXTI12/EXTI15_10, 预留: 数据就绪中断未接, 轮询采集)
 *
 * 补偿算法与 OTP 系数下载流程移植自 Bosch 官方 API
 * (doc/IMUTest/BMM350/bmm350, 即 doc/BMM350 官方参考),
 * 输出补偿后的磁感应强度 (单位 uT, 框架单位 mGauss) 与芯片温度。
 *
 * 注册设备 (RT-Thread sensor 框架, 轮询模式):
 *   mag_bmm350    单位 mGauss (1 uT = 10 mGauss)
 *   temp_bmm350   单位 0.1°C
 */

#ifndef __SENSOR_BMM350_H__
#define __SENSOR_BMM350_H__

#include <rtthread.h>
#include <rtdevice.h>

/* ------------------------- 板级配置 (按需修改) ------------------------- */

/* I2C 总线名 */
#define BMM350_I2C_BUS_NAME         "hwi2c4"

/* 7 位 I2C 地址: ADSEL 拉低 0x14 / 拉高 0x15 (默认低, 探测时自动兼容) */
#define BMM350_I2C_ADDR_DEFAULT     0x14
#define BMM350_I2C_ADDR_ALT         0x15

/* 默认 ODR (Hz) 与采样平均次数档位: 0=不平均, 1=2次, 2=4次, 3=8次
 * (ODR=100Hz 时最大支持 4 次平均, 驱动会自动降档) */
#define BMM350_DEFAULT_ODR_HZ       100
#define BMM350_DEFAULT_AVG          2

/* 是否启用该传感器 (0=跳过初始化, 未接线时置 0 避免探测超时拖慢启动) */
#define BMM350_ENABLE               1

/* ------------------------- 驱动接口 ------------------------- */

int rt_hw_bmm350_init(void);

#endif /* __SENSOR_BMM350_H__ */

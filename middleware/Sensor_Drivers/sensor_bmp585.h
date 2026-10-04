/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BMP585 气压计 RT-Thread 传感器驱动
 *
 * 硬件连接 (2026-10-04 定案: I2C2/PB10/PB11 -> I2C4/PB6/PB9):
 *   I2C4: SCL=PB6, SDA=PB9 (AF6 开漏), 400 kHz
 *   INT  = PE1 (EXTI1, 预留: 数据就绪中断未接, 轮询采集)
 *
 * 按数据手册 BST-BMP585-DS003 (doc/BMP585) 编写:
 *  - CHIP_ID(0x01) = 0x51; 软复位 CMD(0x7E)=0xB6 后等 NVM 就绪
 *  - 输出已内部补偿: 温度 (signed,24,16) °C, 压强 (signed,24,6) Pa
 *  - 正常模式下数据寄存器按 ODR 自动刷新, 直接读取即可
 *
 * 注册设备 (RT-Thread sensor 框架, 轮询模式):
 *   baro_bmp585   单位 Pa
 *   temp_bmp585   单位 0.1°C
 */

#ifndef __SENSOR_BMP585_H__
#define __SENSOR_BMP585_H__

#include <rtthread.h>
#include <rtdevice.h>

/* ------------------------- 板级配置 (按需修改) ------------------------- */

/* I2C 总线名 */
#define BMP585_I2C_BUS_NAME         "hwi2c4"

/* 7 位 I2C 地址: SDO=0 -> 0x46, SDO=1 -> 0x47 (默认 0x46, 探测时自动兼容) */
#define BMP585_I2C_ADDR_DEFAULT     0x46
#define BMP585_I2C_ADDR_ALT         0x47

/* 默认 ODR Hz 与过采样: osr_p/osr_t: 0~7 = 1x/2x/4x/8x/16x/32x/64x/128x
 * (100Hz 下 8x/1x 组合约 2.5ms 转换时间, 余量充足) */
#define BMP585_DEFAULT_ODR_HZ       100
#define BMP585_DEFAULT_OSR_P        3       /* 8x */
#define BMP585_DEFAULT_OSR_T        0       /* 1x */

/* IIR 低通滤波: 0=旁路, 1~7 = 系数 1/3/7/15/31/63/127 */
#define BMP585_IIR_COEFF            0

/* 是否启用该传感器 (0=跳过初始化, 未接线时置 0 避免探测超时拖慢启动) */
#define BMP585_ENABLE               1

/* ------------------------- 驱动接口 ------------------------- */

int rt_hw_bmp585_init(void);

#endif /* __SENSOR_BMP585_H__ */

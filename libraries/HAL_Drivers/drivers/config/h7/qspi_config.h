/*
 * Copyright (c) 2006-2023 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-22     zylx         first version (QUADSPI)
 * 2026-04-13     wdfk-prog    Unify DMA config descriptors
 * 2026-10-01     Flight       H723 OCTOSPI1 重写: 旧 QUADSPI 定义在 H723 上
 *                             不存在 (仅 OCTOSPI1/2), 删除 DMA 配置段 (H7 走
 *                             轮询间接模式), 参数对齐 Flight 板 W25Q64 实测
 */

#ifndef __QSPI_CONFIG_H__
#define __QSPI_CONFIG_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef BSP_USING_QSPI
/*
 * OCTOSPI1 基础配置 (间接模式轮询, 1-1-1 命令):
 *  - ClockPrescaler 由 drv_qspi.c 按框架 max_hz 与 OSPI 内核时钟计算覆盖
 *    (注意 HAL_OSPI 语义: 寄存器存 N-1, SCK = 内核时钟/N)
 *  - DeviceSize 由 medium_size 换算地址位数覆盖 (W25Q64 8MB -> 23)
 *  - SampleShifting 半周期 + ClockMode3 提高 ~92MHz SCK 下的采样裕量;
 *    ChipSelectHighTime=2 (寄存器 1 拍) 给 CS 高电平留时序余量
 */
#ifndef QSPI_BUS_CONFIG
#define QSPI_BUS_CONFIG                                            \
    {                                                              \
        .Instance = OCTOSPI1,                                      \
        .Init.FifoThreshold = 8,                                   \
        .Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE,                \
        .Init.MemoryType = HAL_OSPI_MEMTYPE_MICRON,                \
        .Init.ChipSelectHighTime = 2,                              \
        .Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE,      \
        .Init.ClockMode = HAL_OSPI_CLOCK_MODE_3,                   \
        .Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_HALFCYCLE, \
        .Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE,       \
        .Init.ChipSelectBoundary = 0,                              \
        .Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED,    \
    }
#endif /* QSPI_BUS_CONFIG */
#endif /* BSP_USING_QSPI */

#ifdef __cplusplus
}
#endif

#endif /* __QSPI_CONFIG_H__ */

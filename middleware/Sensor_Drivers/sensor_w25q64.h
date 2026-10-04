/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * W25Q64 SPI NOR Flash 驱动 (OCTOSPI1, 标准 SPI 1-1-1 模式)
 *
 * 硬件连接 (doc/Flight.xlsx 接口配置页):
 *   OCTOSPI1 (OCTOSPIM Port1):
 *     CLK  = PF10 (OCTOSPIM_P1_CLK)
 *     DI   = PF8  (OCTOSPIM_P1_IO0)
 *     DO   = PF9  (OCTOSPIM_P1_IO1)
 *     WP   = PF7  (OCTOSPIM_P1_IO2, 硬件自动管理)
 *     HOLD = PF6  (OCTOSPIM_P1_IO3, 硬件自动管理)
 *     CS   = PG6  (OCTOSPIM_P1_NCS)
 *
 * 按 Winbond W25Q64 数据手册编写:
 *  - 容量 64Mbit = 8MB, JEDEC ID = 0xEF 0x40 0x17
 *  - IO2/IO3 接 WP/HOLD: 芯片仅工作在标准 SPI 模式, OCTOSPI 外设在
 *    非数据阶段自动将 IO2/IO3 驱动为高 ( inactive), 无需软件干预
 *  - SCK 由 BSP drv_qspi.c 按 OSPI 内核时钟与请求上限 (92MHz) 取整分频,
 *    实际落位 ~91.7MHz 档 (日志打印): 读操作使用 Fast Read(0x0B,
 *    ≤104MHz, 8 dummy cycles), 不能使用 50MHz 上限的 Read Data(0x03)
 *  - 页编程 256B/页, 擦除粒度 4KB sector / 32KB / 64KB block / 整片
 *
 * RT-Thread 驱动框架集成: BSP drv_qspi.c 注册 "qspi1" 总线 (OCTOSPI1),
 * 本驱动挂 "qspi10" 设备走 rt_qspi_* 框架 API; INIT_DEVICE_EXPORT 自动
 * 初始化 + 互斥锁保证线程安全, 提供 MSH 调试命令: w25q64 id|read|write|erase
 */

#ifndef __SENSOR_W25Q64_H__
#define __SENSOR_W25Q64_H__

#include <rtthread.h>

/* ------------------------- 器件参数 ------------------------- */

#define W25Q64_PAGE_SIZE            256     /* 页编程粒度 */
#define W25Q64_SECTOR_SIZE          4096    /* 最小擦除单位 */
#define W25Q64_BLOCK32_SIZE         (32 * 1024)
#define W25Q64_BLOCK64_SIZE         (64 * 1024)
#define W25Q64_TOTAL_SIZE           (8 * 1024 * 1024)   /* 64Mbit */

#define W25Q64_JEDEC_MANF_ID        0xEF    /* Winbond */
#define W25Q64_JEDEC_MEM_TYPE       0x40    /* W25Qxx */
#define W25Q64_JEDEC_CAPACITY       0x17    /* 64Mbit */

/* ------------------------- 驱动接口 ------------------------- */

/* 自动初始化入口 (INIT_DEVICE_EXPORT) */
int rt_hw_w25q64_init(void);

/* 初始化是否成功 (探测到正确的 JEDEC ID) */
rt_bool_t w25q64_is_ready(void);

/* 读器件 ID: id[0]=厂商, id[1]=类型, id[2]=容量 */
rt_err_t w25q64_read_jedec_id(rt_uint8_t id[3]);

/* 读 64-bit Unique ID (0x4B) */
rt_err_t w25q64_read_unique_id(rt_uint8_t uid[8]);

/*
 * 读数据 (Fast Read 0x0B, 自动跨页/跨块, 任意长度)。
 * 若上一轮编程/擦除尚未完成会先阻塞等待。
 */
rt_err_t w25q64_read(rt_uint32_t addr, rt_uint8_t *buf, rt_uint32_t len);

/*
 * 写数据 (Page Program 0x02, 自动按 256B 页拆分, 任意长度)。
 * 目标区域必须已擦除 (0xFF), 本驱动不检查。
 */
rt_err_t w25q64_write(rt_uint32_t addr, const rt_uint8_t *buf, rt_uint32_t len);

/* 擦除 (内部等待完成后返回; addr 任意, 内部对齐) */
rt_err_t w25q64_erase_sector(rt_uint32_t addr);
rt_err_t w25q64_erase_block_32k(rt_uint32_t addr);
rt_err_t w25q64_erase_block_64k(rt_uint32_t addr);
rt_err_t w25q64_erase_chip(void);

/* 等待器件空闲 (BUSY=0), 超时返回 -RT_ETIMEOUT */
rt_err_t w25q64_wait_ready(rt_uint32_t timeout_ms);

#endif /* __SENSOR_W25Q64_H__ */

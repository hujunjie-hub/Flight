/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * miniTF (microSD) 卡驱动 (SDMMC1, 4-bit 总线, 轮询传输)
 *
 * 硬件连接 (doc/Flight.xlsx 接口配置页):
 *   SDMMC1:
 *     CLK  = PC12 (SDMMC1_CK)
 *     CMD  = PD2  (SDMMC1_CMD)
 *     D0   = PC8  (SDMMC1_D0)
 *     D1   = PC9  (SDMMC1_D1)
 *     D2   = PC10 (SDMMC1_D2)
 *     D3   = PC11 (SDMMC1_D3)
 *   卡座无 CD (卡检测) 引脚: 上电时序内完成识别, 换卡后经 sdmmc_probe()
 *   手动重识别 (MSH: sdmmc probe)
 *
 * 实现要点 (对标 sensor_w25q64 的存储器件驱动模式):
 *  - HAL SD 轮询传输: 无中断无外部 DMA, FIFO 逐字节拆包, 数据缓冲
 *    无对齐要求; DCache 语义无关 (不走总线 DMA)
 *  - 两段时钟 (SDMMC 内核时钟 = PLL1Q 61.111MHz, MspInit 配置):
 *      识别阶段 CLKDIV=75  -> 61.111/(2*77)  ≈ 397kHz (规格 ≤400kHz)
 *      传输阶段 CLKDIV=1   -> 61.111/(2*3)   ≈ 10.19MHz (Default Speed
 *      规格上限 25MHz 内; CLKDIV=0 的 30.6MHz 超 DS 规格, 未做 CMD6
 *      高速切换不放开 —— 如需 25/50MHz, 需启用 PLL2R 50/100MHz 作
 *      SDMMC 内核时钟, 见驱动头注释)
 *  - 时钟换挡借道 HAL_SD_ConfigWideBusOperation: 该 API 用 hsd->Init
 *    重跑 SDMMC_Init, 先改 Init.ClockDiv 再调用即一次完成 4-bit + 换挡
 *  - 读写擦共用一把互斥锁, 接口不可在中断调用; 所有接口要求
 *    sdmmc_is_ready() 为真 (否则 -RT_EIO, 先 sdmmc_probe())
 *  - 卡型支持: SD V1.1/V2.0 (SDSC/SDHC/SDXC); 不支持 MMC 与 UHS-I
 *    (SDUC >2TB 由 HAL 按 SDXC 同路径处理, 未实测)
 *
 * RT-Thread 驱动框架集成: INIT_DEVICE_EXPORT 自动初始化 (无卡仅告警
 * 不报错, 可移卡后手动 probe), 提供 MSH 调试命令:
 * sdmmc info / probe / read / write / erase
 */

#ifndef __SENSOR_SDMMC_H__
#define __SENSOR_SDMMC_H__

#include <rtthread.h>

/* ------------------------- 时钟配置 ------------------------- */

/* SDMMC 内核时钟 = PLL1Q = 61.111MHz (SystemClock_Config, 见 board.c) */
#define SDMMC_KER_CLK_HZ            61111111UL

/* 识别阶段分频: ker/(2*(div+2)) ≈ 397kHz ≤ 400kHz (SD 物理层规格) */
#define SDMMC_CLKDIV_INIT           75

/* 传输阶段分频: ker/(2*(div+2)) ≈ 10.19MHz (Default Speed ≤25MHz 带内;
 * div=0 为 30.56MHz, 超 DS 规格 25MHz, 未做 CMD6 高速切换不可用) */
#define SDMMC_CLKDIV_XFER           1

/* 轮询传输超时: 慢卡单块写 typ ~50ms / max ~500ms, 按 1s/块 + 基础裕量 */
#define SDMMC_RW_TIMEOUT_BASE_MS    500UL
#define SDMMC_RW_TIMEOUT_PER_BLK_MS 1000UL

/* 擦除后等卡回到 TRANSFER 态的上限 (AU 4MB 慢卡 max 秒级, 给足) */
#define SDMMC_ERASE_TIMEOUT_MS      30000UL

/* ------------------------- 卡信息 ------------------------- */

struct sdmmc_card_info
{
    rt_uint8_t  card_type;      /* 0=SDSC 1=SDHC/SDXC 3=Secured (HAL CARD_*) */
    rt_uint8_t  card_version;   /* 0=V1.x 1=V2.0 */
    rt_uint8_t  card_speed;     /* 0=普通速 0x100=高速 (HAL CARD_*_SPEED) */
    rt_uint16_t block_size;     /* 逻辑块大小 (字节, 通常 512) */
    rt_uint32_t block_nbr;      /* 逻辑块数 */
    rt_uint64_t capacity_bytes; /* block_nbr * block_size */

    /* CID 寄存器解析 (HAL_SD_CardCIDTypeDef 摘要) */
    rt_uint8_t  manuf_id;       /* 厂商 ID */
    rt_uint16_t oem_id;         /* OEM/应用 ID */
    rt_uint32_t prod_sn;        /* 产品序列号 */
    rt_uint8_t  prod_rev;       /* 产品版本 (BCD: 高低半字节各一位) */
    rt_uint8_t  manu_year;      /* 生产年 (2000+x) */
    rt_uint8_t  manu_month;     /* 生产月 (1-12) */
    rt_uint8_t  prod_name[6];   /* 产品名 (5 字符 + 结尾 0) */
};

/* ------------------------- 驱动接口 ------------------------- */

/* 自动初始化入口 (INIT_DEVICE_EXPORT); 无卡时仅告警, 返回 RT_EOK */
int rt_hw_sdmmc_init(void);

/* 卡是否在线且识别完成 (读写擦接口的前置条件) */
rt_bool_t sdmmc_is_ready(void);

/*
 * (重新) 识别卡: 完整 DeInit -> Init -> 4-bit + 传输时钟 -> 卡信息。
 * 上电自动调用一次; 换卡/插卡后手动调用 (sdmmc probe)。
 * 成绪返回 RT_EOK 且 sdmmc_is_ready() 为真。
 */
rt_err_t sdmmc_probe(void);

/* 读卡信息快照 (未就绪返回 -RT_EIO) */
rt_err_t sdmmc_get_info(struct sdmmc_card_info *info);

/*
 * 读 count 个逻辑块到 buf (512B/块, buf 无对齐要求)。
 * sector 为块号 (SDHC/SDXC); SDSC 卡按 HAL 语义自动换算字节地址。
 */
rt_err_t sdmmc_read_blocks(rt_uint8_t *buf, rt_uint32_t sector, rt_uint32_t count);

/* 写 count 个逻辑块 (整块写入; 慢卡单块可耗时数百 ms) */
rt_err_t sdmmc_write_blocks(const rt_uint8_t *buf, rt_uint32_t sector,
                            rt_uint32_t count);

/* 擦除 [start, end] 块号区间 (含端点; SD 卡按 AU 对齐执行, 非 FAT 场景慎用) */
rt_err_t sdmmc_erase_blocks(rt_uint32_t start_sector, rt_uint32_t end_sector);

#endif /* __SENSOR_SDMMC_H__ */

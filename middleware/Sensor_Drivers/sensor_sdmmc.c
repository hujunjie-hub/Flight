/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * miniTF (microSD) 卡驱动实现 (SDMMC1 + HAL SD 轮询)
 *
 * 结构对标 sensor_w25q64: 互斥锁串行化 + 就绪门控 + MSH 调试命令;
 * 差异是 TF 卡为可移介质 —— 上电识别失败不算驱动故障, 不派生重试
 * 线程, 由使用方在插卡后主动 sdmmc_probe()。
 *
 * HAL SD 轮询模式要点:
 *  - HAL_SD_Init: 1-bit + 397kHz 识别 (CMD0/8/55/ACMD41/2/3/9/13...),
 *    内部含电压/状态轮询, 失败经 hsd.ErrorCode 暴露原因
 *  - HAL_SD_ConfigWideBusOperation(4B): ACMD6 开 4-bit, 且用 hsd->Init
 *    重跑 SDMMC_Init —— 调用前改 Init.ClockDiv 即同时完成传输时钟换挡
 *  - ReadBlocks/WriteBlocks/Erase 轮询收发, FIFO 逐字节拆包无对齐要求;
 *    写/擦后需等卡回到 HAL_SD_CARD_TRANSFER (CMD13 探询)
 */

#include <rtthread.h>
#include <stdlib.h>
#include <string.h>
#include "sensor_sdmmc.h"
#include "stm32h7xx_hal.h"          /* HAL_SD_* (hal_conf 已启用 HAL_SD) */

#define LOG_TAG "sensor.sdmmc"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

/* ------------------------- 句柄与状态 ------------------------- */

static SD_HandleTypeDef hsd1;
static struct rt_mutex sdmmc_lock;
static rt_bool_t sdmmc_ready = RT_FALSE;

/* 就绪门控: 接口层统一检查, 避免每处重复提示 */
static rt_err_t sdmmc_check_ready(void)
{
    if (!sdmmc_ready)
    {
        LOG_W("card not ready, run: sdmmc probe");
        return -RT_EIO;
    }
    return RT_EOK;
}

/* 等卡回到 TRANSFER 态 (写/擦后的编程周期, CMD13 探询) */
static rt_err_t sdmmc_wait_transfer(rt_uint32_t timeout_ms)
{
    rt_tick_t start = rt_tick_get();
    const rt_tick_t max_tick = rt_tick_from_millisecond(timeout_ms);

    while ((rt_tick_get() - start) < max_tick)
    {
        HAL_SD_CardStateTypeDef st = HAL_SD_GetCardState(&hsd1);

        if (st == HAL_SD_CARD_TRANSFER)
            return RT_EOK;
        if (st == HAL_SD_CARD_ERROR)
            return -RT_EIO;
        rt_thread_mdelay(2);
    }
    return -RT_ETIMEOUT;
}

/* 轮询传输超时: 基础裕量 + 每块预算 */
static rt_uint32_t sdmmc_rw_timeout(rt_uint32_t count)
{
    return SDMMC_RW_TIMEOUT_BASE_MS + SDMMC_RW_TIMEOUT_PER_BLK_MS * count;
}

/* ------------------------- 识别 ------------------------- */

/*
 * 完整识别流程 (调用方持锁):
 *   DeInit (复位外设/失时钟) -> 1-bit 397kHz Init -> 卡信息 ->
 *   4-bit + 10.19MHz 换挡 -> 等 TRANSFER 态
 * 任何一步失败都整体回滚到未就绪, ErrorCode 记入日志。
 */
static rt_err_t sdmmc_hw_probe(void)
{
    HAL_SD_CardInfoTypeDef card;
    HAL_SD_CardCIDTypeDef cid;

    sdmmc_ready = RT_FALSE;
    (void)HAL_SD_DeInit(&hsd1);          /* 首次调用句柄零初始化, 直接过 */

    hsd1.Instance                   = SDMMC1;
    hsd1.Init.ClockEdge             = SDMMC_CLOCK_EDGE_RISING;
    hsd1.Init.ClockPowerSave        = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    hsd1.Init.BusWide               = SDMMC_BUS_WIDE_1B;    /* 识别必经 1-bit */
    hsd1.Init.HardwareFlowControl   = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd1.Init.ClockDiv              = SDMMC_CLKDIV_INIT;

    if (HAL_SD_Init(&hsd1) != HAL_OK)
    {
        LOG_W("card init failed, ErrorCode=0x%08lX (no card?)",
              (unsigned long)hsd1.ErrorCode);
        (void)HAL_SD_DeInit(&hsd1);
        return -RT_EIO;
    }

    if (HAL_SD_GetCardInfo(&hsd1, &card) != HAL_OK ||
        HAL_SD_GetCardCID(&hsd1, &cid) != HAL_OK)
    {
        LOG_W("get card info failed");
        goto fail;
    }

    /* 4-bit + 传输时钟: ConfigWideBusOperation 用 hsd->Init 重配 CLKCR */
    hsd1.Init.ClockDiv = SDMMC_CLKDIV_XFER;
    if (HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) != HAL_OK)
    {
        LOG_W("4-bit bus switch failed, ErrorCode=0x%08lX",
              (unsigned long)hsd1.ErrorCode);
        goto fail;
    }

    if (sdmmc_wait_transfer(1000) != RT_EOK)
    {
        LOG_W("card not in transfer state after init");
        goto fail;
    }

    sdmmc_ready = RT_TRUE;
    LOG_I("TF card ready: %s, %u MB (%u blocks x %uB), bus 4-bit, manuf %u, "
          "made %u-%02u",
          (card.CardType == CARD_SDHC_SDXC) ? "SDHC/SDXC" :
          (card.CardType == CARD_SDSC) ? "SDSC" : "unknown type",
          (unsigned)((rt_uint64_t)card.LogBlockNbr * card.LogBlockSize
                     / (1024UL * 1024UL)),
          (unsigned)card.LogBlockNbr, (unsigned)card.LogBlockSize,
          (unsigned)cid.ManufacturerID,
          2000 + (unsigned)(cid.ManufactDate >> 4),
          (unsigned)(cid.ManufactDate & 0xF));
    return RT_EOK;

fail:
    (void)HAL_SD_DeInit(&hsd1);
    return -RT_EIO;
}

/* ------------------------- 驱动接口 ------------------------- */

int rt_hw_sdmmc_init(void)
{
    rt_err_t err;

    if (rt_mutex_init(&sdmmc_lock, "sdmmc", RT_IPC_FLAG_PRIO) != RT_EOK)
        return -RT_ERROR;

    err = sdmmc_probe();
    if (err != RT_EOK)
        LOG_W("no TF card at boot, insert then run: sdmmc probe");

    /* 识别失败不算驱动初始化失败 (可移介质) */
    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_sdmmc_init);

rt_bool_t sdmmc_is_ready(void)
{
    return sdmmc_ready;
}

rt_err_t sdmmc_probe(void)
{
    rt_err_t err;

    rt_mutex_take(&sdmmc_lock, RT_WAITING_FOREVER);
    err = sdmmc_hw_probe();
    rt_mutex_release(&sdmmc_lock);
    return err;
}

rt_err_t sdmmc_get_info(struct sdmmc_card_info *info)
{
    rt_err_t err;

    if (info == RT_NULL)
        return -RT_EINVAL;

    rt_mutex_take(&sdmmc_lock, RT_WAITING_FOREVER);
    err = sdmmc_check_ready();
    if (err == RT_EOK)
    {
        const HAL_SD_CardInfoTypeDef *c = &hsd1.SdCard;
        HAL_SD_CardCIDTypeDef cid;

        HAL_SD_GetCardCID(&hsd1, &cid);
        memset(info, 0, sizeof(*info));
        info->card_type     = (rt_uint8_t)c->CardType;
        info->card_version  = (rt_uint8_t)c->CardVersion;
        info->card_speed    = (rt_uint8_t)(c->CardSpeed & 0xFF);
        info->block_size    = (rt_uint16_t)c->LogBlockSize;
        info->block_nbr     = c->LogBlockNbr;
        info->capacity_bytes = (rt_uint64_t)c->LogBlockNbr * c->LogBlockSize;

        info->manuf_id      = cid.ManufacturerID;
        info->oem_id        = cid.OEM_AppliID;
        info->prod_sn       = cid.ProdSN;
        info->prod_rev      = cid.ProdRev;
        info->manu_year     = 2000 + (rt_uint8_t)(cid.ManufactDate >> 4);
        info->manu_month    = (rt_uint8_t)(cid.ManufactDate & 0xF);
        /* ProdName1/2 为大端字符序: 高字节是首字符 */
        info->prod_name[0]  = (rt_uint8_t)(cid.ProdName1 >> 24);
        info->prod_name[1]  = (rt_uint8_t)(cid.ProdName1 >> 16);
        info->prod_name[2]  = (rt_uint8_t)(cid.ProdName1 >> 8);
        info->prod_name[3]  = (rt_uint8_t)(cid.ProdName1);
        info->prod_name[4]  = cid.ProdName2;
        info->prod_name[5]  = '\0';
    }
    rt_mutex_release(&sdmmc_lock);
    return err;
}

rt_err_t sdmmc_read_blocks(rt_uint8_t *buf, rt_uint32_t sector, rt_uint32_t count)
{
    rt_err_t err;

    if (buf == RT_NULL || count == 0)
        return -RT_EINVAL;

    rt_mutex_take(&sdmmc_lock, RT_WAITING_FOREVER);
    err = sdmmc_check_ready();
    if (err == RT_EOK)
    {
        /* 越界检查须防 sector+count 32 位回绕 (大 sector 绕过检查直读写
         * 卡外区块), 与 sensor_w25q64 同款写法 */
        if (sector >= hsd1.SdCard.LogBlockNbr ||
            count > hsd1.SdCard.LogBlockNbr - sector)
        {
            err = -RT_EINVAL;
        }
        else if (HAL_SD_ReadBlocks(&hsd1, buf, sector, count,
                                   sdmmc_rw_timeout(count)) != HAL_OK)
        {
            LOG_E("read %u blk @%u failed, ErrorCode=0x%08lX",
                  (unsigned)count, (unsigned)sector,
                  (unsigned long)hsd1.ErrorCode);
            err = -RT_EIO;
        }
        else
        {
            err = sdmmc_wait_transfer(100);
        }
    }
    rt_mutex_release(&sdmmc_lock);
    return err;
}

rt_err_t sdmmc_write_blocks(const rt_uint8_t *buf, rt_uint32_t sector,
                            rt_uint32_t count)
{
    rt_err_t err;

    if (buf == RT_NULL || count == 0)
        return -RT_EINVAL;

    rt_mutex_take(&sdmmc_lock, RT_WAITING_FOREVER);
    err = sdmmc_check_ready();
    if (err == RT_EOK)
    {
        /* 越界检查须防 sector+count 32 位回绕, 与 read 侧同款 */
        if (sector >= hsd1.SdCard.LogBlockNbr ||
            count > hsd1.SdCard.LogBlockNbr - sector)
        {
            err = -RT_EINVAL;
        }
        else if (HAL_SD_WriteBlocks(&hsd1, buf, sector, count,
                                    sdmmc_rw_timeout(count)) != HAL_OK)
        {
            LOG_E("write %u blk @%u failed, ErrorCode=0x%08lX",
                  (unsigned)count, (unsigned)sector,
                  (unsigned long)hsd1.ErrorCode);
            err = -RT_EIO;
        }
        else
        {
            /* 写完 DATAEND 后卡仍在编程周期, 等 TRANSFER 才算落卡 */
            err = sdmmc_wait_transfer(SDMMC_ERASE_TIMEOUT_MS);
        }
    }
    rt_mutex_release(&sdmmc_lock);
    return err;
}

rt_err_t sdmmc_erase_blocks(rt_uint32_t start_sector, rt_uint32_t end_sector)
{
    rt_err_t err;

    if (start_sector > end_sector)
        return -RT_EINVAL;

    rt_mutex_take(&sdmmc_lock, RT_WAITING_FOREVER);
    err = sdmmc_check_ready();
    if (err == RT_EOK)
    {
        if (end_sector >= hsd1.SdCard.LogBlockNbr)
        {
            err = -RT_EINVAL;
        }
        else if (HAL_SD_Erase(&hsd1, start_sector, end_sector) != HAL_OK)
        {
            LOG_E("erase %u..%u failed, ErrorCode=0x%08lX",
                  (unsigned)start_sector, (unsigned)end_sector,
                  (unsigned long)hsd1.ErrorCode);
            err = -RT_EIO;
        }
        else
        {
            /* SD 卡按擦除组 (AU 对齐) 执行, 命令接受 != 擦完 */
            err = sdmmc_wait_transfer(SDMMC_ERASE_TIMEOUT_MS);
        }
    }
    rt_mutex_release(&sdmmc_lock);
    return err;
}

/* ------------------------- MSH 调试命令 ------------------------- */

static void sdmmc_dump(rt_uint32_t base, const rt_uint8_t *buf, rt_uint32_t len)
{
    for (rt_uint32_t i = 0; i < len; i += 16)
    {
        rt_kprintf("%08lX:", (unsigned long)(base + i));
        for (rt_uint32_t j = 0; j < 16 && (i + j) < len; j++)
            rt_kprintf(" %02X", buf[i + j]);
        rt_kprintf("\n");
    }
}

static void sdmmc(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("usage:\n");
        rt_kprintf("  sdmmc info                 card info (type/CID/capacity)\n");
        rt_kprintf("  sdmmc probe                (re)detect card after insert\n");
        rt_kprintf("  sdmmc read <sec> [n]       read n<=4 blocks, dump first 64B\n");
        rt_kprintf("  sdmmc write <sec> <b0> [b1 ...]   write 1 block, b0.. fill head\n");
        rt_kprintf("  sdmmc erase <sec0> <sec1>  erase block range (DESTRUCTIVE)\n");
        return;
    }

    if (!rt_strcmp(argv[1], "probe"))
    {
        rt_err_t err = sdmmc_probe();

        rt_kprintf("probe: %s\n", err == RT_EOK ? "OK" : "FAIL");
    }
    else if (!rt_strcmp(argv[1], "info"))
    {
        struct sdmmc_card_info info;

        if (sdmmc_get_info(&info) != RT_EOK)
            return;
        rt_kprintf("type    : %s, ver V%u.%u, %s\n",
                   info.card_type == 0 ? "SDSC" :
                   info.card_type == 1 ? "SDHC/SDXC" : "secured/other",
                   info.card_version == 1 ? 2 : 1,
                   info.card_version == 1 ? 0 : 1,
                   info.card_speed == 0 ? "normal speed" : "high speed");
        rt_kprintf("capacity: %u MB (%u blocks x %u B)\n",
                   (unsigned)(info.capacity_bytes / (1024UL * 1024UL)),
                   (unsigned)info.block_nbr, (unsigned)info.block_size);
        rt_kprintf("CID     : manuf %u, oem %c%c, name %s, rev %u.%u\n",
                   (unsigned)info.manuf_id,
                   (char)(info.oem_id >> 8), (char)(info.oem_id & 0xFF),
                   info.prod_name,
                   (unsigned)(info.prod_rev >> 4), (unsigned)(info.prod_rev & 0xF));
        rt_kprintf("          sn %08lX, made %u-%02u\n",
                   (unsigned long)info.prod_sn,
                   (unsigned)info.manu_year, (unsigned)info.manu_month);
    }
    else if (!rt_strcmp(argv[1], "read") && argc >= 3)
    {
        rt_uint8_t buf[4 * 512];
        rt_uint32_t sec = strtoul(argv[2], RT_NULL, 0);
        rt_uint32_t n = (argc >= 4) ? strtoul(argv[3], RT_NULL, 0) : 1;

        if (n < 1)
            n = 1;
        if (n > 4)
            n = 4;
        if (sdmmc_read_blocks(buf, sec, n) == RT_EOK)
        {
            rt_kprintf("read %u block(s) @%u OK, first 64B:\n",
                       (unsigned)n, (unsigned)sec);
            sdmmc_dump(sec * 512UL, buf, 64);
        }
        else
        {
            rt_kprintf("read failed\n");
        }
    }
    else if (!rt_strcmp(argv[1], "write") && argc >= 4)
    {
        rt_uint8_t buf[512];
        rt_uint32_t sec = strtoul(argv[2], RT_NULL, 0);
        rt_uint32_t n = argc - 3;
        rt_err_t err;

        if (n > sizeof(buf))
            n = sizeof(buf);
        memset(buf, 0, sizeof(buf));
        for (rt_uint32_t i = 0; i < n; i++)
            buf[i] = (rt_uint8_t)strtoul(argv[3 + i], RT_NULL, 0);

        err = sdmmc_write_blocks(buf, sec, 1);
        rt_kprintf("write 1 block @%u (head %uB payload): %s\n",
                   (unsigned)sec, (unsigned)n, err == RT_EOK ? "OK" : "FAIL");
    }
    else if (!rt_strcmp(argv[1], "erase") && argc >= 4)
    {
        rt_uint32_t s0 = strtoul(argv[2], RT_NULL, 0);
        rt_uint32_t s1 = strtoul(argv[3], RT_NULL, 0);
        rt_err_t err = sdmmc_erase_blocks(s0, s1);

        rt_kprintf("erase %u..%u: %s\n", (unsigned)s0, (unsigned)s1,
                   err == RT_EOK ? "OK" : "FAIL");
    }
    else
    {
        rt_kprintf("bad args, see: sdmmc\n");
    }
}
MSH_CMD_EXPORT(sdmmc, miniTF SD card debug: info / probe / read / write / erase);

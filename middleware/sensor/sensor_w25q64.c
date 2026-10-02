/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * W25Q64 SPI NOR Flash 驱动实现 (RT-Thread QSPI 框架, 1-1-1 模式)
 *
 * 2026-10-01 重构为驱动框架架构 (原实现自持 OSPI 句柄 HAL 直连):
 *   BSP 层    drv_qspi.c 注册 "qspi1" 总线 (OCTOSPI1 + OCTOSPI Manager,
 *             硬件 NCS, RCC/GPIO/分频自含)
 *   设备层    本驱动 attach "qspi10" 设备, 全部命令经框架 API 下发:
 *             rt_qspi_send           指令 [+24bit 地址] [+数据]     (写侧)
 *             rt_qspi_send_then_recv 指令 [+地址] [+dummy] + 接收   (读侧)
 *             rt_qspi_transfer_message 自组消息 (备用字节阶段)
 *
 * 按 Winbond W25Q64 数据手册编写, 实现要点:
 *  - 所有命令走 1-1-1 模式 (指令/地址/数据各 1 线), 轮询传输, 无中断无 DMA
 *  - 读用 Fast Read(0x0B, ≤104MHz, 8 dummy); 50MHz 上限的 0x03 不用
 *  - Unique ID(0x4B) 帧含 32 个 dummy 时钟, 超 H7 DUMCR 31 周期上限,
 *    以 32-bit 备用字节阶段等效实现 (线上同为 32 个时钟, 芯片地址阶段
 *    后忽略输入); 旧直连驱动写 dummy_cycles=4 是错误值, 本次修正
 *  - 读写擦共用一把互斥锁, 接口不可在中断调用
 */

#include <rtthread.h>
#include <stdlib.h>
#include "sensor_w25q64.h"
#include <drivers/dev_spi.h>            /* rt_qspi_* 框架 API */
#include "drv_qspi.h"                   /* rt_hw_qspi_device_attach */
#include "stm32h7xx_hal.h"              /* w25q64 dbg: OCTOSPIM/OCTOSPI/GPIO 寄存器自诊断 */

#define LOG_TAG "sensor.w25q64"
#define LOG_LVL LOG_LVL_DBG        /* 调试驱动: hex dump (LOG_HEX/DBG 级) 需要 */
#include <ulog.h>

/* ------------------------- W25Qxx 指令集 ------------------------- */

#define W25X_CMD_WRITE_ENABLE       0x06    /* 写使能, 置位 WEL */
#define W25X_CMD_READ_STATUS        0x05    /* 读状态寄存器1: bit0=BUSY bit1=WEL */
#define W25X_CMD_FAST_READ          0x0B    /* 快读: 24bit地址 + 8 dummy + 数据 */
#define W25X_CMD_PAGE_PROGRAM       0x02    /* 页编程: ≤256B, 目标须已擦除 */
#define W25X_CMD_SECTOR_ERASE       0x20    /* 4KB */
#define W25X_CMD_BLOCK_ERASE_32K    0x52
#define W25X_CMD_BLOCK_ERASE_64K    0xD8
#define W25X_CMD_CHIP_ERASE         0xC7
#define W25X_CMD_JEDEC_ID           0x9F
#define W25X_CMD_UNIQUE_ID          0x4B    /* 24bit 地址 + 32 dummy + 64bit */

#define W25X_SR_BUSY                0x01

/* ------------------------- 超时 (数据手册 max + 裕量) ------------------------- */

#define W25Q64_T_PP_MS              10      /* 页编程 max 3ms */
#define W25Q64_T_SE_MS              600     /* 4KB 擦除 max 400ms */
#define W25Q64_T_BE32_MS            2000    /* 32KB 擦除 max 1600ms */
#define W25Q64_T_BE64_MS            2500    /* 64KB 擦除 max 2000ms */
#define W25Q64_T_CE_MS              120000  /* 整片擦除 max 96s (JV 系列) */

/* ------------------------- 框架设备 ------------------------- */

/* BSP 总线 "qspi1" (drv_qspi.c @ OCTOSPI1), 本驱动挂载的设备名 */
#define W25Q64_QSPI_BUS_NAME        "qspi1"
#define W25Q64_QSPI_DEV_NAME        "qspi10"

/* 请求 SCK 上限: W25Q64 全命令 ≤104MHz; BSP 按内核时钟取整分频
 * (drv_qspi.c 日志打印实际 SCK), 92MHz 请求落位 ~91.7MHz 档 */
#define W25Q64_SPI_MAX_HZ           92000000

static struct rt_qspi_device *qspi_dev;
static struct rt_mutex w25q64_lock;
static rt_bool_t w25q64_ready = RT_FALSE;

/* ------------------------- 框架命令封装 ------------------------- */

/* 指令 [+24bit 地址] [+数据]: buf[0]=指令, [1..3]=地址(可选), 其余=数据 */
static rt_err_t w25q64_cmd_send(const rt_uint8_t *buf, rt_size_t len)
{
    return (rt_qspi_send(qspi_dev, buf, len) == (rt_ssize_t)len) ? RT_EOK : -RT_EIO;
}

/* 指令 [+24bit 地址] [+dummy 字节] + 接收: 多出的发送字节按 8 周期/字节折算 */
static rt_err_t w25q64_cmd_recv(const rt_uint8_t *buf, rt_size_t len,
                                rt_uint8_t *data, rt_size_t data_len)
{
    return (rt_qspi_send_then_recv(qspi_dev, buf, len, data, data_len) == (rt_ssize_t)data_len)
           ? RT_EOK : -RT_EIO;
}

static rt_err_t w25q64_write_enable(void)
{
    rt_uint8_t cmd[1] = {W25X_CMD_WRITE_ENABLE};

    return w25q64_cmd_send(cmd, 1);
}

static rt_err_t w25q64_read_status(rt_uint8_t *sr)
{
    rt_uint8_t cmd[1] = {W25X_CMD_READ_STATUS};

    return w25q64_cmd_recv(cmd, 1, sr, 1);
}

/* ------------------------- 驱动接口 ------------------------- */

rt_err_t w25q64_wait_ready(rt_uint32_t timeout_ms)
{
    rt_uint8_t sr = W25X_SR_BUSY;
    rt_tick_t start = rt_tick_get();
    const rt_tick_t max_tick = rt_tick_from_millisecond(timeout_ms);

    while ((rt_tick_get() - start) < max_tick)
    {
        if (w25q64_read_status(&sr) != RT_EOK)
            return -RT_EIO;
        if ((sr & W25X_SR_BUSY) == 0)
            return RT_EOK;
        rt_thread_mdelay(1);
    }

    if ((sr & W25X_SR_BUSY) == 0)
        return RT_EOK;
    return -RT_ETIMEOUT;
}

rt_err_t w25q64_read_jedec_id(rt_uint8_t id[3])
{
    rt_uint8_t cmd[1] = {W25X_CMD_JEDEC_ID};

    if (!w25q64_ready)
        return -RT_EIO;
    if (rt_mutex_take(&w25q64_lock, RT_WAITING_FOREVER) != RT_EOK)
        return -RT_ERROR;

    rt_err_t err = w25q64_cmd_recv(cmd, 1, id, 3);

    rt_mutex_release(&w25q64_lock);
    return err;
}

/*
 * Unique ID (0x4B): 指令 + 24bit 地址 + 32 dummy 时钟 + 64bit 数据。
 * 32 超 H7 DUMCR 上限 (31), 用 32-bit 备用字节阶段等效 —— 框架消息层
 * 自组 (备用字节阶段与 dummy 阶段在线上同为固定时钟数的忽略输入)。
 */
rt_err_t w25q64_read_unique_id(rt_uint8_t uid[8])
{
    struct rt_qspi_message msg;

    if (!w25q64_ready)
        return -RT_EIO;
    if (rt_mutex_take(&w25q64_lock, RT_WAITING_FOREVER) != RT_EOK)
        return -RT_ERROR;

    rt_memset(&msg, 0, sizeof(msg));
    msg.instruction.content = W25X_CMD_UNIQUE_ID;
    msg.instruction.qspi_lines = 1;
    msg.address.content = 0;
    msg.address.size = 24;
    msg.address.qspi_lines = 1;
    msg.alternate_bytes.content = 0;
    msg.alternate_bytes.size = 32;      /* 等效 32 个 dummy 时钟 */
    msg.alternate_bytes.qspi_lines = 1;
    msg.qspi_data_lines = 1;
    msg.parent.recv_buf = uid;
    msg.parent.length = 8;
    msg.parent.cs_take = 1;
    msg.parent.cs_release = 1;

    rt_err_t err = (rt_qspi_transfer_message(qspi_dev, &msg) == 8) ? RT_EOK : -RT_EIO;

    rt_mutex_release(&w25q64_lock);
    return err;
}

/* Fast Read (0x0B): 指令 + 24bit 地址 + 1 dummy 字节 (8 周期) + 数据 */
static rt_err_t w25q64_fast_read(rt_uint32_t addr, rt_uint8_t *buf, rt_uint32_t len)
{
    rt_uint8_t cmd[5];

    cmd[0] = W25X_CMD_FAST_READ;
    cmd[1] = (rt_uint8_t)(addr >> 16);
    cmd[2] = (rt_uint8_t)(addr >> 8);
    cmd[3] = (rt_uint8_t)addr;
    cmd[4] = 0x00;                      /* dummy 字节 */

    return w25q64_cmd_recv(cmd, 5, buf, len);
}

rt_err_t w25q64_read(rt_uint32_t addr, rt_uint8_t *buf, rt_uint32_t len)
{
    rt_err_t err;

    if (!w25q64_ready || buf == RT_NULL || len == 0)
        return -RT_EINVAL;
    /* 防溢出写法: addr+len 在 32bit 上会回绕绕过检查 */
    if (addr >= W25Q64_TOTAL_SIZE || len > W25Q64_TOTAL_SIZE - addr)
        return -RT_EINVAL;
    if (rt_mutex_take(&w25q64_lock, RT_WAITING_FOREVER) != RT_EOK)
        return -RT_ERROR;

    /* 等待可能未完成的编程/擦除, 覆盖最长的整片擦除 */
    err = w25q64_wait_ready(W25Q64_T_CE_MS);
    if (err == RT_EOK)
        err = w25q64_fast_read(addr, buf, len);

    rt_mutex_release(&w25q64_lock);
    return err;
}

/* Page Program (0x02): 指令 + 24bit 地址 + ≤256B 数据, 调用方保证不跨页 */
static rt_err_t w25q64_page_program(rt_uint32_t addr, const rt_uint8_t *buf, rt_uint32_t len)
{
    rt_uint8_t cmd[4 + W25Q64_PAGE_SIZE];

    cmd[0] = W25X_CMD_PAGE_PROGRAM;
    cmd[1] = (rt_uint8_t)(addr >> 16);
    cmd[2] = (rt_uint8_t)(addr >> 8);
    cmd[3] = (rt_uint8_t)addr;
    rt_memcpy(&cmd[4], buf, len);

    return w25q64_cmd_send(cmd, 4 + len);
}

rt_err_t w25q64_write(rt_uint32_t addr, const rt_uint8_t *buf, rt_uint32_t len)
{
    rt_err_t err = RT_EOK;

    if (!w25q64_ready || buf == RT_NULL || len == 0)
        return -RT_EINVAL;
    /* 防溢出写法: addr+len 在 32bit 上会回绕绕过检查 */
    if (addr >= W25Q64_TOTAL_SIZE || len > W25Q64_TOTAL_SIZE - addr)
        return -RT_EINVAL;
    if (rt_mutex_take(&w25q64_lock, RT_WAITING_FOREVER) != RT_EOK)
        return -RT_ERROR;

    while (len > 0)
    {
        /* 本页剩余空间, 页编程不能跨页 */
        rt_uint32_t page_left = W25Q64_PAGE_SIZE - (addr % W25Q64_PAGE_SIZE);
        rt_uint32_t chunk = (len < page_left) ? len : page_left;

        err = w25q64_wait_ready(W25Q64_T_CE_MS);
        if (err != RT_EOK)
            break;

        err = w25q64_write_enable();
        if (err != RT_EOK)
            break;

        err = w25q64_page_program(addr, buf, chunk);
        if (err != RT_EOK)
            break;

        addr += chunk;
        buf += chunk;
        len -= chunk;
    }

    /* 最后等待本次编程完成, 返回时器件空闲 */
    if (err == RT_EOK)
        err = w25q64_wait_ready(W25Q64_T_PP_MS);

    rt_mutex_release(&w25q64_lock);
    return err;
}

static rt_err_t w25q64_erase(rt_uint8_t instr, rt_uint32_t addr, rt_uint32_t t_max_ms)
{
    rt_uint8_t cmd[4];
    rt_err_t err;

    cmd[0] = instr;
    cmd[1] = (rt_uint8_t)(addr >> 16);
    cmd[2] = (rt_uint8_t)(addr >> 8);
    cmd[3] = (rt_uint8_t)addr;

    if (rt_mutex_take(&w25q64_lock, RT_WAITING_FOREVER) != RT_EOK)
        return -RT_ERROR;

    err = w25q64_wait_ready(W25Q64_T_CE_MS);
    if (err == RT_EOK)
        err = w25q64_write_enable();
    if (err == RT_EOK)
        err = w25q64_cmd_send(cmd, 4);
    if (err == RT_EOK)
        err = w25q64_wait_ready(t_max_ms);

    rt_mutex_release(&w25q64_lock);
    return err;
}

rt_err_t w25q64_erase_sector(rt_uint32_t addr)
{
    if (!w25q64_ready || addr >= W25Q64_TOTAL_SIZE)
        return -RT_EINVAL;
    return w25q64_erase(W25X_CMD_SECTOR_ERASE, addr, W25Q64_T_SE_MS);
}

rt_err_t w25q64_erase_block_32k(rt_uint32_t addr)
{
    if (!w25q64_ready || addr >= W25Q64_TOTAL_SIZE)
        return -RT_EINVAL;
    return w25q64_erase(W25X_CMD_BLOCK_ERASE_32K, addr, W25Q64_T_BE32_MS);
}

rt_err_t w25q64_erase_block_64k(rt_uint32_t addr)
{
    if (!w25q64_ready || addr >= W25Q64_TOTAL_SIZE)
        return -RT_EINVAL;
    return w25q64_erase(W25X_CMD_BLOCK_ERASE_64K, addr, W25Q64_T_BE64_MS);
}

rt_err_t w25q64_erase_chip(void)
{
    rt_uint8_t cmd[1] = {W25X_CMD_CHIP_ERASE};
    rt_err_t err;

    if (!w25q64_ready)
        return -RT_EIO;
    if (rt_mutex_take(&w25q64_lock, RT_WAITING_FOREVER) != RT_EOK)
        return -RT_ERROR;

    err = w25q64_wait_ready(W25Q64_T_CE_MS);
    if (err == RT_EOK)
        err = w25q64_write_enable();
    if (err == RT_EOK)
        err = w25q64_cmd_send(cmd, 1);
    if (err == RT_EOK)
        err = w25q64_wait_ready(W25Q64_T_CE_MS);

    rt_mutex_release(&w25q64_lock);
    return err;
}

rt_bool_t w25q64_is_ready(void)
{
    return w25q64_ready;
}

/* ------------------------- 初始化 ------------------------- */

int rt_hw_w25q64_init(void)
{
    struct rt_qspi_configuration cfg = {0};
    rt_uint8_t id[3] = {0};
    rt_uint8_t retry;

    if (rt_mutex_init(&w25q64_lock, "w25q64", RT_IPC_FLAG_PRIO) != RT_EOK)
        return -RT_ERROR;

    /* 挂载 QSPI 设备到 BSP 总线 (qspi1 @ OCTOSPI1, 硬件 NCS=PG6, 1 线) */
    if (rt_hw_qspi_device_attach(W25Q64_QSPI_BUS_NAME, W25Q64_QSPI_DEV_NAME,
                                 PIN_NONE, 1, RT_NULL, RT_NULL) != RT_EOK)
    {
        LOG_E("QSPI device attach failed");
        return -RT_ERROR;
    }
    qspi_dev = (struct rt_qspi_device *)rt_device_find(W25Q64_QSPI_DEV_NAME);
    if (qspi_dev == RT_NULL)
    {
        LOG_E("QSPI device %s not found", W25Q64_QSPI_DEV_NAME);
        return -RT_ERROR;
    }

    /* 模式3/8bit/≤92MHz; medium_size 供框架判 24bit 地址 (8MB ≤ 16MB 边界) */
    cfg.parent.mode = RT_SPI_MODE_3 | RT_SPI_MSB;
    cfg.parent.data_width = 8;
    cfg.parent.max_hz = W25Q64_SPI_MAX_HZ;
    cfg.medium_size = W25Q64_TOTAL_SIZE;
    cfg.ddr_mode = 0;
    cfg.qspi_dl_width = 1;
    if (rt_qspi_configure(qspi_dev, &cfg) != RT_EOK)
    {
        LOG_E("QSPI configure failed");
        return -RT_ERROR;
    }

    /* JEDEC ID 校验 (0xEF 0x40 0x17), 重试覆盖上电复位窗口。
     * 注意走内部 w25q64_cmd_recv (不带 w25q64_ready 门禁): ready 标志
     * 在探测成功后才置位, 公开 API 在此恒拒 (-RT_EIO) 且不碰总线 */
    {
        rt_uint8_t cmd[1] = {W25X_CMD_JEDEC_ID};

        for (retry = 0; retry < 3; retry++)
        {
            if (w25q64_cmd_recv(cmd, 1, id, 3) == RT_EOK &&
                id[0] == W25Q64_JEDEC_MANF_ID &&
                id[1] == W25Q64_JEDEC_MEM_TYPE &&
                id[2] == W25Q64_JEDEC_CAPACITY)
                break;
            rt_thread_mdelay(20);
        }
    }

    if (retry >= 3)
    {
        LOG_E("W25Q64 not found (JEDEC=%02X %02X %02X), check OCTOSPI1 wiring "
              "CLK=PF10 IO0=PF8 IO1=PF9 WP=PF7 HOLD=PF6 CS=PG6",
              id[0], id[1], id[2]);
        return -RT_ERROR;
    }

    w25q64_ready = RT_TRUE;
    LOG_I("W25Q64 ready, 8MB via %s (OCTOSPI1, 1-1-1)", W25Q64_QSPI_DEV_NAME);

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_w25q64_init);

/* ------------------------- MSH 调试命令 ------------------------- */

/* ulog 十六进制转储 (DEBUG 级): 每行 16 字节, 显示真实 flash 地址 */
static void w25q64_dump(rt_uint32_t addr, const rt_uint8_t *buf, rt_uint32_t len)
{
    ulog_hexdump(LOG_TAG, 16, buf, (rt_size_t)len, (rt_base_t)addr);
}

/* ------------------------- 取证子命令 (w25q64 dbg) ------------------------- */

/*
 * 固件侧总线自诊断 (不依赖 SWD, 避开调试器争抢): 转储 OCTOSPIM 路由 /
 * OCTOSPI1 配置 / 关键 GPIO 寄存器, 并在当前 SCK 与 13.75MHz 慢速两档
 * 下各发一次 JEDEC 探测 (区分时序问题 vs 芯片不在线)。DCR2 直写仅此
 * 诊断路径 (总线空闲时调用), 运行路径一律走框架。
 */
static void w25q64_dbg(void)
{
    rt_uint8_t id[3] = {0};
    rt_uint8_t cmd[1] = {W25X_CMD_JEDEC_ID};
    rt_uint32_t dcr2_saved = OCTOSPI1->DCR2;
    int i;

    rt_kprintf("OCTOSPIM: CR=0x%08X", OCTOSPIM->CR);
    for (i = 0; i < 3; i++)
        rt_kprintf(" PCR[%d]=0x%08X", i + 1, OCTOSPIM->PCR[i]);
    rt_kprintf("\n");
    /* PCR 位含义: bit1 CLKSRC/IOxSRC/NCSSRC 源选择, bit8 NCSEN, bit12 CLKEN,
     * bit13 IOLEN, bit14 IOHEN (H723 端口1=OCTOSPI1; PCR 实际位序见 RM0468) */

    rt_kprintf("OCTOSPI1: CR=0x%08X DCR1=0x%08X DCR2=0x%08X DCR3=0x%08X\n",
               OCTOSPI1->CR, OCTOSPI1->DCR1, OCTOSPI1->DCR2, OCTOSPI1->DCR3);
    rt_kprintf("          SR=0x%08X CCR=0x%08X TCR=0x%08X IR=0x%08X DLR=0x%08X\n",
               OCTOSPI1->SR, OCTOSPI1->CCR, OCTOSPI1->TCR, OCTOSPI1->IR, OCTOSPI1->DLR);

    rt_kprintf("GPIOF: MODER=0x%08X OSPEEDR=0x%08X AFR[1]=0x%08X IDR=0x%08X\n",
               GPIOF->MODER, GPIOF->OSPEEDR, GPIOF->AFR[1], GPIOF->IDR);
    rt_kprintf("       (PF10 CLK=%d PF9 IO1/MISO=%d PF8 IO0=%d PF7 IO2/WP=%d PF6 IO3/HOLD=%d)\n",
               (GPIOF->IDR >> 10) & 1, (GPIOF->IDR >> 9) & 1, (GPIOF->IDR >> 8) & 1,
               (GPIOF->IDR >> 7) & 1, (GPIOF->IDR >> 6) & 1);
    rt_kprintf("GPIOG: MODER=0x%08X AFR[1]=0x%08X AFR[0]=0x%08X IDR=0x%08X (PG6 NCS=%d)\n",
               GPIOG->MODER, GPIOG->AFR[1], GPIOG->AFR[0], GPIOG->IDR, (GPIOG->IDR >> 6) & 1);

    if (qspi_dev == RT_NULL)
    {
        rt_kprintf("qspi10 not attached, init=%d\n", w25q64_ready);
        return;
    }

    /* 当前 SCK 档 JEDEC */
    (void)w25q64_cmd_recv(cmd, 1, id, 3);
    rt_kprintf("JEDEC @DCR2=%u (SCK=%u kHz): %02X %02X %02X\n",
               dcr2_saved, 275000u / (dcr2_saved + 1u) / 1000u, id[0], id[1], id[2]);

    /* 慢速档 13.75MHz (DCR2=19) 再探一次: 仍全零 => 与时序无关 */
    OCTOSPI1->DCR2 = 19;
    rt_memset(id, 0, sizeof(id));
    (void)w25q64_cmd_recv(cmd, 1, id, 3);
    rt_kprintf("JEDEC @DCR2=19 (SCK=13750 kHz): %02X %02X %02X\n", id[0], id[1], id[2]);

    OCTOSPI1->DCR2 = dcr2_saved;

    /* BSP HAL 直连探针: 定位 Command/Receive 哪一级失败 */
    stm32_qspi_bus_diag();
}

static void w25q64(int argc, char **argv)
{
    rt_uint8_t buf[64];

    if (argc < 2)
    {
        LOG_I("usage:");
        LOG_I("  w25q64 id                 read JEDEC/Unique ID");
        LOG_I("  w25q64 read <addr> [len]  hex dump (default 64B)");
        LOG_I("  w25q64 write <addr> <b0> [b1 ...]  write bytes (max 64)");
        LOG_I("  w25q64 erase <addr> <4k|32k|64k|chip>");
        LOG_I("  w25q64 dbg                dump OCTOSPIM/OSPI/GPIO regs + slow-SCK JEDEC");
        return;
    }

    if (!rt_strcmp(argv[1], "dbg"))
    {
        w25q64_dbg();
    }
    else if (!rt_strcmp(argv[1], "id"))
    {
        rt_uint8_t uid[8];

        if (w25q64_read_jedec_id(buf) == RT_EOK)
            LOG_I("JEDEC ID: %02X %02X %02X (expect EF 40 17)",
                  buf[0], buf[1], buf[2]);
        else
            LOG_E("read JEDEC ID failed");
        if (w25q64_read_unique_id(uid) == RT_EOK)
            LOG_I("Unique ID: %02X%02X%02X%02X%02X%02X%02X%02X",
                  uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6], uid[7]);
    }
    else if (!rt_strcmp(argv[1], "read") && argc >= 3)
    {
        rt_uint32_t addr = strtoul(argv[2], RT_NULL, 0);
        rt_uint32_t len = (argc >= 4) ? strtoul(argv[3], RT_NULL, 0) : sizeof(buf);

        if (len > sizeof(buf))
            len = sizeof(buf);
        if (w25q64_read(addr, buf, len) == RT_EOK)
            w25q64_dump(addr, buf, len);
        else
            LOG_E("read failed");
    }
    else if (!rt_strcmp(argv[1], "write") && argc >= 4)
    {
        rt_uint32_t addr = strtoul(argv[2], RT_NULL, 0);
        rt_uint32_t len = argc - 3;
        rt_err_t err;

        if (len > sizeof(buf))
            len = sizeof(buf);
        for (rt_uint32_t i = 0; i < len; i++)
            buf[i] = (rt_uint8_t)strtoul(argv[3 + i], RT_NULL, 0);

        err = w25q64_write(addr, buf, len);
        LOG_I("write %lu bytes @ 0x%08X: %s",
              (unsigned long)len, addr, err == RT_EOK ? "OK" : "FAIL");
    }
    else if (!rt_strcmp(argv[1], "erase") && argc >= 3)
    {
        rt_err_t err;

        if (!rt_strcmp(argv[2], "chip"))
        {
            LOG_I("chip erasing (may take ~25s)...");
            err = w25q64_erase_chip();
        }
        else if (argc >= 4)
        {
            rt_uint32_t addr = strtoul(argv[2], RT_NULL, 0);

            if (!rt_strcmp(argv[3], "4k"))
                err = w25q64_erase_sector(addr);
            else if (!rt_strcmp(argv[3], "32k"))
                err = w25q64_erase_block_32k(addr);
            else if (!rt_strcmp(argv[3], "64k"))
                err = w25q64_erase_block_64k(addr);
            else
                err = -RT_EINVAL;
        }
        else
            err = -RT_EINVAL;

        if (err == RT_EOK)
            LOG_I("erase: OK");
        else
            LOG_E("erase: FAIL");
    }
    else
    {
        LOG_W("bad args");
    }
}
MSH_CMD_EXPORT(w25q64, W25Q64 SPI flash debug: id / read / write / erase);

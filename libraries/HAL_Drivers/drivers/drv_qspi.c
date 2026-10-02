/*
 * Copyright (c) 2006-2025 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2018-11-27     zylx           first version (QUADSPI, 间接模式)
 * 2025-12-14     LinuxMint-User resolve QSPI interface type mismatch
 * 2026-10-01     Flight         重写为 STM32H723 OCTOSPI1 版: QUADSPI 符号在
 *                              H723 上不存在 (仅 OCTOSPI1/2); 间接模式轮询
 *                              传输, NCS 由 OCTOSPI Manager 硬件驱动 (P1),
 *                              GPIO/RCC/OSPIM 初始化自含 (本工程无 CubeMX
 *                              生成的 OSPI MspInit); 删除 DMA/软 CS 路径
 */

#include "board.h"
#include "drv_qspi.h"
#include "drv_config.h"

#ifdef RT_USING_QSPI

#define DRV_DEBUG
#define LOG_TAG              "drv.qspi"
#include <drv_log.h>

#if defined(BSP_USING_QSPI)

/* 轮询传输超时 ms: 单条命令 (指令+地址+数据) 级别; 芯片级擦除/编程等待
 * 由上层驱动按状态寄存器轮询 (w25q64_wait_ready), 不经过本值 */
#define QSPI_CMD_TIMEOUT_MS       100

struct stm32_qspi_bus
{
    OSPI_HandleTypeDef OSPI_Handler;
    rt_bool_t configured;          /* 首次 configure 完成外设初始化后不再重入 */
};

struct rt_spi_bus _qspi_bus1;
static struct stm32_qspi_bus _stm32_qspi_bus;

/* ---------------- GPIO/RCC 初始化 (自含) ---------------- */

/*
 * Flight 板 OCTOSPI Manager Port1 引脚 (doc/Flight.xlsx + CubeMX DB
 * GPIO-STM32H72_gpio_v1_0_Modes.xml 逐脚 AF 实证, 2026-10-01):
 *   PF10=CLK -> AF9_OCTOSPIM_P1 (注意! 其余脚才是 AF10, 旧驱动全部按
 *             AF10 配置导致时钟根本未接引脚, 芯片零响应的根因)
 *   PF8=IO0(DI), PF9=IO1(DO) -> AF10_OCTOSPIM_P1
 *   PF7=IO2(WP), PF6=IO3(HOLD) -> 普通 GPIO 输出高!
 *     1-1-1 模式下 OCTOSPI 不驱动 IO2/IO3 (AF 态实测输出低), W25Q64 的
 *     /WP 与 /HOLD 被拉低; 本总线不跑 2/4 线, 改配 GPIO 推挽输出高
 *     (inactive) 满足 W25Q64 时序。将来启用 2/4 线须切回 AF10 并管理电平。
 *   PG6=NCS -> AF10_OCTOSPIM_P1 (OCTOSPI Manager 硬件驱动)
 */
static void stm32_qspi_pin_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /* PF10 CLK: AF9 (CubeMX DB 实证, 非与其余数据脚相同的 AF10) */
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF9_OCTOSPIM_P1;
    gpio.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOF, &gpio);

    /* PF8 IO0 / PF9 IO1 + PG6 NCS: AF10 复用推挽, 甚高速 */
    gpio.Alternate = GPIO_AF10_OCTOSPIM_P1;
    gpio.Pin = GPIO_PIN_9 | GPIO_PIN_8;
    HAL_GPIO_Init(GPIOF, &gpio);
    gpio.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOG, &gpio);

    /* PF7 IO2(WP) / PF6 IO3(HOLD): GPIO 输出高, 常态 inactive */
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pin = GPIO_PIN_7 | GPIO_PIN_6;
    HAL_GPIO_Init(GPIOF, &gpio);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7 | GPIO_PIN_6, GPIO_PIN_SET);
}

/* ---------------- 框架回调: 总线初始化 ---------------- */

/*
 * OSPI 内核时钟: 本 HAL 的 HAL_RCCEx_GetPeriphCLKFreq 无 OSPI 分支
 * (恒返回 0), 自读 D1CCIPR.OCTOSPISEL 计算复位默认源 D1HCLK (=HCLK);
 * PLL/CLKP 源告警回退 HCLK (本工程 SystemClock_Config 未改过 OSPISEL,
 * 该路径仅为显式说明, 若有工程切换源需回来补 PLL1Q/PLL2R 换算)。
 */
static rt_uint32_t qspi_kernel_clk(void)
{
    rt_uint32_t sel = __HAL_RCC_GET_OSPI_SOURCE();

    if (sel != RCC_OSPICLKSOURCE_D1HCLK)
    {
        LOG_W("unhandled OSPI kernel clock source 0x%X, assume HCLK", sel);
    }
    return HAL_RCC_GetHCLKFreq();
}

/*
 * 首次 configure 时初始化 OCTOSPI1: RCC + GPIO + 外设 + OCTOSPI Manager。
 * 分频按 OSPI 内核时钟计算 (SCK = 内核时钟 / (prescaler + 1) <= max_hz),
 * 实际 SCK 以本函数日志为准。
 */
static rt_err_t stm32_qspi_init(struct rt_qspi_device *device, struct rt_qspi_configuration *qspi_cfg)
{
    rt_uint32_t kerclk;
    rt_uint32_t i = 2;
    OSPI_HandleTypeDef *hospi = &_stm32_qspi_bus.OSPI_Handler;
    OSPIM_CfgTypeDef ospim = {0};
    struct rt_spi_configuration *cfg = &qspi_cfg->parent;

    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(qspi_cfg != RT_NULL);

    if (_stm32_qspi_bus.configured)
        return RT_EOK;

    __HAL_RCC_OSPI1_CLK_ENABLE();
    __HAL_RCC_OCTOSPIM_CLK_ENABLE();
    stm32_qspi_pin_init();

    {
        OSPI_HandleTypeDef init = QSPI_BUS_CONFIG;
        *hospi = init;
    }

    kerclk = qspi_kernel_clk();
    /* HAL_OSPI 语义: 寄存器存 N-1, SCK = 内核时钟/N (HAL_OSPI_Init 写入时
     * 减 1)。N 从 2 起保证有分频 (N=1 寄存器 0 = 不分频, 对 NOR 不可用) */
    while (cfg->max_hz < kerclk / i)
    {
        i++;
        if (i == 255)
        {
            LOG_E("QSPI init failed, max_hz(%d Hz) too low for kernel clock %d Hz",
                  cfg->max_hz, kerclk);
            return -RT_ERROR;
        }
    }
    hospi->Init.ClockPrescaler = i;

    /* 框架 CPOL/CPHA -> OSPI ClockMode (1-1-1 SPI 模式 0/3) */
    if (!(cfg->mode & RT_SPI_CPOL))
        hospi->Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
    else
        hospi->Init.ClockMode = HAL_OSPI_CLOCK_MODE_3;

    /* 器件容量: DeviceSize = 地址位数 (W25Q64 8MB=2^23 -> 23, HAL 写
     * 寄存器时减 1 成 DEVSIZE=22) */
    hospi->Init.DeviceSize = POSITION_VAL(qspi_cfg->medium_size);

    if (HAL_OSPI_Init(hospi) != HAL_OK)
    {
        LOG_E("HAL_OSPI_Init failed");
        return -RT_ERROR;
    }

    /* 全部信号位于 Port1 低字节 (PF6-PF10 + PG6), 高半字节/DQS 未用 */
    ospim.ClkPort = 1;
    ospim.NCSPort = 1;
    ospim.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
    ospim.IOHighPort = HAL_OSPIM_IOPORT_NONE;
    ospim.DQSPort = 0;
    ospim.Req2AckTime = 1;
    if (HAL_OSPIM_Config(hospi, &ospim, QSPI_CMD_TIMEOUT_MS) != HAL_OK)
    {
        LOG_E("HAL_OSPIM_Config failed");
        return -RT_ERROR;
    }

    _stm32_qspi_bus.configured = RT_TRUE;
    LOG_I("qspi1 ready @ OCTOSPI1, kernel=%d Hz prescaler=%d SCK=%d Hz",
          kerclk, i, kerclk / i);
    return RT_EOK;
}

/* ---------------- 框架回调: 消息传输 ---------------- */

/* rt_qspi_message -> OSPI_RegularCmdTypeDef (间接模式常规命令) */
static rt_err_t qspi_send_cmd(OSPI_HandleTypeDef *hospi, struct rt_qspi_message *message)
{
    OSPI_RegularCmdTypeDef cmd;

    RT_ASSERT(hospi != RT_NULL);
    RT_ASSERT(message != RT_NULL);

    rt_memset(&cmd, 0, sizeof(cmd));
    cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
    cmd.FlashId = HAL_OSPI_FLASH_ID_1;

    cmd.Instruction = message->instruction.content;
    cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
    switch (message->instruction.qspi_lines)
    {
    case 0:  cmd.InstructionMode = HAL_OSPI_INSTRUCTION_NONE; break;
    case 2:  cmd.InstructionMode = HAL_OSPI_INSTRUCTION_2_LINES; break;
    case 4:  cmd.InstructionMode = HAL_OSPI_INSTRUCTION_4_LINES; break;
    default: cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE; break;
    }

    cmd.Address = message->address.content;
    cmd.AddressSize = (message->address.size == 32) ? HAL_OSPI_ADDRESS_32_BITS
                                                    : HAL_OSPI_ADDRESS_24_BITS;
    switch (message->address.qspi_lines)
    {
    case 0:  cmd.AddressMode = HAL_OSPI_ADDRESS_NONE; break;
    case 2:  cmd.AddressMode = HAL_OSPI_ADDRESS_2_LINES; break;
    case 4:  cmd.AddressMode = HAL_OSPI_ADDRESS_4_LINES; break;
    default: cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE; break;
    }

    cmd.AlternateBytes = message->alternate_bytes.content;
    switch (message->alternate_bytes.size)
    {
    case 8:  cmd.AlternateBytesSize = HAL_OSPI_ALTERNATE_BYTES_8_BITS; break;
    case 16: cmd.AlternateBytesSize = HAL_OSPI_ALTERNATE_BYTES_16_BITS; break;
    case 32: cmd.AlternateBytesSize = HAL_OSPI_ALTERNATE_BYTES_32_BITS; break;
    default: cmd.AlternateBytesSize = HAL_OSPI_ALTERNATE_BYTES_24_BITS; break;
    }
    switch (message->alternate_bytes.qspi_lines)
    {
    case 0:  cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE; break;
    case 2:  cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_2_LINES; break;
    case 4:  cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_4_LINES; break;
    default: cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_1_LINE; break;
    }

    /* H7 DUMCR 上限 31 周期; 更长的固定等待 (如 W25X Unique ID 需 32) 由
     * 消息层用备用字节阶段等效表达 */
    cmd.DummyCycles = message->dummy_cycles;
    cmd.NbData = message->parent.length;
    switch (message->qspi_data_lines)
    {
    case 0:  cmd.DataMode = HAL_OSPI_DATA_NONE; break;
    case 2:  cmd.DataMode = HAL_OSPI_DATA_2_LINES; break;
    case 4:  cmd.DataMode = HAL_OSPI_DATA_4_LINES; break;
    default: cmd.DataMode = HAL_OSPI_DATA_1_LINE; break;
    }

    cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
    cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

    if (HAL_OSPI_Command(hospi, &cmd, QSPI_CMD_TIMEOUT_MS) != HAL_OK)
    {
        LOG_E("OSPI command failed(%d)!", hospi->ErrorCode);
        return -RT_ERROR;
    }
    return RT_EOK;
}

static rt_ssize_t qspixfer(struct rt_spi_device *device, struct rt_spi_message *message)
{
    rt_ssize_t result = 0;
    HAL_StatusTypeDef hal_ret;
    struct rt_qspi_message *qspi_message = (struct rt_qspi_message *)message;
    struct stm32_qspi_bus *qspi_bus;
    OSPI_HandleTypeDef *hospi;
    const rt_uint8_t *sndb = message->send_buf;
    rt_uint8_t *rcvb = message->recv_buf;
    rt_int32_t length = message->length;

    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(device->bus != RT_NULL);

    qspi_bus = device->bus->parent.user_data;
    hospi = &qspi_bus->OSPI_Handler;

    /* NCS 由 OCTOSPI 外设硬件驱动: 一条消息 = 一条常规命令 = 一个完整
     * CS 压低周期 (指令+地址+备用字节+dummy+数据), 无软件 CS 路径 */
    if (qspi_send_cmd(hospi, qspi_message) != RT_EOK)
    {
        hospi->State = HAL_OSPI_STATE_READY;
        return -RT_ERROR;
    }

    if (length != 0)
    {
        if (sndb != RT_NULL)
            hal_ret = HAL_OSPI_Transmit(hospi, (rt_uint8_t *)sndb, QSPI_CMD_TIMEOUT_MS);
        else
            hal_ret = HAL_OSPI_Receive(hospi, rcvb, QSPI_CMD_TIMEOUT_MS);

        if (hal_ret == HAL_OK)
        {
            result = length;
        }
        else
        {
            LOG_E("OSPI %s failed(%d)!", sndb != RT_NULL ? "send" : "recv",
                  hospi->ErrorCode);
            hospi->State = HAL_OSPI_STATE_READY;
            result = -RT_ERROR;
        }
    }
    else
    {
        /* 纯指令/指令+地址命令 (写使能/擦除): DataMode=NONE 时
         * HAL_OSPI_Command 内已启动传输并等待 TC 完成 */
        result = 1;
    }

    return result;
}

static rt_err_t qspi_configure(struct rt_spi_device *device, struct rt_spi_configuration *configuration)
{
    struct rt_qspi_device *qspi_device = (struct rt_qspi_device *)device;

    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(configuration != RT_NULL);

    return stm32_qspi_init(qspi_device, &qspi_device->config);
}

static const struct rt_spi_ops stm32_qspi_ops =
{
    .configure = qspi_configure,
    .xfer = qspixfer,
};

static int stm32_qspi_register_bus(struct stm32_qspi_bus *qspi_bus, const char *name)
{
    RT_ASSERT(qspi_bus != RT_NULL);
    RT_ASSERT(name != RT_NULL);

    _qspi_bus1.parent.user_data = qspi_bus;
    return rt_qspi_bus_register(&_qspi_bus1, name, &stm32_qspi_ops);
}

/**
  * @brief  This function attach device to QSPI bus.
  * @param  device_name      QSPI device name
  * @param  cs_pin           reserved (NCS driven by OCTOSPI Manager hardware)
  * @param  data_line_width  QSPI data lines width, such as 1, 2, 4
  * @param  enter_qspi_mode  Callback function that lets FLASH enter QSPI mode
  * @param  exit_qspi_mode   Callback function that lets FLASH exit QSPI mode
  * @retval 0 : success
  *        -1 : failed
  */
rt_err_t rt_hw_qspi_device_attach(const char *bus_name, const char *device_name, rt_base_t cs_pin, rt_uint8_t data_line_width, void (*enter_qspi_mode)(), void (*exit_qspi_mode)())
{
    struct rt_qspi_device *qspi_device = RT_NULL;
    rt_err_t result = RT_EOK;

    RT_ASSERT(bus_name != RT_NULL);
    RT_ASSERT(device_name != RT_NULL);
    RT_ASSERT(data_line_width == 1 || data_line_width == 2 || data_line_width == 4);
    RT_UNUSED(cs_pin);

    qspi_device = (struct rt_qspi_device *)rt_malloc(sizeof(struct rt_qspi_device));
    if (qspi_device == RT_NULL)
    {
        LOG_E("no memory, qspi bus attach device failed!");
        return -RT_ENOMEM;
    }
    rt_memset(qspi_device, 0, sizeof(struct rt_qspi_device));

    /* Safe type conversion to resolve interface contract mismatch.
     * Caller ensures the function pointer is compatible via adapter pattern.
     */
    if (enter_qspi_mode != RT_NULL)
    {
        qspi_device->enter_qspi_mode = (void (*)(struct rt_qspi_device *))enter_qspi_mode;
    }

    if (exit_qspi_mode != RT_NULL)
    {
        qspi_device->exit_qspi_mode = (void (*)(struct rt_qspi_device *))exit_qspi_mode;
    }

    qspi_device->config.qspi_dl_width = data_line_width;

    /* NCS 由硬件驱动, 不走软件 CS 引脚 */
    result = rt_spi_bus_attach_device_cspin(&qspi_device->parent, device_name, bus_name, PIN_NONE, RT_NULL);

    if (result != RT_EOK)
    {
        rt_free(qspi_device);
    }

    return result;
}

static int rt_hw_qspi_bus_init(void)
{
    return stm32_qspi_register_bus(&_stm32_qspi_bus, "qspi1");
}
INIT_BOARD_EXPORT(rt_hw_qspi_bus_init);

/*
 * HAL 直连诊断 (w25q64 dbg 调用): 绕过框架直接驱动 HAL_OSPI, 逐步打印
 * 返回码/状态机/寄存器痕迹, 定位 Command/Receive 哪一级失败。
 * 仅诊断用, 会发起一次真实 JEDEC 传输 (总线空闲时调用)。
 */
void stm32_qspi_bus_diag(void)
{
    OSPI_HandleTypeDef *h = &_stm32_qspi_bus.OSPI_Handler;
    OSPI_RegularCmdTypeDef cmd = {0};
    rt_uint8_t buf[3] = {0xAA, 0x55, 0x77};
    HAL_StatusTypeDef ret;

    if (!_stm32_qspi_bus.configured)
    {
        rt_kprintf("qspi diag: bus not configured yet\n");
        return;
    }

    rt_kprintf("qspi diag: State=%d ErrorCode=0x%X SR=0x%08X\n",
               h->State, h->ErrorCode, OCTOSPI1->SR);

    /* JEDEC READ (0x9F): 1-1-1, 无地址, 收 3 字节 */
    cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
    cmd.FlashId = HAL_OSPI_FLASH_ID_1;
    cmd.Instruction = 0x9F;
    cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
    cmd.AddressMode = HAL_OSPI_ADDRESS_NONE;
    cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode = HAL_OSPI_DATA_1_LINE;
    cmd.NbData = 3;
    cmd.DummyCycles = 0;
    cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
    cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

    ret = HAL_OSPI_Command(h, &cmd, QSPI_CMD_TIMEOUT_MS);
    rt_kprintf("qspi diag: Command ret=%d err=0x%X State=%d\n", ret, h->ErrorCode, h->State);
    rt_kprintf("  CR=0x%08X CCR=0x%08X TCR=0x%08X IR=0x%08X DLR=0x%08X AR=0x%08X SR=0x%08X\n",
               OCTOSPI1->CR, OCTOSPI1->CCR, OCTOSPI1->TCR, OCTOSPI1->IR,
               OCTOSPI1->DLR, OCTOSPI1->AR, OCTOSPI1->SR);

    if (ret == HAL_OK)
    {
        ret = HAL_OSPI_Receive(h, buf, QSPI_CMD_TIMEOUT_MS);
        rt_kprintf("qspi diag: Receive ret=%d err=0x%X State=%d data=%02X %02X %02X\n",
                   ret, h->ErrorCode, h->State, buf[0], buf[1], buf[2]);
        rt_kprintf("  SR=0x%08X FCR write to clear FTF/TCF\n", OCTOSPI1->SR);
        OCTOSPI1->FCR = OCTOSPI_SR_TCF | OCTOSPI_SR_FTF | OCTOSPI_SR_TEF;
    }
}

#endif /* BSP_USING_QSPI */
#endif /* RT_USING_QSPI */

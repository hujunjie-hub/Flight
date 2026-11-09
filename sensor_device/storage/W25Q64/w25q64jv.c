/* W25Q64JV on OCTOSPI1, indirect 1-1-4 commands, RT-Thread native MTD NOR.
 * No erase/program is performed at boot except volatile QE enable.
 */
#include <board.h>
#include <drivers/mtd_nor.h>
#include <string.h>

#define NOR_FLASH_DEVICE_NAME "nor0"
#define NOR_FLASH_SIZE (8u * 1024u * 1024u)

static OSPI_HandleTypeDef hospi1;

void OCTOSPI1_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_OSPI_IRQHandler(&hospi1);
    rt_interrupt_leave();
}
static struct rt_mtd_nor_device g_nor;
static struct rt_mutex g_nor_lock;
static uint32_t g_jedec;

static int command(uint8_t opcode, uint32_t address, uint8_t addressed,
                   uint32_t length, uint8_t quad, uint8_t dummy)
{
    OSPI_RegularCmdTypeDef cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
    cmd.FlashId = HAL_OSPI_FLASH_ID_1;
    cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
    cmd.Instruction = opcode;
    cmd.AddressMode = addressed ? HAL_OSPI_ADDRESS_1_LINE : HAL_OSPI_ADDRESS_NONE;
    cmd.AddressSize = HAL_OSPI_ADDRESS_24_BITS;
    cmd.Address = address;
    cmd.DataMode = length ? (quad ? HAL_OSPI_DATA_4_LINES : HAL_OSPI_DATA_1_LINE)
                          : HAL_OSPI_DATA_NONE;
    cmd.NbData = length;
    cmd.DummyCycles = dummy;
    cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;
    return HAL_OSPI_Command(&hospi1, &cmd, 10u) == HAL_OK ? RT_EOK : -RT_EIO;
}

static int status(uint8_t opcode, uint8_t *value)
{
    if (command(opcode, 0, 0, 1, 0, 0))
    {
        return -RT_EIO;
    }
    return HAL_OSPI_Receive(&hospi1, value, 10u) == HAL_OK ? RT_EOK : -RT_EIO;
}

static int wait_ready(uint32_t timeout_ms)
{
    rt_tick_t start = rt_tick_get();
    uint8_t value;
    do
    {
        if (status(0x05u, &value))
        {
            return -RT_EIO;
        }
        if (!(value & 1u))
        {
            return RT_EOK;
        }
        rt_thread_mdelay(1u);
    } while ((rt_tick_t)(rt_tick_get() - start) < rt_tick_from_millisecond(timeout_ms));
    return -RT_ETIMEOUT;
}

static int write_enable(void)
{
    uint8_t value;
    if (wait_ready(500u) || command(0x06u, 0, 0, 0, 0, 0) || status(0x05u, &value))
    {
        return -RT_EIO;
    }
    return (value & 2u) ? RT_EOK : -RT_EIO;
}

static int in_range(rt_off_t offset, rt_size_t length)
{
    return offset >= 0 && (uint32_t)offset <= NOR_FLASH_SIZE &&
           length <= NOR_FLASH_SIZE - (uint32_t)offset;
}

static rt_err_t nor_id(struct rt_mtd_nor_device *device)
{
    (void)device;
    return (rt_err_t)g_jedec;
}

static rt_ssize_t nor_read(struct rt_mtd_nor_device *device, rt_off_t offset,
                           uint8_t *data, rt_size_t length)
{
    int result;
    (void)device;
    if (!in_range(offset, length) || (!data && length))
    {
        return -RT_EINVAL;
    }
    if (!length)
    {
        return 0;
    }
    rt_mutex_take(&g_nor_lock, RT_WAITING_FOREVER);
    result = wait_ready(500u);
    if (!result)
    {
        result = command(0x6bu, (uint32_t)offset, 1, (uint32_t)length, 1, 8);
    }
    if (!result && HAL_OSPI_Receive(&hospi1, data, 100u) != HAL_OK)
    {
        result = -RT_EIO;
    }
    if (result)
    {
        (void)HAL_OSPI_Abort(&hospi1);
    }
    rt_mutex_release(&g_nor_lock);
    return result ? result : (rt_ssize_t)length;
}

static rt_ssize_t nor_write(struct rt_mtd_nor_device *device, rt_off_t offset,
                            const uint8_t *data, rt_size_t length)
{
    rt_size_t done = 0;
    int result = 0;
    (void)device;
    if (!in_range(offset, length) || (!data && length))
    {
        return -RT_EINVAL;
    }
    rt_mutex_take(&g_nor_lock, RT_WAITING_FOREVER);
    while (done < length)
    {
        uint32_t address = (uint32_t)offset + (uint32_t)done;
        uint32_t count = 256u - (address & 255u);
        if (count > length - done)
        {
            count = (uint32_t)(length - done);
        }
        if ((result = write_enable()) ||
            (result = command(0x32u, address, 1, count, 1, 0)))
        {
            break;
        }
        if (HAL_OSPI_Transmit(&hospi1, (uint8_t *)data + done, 10u) != HAL_OK)
        {
            result = -RT_EIO;
            break;
        }
        if ((result = wait_ready(10u)))
        {
            break;
        }
        done += count;
    }
    if (result)
    {
        (void)HAL_OSPI_Abort(&hospi1);
    }
    rt_mutex_release(&g_nor_lock);
    return result ? result : (rt_ssize_t)done;
}

static rt_err_t nor_erase(struct rt_mtd_nor_device *device, rt_off_t offset,
                          rt_size_t length)
{
    rt_size_t done = 0;
    int result = 0;
    (void)device;
    if (!in_range(offset, length) || ((uint32_t)offset & 4095u) || (length & 4095u))
    {
        return -RT_EINVAL;
    }
    rt_mutex_take(&g_nor_lock, RT_WAITING_FOREVER);
    while (done < length)
    {
        if ((result = write_enable()) ||
            (result = command(0x20u, (uint32_t)offset + (uint32_t)done, 1, 0, 0, 0)) ||
            (result = wait_ready(500u)))
        {
            break;
        }
        done += 4096u;
    }
    if (result)
    {
        (void)HAL_OSPI_Abort(&hospi1);
    }
    rt_mutex_release(&g_nor_lock);
    return result;
}

static const struct rt_mtd_nor_driver_ops nor_ops =
{
    .read_id = nor_id,
    .read = nor_read,
    .write = nor_write,
    .erase_block = nor_erase,
};

static int w25q64jv_init(void)
{
    OSPIM_CfgTypeDef io;
    uint8_t id[3];
    uint8_t sr2;

    memset(&hospi1, 0, sizeof(hospi1));
    hospi1.Instance = OCTOSPI1;
    hospi1.Init.FifoThreshold = 8u;
    hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
    hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MICRON;
    hospi1.Init.DeviceSize = 23u;
    hospi1.Init.ChipSelectHighTime = 2u;
    hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_3;
    hospi1.Init.ClockPrescaler = 3u;
    hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_HALFCYCLE;
    hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
    if (HAL_OSPI_Init(&hospi1) != HAL_OK)
    {
        return -RT_EIO;
    }

    memset(&io, 0, sizeof(io));
    io.ClkPort = 1u;
    io.NCSPort = 1u;
    io.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
    io.Req2AckTime = 1u;
    if (HAL_OSPIM_Config(&hospi1, &io, 10u) != HAL_OK)
    {
        return -RT_EIO;
    }

    /* Release power-down; do not reset or erase an in-progress operation. */
    if (command(0xabu, 0, 0, 0, 0, 0))
    {
        return -RT_EIO;
    }
    rt_thread_mdelay(1u);
    if (wait_ready(500u) || command(0x9fu, 0, 0, 3, 0, 0) ||
        HAL_OSPI_Receive(&hospi1, id, 10u) != HAL_OK)
    {
        return -RT_EIO;
    }
    g_jedec = ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | id[2];
    if (g_jedec != 0xef4017u && g_jedec != 0xef6017u)
    {
        return -RT_EIO;
    }

    if (status(0x35u, &sr2))
    {
        return -RT_EIO;
    }
    if (!(sr2 & 2u))
    {
        /* 50h enables VOLATILE status write; preserve protection/SRP bits. */
        sr2 |= 2u;
        if (command(0x50u, 0, 0, 0, 0, 0) || command(0x31u, 0, 0, 1, 0, 0) ||
            HAL_OSPI_Transmit(&hospi1, &sr2, 10u) != HAL_OK)
        {
            return -RT_EIO;
        }
        rt_thread_mdelay(1u);
        if (status(0x35u, &sr2) || !(sr2 & 2u))
        {
            return -RT_EIO;
        }
    }

    if (rt_mutex_init(&g_nor_lock, "nor", RT_IPC_FLAG_PRIO) != RT_EOK)
    {
        return -RT_EIO;
    }
    g_nor.block_size = 4096u;
    g_nor.block_start = 0u;
    g_nor.block_end = NOR_FLASH_SIZE / 4096u;
    g_nor.ops = &nor_ops;
    return rt_mtd_nor_register_device(NOR_FLASH_DEVICE_NAME, &g_nor);
}
INIT_DEVICE_EXPORT(w25q64jv_init);

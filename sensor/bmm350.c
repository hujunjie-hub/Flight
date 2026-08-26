/*
 * BMM350 Magnetometer Driver
 *
 * Hardware Interface:
 *   I2C1 (PB6=SCL, PB7=SDA, PB5=INT)
 *   I2C Address: 0x14 (SDO/SA0 = GND) or 0x15 (SDO/SA0 = VDDIO)
 *
 * Configuration:
 *   I2C Clock: 400kHz (Fast Mode)
 *   Sample Rate: 100Hz
 *   Data Ready Interrupt: PB5 (EXTI5)
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define BMM350_I2C_DEVICE      "i2c1"
#define BMM350_I2C_ADDR        0x14
#define BMM350_INT_PIN         GET_PIN(B, 5)

/* BMM350 Register Map */
#define BMM350_REG_CHIP_ID         0x00
#define BMM350_REG_PMU_CMD_AGP     0x02
#define BMM350_REG_PMU_CMD_AGY     0x03
#define BMM350_REG_PMU_CMD         0x04
#define BMM350_REG_PMU_CMD_STATUS  0x05
#define BMM350_REG_PMU_CMD_SELF_TEST 0x06
#define BMM350_REG_PMU_CMD_Adj     0x07

#define BMM350_REG_PMU_CMD_RV      0x03
#define BMM350_REG_PMU_CMD_AG      0x04
#define BMM350_REG_PMU_CMD_NVM     0x05

#define BMM350_REG_MAG_X_LSB       0x31
#define BMM350_REG_MAG_X_MSB       0x32
#define BMM350_REG_MAG_Y_LSB       0x33
#define BMM350_REG_MAG_Y_MSB       0x34
#define BMM350_REG_MAG_Z_LSB       0x35
#define BMM350_REG_MAG_Z_MSB       0x36

#define BMM350_REG_TEMP_XYZ_LSB    0x37
#define BMM350_REG_TEMP_XYZ_MSB    0x38

#define BMM350_REG_INT_STATUS      0x30
#define BMM350_REG_INT_CTRL        0x2E
#define BMM350_REG_INT_EN          0x2D

#define BMM350_REG_ODR             0x3C
#define BMM350_REG_OPMODE          0x3B
#define BMM350_REG_PMU_INIT        0x06

#define BMM350_CHIP_ID_VALUE       0x33

/* ODR Settings */
#define BMM350_ODR_1HZ             0x00
#define BMM350_ODR_2HZ             0x01
#define BMM350_ODR_6HZ             0x02
#define BMM350_ODR_8HZ             0x03
#define BMM350_ODR_10HZ            0x04
#define BMM350_ODR_12HZ            0x05
#define BMM350_ODR_15HZ            0x06
#define BMM350_ODR_20HZ            0x07
#define BMM350_ODR_25HZ            0x08
#define BMM350_ODR_30HZ            0x09
#define BMM350_ODR_50HZ            0x0A
#define BMM350_ODR_100HZ           0x0B
#define BMM350_ODR_200HZ           0x0C
#define BMM350_ODR_400HZ           0x0D

/* OPMODE Settings */
#define BMM350_OPMODE_NORMAL       0x00
#define BMM350_OPMODE_FORCED       0x01
#define BMM350_OPMODE_SUSPEND      0x02
#define BMM350_OPMODE_HMC          0x03

/* Magnetometer Data Structure */
struct bmm350_mag_data
{
    rt_int32_t x;       /* uT (micro Tesla) */
    rt_int32_t y;
    rt_int32_t z;
    rt_int32_t temperature; /* 0.01 degrees C */
    rt_uint32_t timestamp;
    rt_bool_t valid;
};

struct bmm350_device
{
    struct rt_i2c_bus_device *i2c_bus;
    struct bmm350_mag_data data;
    rt_sem_t data_ready_sem;
    rt_mutex_t i2c_lock;
    rt_uint8_t rx_buf[8];
};

static struct bmm350_device bmm350_dev = {0};

/* I2C Write Register */
static rt_err_t bmm350_write_reg(rt_uint8_t reg, rt_uint8_t val)
{
    rt_uint8_t buf[2];
    buf[0] = reg;
    buf[1] = val;

    if (rt_i2c_master_send(bmm350_dev.i2c_bus, BMM350_I2C_ADDR, RT_I2C_WR, buf, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

/* I2C Read Register */
static rt_err_t bmm350_read_reg(rt_uint8_t reg, rt_uint8_t *val)
{
    rt_uint8_t buf = reg;

    if (rt_i2c_master_send(bmm350_dev.i2c_bus, BMM350_I2C_ADDR, RT_I2C_WR, &buf, 1) != 1)
    {
        return -RT_ERROR;
    }

    if (rt_i2c_master_recv(bmm350_dev.i2c_bus, BMM350_I2C_ADDR, RT_I2C_RD, val, 1) != 1)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

/* I2C Read Multiple Registers */
static rt_err_t bmm350_read_regs(rt_uint8_t reg, rt_uint8_t *buf, rt_uint8_t len)
{
    rt_uint8_t cmd = reg;

    if (rt_i2c_master_send(bmm350_dev.i2c_bus, BMM350_I2C_ADDR, RT_I2C_WR, &cmd, 1) != 1)
    {
        return -RT_ERROR;
    }

    if (rt_i2c_master_recv(bmm350_dev.i2c_bus, BMM350_I2C_ADDR, RT_I2C_RD, buf, len) != len)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

/* Soft Reset */
static rt_err_t bmm350_soft_reset(void)
{
    rt_err_t ret;

    ret = bmm350_write_reg(BMM350_REG_PMU_CMD, 0xB6);
    rt_thread_mdelay(50);

    return ret;
}

/* Set ODR */
static rt_err_t bmm350_set_odr(rt_uint8_t odr)
{
    return bmm350_write_reg(BMM350_REG_ODR, odr);
}

/* Set Operation Mode */
static rt_err_t bmm350_set_opmode(rt_uint8_t mode)
{
    return bmm350_write_reg(BMM350_REG_OPMODE, mode);
}

/* Data Ready Interrupt Callback */
static void bmm350_int_irq_callback(void *args)
{
    rt_sem_release(bmm350_dev.data_ready_sem);
}

/* Read Magnetometer Data */
static rt_err_t bmm350_read_mag(struct bmm350_mag_data *data)
{
    rt_uint8_t buf[7];
    rt_int16_t raw_x, raw_y, raw_z;

    if (data == RT_NULL)
        return -RT_ERROR;

    /* Read status + XYZ data (7 bytes: 0x31-0x37) */
    if (bmm350_read_regs(BMM350_REG_MAG_X_LSB, buf, 7) != RT_EOK)
    {
        return -RT_ERROR;
    }

    /* Check data ready (bit 0 of status) */
    if (!(buf[0] & 0x01))
    {
        data->valid = RT_FALSE;
        return RT_EOK;
    }

    /* Parse XYZ data (24-bit signed, in LSB format) */
    raw_x = (rt_int16_t)((buf[2] << 8) | buf[1]);
    raw_y = (rt_int16_t)((buf[4] << 8) | buf[3]);
    raw_z = (rt_int16_t)((buf[6] << 8) | buf[5]);

    /* Convert to micro Tesla (0.3uT/LSB for BMM350) */
    data->x = raw_x * 3 / 10;
    data->y = raw_y * 3 / 10;
    data->z = raw_z * 3 / 10;

    data->timestamp = rt_tick_get();
    data->valid = RT_TRUE;

    return RT_EOK;
}

/* Device Init */
static rt_err_t bmm350_init(void)
{
    rt_err_t ret;
    rt_uint8_t chip_id = 0;

    /* Find I2C bus */
    bmm350_dev.i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(BMM350_I2C_DEVICE);
    if (bmm350_dev.i2c_bus == RT_NULL)
    {
        rt_kprintf("[BMM350] I2C bus not found: %s\n", BMM350_I2C_DEVICE);
        return -RT_ERROR;
    }

    /* Create semaphore */
    bmm350_dev.data_ready_sem = rt_sem_create("bmm_sem", 0, RT_IPC_FLAG_FIFO);
    if (bmm350_dev.data_ready_sem == RT_NULL)
    {
        rt_kprintf("[BMM350] Semaphore create failed\n");
        return -RT_ERROR;
    }

    /* Configure INT pin */
    rt_pin_mode(BMM350_INT_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(BMM350_INT_PIN, PIN_IRQ_MODE_RISING, bmm350_int_irq_callback, RT_NULL);
    rt_pin_irq_enable(BMM350_INT_PIN, PIN_IRQ_ENABLE);

    /* Software reset */
    ret = bmm350_soft_reset();
    if (ret != RT_EOK)
    {
        rt_kprintf("[BMM350] Soft reset failed\n");
        return ret;
    }

    /* Read Chip ID */
    ret = bmm350_read_reg(BMM350_REG_CHIP_ID, &chip_id);
    if (ret != RT_EOK || chip_id != BMM350_CHIP_ID_VALUE)
    {
        rt_kprintf("[BMM350] Chip ID mismatch: 0x%02X (expected 0x%02X)\n", chip_id, BMM350_CHIP_ID_VALUE);
        return -RT_ERROR;
    }

    /* Set ODR to 100Hz */
    ret = bmm350_set_odr(BMM350_ODR_100HZ);
    if (ret != RT_EOK)
    {
        rt_kprintf("[BMM350] Set ODR failed\n");
        return ret;
    }

    /* Set Normal mode */
    ret = bmm350_set_opmode(BMM350_OPMODE_NORMAL);
    if (ret != RT_EOK)
    {
        rt_kprintf("[BMM350] Set OPMODE failed\n");
        return ret;
    }

    rt_kprintf("[BMM350] Init OK, ChipID: 0x%02X\n", chip_id);

    return RT_EOK;
}

/* Sensor Thread Entry */
static void bmm350_thread_entry(void *parameter)
{
    struct bmm350_mag_data data;
    rt_err_t ret;

    rt_kprintf("[BMM350] Thread started\n");

    while (1)
    {
        /* Wait for data ready interrupt */
        ret = rt_sem_recv(bmm350_dev.data_ready_sem, 1, RT_WAITING_FOREVER);
        if (ret < 1)
        {
            continue;
        }

        /* Read magnetometer data */
        ret = bmm350_read_mag(&data);
        if (ret == RT_EOK && data.valid)
        {
            /* Store latest data */
            rt_enter_critical();
            bmm350_dev.data = data;
            rt_exit_critical();
        }
    }
}

/* Get latest magnetometer data */
rt_err_t bmm350_get_data(struct bmm350_mag_data *data)
{
    if (data == RT_NULL)
        return -RT_ERROR;

    rt_enter_critical();
    *data = bmm350_dev.data;
    rt_exit_critical();

    return RT_EOK;
}

/* Register magnetometer driver */
static int bmm350_register(void)
{
    rt_err_t ret;
    rt_thread_t thread;

    ret = bmm350_init();
    if (ret != RT_EOK)
    {
        rt_kprintf("[BMM350] Init failed: %d\n", ret);
        return ret;
    }

    /* Create magnetometer data reading thread */
    thread = rt_thread_create("bmm_read", bmm350_thread_entry, RT_NULL,
                              4096, RT_THREAD_PRIORITY_MAX - 6, 10);
    if (thread == RT_NULL)
    {
        rt_kprintf("[BMM350] Thread create failed\n");
        return -RT_ERROR;
    }

    rt_thread_startup(thread);

    return RT_EOK;
}

INIT_APP_EXPORT(bmm350_register);

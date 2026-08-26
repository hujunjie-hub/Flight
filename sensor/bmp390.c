/*
 * BMP390 Barometric Pressure Sensor Driver
 *
 * Hardware Interface:
 *   SPI3 (PB3=SCK, PB2=SDI/MOSI, PB4=SDO/MISO, PB1=CS)
 *   EXTI PB0 (Data Ready interrupt)
 *
 * Configuration:
 *   SPI Clock: 3.819MHz
 *   Sample Rate: 50Hz
 *   SPI Mode: Mode 0 (CPOL=0, CPHA=0), MSB first
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define BMP390_CS_PIN           GET_PIN(B, 1)
#define BMP390_CS_LOW()         rt_pin_write(BMP390_CS_PIN, PIN_LOW)
#define BMP390_CS_HIGH()        rt_pin_write(BMP390_CS_PIN, PIN_HIGH)

#define BMP390_SPI_DEVICE       "spi3"
#define BMP390_INT_PIN          GET_PIN(B, 0)

/* BMP390 Register Map */
#define BMP390_REG_CHIP_ID          0x00
#define BMP390_REG_STATUS           0x03
#define BMP390_REG_PRESS_DATA_3     0x20
#define BMP390_REG_PRESS_DATA_2     0x21
#define BMP390_REG_PRESS_DATA_1     0x22
#define BMP390_REG_TEMP_DATA_3      0x23
#define BMP390_REG_TEMP_DATA_2      0x24
#define BMP390_REG_TEMP_DATA_1      0x25
#define BMP390_REG_SENSORTIME_3     0x26
#define BMP390_REG_SENSORTIME_2     0x27
#define BMP390_REG_SENSORTIME_1     0x28
#define BMP390_REG_EVENT            0x32
#define BMP390_REG_INT_STATUS       0x27
#define BMP390_REG_CONFIG           0x1F
#define BMP390_REG_ODR              0x1D
#define BMP390_REG_OSR              0x1C
#define BMP390_REG_OVR_CONFIG       0x1E
#define BMP390_REG_IIR_CONFIG       0x31
#define BMP390_REG_T_STANDBY        0x1E
#define BMP390_REG_CMD              0x7E
#define BMP390_REG_CMD_EXT          0x7F

#define BMP390_CHIP_ID_VALUE        0x60

/* Commands */
#define BMP390_CMD_SOFT_RESET        0xB6
#define BMP390_CMD_FIFO_FLUSH        0xB0
#define BMP390_CMD_START_NORMAL      0x30
#define BMP390_CMD_START_FORCED      0x10
#define BMP390_CMD_START_FORCED_AUTO 0x30

/* ODR Settings */
#define BMP390_ODR_200HZ            0x00
#define BMP390_ODR_100HZ            0x01
#define BMP390_ODR_50HZ             0x02
#define BMP390_ODR_25HZ             0x03
#define BMP390_ODR_12_5HZ           0x04
#define BMP390_ODR_6_25HZ           0x05
#define BMP390_ODR_3_1HZ            0x06
#define BMP390_ODR_1_5HZ            0x07
#define BMP390_ODR_0_78HZ           0x08
#define BMP390_ODR_0_39HZ           0x09
#define BMP390_ODR_0_2HZ            0x0A
#define BMP390_ODR_0_1HZ            0x0B
#define BMP390_ODR_0_05HZ           0x0C
#define BMP390_ODR_0_02HZ           0x0D
#define BMP390_ODR_0_01HZ           0x0E
#define BMP390_ODR_0_006HZ          0x0F

/* OSR Settings */
#define BMP390_OSR_NONE             0x00
#define BMP390_OSR_2X               0x01
#define BMP390_OSR_4X               0x02
#define BMP390_OSR_8X               0x03
#define BMP390_OSR_16X              0x04
#define BMP390_OSR_32X              0x05

/* IIR Filter */
#define BMP390_IIR_COEFF_0          0x00
#define BMP390_IIR_COEFF_1          0x01
#define BMP390_IIR_COEFF_3          0x02
#define BMP390_IIR_COEFF_7          0x03
#define BMP390_IIR_COEFF_15         0x04
#define BMP390_IIR_COEFF_31         0x05
#define BMP390_IIR_COEFF_63         0x06
#define BMP390_IIR_COEFF_127        0x07

/* BMP390 Data Structure */
struct bmp390_data
{
    rt_int32_t pressure;        /* Pa (Pascals) */
    rt_int32_t temperature;     /* 0.01 degrees C */
    rt_int32_t altitude;        /* cm (calculated from pressure) */
    rt_uint32_t timestamp;
    rt_bool_t valid;
};

struct bmp390_device
{
    struct rt_spi_device *spi_dev;
    struct bmp390_data data;
    rt_sem_t data_ready_sem;
    rt_mutex_t spi_lock;
    rt_uint8_t rx_buf[8];
    rt_uint8_t tx_buf[8];
};

static struct bmp390_device bmp390_dev = {0};

/* Compensation coefficients (read from NVM) */
static rt_int32_t bmp390_t1, bmp390_t2, bmp390_t3;
static rt_int32_t bmp390_p1, bmp390_p2, bmp390_p3, bmp390_p4, bmp390_p5;
static rt_int32_t bmp390_p6, bmp390_p7, bmp390_p8, bmp390_p9, bmp390_p10, bmp390_p11;

/* SPI Read Register */
static rt_err_t bmp390_read_reg(rt_uint8_t reg, rt_uint8_t *val)
{
    rt_uint8_t tx_buf[2];
    rt_uint8_t rx_buf[2];

    tx_buf[0] = reg | 0x80;  /* MSB=1 for read */
    tx_buf[1] = 0x00;

    BMP390_CS_LOW();
    rt_spi_transfer(bmp390_dev.spi_dev, tx_buf, rx_buf, 2);
    BMP390_CS_HIGH();

    *val = rx_buf[1];

    return RT_EOK;
}

/* SPI Read Multiple Registers */
static rt_err_t bmp390_read_regs(rt_uint8_t reg, rt_uint8_t *buf, rt_uint8_t len)
{
    rt_uint8_t tx_buf[8];
    rt_uint8_t rx_buf[8];

    tx_buf[0] = reg | 0x80;  /* MSB=1 for read */

    BMP390_CS_LOW();
    rt_spi_transfer(bmp390_dev.spi_dev, tx_buf, rx_buf, len + 1);
    BMP390_CS_HIGH();

    rt_memcpy(buf, &rx_buf[1], len);

    return RT_EOK;
}

/* SPI Write Register */
static rt_err_t bmp390_write_reg(rt_uint8_t reg, rt_uint8_t val)
{
    rt_uint8_t tx_buf[2];

    tx_buf[0] = reg & 0x7F;  /* MSB=0 for write */
    tx_buf[1] = val;

    BMP390_CS_LOW();
    rt_spi_transmit(bmp390_dev.spi_dev, tx_buf, 2);
    BMP390_CS_HIGH();

    return RT_EOK;
}

/* Read compensation coefficients from NVM */
static rt_err_t bmp390_read_nvm(void)
{
    rt_uint8_t buf[26];
    rt_uint32_t raw_t[3], raw_p[11];

    /* Read NVM data */
    if (bmp390_read_regs(0x31, buf, 26) != RT_EOK)
    {
        return -RT_ERROR;
    }

    /* Parse temperature compensation (3 x 24-bit) */
    raw_t[0] = ((rt_uint32_t)buf[0] << 16) | ((rt_uint32_t)buf[1] << 8) | buf[2];
    raw_t[1] = ((rt_uint32_t)buf[3] << 16) | ((rt_uint32_t)buf[4] << 8) | buf[5];
    raw_t[2] = ((rt_uint32_t)buf[6] << 16) | ((rt_uint32_t)buf[7] << 8) | buf[8];

    /* Parse pressure compensation (11 x 24-bit) */
    for (int i = 0; i < 11; i++)
    {
        int idx = 9 + i * 3;
        raw_p[i] = ((rt_uint32_t)buf[idx] << 16) | ((rt_uint32_t)buf[idx + 1] << 8) | buf[idx + 2];
    }

    /* Convert to signed 32-bit */
    bmp390_t1 = (rt_int32_t)raw_t[0];
    bmp390_t2 = (rt_int32_t)raw_t[1];
    bmp390_t3 = (rt_int32_t)raw_t[2];

    bmp390_p1 = (rt_int32_t)raw_p[0];
    bmp390_p2 = (rt_int32_t)raw_p[1];
    bmp390_p3 = (rt_int32_t)raw_p[2];
    bmp390_p4 = (rt_int32_t)raw_p[3];
    bmp390_p5 = (rt_int32_t)raw_p[4];
    bmp390_p6 = (rt_int32_t)raw_p[5];
    bmp390_p7 = (rt_int32_t)raw_p[6];
    bmp390_p8 = (rt_int32_t)raw_p[7];
    bmp390_p9 = (rt_int32_t)raw_p[8];
    bmp390_p10 = (rt_int32_t)raw_p[9];
    bmp390_p11 = (rt_int32_t)raw_p[10];

    return RT_EOK;
}

/* Compensate temperature */
static rt_int32_t bmp390_compensate_temp(rt_int32_t adc_t)
{
    rt_int32_t t1 = bmp390_t1;
    rt_int32_t t2 = bmp390_t2;
    rt_int32_t t3 = bmp390_t3;

    rt_int64_t temp1, temp2, temp3;

    temp1 = ((rt_int64_t)adc_t) - ((rt_int64_t)t1);
    temp2 = temp1 * ((rt_int64_t)t2);
    temp3 = (temp1 * temp1 * ((rt_int64_t)t3)) >> 30;

    return (rt_int32_t)((temp2 + temp3) / 100);
}

/* Compensate pressure */
static rt_int32_t bmp390_compensate_press(rt_int32_t adc_p, rt_int32_t t_comp)
{
    rt_int32_t p1 = bmp390_p1;
    rt_int32_t p2 = bmp390_p2;
    rt_int32_t p3 = bmp390_p3;
    rt_int32_t p4 = bmp390_p4;
    rt_int32_t p5 = bmp390_p5;
    rt_int32_t p6 = bmp390_p6;
    rt_int32_t p7 = bmp390_p7;
    rt_int32_t p8 = bmp390_p8;
    rt_int32_t p9 = bmp390_p9;
    rt_int32_t p10 = bmp390_p10;
    rt_int32_t p11 = bmp390_p11;

    rt_int64_t press1, press2, press3;

    press1 = ((rt_int64_t)adc_p) / 16384.0;
    press2 = ((rt_int64_t)p1) * press1;
    press3 = press1 * press1;
    press2 = press2 + (((rt_int64_t)p2) * press3) / 1048576.0;
    press3 = press3 * press1;
    press2 = press2 + (((rt_int64_t)p3) * press3) / 4294967296.0;
    press3 = press3 * press1;
    press2 = press2 + (((rt_int64_t)p4) * press3) / 1099511627776.0;
    press3 = press3 * press1;
    press2 = press2 + (((rt_int64_t)p5) * press3) / 4503599627370496.0;
    press3 = press3 * press1;
    press2 = press2 + (((rt_int64_t)p6) * press3) / 18014398509481984.0;

    press3 = ((rt_int64_t)t_comp) / 65536.0;
    press3 = press3 * press3;
    press2 = press2 + (((rt_int64_t)p7) * press3) / 16.0;
    press3 = press3 * ((rt_int64_t)t_comp) / 65536.0;
    press2 = press2 + (((rt_int64_t)p8) * press3) / 2048.0;
    press3 = press3 * ((rt_int64_t)t_comp) / 65536.0;
    press2 = press2 + (((rt_int64_t)p9) * press3) / 65536.0;
    press3 = press3 * ((rt_int64_t)t_comp) / 65536.0;
    press2 = press2 + (((rt_int64_t)p10) * press3) / 4294967296.0;
    press3 = press3 * ((rt_int64_t)t_comp) / 65536.0;
    press2 = press2 + (((rt_int64_t)p11) * press3) / 1099511627776.0;

    return (rt_int32_t)press2;
}

/* Data Ready Interrupt Callback */
static void bmp390_int_irq_callback(void *args)
{
    rt_sem_release(bmp390_dev.data_ready_sem);
}

/* Read Pressure and Temperature */
static rt_err_t bmp390_read_data(struct bmp390_data *data)
{
    rt_uint8_t buf[6];
    rt_int32_t adc_p, adc_t;

    if (data == RT_NULL)
        return -RT_ERROR;

    /* Read press and temp data (6 bytes: 0x20-0x25) */
    if (bmp390_read_regs(BMP390_REG_PRESS_DATA_3, buf, 6) != RT_EOK)
    {
        return -RT_ERROR;
    }

    /* Parse ADC values (24-bit signed) */
    adc_p = ((rt_int32_t)buf[0] << 16) | ((rt_int32_t)buf[1] << 8) | buf[2];
    adc_t = ((rt_int32_t)buf[3] << 16) | ((rt_int32_t)buf[4] << 8) | buf[5];

    /* Compensate temperature */
    data->temperature = bmp390_compensate_temp(adc_t);

    /* Compensate pressure */
    data->pressure = bmp390_compensate_press(adc_p, data->temperature);

    /* Calculate approximate altitude from pressure */
    data->altitude = (rt_int32_t)(44330.0 * (1.0 - pow((double)data->pressure / 101325.0, 1.0 / 5.255)));

    data->timestamp = rt_tick_get();
    data->valid = RT_TRUE;

    return RT_EOK;
}

/* Device Init */
static rt_err_t bmp390_init(void)
{
    rt_err_t ret;
    rt_uint8_t chip_id = 0;

    /* Configure CS pin */
    rt_pin_mode(BMP390_CS_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(BMP390_CS_PIN, PIN_HIGH);

    /* Find SPI device */
    bmp390_dev.spi_dev = (struct rt_spi_device *)rt_device_find(BMP390_SPI_DEVICE);
    if (bmp390_dev.spi_dev == RT_NULL)
    {
        rt_kprintf("[BMP390] SPI device not found: %s\n", BMP390_SPI_DEVICE);
        return -RT_ERROR;
    }

    /* Configure SPI: Mode 0, MSB first, 8-bit, 3.819MHz */
    struct rt_spi_configuration spi_cfg;
    spi_cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB;
    spi_cfg.data_width = 8;
    spi_cfg.max_hz = 3819000;
    ret = rt_spi_configure(bmp390_dev.spi_dev, &spi_cfg);
    if (ret != RT_EOK)
    {
        rt_kprintf("[BMP390] SPI configure failed: %d\n", ret);
        return ret;
    }

    /* Create semaphore */
    bmp390_dev.data_ready_sem = rt_sem_create("bmp_sem", 0, RT_IPC_FLAG_FIFO);
    if (bmp390_dev.data_ready_sem == RT_NULL)
    {
        rt_kprintf("[BMP390] Semaphore create failed\n");
        return -RT_ERROR;
    }

    /* Configure INT pin */
    rt_pin_mode(BMP390_INT_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(BMP390_INT_PIN, PIN_IRQ_MODE_RISING, bmp390_int_irq_callback, RT_NULL);
    rt_pin_irq_enable(BMP390_INT_PIN, PIN_IRQ_ENABLE);

    /* Software reset */
    bmp390_write_reg(BMP390_REG_CMD, BMP390_CMD_SOFT_RESET);
    rt_thread_mdelay(50);

    /* Read Chip ID */
    ret = bmp390_read_reg(BMP390_REG_CHIP_ID, &chip_id);
    if (ret != RT_EOK || chip_id != BMP390_CHIP_ID_VALUE)
    {
        rt_kprintf("[BMP390] Chip ID mismatch: 0x%02X (expected 0x%02X)\n", chip_id, BMP390_CHIP_ID_VALUE);
        return -RT_ERROR;
    }

    /* Read NVM compensation coefficients */
    ret = bmp390_read_nvm();
    if (ret != RT_EOK)
    {
        rt_kprintf("[BMP390] NVM read failed\n");
        return ret;
    }

    /* Configure ODR to 50Hz */
    bmp390_write_reg(BMP390_REG_ODR, BMP390_ODR_50HZ);

    /* Configure OSR: 16x pressure, 2x temperature */
    bmp390_write_reg(BMP390_REG_OSR, (BMP390_OSR_16X << 3) | BMP390_OSR_2X);

    /* Configure IIR filter: coefficient 3 */
    bmp390_write_reg(BMP390_REG_IIR_CONFIG, BMP390_IIR_COEFF_3);

    /* Configure FIFO: disabled */
    bmp390_write_reg(BMP390_REG_OVR_CONFIG, 0x00);

    /* Start normal mode */
    bmp390_write_reg(BMP390_REG_CMD, BMP390_CMD_START_NORMAL);
    rt_thread_mdelay(50);

    rt_kprintf("[BMP390] Init OK, ChipID: 0x%02X\n", chip_id);

    return RT_EOK;
}

/* Sensor Thread Entry */
static void bmp390_thread_entry(void *parameter)
{
    struct bmp390_data data;
    rt_err_t ret;

    rt_kprintf("[BMP390] Thread started\n");

    while (1)
    {
        /* Wait for data ready interrupt */
        ret = rt_sem_recv(bmp390_dev.data_ready_sem, 1, RT_WAITING_FOREVER);
        if (ret < 1)
        {
            continue;
        }

        /* Read pressure and temperature data */
        ret = bmp390_read_data(&data);
        if (ret == RT_EOK && data.valid)
        {
            /* Store latest data */
            rt_enter_critical();
            bmp390_dev.data = data;
            rt_exit_critical();
        }
    }
}

/* Get latest barometer data */
rt_err_t bmp390_get_data(struct bmp390_data *data)
{
    if (data == RT_NULL)
        return -RT_ERROR;

    rt_enter_critical();
    *data = bmp390_dev.data;
    rt_exit_critical();

    return RT_EOK;
}

/* Register barometer driver */
static int bmp390_register(void)
{
    rt_err_t ret;
    rt_thread_t thread;

    ret = bmp390_init();
    if (ret != RT_EOK)
    {
        rt_kprintf("[BMP390] Init failed: %d\n", ret);
        return ret;
    }

    /* Create barometer data reading thread */
    thread = rt_thread_create("bmp_read", bmp390_thread_entry, RT_NULL,
                              4096, RT_THREAD_PRIORITY_MAX - 8, 10);
    if (thread == RT_NULL)
    {
        rt_kprintf("[BMP390] Thread create failed\n");
        return -RT_ERROR;
    }

    rt_thread_startup(thread);

    return RT_EOK;
}

INIT_APP_EXPORT(bmp390_register);

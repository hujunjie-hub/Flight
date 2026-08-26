/*
 * ADIS16505 IMU Driver
 *
 * Hardware Interface:
 *   SPI1 (PA5=SCLK, PA6=DOUT/MISO, PA7=DIN/MOSI, PC4=CS)
 *   EXTI PA4 (Data Ready interrupt)
 *   DMA1_Stream0 (RX), DMA1_Stream1 (TX)
 *
 * Configuration:
 *   SPI Clock: 954.861kHz
 *   Sample Rate: 1000Hz
 *   SPI Mode: Mode 3 (CPOL=1, CPHA=1), MSB first, 16-bit
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include < drv_spi.h>

#define ADIS16505_CS_PIN        GET_PIN(C, 4)
#define ADIS16505_CS_LOW()      rt_pin_write(ADIS16505_CS_PIN, PIN_LOW)
#define ADIS16505_CS_HIGH()     rt_pin_write(ADIS16505_CS_PIN, PIN_HIGH)

#define ADIS16505_SPI_DEVICE    "spi1"
#define ADIS16505_DR_PIN        GET_PIN(A, 4)
#define ADIS16505_DR_IRQ        EXTI4_IRQn

/* ADIS16505 Register Map (16-bit address, bit15=R/W, bit14-8=addr, bit7-0=MSB) */
#define ADIS16505_REG_ID                0x7200
#define ADIS16505_REG_PROD_ID           0x7200
#define ADIS16505_REG_REV_ID            0x7202
#define ADIS16505_REG_SERIAL_NUM        0x7204
#define ADIS16505_REG_TEMP_OUT          0x7206
#define ADIS16505_REG_X_GYRO_OUT        0x7208
#define ADIS16505_REG_Y_GYRO_OUT        0x720A
#define ADIS16505_REG_Z_GYRO_OUT        0x720C
#define ADIS16505_REG_X_ACCL_OUT        0x720E
#define ADIS16505_REG_Y_ACCL_OUT        0x7210
#define ADIS16505_REG_Z_ACCL_OUT        0x7212
#define ADIS16505_REG_X_MAGN_OUT        0x7224
#define ADIS16505_REG_Y_MAGN_OUT        0x7226
#define ADIS16505_REG_Z_MAGN_OUT        0x7228
#define ADIS16505_REG_TIMER_OUT         0x722A
#define ADIS16505_REG_DIAG_STS          0x7230
#define ADIS16505_REG_GLOB_CMD          0x7232
#define ADIS16505_REG_FN_CTRL           0x7234
#define ADIS16505_REG_CONFIG            0x7238
#define ADIS16505_REG_SAMPLE_RATE       0x723A

#define ADIS16505_PROD_ID_VALUE         0x4075

/* Diagnostic Status Bits */
#define ADIS16505_DIAG_ACCL_FAIL        (1 << 0)
#define ADIS16505_DIAG_GYRO_FAIL        (1 << 1)
#define ADIS16505_DIAG_MAG_FAIL         (1 << 2)
#define ADIS16505_DIAG_SRAM_FAIL        (1 << 3)
#define ADIS16505_DIAG_SENSOR_OVERRANGE (1 << 4)
#define ADIS16505_DIAG_SPI_COMM_ERR     (1 << 5)
#define ADIS16505_DIAG_FLASH_UPDATE     (1 << 6)
#define ADIS16505_DIAG_ROM_ERROR        (1 << 7)

/* Global Command Bits */
#define ADIS16505_GLOB_CMD_SW_RESET     (1 << 8)
#define ADIS16505_GLOB_CMD.factory_reset= (1 << 6)
#define ADIS16505_GLOB_CMD_CLR_STAT     (1 << 5)
#define ADIS16505_GLOB_CMD.factory_cal  = (1 << 1)

/* IMU Data Structure */
struct adis16505_data
{
    rt_int16_t x_gyro;
    rt_int16_t y_gyro;
    rt_int16_t z_gyro;
    rt_int16_t x_accl;
    rt_int16_t y_accl;
    rt_int16_t z_accl;
    rt_int16_t x_magn;
    rt_int16_t y_magn;
    rt_int16_t z_magn;
    rt_int16_t temperature;
    rt_uint32_t timer;
    rt_uint32_t timestamp;
};

struct adis16505_device
{
    struct rt_spi_device *spi_dev;
    struct rt_device     *irq_dev;
    struct adis16505_data data;
    rt_sem_t              data_ready_sem;
    rt_mutex_t            spi_lock;
    rt_uint8_t            rx_buf[20];
    rt_uint8_t            tx_buf[20];
};

static struct adis16505_device adis16505_dev = {0};

/* SPI Write: bit15=1 for write */
static rt_err_t adis16505_write_reg(rt_uint16_t reg, rt_uint16_t val)
{
    rt_uint8_t tx_buf[4];
    rt_uint8_t rx_buf[4];

    tx_buf[0] = (rt_uint8_t)((reg >> 8) | 0x80);
    tx_buf[1] = (rt_uint8_t)(reg & 0xFF);
    tx_buf[2] = (rt_uint8_t)(val >> 8);
    tx_buf[3] = (rt_uint8_t)(val & 0xFF);

    ADIS16505_CS_LOW();
    rt_spi_transfer(adis16505_dev.spi_dev, tx_buf, rx_buf, 4);
    ADIS16505_CS_HIGH();

    return RT_EOK;
}

/* SPI Read: bit15=0 for read, need dummy read then data read */
static rt_err_t adis16505_read_reg(rt_uint16_t reg, rt_uint16_t *val)
{
    rt_uint8_t tx_buf[4];
    rt_uint8_t rx_buf[4];

    tx_buf[0] = (rt_uint8_t)((reg >> 8) & 0x7F);
    tx_buf[1] = (rt_uint8_t)(reg & 0xFF);
    tx_buf[2] = 0x00;
    tx_buf[3] = 0x00;

    ADIS16505_CS_LOW();
    rt_spi_transfer(adis16505_dev.spi_dev, tx_buf, rx_buf, 4);
    ADIS16505_CS_HIGH();

    /* Second read to get actual data */
    tx_buf[0] = (rt_uint8_t)((reg >> 8) & 0x7F);
    tx_buf[1] = (rt_uint8_t)(reg & 0xFF);
    tx_buf[2] = 0x00;
    tx_buf[3] = 0x00;

    ADIS16505_CS_LOW();
    rt_spi_transfer(adis16505_dev.spi_dev, tx_buf, rx_buf, 4);
    ADIS16505_CS_HIGH();

    *val = ((rt_uint16_t)rx_buf[2] << 8) | rx_buf[3];

    return RT_EOK;
}

/* Burst Read: read all sensor data in one burst (20 bytes) */
static rt_err_t adis16505_burst_read(struct adis16505_data *data)
{
    rt_uint8_t tx_buf[20];
    rt_uint8_t rx_buf[20];
    rt_uint16_t diag;
    int i;

    rt_memset(tx_buf, 0, sizeof(tx_buf));

    /* ADIS16505 burst read: send register 0x7206 (DIAG_STS) address, read 20 bytes */
    tx_buf[0] = 0x00;  /* burst read command */
    tx_buf[1] = 0x00;

    ADIS16505_CS_LOW();
    rt_spi_transfer(adis16505_dev.spi_dev, tx_buf, rx_buf, 20);
    ADIS16505_CS_HIGH();

    /* Parse burst data */
    /* Byte 0-1: DIAG_STS */
    diag = ((rt_uint16_t)rx_buf[0] << 8) | rx_buf[1];
    /* Byte 2-3: X_GYRO */
    data->x_gyro = (rt_int16_t)(((rt_uint16_t)rx_buf[2] << 8) | rx_buf[3]);
    /* Byte 4-5: Y_GYRO */
    data->y_gyro = (rt_int16_t)(((rt_uint16_t)rx_buf[4] << 8) | rx_buf[5]);
    /* Byte 6-7: Z_GYRO */
    data->z_gyro = (rt_int16_t)(((rt_uint16_t)rx_buf[6] << 8) | rx_buf[7]);
    /* Byte 8-9: X_ACCL */
    data->x_accl = (rt_int16_t)(((rt_uint16_t)rx_buf[8] << 8) | rx_buf[9]);
    /* Byte 10-11: Y_ACCL */
    data->y_accl = (rt_int16_t)(((rt_uint16_t)rx_buf[10] << 8) | rx_buf[11]);
    /* Byte 12-13: Z_ACCL */
    data->z_accl = (rt_int16_t)(((rt_uint16_t)rx_buf[12] << 8) | rx_buf[13]);
    /* Byte 14-15: X_MAGN */
    data->x_magn = (rt_int16_t)(((rt_uint16_t)rx_buf[14] << 8) | rx_buf[15]);
    /* Byte 16-17: Y_MAGN */
    data->y_magn = (rt_int16_t)(((rt_uint16_t)rx_buf[16] << 8) | rx_buf[17]);
    /* Byte 18-19: Z_MAGN + TEMP (combined) */
    data->z_magn = (rt_int16_t)(((rt_uint16_t)rx_buf[18] << 8) | rx_buf[19]);

    data->timestamp = rt_tick_get();

    return RT_EOK;
}

/* DR interrupt callback */
static void adis16505_dr_irq_callback(void *args)
{
    rt_sem_release(adis16505_dev.data_ready_sem);
}

/* Device Init */
static rt_err_t adis16505_init(void)
{
    rt_err_t ret;
    rt_uint16_t prod_id = 0;

    /* Configure CS pin as output, set HIGH */
    rt_pin_mode(ADIS16505_CS_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(ADIS16505_CS_PIN, PIN_HIGH);

    /* Find SPI device */
    adis16505_dev.spi_dev = (struct rt_spi_device *)rt_device_find(ADIS16505_SPI_DEVICE);
    if (adis16505_dev.spi_dev == RT_NULL)
    {
        rt_kprintf("[ADIS16505] SPI device not found: %s\n", ADIS16505_SPI_DEVICE);
        return -RT_ERROR;
    }

    /* Configure SPI: Mode 3, MSB first, 16-bit data width, 954.861kHz */
    struct rt_spi_configuration spi_cfg;
    spi_cfg.mode = RT_SPI_MODE_3 | RT_SPI_MSB;
    spi_cfg.data_width = 16;
    spi_cfg.max_hz = 954861;
    ret = rt_spi_configure(adis16505_dev.spi_dev, &spi_cfg);
    if (ret != RT_EOK)
    {
        rt_kprintf("[ADIS16505] SPI configure failed: %d\n", ret);
        return ret;
    }

    /* Create semaphore for data ready */
    adis16505_dev.data_ready_sem = rt_sem_create("adis_sem", 0, RT_IPC_FLAG_FIFO);
    if (adis16505_dev.data_ready_sem == RT_NULL)
    {
        rt_kprintf("[ADIS16505] Semaphore create failed\n");
        return -RT_ERROR;
    }

    /* Configure DR pin as input with EXTI interrupt */
    rt_pin_mode(ADIS16505_DR_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(ADIS16505_DR_PIN, PIN_IRQ_MODE_FALLING, adis16505_dr_irq_callback, RT_NULL);
    rt_pin_irq_enable(ADIS16505_DR_PIN, PIN_IRQ_ENABLE);

    /* Reset delay - wait for sensor to boot */
    rt_thread_mdelay(250);

    /* Verify Product ID */
    ret = adis16505_read_reg(ADIS16505_REG_PROD_ID, &prod_id);
    if (ret != RT_EOK || prod_id != ADIS16505_PROD_ID_VALUE)
    {
        rt_kprintf("[ADIS16505] Product ID mismatch: 0x%04X (expected 0x%04X)\n", prod_id, ADIS16505_PROD_ID_VALUE);
        return -RT_ERROR;
    }

    /* Software reset */
    adis16505_write_reg(ADIS16505_REG_GLOB_CMD, ADIS16505_GLOB_CMD_SW_RESET);
    rt_thread_mdelay(500);

    /* Configure function control: enable all sensors */
    adis16505_write_reg(ADIS16505_REG_FN_CTRL, 0x0000);

    /* Configure: set sample rate */
    adis16505_write_reg(ADIS16505_REG_CONFIG, 0x0000);

    rt_kprintf("[ADIS16505] Init OK, ProdID: 0x%04X\n", prod_id);

    return RT_EOK;
}

/* Read IMU data (called from sensor thread) */
static rt_err_t adis16505_read(struct adis16505_data *data)
{
    rt_err_t ret;

    if (data == RT_NULL)
        return -RT_ERROR;

    ret = adis16505_burst_read(data);

    return ret;
}

/* Get IMU data with waiting */
rt_err_t adis16505_fetch_data(struct adis16505_data *data)
{
    rt_err_t ret;

    if (data == RT_NULL)
        return -RT_ERROR;

    /* Wait for DR interrupt (1ms timeout for 1000Hz) */
    ret = rt_sem_recv(adis16505_dev.data_ready_sem, 1, RT_WAITING_FOREVER);
    if (ret < 1)
    {
        return -RT_ETIMEOUT;
    }

    return adis16505_read(data);
}

/* Sensor thread entry */
static void adis16505_thread_entry(void *parameter)
{
    struct adis16505_data data;
    rt_err_t ret;

    rt_kprintf("[ADIS16505] Thread started\n");

    while (1)
    {
        ret = adis16505_fetch_data(&data);
        if (ret == RT_EOK)
        {
            /* Store latest data */
            rt_enter_critical();
            adis16505_dev.data = data;
            rt_exit_critical();
        }
    }
}

/* Get latest IMU data */
rt_err_t adis16505_get_data(struct adis16505_data *data)
{
    if (data == RT_NULL)
        return -RT_ERROR;

    rt_enter_critical();
    *data = adis16505_dev.data;
    rt_exit_critical();

    return RT_EOK;
}

/* Register IMU sensor with RT-Thread device framework */
static int adis16505_register(void)
{
    rt_err_t ret;
    rt_thread_t thread;

    ret = adis16505_init();
    if (ret != RT_EOK)
    {
        rt_kprintf("[ADIS16505] Init failed: %d\n", ret);
        return ret;
    }

    /* Create IMU data reading thread */
    thread = rt_thread_create("adis_read", adis16505_thread_entry, RT_NULL,
                              4096, RT_THREAD_PRIORITY_MAX - 2, 10);
    if (thread == RT_NULL)
    {
        rt_kprintf("[ADIS16505] Thread create failed\n");
        return -RT_ERROR;
    }

    rt_thread_startup(thread);

    return RT_EOK;
}

INIT_APP_EXPORT(adis16505_register);

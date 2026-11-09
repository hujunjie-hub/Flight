/* ADIS16505-2 IMU on SPI1 with TIM2 CH4 DRDY capture.
 *
 * Native RT-Thread device "imu": init configures the part, open enables
 * DRDY capture, read returns one complete burst, control returns diagnostics.
 * The 100 Hz DMA burst is started from the TIM2 capture ISR through the
 * bounded SPI1 device-specific extension in drv_spi.c; the completion worker
 * only moves ownership and notifies readers through rx_indicate.
 */
#include "adis16505.h"

static uint16_t read_be16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

static int32_t read_be32_words(uint16_t low, uint16_t high)
{
    return (int32_t)(((uint32_t)high << 16) | (uint32_t)low);
}

static int16_t as_int16(uint16_t value)
{
    return (int16_t)value;
}

sensor_status_t adis16505_payload_checksum(const uint8_t *burst, size_t burst_len)
{
    /* The burst ends with a 16-bit big-endian checksum equal to the modulo
     * 65536 sum of every preceding byte (datasheet p.19-20; the same rule is
     * used by the native transport check). */
    uint16_t checksum = 0u;
    uint16_t sum = 0u;
    size_t index;

    if (burst == NULL) {
        return SENSOR_ERR_ARGUMENT;
    }
    if ((burst_len != 20u) && (burst_len != 32u)) {
        return SENSOR_ERR_LENGTH;
    }
    checksum = read_be16(&burst[burst_len - 2u]);
    for (index = 0; index < burst_len - 2u; ++index) {
        sum = (uint16_t)(sum + (uint16_t)burst[index]);
    }
    return (sum == checksum) ? SENSOR_OK : SENSOR_ERR_CHECKSUM;
}

Adis16505Encoding adis16505_default_encoding(void)
{
    return (Adis16505Encoding){
        .extended = true,
        .gyro_lsb_per_dps = 2621440.0,
        .accel_lsb_per_mps2 = 26756268.0,
        .temp_lsb_per_c = 10.0,
    };
}

sensor_status_t adis16505_decode_payload(const Adis16505Encoding *encoding,
                                        const uint8_t *burst, size_t burst_len, ImuReading *reading)
{
    uint16_t words[16];
    int32_t gyro_raw[3];
    int32_t accel_raw[3];
    int16_t temperature_raw;
    sensor_status_t status;
    size_t word_count;
    size_t index;

    if ((encoding == NULL) || (burst == NULL) || (reading == NULL)) {
        return SENSOR_ERR_ARGUMENT;
    }
    word_count = encoding->extended ? 16u : 10u;
    if (burst_len != word_count * 2u) {
        return SENSOR_ERR_LENGTH;
    }
    if (!(encoding->gyro_lsb_per_dps > 0.0) || !(encoding->accel_lsb_per_mps2 > 0.0) ||
        !(encoding->temp_lsb_per_c > 0.0)) {
        return SENSOR_ERR_NOT_CONFIGURED;
    }

    status = adis16505_payload_checksum(burst, burst_len);
    if (status != SENSOR_OK) {
        return status;
    }

    for (index = 0; index < word_count; ++index) {
        words[index] = read_be16(&burst[index * 2u]);
    }

    if (encoding->extended) {
        gyro_raw[0] = read_be32_words(words[1], words[2]);
        gyro_raw[1] = read_be32_words(words[3], words[4]);
        gyro_raw[2] = read_be32_words(words[5], words[6]);
        accel_raw[0] = read_be32_words(words[7], words[8]);
        accel_raw[1] = read_be32_words(words[9], words[10]);
        accel_raw[2] = read_be32_words(words[11], words[12]);
        temperature_raw = as_int16(words[13]);
    } else {
        gyro_raw[0] = as_int16(words[1]);
        gyro_raw[1] = as_int16(words[2]);
        gyro_raw[2] = as_int16(words[3]);
        accel_raw[0] = as_int16(words[4]);
        accel_raw[1] = as_int16(words[5]);
        accel_raw[2] = as_int16(words[6]);
        temperature_raw = as_int16(words[7]);
    }

    for (index = 0; index < 3; ++index) {
        reading->gyro_dps[index] = (float)((double)gyro_raw[index] / encoding->gyro_lsb_per_dps);
        reading->accel_mps2[index] = (float)((double)accel_raw[index] / encoding->accel_lsb_per_mps2);
    }

    reading->temperature_c = (float)((double)temperature_raw * encoding->temp_lsb_per_c);
    reading->sequence = words[encoding->extended ? 14 : 8];
    reading->diagnostic_clear = words[0] == 0u;
    return SENSOR_OK;
}

#define IMU_TRANSFER_WORDS 17u
#define IMU_PAYLOAD_BYTES 32u

static inline void imu_burst_to_bytes(const uint16_t words[IMU_TRANSFER_WORDS],
                                      uint8_t out[2u * IMU_TRANSFER_WORDS])
{
    for (unsigned i = 0; i < IMU_TRANSFER_WORDS; ++i)
    {
        out[2u * i] = (uint8_t)(words[i] >> 8);
        out[2u * i + 1u] = (uint8_t)words[i];
    }
}

static inline int imu_burst_checksum_valid(const uint8_t bytes[2u * IMU_TRANSFER_WORDS])
{
    return adis16505_payload_checksum(bytes + 2, IMU_PAYLOAD_BYTES) == SENSOR_OK;
}

#if !defined(FLIGHT_DRIVER_CODEC_TEST)
#include "timer_capture.h"
#include "board_device.h"
#include "drv_spi.h"
#include <string.h>

#define IMU_SLOTS 4u
#define IMU_CAP_VALID 0x01u
#define IMU_CAP_OVERCAPTURE 0x02u

static struct rt_spi_device g_imu_spi;
static struct rt_device g_imu_dev;
static struct rt_mutex g_imu_lock;
static struct rt_semaphore g_imu_sem;
static uint32_t g_tick_hz;
static struct rt_thread g_imu_thread;
static uint8_t g_imu_stack[1024] __attribute__((aligned(8)));
static uint16_t g_imu_tx[32] DMA_BUFFER;
static uint16_t g_imu_rx[IMU_SLOTS][32] DMA_BUFFER;
static rt_device_t g_timebase;

static struct
{
    struct timebase_capture capture;
    uint64_t start;
    uint64_t dma_complete;
    uint64_t eot;
    uint32_t error;
} g_meta[IMU_SLOTS];

static struct imu_raw_sample g_ring[IMU_SLOTS];
static volatile uint32_t g_head;      /* DMA producer (ISR) */
static volatile uint32_t g_tail;      /* ingest worker */
static volatile uint32_t g_sample_head;
static volatile uint32_t g_sample_tail;
static volatile uint8_t g_inflight;
static uint8_t g_ready;
static uint8_t g_opened;
static struct imu_stats g_stats;

static int timebase_now(uint64_t *ticks)
{
    struct timebase_instant instant;
    if (!g_timebase)
    {
        return -1;
    }
    if (rt_device_read(g_timebase, 0, &instant, sizeof(instant)) != (rt_ssize_t)sizeof(instant))
    {
        return -1;
    }
    *ticks = instant.ticks;
    return 0;
}

static int imu_word(uint16_t tx, uint16_t *rx)
{
    uint16_t dummy;
    if (!rx)
    {
        rx = &dummy;
    }
    if (rt_spi_transfer(&g_imu_spi, &tx, rx, 1u) != 1)
    {
        return -RT_EIO;
    }
    rt_hw_us_delay(16u);
    return 0;
}

static int imu_reg_read(uint8_t reg, uint16_t *value)
{
    if (imu_word((uint16_t)reg << 8, 0))
    {
        return -RT_EIO;
    }
    return imu_word(0u, value);
}

static int imu_reg_write(uint8_t reg, uint16_t value)
{
    if (imu_word(((uint16_t)(reg | 0x80u) << 8) | (value & 255u), 0) ||
        imu_word(((uint16_t)(reg | 0x81u) << 8) | (value >> 8), 0))
    {
        return -RT_EIO;
    }
    rt_hw_us_delay(200u);
    return 0;
}

static void imu_done(uint32_t error, void *context)
{
    uint64_t now;
    (void)context;

    IMU_CHIP_SELECT_PORT->BSRR = IMU_CHIP_SELECT_MASK; /* HAL callback follows SPI EOT, CS may rise now */
    g_meta[g_head & (IMU_SLOTS - 1u)].error = error;
    if (!timebase_now(&now))
    {
        g_meta[g_head & (IMU_SLOTS - 1u)].eot = now;
    }
    __DMB();
    g_head++;
    g_inflight = 0u;
    (void)rt_sem_release(&g_imu_sem); /* priority 4; TIM2 never calls the RTOS */
}

/* Weak in drv_spi.c; the IMU owns the SPI1 DMA-complete timestamp. */
void board_spi1_dma_rx_tc_isr(void)
{
    uint64_t now;
    if (g_inflight && !timebase_now(&now))
    {
        g_meta[g_head & (IMU_SLOTS - 1u)].dma_complete = now;
    }
}

/* TIM2 CH4 DRDY starts one SPI burst. */
static void imu_capture(const struct timebase_capture *capture)
{
    uint64_t now;
    uint32_t slot;

    if (!g_ready || g_inflight || (uint32_t)(g_head - g_tail) >= IMU_SLOTS)
    {
        g_stats.dropped++;
        return;
    }
    slot = g_head & (IMU_SLOTS - 1u);
    g_inflight = 1u;
    memset(&g_meta[slot], 0, sizeof(g_meta[slot]));
    g_meta[slot].capture = *capture;
    if (!timebase_now(&now))
    {
        g_meta[slot].start = now;
    }
    IMU_CHIP_SELECT_PORT->BSRR = (uint32_t)IMU_CHIP_SELECT_MASK << 16;
    if (stm32_spi1_isr_submit_prearmed(g_imu_tx, g_imu_rx[slot], IMU_TRANSFER_WORDS) != RT_EOK)
    {
        IMU_CHIP_SELECT_PORT->BSRR = IMU_CHIP_SELECT_MASK;
        g_inflight = 0u;
        g_stats.errors++;
        g_stats.dropped++;
    }
}

static rt_err_t imu_set_capture(void (*callback)(const struct timebase_capture *))
{
    struct timebase_imu_capture_config config = {callback};
    return rt_device_control(g_timebase, TIMEBASE_CTRL_SET_IMU_CAPTURE, &config);
}

static void imu_abort_transfer(void)
{
    if (g_inflight)
    {
        (void)stm32_spi1_fast_recover();
        IMU_CHIP_SELECT_PORT->BSRR = IMU_CHIP_SELECT_MASK;
        g_inflight = 0u;
        g_stats.errors++;
    }
}

static void imu_ingest(void *parameter)
{
    (void)parameter;
    for (;;)
    {
        rt_err_t (*notify)(rt_device_t, rt_size_t);
        uint32_t slot;
        uint32_t pushed = 0u;
        uint32_t head;
        struct imu_raw_sample sample;
        uint8_t wire[34];

        (void)rt_sem_take(&g_imu_sem, rt_tick_from_millisecond(2));
        rt_mutex_take(&g_imu_lock, RT_WAITING_FOREVER);
        head = g_head;
        while (g_tail != head)
        {
            slot = g_tail & (IMU_SLOTS - 1u);
            __DMB();
            memset(&sample, 0, sizeof(sample));
            imu_burst_to_bytes(g_imu_rx[slot], wire);
            /* Remove only the command-phase dummy; M2 consumes the 32-byte response. */
            memcpy(sample.data, wire + 2u, IMU_PAYLOAD_BYTES);
            sample.length = IMU_PAYLOAD_BYTES;
            sample.sequence = g_meta[slot].capture.sequence;
            sample.capture_tick = g_meta[slot].capture.tick64;
            sample.dma_start_tick = g_meta[slot].start;
            sample.dma_complete_tick = g_meta[slot].dma_complete;
            sample.eot_tick = g_meta[slot].eot;
            sample.flags = IMU_FLAG_VALID;
            if (!g_meta[slot].dma_complete || g_meta[slot].error ||
                (g_meta[slot].capture.flags & IMU_CAP_OVERCAPTURE))
            {
                sample.flags |= IMU_FLAG_TRANSPORT_ERROR;
                g_stats.errors++;
            }
            if (!imu_burst_checksum_valid(wire))
            {
                g_stats.checksum_errors++;
            }
            g_tail++;

            if ((uint32_t)(g_sample_head - g_sample_tail) >= IMU_SLOTS)
            {
                g_stats.raw_dropped++;
            }
            else
            {
                g_ring[g_sample_head & (IMU_SLOTS - 1u)] = sample;
                __DMB();
                g_sample_head++;
                g_stats.received++;
                pushed = 1u;
            }
        }

        if (g_opened && g_inflight)
        {
            uint64_t now;
            uint64_t start = g_meta[g_head & (IMU_SLOTS - 1u)].start;
            uint32_t expected_head = g_head;
            if (!timebase_now(&now) && now - start > (uint64_t)g_tick_hz / 1000u)
            {
                (void)imu_set_capture(NULL);
                /* A completion may have preempted the timeout check. Do not
                 * abort the following frame or count a completed frame twice. */
                if (g_head == expected_head)
                {
                    imu_abort_transfer();
                }
                (void)imu_set_capture(imu_capture);
            }
        }
        notify = g_imu_dev.rx_indicate;
        rt_mutex_release(&g_imu_lock);
        if (pushed && notify)
        {
            (void)notify(&g_imu_dev, sizeof(struct imu_raw_sample));
        }
    }
}

static int imu_configure(void)
{
    struct rt_spi_configuration cfg;
    uint16_t value;
    int status;

    rt_pin_mode(IMU_CHIP_SELECT_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(IMU_CHIP_SELECT_PIN, PIN_HIGH);
    rt_pin_mode(IMU_RESET_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(IMU_RESET_PIN, PIN_LOW);
    rt_thread_mdelay(1);
    rt_pin_write(IMU_RESET_PIN, PIN_HIGH);
    rt_thread_mdelay(310);

    status = rt_spi_bus_attach_device_cspin(&g_imu_spi, "imu_spi", IMU_SPI_BUS_NAME, IMU_CHIP_SELECT_PIN, 0);
    if (status)
    {
        return status;
    }

    memset(&cfg, 0, sizeof(cfg));
    cfg.data_width = 16;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_3 | RT_SPI_MSB;
    cfg.max_hz = 1000000u;
    if (rt_spi_configure(&g_imu_spi, &cfg) != RT_EOK)
    {
        goto detach_spi;
    }

    if (imu_reg_read(0x72u, &value) || value != 16505u)
    {
        goto detach_spi;
    }
    if (imu_reg_write(0x5cu, 0u) || imu_reg_write(0x64u, 1u) ||
        imu_reg_write(0x60u, 0x0201u))
    {
        goto detach_spi;
    }
    if (imu_reg_read(0x5cu, &value) || value != 0u ||
        imu_reg_read(0x64u, &value) || value != 1u ||
        imu_reg_read(0x60u, &value) || value != 0x0201u)
    {
        goto detach_spi;
    }
    if (imu_reg_write(0x68u, 4u)) /* volatile self test, no flash writes */
    {
        goto detach_spi;
    }
    rt_thread_mdelay(24);
    if (imu_reg_read(0x02u, &value) || (value & 0x07ecu))
    {
        goto detach_spi;
    }

    memset(g_imu_tx, 0, sizeof(g_imu_tx));
    memset(g_imu_rx, 0, sizeof(g_imu_rx));
    g_imu_tx[0] = 0x6800u;
    status = stm32_spi1_fast_claim(&g_imu_spi, imu_done, NULL);
    if (status == RT_EOK)
    {
        return RT_EOK;
    }

detach_spi:
    (void)rt_spi_bus_detach_device(&g_imu_spi);
    return -RT_EIO;
}

static rt_err_t imu_init(rt_device_t dev)
{
    (void)dev;
    if (g_ready)
    {
        return RT_EOK;
    }

    g_timebase = rt_device_find(TIMEBASE_DEVICE_NAME);
    if (!g_timebase || rt_device_open(g_timebase, RT_DEVICE_OFLAG_RDONLY) != RT_EOK)
    {
        return -RT_EIO;
    }

    if (rt_device_control(g_timebase, TIMEBASE_CTRL_GET_TICK_HZ, &g_tick_hz) != RT_EOK ||
        !g_tick_hz || rt_mutex_init(&g_imu_lock, "imu_lock", RT_IPC_FLAG_PRIO) != RT_EOK)
    {
        goto close_timebase;
    }
    if (rt_sem_init(&g_imu_sem, "imu_dma", 0, RT_IPC_FLAG_PRIO) != RT_EOK)
    {
        goto detach_mutex;
    }
    g_head = 0u;
    g_tail = 0u;
    g_sample_head = 0u;
    g_sample_tail = 0u;
    g_inflight = 0u;
    memset(&g_stats, 0, sizeof(g_stats));

    if (rt_thread_init(&g_imu_thread, "imu_ingest", imu_ingest, 0, g_imu_stack,
                       sizeof(g_imu_stack), 2u, 1u) != RT_EOK)
    {
        goto detach_sem;
    }
    if (rt_thread_startup(&g_imu_thread) != RT_EOK || imu_configure() != RT_EOK)
    {
        (void)rt_thread_detach(&g_imu_thread);
        goto detach_sem;
    }
    g_ready = 1u;
    return RT_EOK;

detach_sem:
    (void)rt_sem_detach(&g_imu_sem);
detach_mutex:
    (void)rt_mutex_detach(&g_imu_lock);
close_timebase:
    (void)rt_device_close(g_timebase);
    g_timebase = NULL;
    return -RT_EIO;
}

static rt_err_t imu_open(rt_device_t dev, rt_uint16_t oflag)
{
    rt_err_t result;
    (void)dev;
    (void)oflag;
    if (!g_ready)
    {
        return -RT_EIO;
    }
    rt_mutex_take(&g_imu_lock, RT_WAITING_FOREVER);
    result = imu_set_capture(imu_capture);
    g_opened = result == RT_EOK;
    rt_mutex_release(&g_imu_lock);
    return result;
}

static rt_err_t imu_close(rt_device_t dev)
{
    (void)dev;
    rt_mutex_take(&g_imu_lock, RT_WAITING_FOREVER);
    (void)imu_set_capture(NULL);
    g_opened = 0u;
    imu_abort_transfer();
    g_tail = g_head;
    g_sample_tail = g_sample_head;
    rt_mutex_release(&g_imu_lock);
    return RT_EOK;
}

static rt_ssize_t imu_read(rt_device_t dev, rt_off_t pos, void *buffer,
                            rt_size_t size)
{
    struct imu_raw_sample *sample = buffer;
    (void)dev;
    (void)pos;
    if (!sample || size < sizeof(*sample))
    {
        return -RT_EINVAL;
    }

    rt_mutex_take(&g_imu_lock, RT_WAITING_FOREVER);
    if (g_sample_tail == g_sample_head)
    {
        rt_mutex_release(&g_imu_lock);
        return 0;
    }
    *sample = g_ring[g_sample_tail & (IMU_SLOTS - 1u)];
    __DMB();
    g_sample_tail++;
    rt_mutex_release(&g_imu_lock);
    return (rt_ssize_t)sizeof(*sample);
}

static rt_err_t imu_control(rt_device_t dev, int cmd, void *args)
{
    (void)dev;
    switch (cmd)
    {
    case IMU_CTRL_GET_STATS:
        if (!args)
        {
            return -RT_EINVAL;
        }
        *(struct imu_stats *)args = g_stats;
        return RT_EOK;
    default:
        return -RT_ENOSYS;
    }
}

static const struct rt_device_ops imu_ops =
{
    .init = imu_init,
    .open = imu_open,
    .close = imu_close,
    .read = imu_read,
    .control = imu_control,
};

static int imu_register(void)
{
    g_imu_dev.type = RT_Device_Class_Sensor;
    g_imu_dev.ops = &imu_ops;
    return rt_device_register(&g_imu_dev, IMU_DEVICE_NAME,
                              RT_DEVICE_FLAG_RDONLY | RT_DEVICE_FLAG_STANDALONE);
}
INIT_DEVICE_EXPORT(imu_register);
#endif /* !FLIGHT_DRIVER_CODEC_TEST */

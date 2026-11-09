#include "sample_acquisition/sensor_input.h"
#include "adis16505.h"
#include "ports/port_api.h"

static rt_device_t imu;
static void (*imu_notify)(void);

/* Simulated samples and their arrival times travel together. */
enum { IMU_INJECT_CAPACITY = 8 };
typedef struct {
    ImuSample sample;
    uint64_t arrival_us;
} InjectedImu;
static struct rt_messagequeue inject_queue;
rt_align(RT_ALIGN_SIZE) static uint8_t inject_pool[RT_MQ_BUF_SIZE(sizeof(InjectedImu), IMU_INJECT_CAPACITY)];
static volatile uint32_t inject_dropped;
static volatile uint8_t external_source;

void sensor_imu_use_external_source(bool enable)
{
    if (!imu)
        return;
    rt_enter_critical();
    (void)rt_mq_control(&inject_queue, RT_IPC_CMD_RESET, RT_NULL);
    external_source = enable ? 1u : 0u;
    rt_exit_critical();
}

Status sensor_imu_inject(const ImuSample *sample, uint64_t arrival_us)
{
    if (!external_source || !sample)
    {
        return STATUS_INVALID_ARGUMENT;
    }
    const InjectedImu message = {.sample = *sample, .arrival_us = arrival_us};
    if (rt_mq_send(&inject_queue, &message, sizeof(message)) != RT_EOK)
    {
        ++inject_dropped;
        return STATUS_BUFFER_FULL;
    }
    return STATUS_OK;
}

uint32_t sensor_imu_injected_dropped(void)
{
    return inject_dropped;
}

static rt_err_t imu_ready(rt_device_t device, rt_size_t size)
{
    void (*notify)(void) = imu_notify;
    (void)device;
    (void)size;
    if (notify)
    {
        notify();
    }
    return RT_EOK;
}

Status sensor_imu_start(void (*notify)(void))
{
    rt_device_t device = rt_device_find(IMU_DEVICE_NAME);
    if (imu || !device || rt_device_open(device, RT_DEVICE_OFLAG_RDONLY) != RT_EOK)
    {
        return STATUS_NOT_READY;
    }
    if (rt_mq_init(&inject_queue, "hilimu", inject_pool, sizeof(InjectedImu),
                   sizeof(inject_pool), RT_IPC_FLAG_FIFO) != RT_EOK)
    {
        (void)rt_device_close(device);
        return STATUS_NOT_READY;
    }
    inject_dropped = 0u;
    imu_notify = notify;
    if (rt_device_set_rx_indicate(device, imu_ready) != RT_EOK)
    {
        imu_notify = NULL;
        (void)rt_mq_detach(&inject_queue);
        (void)rt_device_close(device);
        return STATUS_NOT_READY;
    }
    imu = device;
    return STATUS_OK;
}

void sensor_imu_stop(void)
{
    if (imu)
    {
        imu_notify = NULL;
        external_source = 0u;
        (void)rt_mq_detach(&inject_queue);
        (void)rt_device_close(imu);
        imu = NULL;
    }
}

void sensor_imu_default_encoding(sensor_imu_config_t *config)
{
    const Adis16505Encoding encoding = adis16505_default_encoding();
    config->burst32 = encoding.extended;
    config->gyro_lsb_per_dps = encoding.gyro_lsb_per_dps;
    config->accel_lsb_per_mps2 = encoding.accel_lsb_per_mps2;
    config->temp_lsb_per_c = encoding.temp_lsb_per_c;
}

Status sensor_imu_read(const sensor_imu_config_t *config, sensor_context_t *context,
                            ImuSample *out, uint64_t *arrival_us)
{
    struct imu_raw_sample raw;
    uint64_t capture_us;
    Status result;

    if (external_source)
    {
        InjectedImu message;
        if (rt_mq_recv(&inject_queue, &message, sizeof(message), 0) != (rt_ssize_t)sizeof(message))
        {
            return STATUS_NOT_READY;
        }
        *out = message.sample;
        *arrival_us = message.arrival_us;
        return STATUS_OK;
    }
    if (!imu || rt_device_read(imu, 0, &raw, sizeof(raw)) != (rt_ssize_t)sizeof(raw))
    {
        return STATUS_NOT_READY;
    }
    *arrival_us = 0u;
    if (clock_now_us(arrival_us) || (raw.flags & IMU_FLAG_TRANSPORT_ERROR) ||
        clock_ticks_to_mono_us(raw.capture_tick, context->time_epoch, &capture_us))
    {
        return STATUS_INVALID_ARGUMENT;
    }
    if (!config)
        return SENSOR_ERR_ARGUMENT;
    const Adis16505Encoding encoding = {
        .extended = config->burst32,
        .gyro_lsb_per_dps = config->gyro_lsb_per_dps,
        .accel_lsb_per_mps2 = config->accel_lsb_per_mps2,
        .temp_lsb_per_c = config->temp_lsb_per_c,
    };
    ImuReading reading;
    result = adis16505_decode_payload(&encoding, raw.data, raw.length, &reading);
    if (result == SENSOR_OK)
        result = sensor_imu_build_sample(config, context, &reading, capture_us, *arrival_us, out);
    if (result == STATUS_OK)
    {
        /* Keep the 32-bit DRDY sequence and its gaps across DATA_CNTR wrap. */
        out->sample_sequence = raw.sequence;
        out->header.event_time_uncertainty_us += 1u;
    }
    return result;
}

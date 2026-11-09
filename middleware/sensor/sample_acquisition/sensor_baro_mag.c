#include "bmp585.h"
#include "bmm350.h"
#include "board_identity.h"
#include "calibration/sensor_calibration_service.h"
#include "sample_acquisition/sensor_input.h"
#include "ports/port_api.h"
#include "sample_normalization/sensor_sample.h"
#include "measurement_filter/sensor_filter.h"
#include "messages/message_ids.h"
#include <string.h>

/* Slow sensors are native RT-Thread devices. The capture tick is converted by
 * the Time Service; polling time is never presented as a measurement epoch. */
static struct
{
    uint32_t baro;
    uint32_t mag;
    uint32_t rejected;
    uint32_t untrusted_time;
    float pressure_pa;
    float altitude_m;
    float mag_ut[3];
} slow;

static sensor_context_t baro_context;
static float baro_reference_pa = 101325.0f;
static float mag_offset_ut[3];
static float mag_scale[3] = {1.0f, 1.0f, 1.0f};

void sensor_baro_set_reference(float pressure_pa)
{
    baro_reference_pa = pressure_pa;
}

void sensor_mag_set_calibration(const float offset_ut[3], const float scale[3])
{
    if (!offset_ut || !scale)
        return;
    rt_enter_critical();
    for (unsigned axis = 0; axis < 3; ++axis) {
        mag_offset_ut[axis] = offset_ut[axis];
        mag_scale[axis] = (scale[axis] > 0.1f && scale[axis] < 10.0f) ? scale[axis] : 1.0f;
    }
    rt_exit_critical();
}

void sensor_baro_mag_poll(uint64_t now, SensorMeasurementSink submit_measurement)
{
    static rt_device_t baro;
    static rt_device_t mag;
    static uint64_t last_read;
    static struct barometer_raw_sample baro_sample;
    static struct magnetometer_compensated mag_sample;
    static SensorMeasurement measurement;

    if (now - last_read < 10000)
    {
        return;
    }
    last_read = now;

    if (!baro)
    {
        baro = rt_device_find(BAROMETER_DEVICE_NAME);
        if (baro && rt_device_open(baro, RT_DEVICE_OFLAG_RDONLY) != RT_EOK)
        {
            baro = RT_NULL;
        }
    }
    if (baro && rt_device_read(baro, 0, &baro_sample, sizeof(baro_sample)) ==
                     (rt_ssize_t)sizeof(baro_sample))
    {
        BarometerReading reading;
        uint64_t capture_us = now;
        ClockInstant clock;

        baro_context.boot_id = board_boot_id();
        baro_context.time_epoch = 1u;
        baro_context.profile_id = PRODUCT_PROFILE_ID;
        baro_context.sensor_instance_id = 3u;
        memset(&measurement, 0, sizeof(measurement));

        if (clock_read(&clock) == STATUS_OK)
        {
            baro_context.boot_id = clock.boot_id;
            baro_context.time_epoch = clock.time_epoch;
            (void)clock_ticks_to_mono_us(baro_sample.capture_tick,
                                            clock.time_epoch, &capture_us);
        }

        const sensor_baro_config_t baro_config = {baro_reference_pa, .25, 1};
        if (bmp585_decode_payload(baro_sample.data, &reading) == SENSOR_OK &&
            sensor_baro_build_sample(&baro_config, &baro_context, &reading, capture_us, now,
                                     &measurement.payload.baro) == SENSOR_OK)
        {
            ContractHeader *h = &measurement.payload.baro.header;
            h->timestamp_quality = TIMESTAMP_QUALITY_RECEIVE_TIME_ESTIMATED;
            h->event_time_uncertainty_us = 10000u;
            h->timestamp_flags = TIMESTAMP_FLAG_UNCERTAINTY_BOUNDED;
            measurement.header = *h;
            measurement.sensor_instance_id = 3u;
            measurement.measurement_kind = PRODUCT_MEASUREMENT_BARO;
            slow.baro++;
            slow.altitude_m = measurement.payload.baro.altitude_m;
            slow.pressure_pa = (float)reading.pressure_pa;
            sensor_calibration_feed_baro((float)reading.pressure_pa, now);
            if (h->timestamp_quality == TIMESTAMP_QUALITY_RECEIVE_TIME_ESTIMATED ||
                h->event_time_uncertainty_us > 100u)
            {
                slow.untrusted_time++;
            }
            else if (submit_measurement(&measurement))
            {
                slow.rejected++;
            }
        }
        else
        {
            slow.rejected++;
        }
    }

    if (!mag)
    {
        mag = rt_device_find(MAG_DEVICE_NAME);
        if (mag && rt_device_open(mag, RT_DEVICE_OFLAG_RDONLY) != RT_EOK)
        {
            mag = RT_NULL;
        }
    }
    if (mag && rt_device_control(mag, MAG_CTRL_READ_COMPENSATED, &mag_sample) == RT_EOK)
    {
        if (sensor_filter_magnetometer(mag_sample.xyz_ut, mag_sample.temperature_c))
        {
            rt_enter_critical();
            for (unsigned axis = 0; axis < 3; ++axis)
                slow.mag_ut[axis] = (mag_sample.xyz_ut[axis] - mag_offset_ut[axis]) * mag_scale[axis];
            rt_exit_critical();
            sensor_calibration_feed_mag(slow.mag_ut, now);
            slow.mag++;
        }
        else
        {
            slow.rejected++;
        }
        /* No verified installation rotation yet: diagnostics stay in sensor axes. */
    }
}

static void sensors(void)
{
    rt_kprintf("baro=%u mag=%u rejected=%u untrusted_sample_time=%u\n",
               slow.baro, slow.mag, slow.rejected, slow.untrusted_time);
    rt_kprintf("pressure=%d Pa altitude=%d cm mag_sensor=(%d,%d,%d) nT\n",
               (int)slow.pressure_pa, (int)(slow.altitude_m * 100),
               (int)(slow.mag_ut[0] * 1000), (int)(slow.mag_ut[1] * 1000),
               (int)(slow.mag_ut[2] * 1000));
}
MSH_CMD_EXPORT(sensors, Flight slow sensor diagnostics and timing rejections);

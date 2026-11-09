/* M2 acquisition delivers standard samples to the application.
 * Each adapter owns its native device access and capture-time mapping. */

#ifndef SENSOR_INPUT_H
#define SENSOR_INPUT_H

#include "messages/common_messages.h"
#include "sample_normalization/sensor_sample.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef Status (*SensorMeasurementSink)(const SensorMeasurement *measurement);

void sensor_gnss_poll(uint64_t now, SensorMeasurementSink submit_measurement);
/* Fresh receiver status for display. Its timestamp is local receipt time.
 * Read only from the communication task that owns sensor_gnss_poll(). */
bool sensor_gnss_latest(uint64_t now, GnssSample *sample, double *altitude_msl_m);
void sensor_gnss_print_diagnostics(void);
/* Called only by the communication task, with a complete CRC-checked frame. */
void sensor_gnss_queue_rtcm(const uint8_t *data, size_t length);
typedef struct {
    uint64_t arrival_us;
    uint32_t unmapped, rejected;
    uint16_t flags;
    uint8_t fix_type, satellites;
} GnssLogStatus;
/* Latest decoded BESTNAV quality, including frames excluded from fusion. */
void sensor_gnss_log_status(GnssLogStatus *out);
void sensor_baro_mag_poll(uint64_t now, SensorMeasurementSink submit_measurement);
/* Apply a runtime hard/soft-iron calibration after the driver compensation. */
void sensor_baro_set_reference(float pressure_pa);
void sensor_mag_set_calibration(const float offset_ut[3], const float scale[3]);

/* One acquisition owner, with a notification after a complete native burst. */
Status sensor_imu_start(void (*notify)(void));
void sensor_imu_stop(void);
void sensor_imu_default_encoding(sensor_imu_config_t *config);

/* STATUS_NOT_READY means that there is no queued burst. Other errors report
 * a rejected burst. arrival_us is sampled immediately after the device read. */
Status sensor_imu_read(const sensor_imu_config_t *config, sensor_context_t *context,
                            ImuSample *out, uint64_t *arrival_us);

/* HIL/SIL driver double (spec 02 section 4.9): the simulated measurement enters
 * at this M2 standard-sample exit, so M3 still consumes it through
 * estimator_push_imu and no PlantTruth reaches the estimator. Selecting the
 * external source stops the device queue from being read; the samples already
 * in flight are discarded and the injected FIFO starts empty. task_sim is the
 * only producer; task_sensor is the only consumer. */
void sensor_imu_use_external_source(bool enable);

/* Non-blocking. STATUS_BUFFER_FULL means the consumer is behind; the caller
 * counts the gap and must not retry the same sample. */
Status sensor_imu_inject(const ImuSample *sample, uint64_t arrival_us);

uint32_t sensor_imu_injected_dropped(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_INPUT_H */

/* Standard sensor samples, calibration application and timestamp metadata. */
#include "sample_normalization/sensor_sample.h"

#include "messages/message_ids.h"
#include "calibration/sensor_calibration.h"
#include "measurement_filter/sensor_filter.h"

#include <math.h>
#include <string.h>

#define SENSOR_DEG_TO_RAD (3.14159265358979323846 / 180.0)

static void fill_header(ContractHeader *header, uint16_t message_type, uint16_t source_id,
                        uint16_t frame_id, sensor_context_t *context, uint64_t sample_us,
                        uint64_t publish_us, uint8_t timestamp_quality, uint8_t timestamp_flags,
                        uint32_t event_time_uncertainty_us)
{
    memset(header, 0, sizeof(*header));
    header->schema_major = (uint16_t)CONTRACT_SCHEMA_MAJOR;
    header->schema_minor = (uint16_t)CONTRACT_SCHEMA_MINOR;
    header->message_type = message_type;
    header->source_id = source_id;
    header->sequence = context->sequence++;
    header->profile_id = context->profile_id;
    header->frame_id = frame_id;
    header->boot_id = context->boot_id;
    header->clock_domain_id = CLOCK_DOMAIN_FCU_MONOTONIC;
    header->time_epoch = context->time_epoch;
    header->timestamp_sample_us = sample_us;
    header->timestamp_publish_us = publish_us;
    /* Expiry belongs to the physical epoch, never to the consumer's read. */
    header->valid_until_us = sample_us + (source_id == SOURCE_IMU ? 5000u : 20000u);
    header->event_time_uncertainty_us = event_time_uncertainty_us;
    header->timestamp_quality = timestamp_quality;
    header->timestamp_flags = timestamp_flags;
    header->validity_flags = 0;
}

sensor_status_t sensor_imu_build_sample(const sensor_imu_config_t *config, sensor_context_t *context,
                                        const ImuReading *reading, uint64_t dri_capture_us,
                                        uint64_t publish_us, ImuSample *out)
{
    float gyro_calibrated[3], accel_calibrated[3], gyro_body[3], accel_body[3];
    double corrected_delay_us;
    uint64_t sample_time_us;
    uint8_t timestamp_flags = 0u;
    size_t index;

    if (!config || !context || !reading || !out) {
        return SENSOR_ERR_ARGUMENT;
    }

    sensor_apply_calibration(config->gyro_bias_dps, config->gyro_scale, config->gyro_misalignment, reading->gyro_dps,
                      gyro_calibrated);
    sensor_apply_calibration(config->accel_bias_mps2, config->accel_scale, config->accel_misalignment,
                      reading->accel_mps2, accel_calibrated);
    sensor_rotate_vector(config->install_rotation, gyro_calibrated, gyro_body);
    sensor_rotate_vector(config->install_rotation, accel_calibrated, accel_body);

    for (index = 0; index < 3; ++index) {
        out->gyro_frd_radps[index] = (float)((double)gyro_body[index] * SENSOR_DEG_TO_RAD);
        out->accel_frd_mps2[index] = accel_body[index];
    }
    out->temperature_c = reading->temperature_c;

    /* Sample-time correction: capture time of the DRDY edge minus the
     * calibrated pipeline/group delay; never underflow below zero. */
    corrected_delay_us = config->pipeline_delay_us;
    if (!isfinite(corrected_delay_us) || (corrected_delay_us < 0.0)) {
        corrected_delay_us = 0.0;
    }
    if ((double)dri_capture_us > corrected_delay_us) {
        sample_time_us = dri_capture_us - (uint64_t)(corrected_delay_us + 0.5);
    } else {
        sample_time_us = 0u;
    }
    if (corrected_delay_us > 0.0) {
        timestamp_flags |= (uint8_t)TIMESTAMP_FLAG_PIPELINE_CORRECTED;
    }
    if (config->pipeline_delay_uncertainty_us > 0.0) {
        timestamp_flags |= (uint8_t)TIMESTAMP_FLAG_UNCERTAINTY_BOUNDED;
    }

    fill_header(&out->header, (uint16_t)MSG_IMU_SAMPLE, (uint16_t)SOURCE_IMU,
                (uint16_t)FRAME_BODY_FRD, context, sample_time_us, publish_us,
                (corrected_delay_us > 0.0) ? (uint8_t)TIMESTAMP_QUALITY_SENSOR_MODEL_CORRECTED
                                           : (uint8_t)TIMESTAMP_QUALITY_HARDWARE_CAPTURED,
                timestamp_flags | TIMESTAMP_FLAG_UNCERTAINTY_BOUNDED,
                (uint32_t)(config->pipeline_delay_uncertainty_us + 0.5));
    out->sensor_instance_id = context->sensor_instance_id;
    out->sample_sequence = reading->sequence;
    out->calibration_version = config->calibration_version;
    out->quality_flags = (uint32_t)(IMU_Q_GYRO_VALID | IMU_Q_ACCEL_VALID | IMU_Q_TEMP_VALID);
    if (reading->diagnostic_clear) {
        out->quality_flags |= (uint32_t)IMU_Q_CALIBRATED;
    }
    out->header.validity_flags = VALIDITY_PAYLOAD_VALID | VALIDITY_TIME_VALID;

    return SENSOR_OK;
}

double sensor_baro_pressure_to_altitude(double pressure_pa, double p0_pa)
{
    if (!(pressure_pa > 0.0) || !(p0_pa > 0.0)) {
        return 0.0;
    }
    return 44330.0 * (1.0 - pow(pressure_pa / p0_pa, 1.0 / 5.255));
}

sensor_status_t sensor_baro_build_sample(const sensor_baro_config_t *config, sensor_context_t *context,
                                         const BarometerReading *reading, uint64_t sample_time_us,
                                         uint64_t publish_us, BaroSample *out)
{
    if (!config || !context || !reading || !out) {
        return SENSOR_ERR_ARGUMENT;
    }
    if (!(config->p0_pa > 0.0) || !(config->variance_m2 > 0.0)) {
        return SENSOR_ERR_NOT_CONFIGURED;
    }

    if (!sensor_filter_barometer(reading)) {
        return SENSOR_ERR_RANGE;
    }

    double altitude_m = sensor_baro_pressure_to_altitude(reading->pressure_pa, config->p0_pa);

    fill_header(&out->header, (uint16_t)MSG_BARO_SAMPLE, (uint16_t)SOURCE_BARO,
                (uint16_t)FRAME_UNSPECIFIED, context, sample_time_us, publish_us,
                (uint8_t)TIMESTAMP_QUALITY_HARDWARE_CAPTURED, TIMESTAMP_FLAG_UNCERTAINTY_BOUNDED,
                0u);
    out->sensor_instance_id = context->sensor_instance_id;
    out->calibration_version = config->calibration_version;
    out->altitude_m = (float)altitude_m;
    out->variance_m2 = (float)config->variance_m2;
    out->quality_flags = (uint32_t)(BARO_Q_ALTITUDE_VALID | BARO_Q_VARIANCE_VALID |
                                    BARO_Q_REFERENCE_VALID);
    out->header.validity_flags = VALIDITY_PAYLOAD_VALID | VALIDITY_TIME_VALID;
    return SENSOR_OK;
}

void sensor_gnss_reset(sensor_gnss_state_t *state)
{
    if (state != NULL) {
        state->has_previous = false;
        state->latitude_deg = 0.0;
        state->longitude_deg = 0.0;
        state->sample_time_us = 0u;
    }
}

sensor_status_t sensor_gnss_build_sample(const sensor_gnss_config_t *config, sensor_context_t *context,
                                 sensor_gnss_state_t *state, const sensor_gnss_pvt_t *pvt,
                                 uint64_t sample_time_us, uint64_t arrival_us, uint32_t mapping_epoch,
                                 GnssSample *out)
{
    bool position_valid;
    bool velocity_valid;
    bool position_cov_valid = true;
    bool velocity_cov_valid = true;
    int index;
    uint8_t timestamp_flags = 0u;

    if ((config == NULL) || (context == NULL) || (state == NULL) || (pvt == NULL) || (out == NULL)) {
        return SENSOR_ERR_ARGUMENT;
    }
    if (!(config->max_speed_mps > 0.0) || !(config->stale_age_us > 0u) ||
        !(config->max_variance_m2 > 0.0)) {
        return SENSOR_ERR_NOT_CONFIGURED;
    }
    sensor_status_t status = sensor_filter_gnss(config, state, pvt, sample_time_us, arrival_us);
    if (status != SENSOR_OK) {
        return status;
    }

    position_valid = (pvt->valid_flags & SENSOR_GNSS_PVT_POSITION_VALID) != 0u &&
                     (pvt->fix_type != GNSS_FIX_NONE);
    velocity_valid = (pvt->valid_flags & SENSOR_GNSS_PVT_VELOCITY_VALID) != 0u &&
                     (pvt->fix_type != GNSS_FIX_NONE);

    for (index = 0; index < 3; ++index) {
        const float variance = pvt->position_variance_m2[index];
        if (!isfinite((double)variance) || !(variance > 0.0f) || (variance > config->max_variance_m2)) {
            position_cov_valid = false;
        }
    }
    for (index = 0; index < 3; ++index) {
        const float variance = pvt->velocity_variance_m2ps2[index];
        if (!isfinite((double)variance) || !(variance > 0.0f) || (variance > config->max_variance_m2)) {
            velocity_cov_valid = false;
        }
    }
    if ((pvt->valid_flags & SENSOR_GNSS_PVT_POSITION_COV_VALID) == 0u) {
        position_cov_valid = false;
    }
    if ((pvt->valid_flags & SENSOR_GNSS_PVT_VELOCITY_COV_VALID) == 0u) {
        velocity_cov_valid = false;
    }

    memset(out, 0, sizeof(*out));
    if (mapping_epoch != 0u) {
        timestamp_flags |= (uint8_t)TIMESTAMP_FLAG_EXTERNAL_CLOCK_MAPPED;
    }
    fill_header(&out->header, (uint16_t)MSG_GNSS_SAMPLE, (uint16_t)SOURCE_GNSS,
                (uint16_t)FRAME_WGS84_GEODETIC, context, sample_time_us, arrival_us,
                (mapping_epoch != 0u) ? (uint8_t)TIMESTAMP_QUALITY_EXTERNAL_MAPPED
                                      : (uint8_t)TIMESTAMP_QUALITY_RECEIVE_TIME_ESTIMATED,
                timestamp_flags, 0u);
    out->header.source_clock_domain_id = CLOCK_DOMAIN_GNSS_GPS;
    out->header.source_mapping_epoch = mapping_epoch;
    out->sensor_instance_id = context->sensor_instance_id;
    out->latitude_deg = pvt->latitude_deg;
    out->longitude_deg = pvt->longitude_deg;
    out->altitude_m = (float)pvt->altitude_m;
    for (index = 0; index < 3; ++index) {
        out->velocity_ned_mps[index] = pvt->velocity_ned_mps[index];
        out->position_variance_m2[index] = pvt->position_variance_m2[index];
        out->velocity_variance_m2ps2[index] = pvt->velocity_variance_m2ps2[index];
    }
    out->fix_type = pvt->fix_type;
    out->satellite_count = pvt->satellites;

    out->quality_flags = 0u;
    if (position_valid) {
        out->quality_flags |= (uint32_t)GNSS_Q_POSITION_VALID;
    }
    if (velocity_valid) {
        out->quality_flags |= (uint32_t)GNSS_Q_VELOCITY_VALID;
    }
    if (position_valid && position_cov_valid) {
        out->quality_flags |= (uint32_t)GNSS_Q_POSITION_COV_VALID;
    }
    if (velocity_valid && velocity_cov_valid) {
        out->quality_flags |= (uint32_t)GNSS_Q_VELOCITY_COV_VALID;
    }
    out->header.validity_flags = VALIDITY_TIME_VALID;
    if (position_valid || velocity_valid) {
        out->header.validity_flags |= VALIDITY_PAYLOAD_VALID;
    }

    state->has_previous = true;
    state->latitude_deg = pvt->latitude_deg;
    state->longitude_deg = pvt->longitude_deg;
    state->sample_time_us = sample_time_us;
    return SENSOR_OK;
}

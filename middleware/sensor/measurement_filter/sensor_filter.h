#ifndef SENSOR_FILTER_H
#define SENSOR_FILTER_H

#include "sample_normalization/sensor_sample.h"

bool sensor_filter_barometer(const BarometerReading *reading);
bool sensor_filter_magnetometer(const float xyz_ut[3], float temperature_c);
sensor_status_t sensor_filter_gnss(const sensor_gnss_config_t *config,
                                  const sensor_gnss_state_t *state, const sensor_gnss_pvt_t *pvt,
                                  uint64_t sample_time_us, uint64_t arrival_us);

#endif

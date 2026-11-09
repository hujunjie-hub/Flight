#ifndef SENSOR_READING_H
#define SENSOR_READING_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    SENSOR_OK = 0,
    SENSOR_ERR_ARGUMENT = 1,
    SENSOR_ERR_LENGTH = 2,
    SENSOR_ERR_CHECKSUM = 3,
    SENSOR_ERR_RANGE = 4,
    SENSOR_ERR_NOT_CONFIGURED = 5,
    SENSOR_ERR_JUMP = 6,
    SENSOR_ERR_STALE = 7
} sensor_status_t;

typedef struct {
    float gyro_dps[3];
    float accel_mps2[3];
    float temperature_c;
    uint32_t sequence;
    bool diagnostic_clear;
} ImuReading;

typedef struct {
    double pressure_pa;
    double temperature_c;
} BarometerReading;

#endif

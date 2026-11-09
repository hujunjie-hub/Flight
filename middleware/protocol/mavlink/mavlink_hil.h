#ifndef MAVLINK_HIL_H
#define MAVLINK_HIL_H

#include "mavlink/mavlink_frame.h"

#define MAVLINK_HIL_MAX_PAYLOAD 81u /* HIL_ACTUATOR_CONTROLS is the widest */
#define MAVLINK_HIL_RX_CAPACITY 320u
#define MAVLINK_HIL_MSG_HEARTBEAT 0u
#define MAVLINK_HIL_MSG_ACTUATOR_CONTROLS 93u
#define MAVLINK_HIL_MSG_SENSOR 107u
#define MAVLINK_HIL_MSG_GPS 113u

typedef struct {
    uint8_t data[MAVLINK_HIL_RX_CAPACITY];
    uint16_t length;
    uint32_t dropped;
} mavlink_hil_rx_t;

typedef struct {
    uint32_t id;
    uint8_t sequence;
    uint8_t length;
    uint8_t payload[MAVLINK_HIL_MAX_PAYLOAD];
} mavlink_hil_frame_t;

typedef struct {
    uint64_t sim_time_us;
    float accel_mps2[3];
    float gyro_radps[3];
    float mag_gauss[3];
    float abs_pressure_hpa;
    float pressure_alt_m;
    float temperature_c;
    uint32_t fields_updated;
} mavlink_hil_sensor_t;

typedef struct {
    uint64_t sim_time_us;
    double latitude_deg;
    double longitude_deg;
    float altitude_m;
    float eph_m;
    float epv_m;
    float velocity_ned_mps[3];
    uint8_t fix_type;
    uint8_t satellites;
} mavlink_hil_gps_t;

typedef struct {
    float quaternion[4]; /* Body FRD to earth NED, wxyz. */
    float gyro_radps[3];
    float specific_force_ned_mps2[3];
    float velocity_ned_mps[3];
    float position_ned_m[3];
    double latitude_deg;
    double longitude_deg;
    float altitude_m;
} mavlink_fmt_state_t;

uint16_t mavlink_hil_rx_feed(mavlink_hil_rx_t *rx, const uint8_t *data, uint16_t count);
int mavlink_hil_rx_next(mavlink_hil_rx_t *rx, mavlink_hil_frame_t *out);
int mavlink_hil_frame_is_sensor(const mavlink_hil_frame_t *frame);
int mavlink_hil_frame_is_gps(const mavlink_hil_frame_t *frame);
void mavlink_hil_decode_sensor(const mavlink_hil_frame_t *frame, mavlink_hil_sensor_t *out);
void mavlink_hil_decode_gps(const mavlink_hil_frame_t *frame, mavlink_hil_gps_t *out);
/* FMT dialect input used by the Host connector. Standard UART HIL uses HIL_SENSOR/GPS. */
int mavlink_hil_decode_fmt_state(const uint8_t *frame, size_t size, mavlink_fmt_state_t *out);
size_t mavlink_hil_build_actuator(uint64_t time_us, const float controls[4], uint8_t mode,
                                     uint8_t sequence, uint8_t *out, size_t capacity);
size_t mavlink_hil_build_heartbeat(uint8_t sequence, uint8_t *out, size_t capacity);

#endif
